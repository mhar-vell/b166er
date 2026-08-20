#!/usr/bin/env python3
"""
fuzzy_wb_controller — controlador whole-body com ganho Fuzzy adaptativo.

Substitui o ganho fixo K do whole_body_planner por um schedulador Mamdani
que ajusta (k_pos, k_orient, lambda_dls) em função do erro e sua variação.

Lei de controle:
  q̇_wb = J_wb†(λ) · K_fuzzy · err_EE  +  (I − J_wb† J_wb) · q̇₀

  K_fuzzy = diag(k_pos, k_pos, k_pos, k_orient, k_orient, k_orient)
  λ       = lambda_dls adaptativo

Manobra girar-avançar-girar (ver ~maneuver_enable): o DLS resolve a base
como holonômica; quando isso pede componente lateral que uma base
skid-steer não realiza, o controlador entra em ALIGN (para, gira até
apontar pra direção que o DLS pediu) antes de voltar a ADVANCE (controle
whole-body normal). Só ativo com ~nonholonomic=true.

Tópicos
-------
  Subscreve:
    /b166er/robot_state   (RobotState)   — estado completo
    /b166er/ee_target     (PoseStamped)  — pose alvo do EE
  Publica:
    /cmd_vel              (Twist)        — base Pioneer
    /b166er/arm_vel_cmd   (JointState)  — velocidade das juntas
    /b166er/fuzzy_gains   (Float64MultiArray) — [k_pos, k_orient, lambda]
    /b166er/wb_jacobian   (Float64MultiArray) — J_wb para diagnóstico
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Bool, Float64
from tf.transformations import quaternion_matrix

from b166er_whole_body_control.msg import RobotState
from b166er_whole_body_control.kinematics import (
    JOINT_NAMES, JOINT_LOWER, JOINT_UPPER, T_BASELINK_ARM, T_T265_TOOLTIP,
    whole_body_jacobian, arm_jacobian_world,
    dls_pseudoinverse, null_space_projector,
    joint_limit_gradient,
    pose_error,
)
from b166er_whole_body_control.fuzzy_gain import FuzzyGain


# ---------------------------------------------------------------------------
# Saturação de velocidade
# ---------------------------------------------------------------------------
MAX_BASE_LIN = 0.3    # m/s
MAX_BASE_ANG = 0.5    # rad/s
MAX_ARM_VEL  = 0.8    # rad/s por junta

# Teto MUITO mais baixo para a manipulação com a base travada. Com o
# teto normal, o braço saiu da postura recolhida para um alvo a 1,2 m
# usando ~1,9 rad/s em várias juntas ao mesmo tempo: o erro caiu
# 1,22 → 0,15 m em 2 s (o controle funciona), mas o CG disparou para a
# frente e o momento dinâmico tombou o Pioneer (2026-08-13). Manipular
# perto de uma parede é movimento deliberado, não corrida — e o braço
# de 28 kg sobre uma base de 41 kg tem autoridade de sobra para
# derrubar o conjunto.
MAX_ARM_VEL_LOCKED = 0.25  # rad/s por junta

# Teto de velocidade CARTESIANA na manipulação com base travada. O ganho
# Fuzzy é agressivo por projeto (aproxima rápido de longe), mas aqui o
# erro começa em ~0,3 m e o ganho pedia velocidades que faziam o braço
# ultrapassar e caçar o alvo (erro oscilando 0,15 ↔ 0,46 m em vez de
# convergir, 2026-08-13). Perto de uma parede energizada, aproximação
# lenta e monotônica vale mais que velocidade.
MAX_CART_VEL_LOCKED = 0.06  # m/s

# Peso da BASE na resolução da redundância whole-body durante a
# manipulação fina. O DLS puro distribui pela solução de norma mínima e
# não sabe que mover a base é caro: medido em 2026-08-13, ele levou o
# chassi de 0,80 m para 0,53 m da parede (encostando no limite de
# exclusão) em vez de usar o braço, porque deslocar a base "custava"
# menos em norma de velocidade.
#
# Com a pseudo-inversa PONDERADA, mover a base custa BASE_WEIGHT vezes
# mais que mover uma junta do braço. O braço faz o trabalho fino; a
# base só entra quando o braço não dá conta — que é o comportamento
# whole-body desejado, e preserva a margem de segurança.
BASE_WEIGHT = 12.0
K_NULL       = 0.3    # ganho do objetivo secundário (espaço nulo)

# Piso de velocidade linear da base. Com o braço horizontal, a direção x do
# EE é singular para o braço (∂x/∂J2 = 0) e só a base corrige o erro — mas
# comandos < ~5 mm/s não vencem o escorregamento estático do contato
# (Gazebo/ODE) nem folgas reais, e a base estaciona fora da tolerância
# (~7 mm observados). Fora da tolerância, o comando preserva o sentido mas
# nunca fica abaixo deste módulo. Sem risco de ciclo-limite: a banda de
# parada (tol_pos=5 mm para cada lado) é maior que o passo por ciclo
# (10 mm/s ÷ 20 Hz = 0,5 mm).
MIN_BASE_LIN = 0.010  # m/s


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _odom_to_matrix(odom):
    p = odom.pose.pose.position
    o = odom.pose.pose.orientation
    T = quaternion_matrix([o.x, o.y, o.z, o.w])
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _pose_stamped_to_matrix(ps):
    p = ps.pose.position
    o = ps.pose.orientation
    T = quaternion_matrix([o.x, o.y, o.z, o.w])
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _saturate_base(v_lin, v_ang):
    v_lin = float(np.clip(v_lin, -MAX_BASE_LIN, MAX_BASE_LIN))
    v_ang = float(np.clip(v_ang, -MAX_BASE_ANG, MAX_BASE_ANG))
    return v_lin, v_ang


def _lin_floor(v):
    """Aplica o piso MIN_BASE_LIN preservando o sentido.

    Só atua quando o comando é significativo (>1e-5): um v≈0 legítimo
    (erro ortogonal ao heading, a corrigir por rotação) não é inflado.
    """
    if 1e-5 < abs(v) < MIN_BASE_LIN:
        return MIN_BASE_LIN if v > 0 else -MIN_BASE_LIN
    return v


# ---------------------------------------------------------------------------
# Nó principal
# ---------------------------------------------------------------------------

class FuzzyWBController:

    def __init__(self):
        rospy.init_node('fuzzy_wb_controller')

        self._rate_hz = rospy.get_param('~rate',          20.0)
        self._nonholo = rospy.get_param('~nonholonomic',  True)
        self._ns_gain = rospy.get_param('~null_space_gain', K_NULL)

        # Tolerâncias de parada
        self._tol_pos    = rospy.get_param('~tol_pos',    0.005)   # m
        self._tol_orient = rospy.get_param('~tol_orient', 0.02)    # rad

        # Manobra girar-avançar-girar: a base é skid-steer (2 DOF de
        # controle — avanço + giro), mas o DLS resolve o erro tratando-a
        # como holonômica (x, y, θ livres, ver base_jacobian_world). Sempre
        # que a solução holonômica pede componente lateral significativa
        # (perpendicular ao heading), a base sozinha não consegue realizar
        # isso — historicamente ela "empacava" com o piso de velocidade sem
        # convergir (ver ROADMAP, limitação da Fase 3). Em vez de deixar
        # essa componente ser descartada silenciosamente na projeção não-
        # -holonômica, entra em ALIGN: para, gira até apontar pra direção
        # que o DLS pediu, e só então retoma o avanço normal (ADVANCE) —
        # cria um ponto de parada determinístico, também o lugar certo para
        # travas de segurança futuras (tilt do IMU) antes de aplicar força.
        self._maneuver_enable       = rospy.get_param('~maneuver_enable', True)
        self._heading_engage_tol    = rospy.get_param('~heading_engage_tol', 0.35)  # rad (~20°)
        self._heading_exit_tol      = rospy.get_param('~heading_exit_tol',  0.08)   # rad (~4.5°)
        self._align_min_planar_dist = rospy.get_param('~align_min_planar_dist', 0.05)  # m, piso p/ confiar no rumo até o alvo
        self._align_k                = rospy.get_param('~align_k', 1.5)  # rad/s por rad de erro de heading
        # Teto de tempo do ALIGN: numa base skid-steer real (e no ODE do
        # Gazebo) girar "no lugar" desliza — em giros longos e sustentados
        # o deslocamento translacional real desloca o rumo até o alvo mais
        # rápido do que o giro consegue alcançar, e o ALIGN nunca satisfaz
        # a tolerância de saída (observado em Gazebo com alvo muito
        # lateral). Ao vencer o teto, desiste e libera o ADVANCE — girar
        # pra sempre é pior que seguir com o melhor esforço contínuo.
        self._align_max_duration    = rospy.get_param('~align_max_duration', 8.0)  # s
        self._maneuver_state        = 'ADVANCE'   # ou 'ALIGN' — histerese entre os dois
        self._align_t_start         = None

        # Estado
        self._robot_state = None
        self._target      = None
        self._enabled     = False
        self._reached     = False

        # Erro anterior (para derivada)
        self._err_prev  = None
        self._t_prev    = None
        self._delta_err = 0.0

        # Motor Fuzzy
        self._fuzzy = FuzzyGain()

        # Publicadores
        self._pub_cmdvel  = rospy.Publisher('/cmd_vel',
                                            Twist, queue_size=1)
        self._pub_armvel  = rospy.Publisher('/b166er/arm_vel_cmd',
                                            JointState, queue_size=1)
        self._pub_gains   = rospy.Publisher('/b166er/fuzzy_gains',
                                            Float64MultiArray, queue_size=1)
        self._pub_jacobian = rospy.Publisher('/b166er/wb_jacobian',
                                             Float64MultiArray, queue_size=1)

        # Arbitragem de /cmd_vel: quando um orquestrador (chave_mission)
        # assume a navegação da base, este nó precisa CALAR — ver o
        # stand-down no spin(). Default true: comportamento antigo,
        # standalone, preservado.
        self._wb_enabled = True

        # Trava de base: quando ligada, a base fica PARADA e só o braço
        # (5 DOF) serve o alvo Cartesiano. Existe porque o whole-body
        # não tem nenhuma noção de obstáculo — na missão da chave ele
        # dirigiu a base contra a parede tentando alcançar o olhal e
        # tombou o robô (2026-08-13). Depois que a base já está
        # posicionada na pose de standoff, mover a base durante a
        # manipulação fina não agrega e só arrisca colisão.
        self._base_locked = rospy.get_param('~base_locked', False)

        # Servo da PONTA DA FERRAMENTA (não do T265). Com a orientação
        # fora do controle (ver etapa 4a), a conversão "alvo da ponta →
        # alvo do T265" deixa de valer: ela pressupõe uma orientação
        # fixa e conhecida, e a orientação passa a variar livremente.
        # Com isso ligado, o alvo publicado em /b166er/ee_target é lido
        # como a posição desejada da PONTA, e o erro/Jacobiana são
        # calculados nela.
        self._servo_tooltip = rospy.get_param('~servo_tooltip', False)

        # RESTRIÇÃO DE APROXIMAÇÃO (2026-08-13). Substitui a trava total
        # de base. A trava resolvia a colisão com a parede, mas tirava a
        # base da solução: sobravam 5 juntas para a tarefa inteira e o
        # DLS saturava J2/J3 nos batentes, congelando o erro em ~10 cm.
        # Whole-body existe justamente para distribuir a tarefa entre
        # base e braço — o Marco notou isso observando a simulação ("o
        # Pioneer poderia atuar de forma mais otimizada").
        #
        # Em vez de travar, restringe: a base participa dos 8 DOF, mas a
        # componente de velocidade que a levaria para DENTRO de um plano
        # de exclusão é projetada fora. O plano vem em
        # /b166er/base_keepout (posição = ponto no plano; eixo X da
        # orientação = normal apontando para o LADO SEGURO).
        self._keepout_p = None      # ponto do plano (mundo)
        self._keepout_n = None      # normal unitária, lado seguro
        self._keepout_d = rospy.get_param('~base_keepout_dist', 0.55)

        # MEDIDA DIRETA do Hokuyo (laser_safety). Preferida ao plano
        # inferido pela tag: o plano depende da estimativa da parede,
        # que carrega o erro da detecção; o laser mede o obstáculo real
        # à frente, com ~1,5 cm de erro (calibrado em 2026-08-13).
        # Também protege contra obstáculos que a tag não descreve.
        self._front_clearance = None
        self._use_laser = rospy.get_param('~use_laser_keepout', True)

        # Subscritores
        rospy.Subscriber('/b166er/robot_state', RobotState,   self._cb_state)
        rospy.Subscriber('/b166er/ee_target',   PoseStamped,  self._cb_target)
        rospy.Subscriber('/b166er/wb_enable',   Bool,         self._cb_wb_enable)
        rospy.Subscriber('/b166er/base_lock',   Bool,         self._cb_base_lock)
        rospy.Subscriber('/b166er/servo_tooltip', Bool,       self._cb_servo_tooltip)
        rospy.Subscriber('/b166er/base_keepout', PoseStamped, self._cb_keepout)
        rospy.Subscriber('/b166er/front_clearance', Float64, self._cb_clearance)

        rospy.loginfo('[fuzzy_wb_controller] pronto — aguardando /b166er/ee_target')

    # ------------------------------------------------------------------
    def _cb_state(self, msg):
        self._robot_state = msg

    def _cb_base_lock(self, msg):
        if msg.data != self._base_locked:
            rospy.loginfo('[fuzzy_wb_ctrl] base %s',
                          'TRAVADA (só braço serve o alvo)' if msg.data
                          else 'liberada (whole-body 8-DOF)')
        self._base_locked = msg.data

    def _cb_keepout(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        R = quaternion_matrix([o.x, o.y, o.z, o.w])[:3, :3]
        self._keepout_p = np.array([p.x, p.y, p.z])
        self._keepout_n = R[:, 0]          # eixo X = normal do lado seguro
        rospy.loginfo('[fuzzy_wb_ctrl] plano de exclusão: ponto=%s normal=%s '
                      'dist_min=%.2f m', self._keepout_p.round(3),
                      self._keepout_n.round(3), self._keepout_d)

    def _cb_clearance(self, msg):
        self._front_clearance = msg.data

    def _apply_keepout(self, q_dot_base, p_base, theta):
        """Projeta fora a componente de velocidade que invade o plano.

        Deixa a base LIVRE para tudo o mais — só remove o movimento que
        a aproximaria além do limite. É uma restrição de velocidade, não
        uma trava: a redundância dos 8 DOF continua disponível ao DLS.
        """
        # ── Restrição por MEDIDA (Hokuyo) ──
        # Bloqueia avanço quando o laser vê obstáculo dentro do limite,
        # independentemente do que a estimativa da tag diga. É a defesa
        # que não depende de inferência nenhuma.
        if self._use_laser and self._front_clearance is not None:
            if self._front_clearance < self._keepout_d:
                # ATENÇÃO AO FRAME: q_dot_base vem da Jacobiana
                # whole-body no frame do MUNDO ([ẋ, ẏ, θ̇]), não no
                # frame do robô. A primeira versão testava
                # q_dot_base[0] > 0 achando que era "avanço" — mas isso
                # é a componente X do mundo. Com o robô encarando a
                # parede em +Y, ele avançava em ẏ e a checagem nunca
                # disparava: o robô foi de 0,80 m a 0,31 m da parede e
                # tombou (pitch de 1,31 rad, 2026-08-13).
                #
                # O correto é projetar no HEADING e remover só a
                # componente que aproxima.
                fwd = np.array([np.cos(theta), np.sin(theta)])
                v_fwd = float(np.dot(q_dot_base[:2], fwd))
                if v_fwd > 0.0:
                    q_dot_base = q_dot_base.copy()
                    q_dot_base[:2] = q_dot_base[:2] - v_fwd * fwd
                    rospy.logwarn_throttle(2.0,
                        '[fuzzy_wb] laser: obstáculo a %.2f m (mín %.2f) — '
                        'avanço de %.3f m/s bloqueado',
                        self._front_clearance, self._keepout_d, v_fwd)

        if self._keepout_p is None:
            return q_dot_base
        n_xy = np.array([self._keepout_n[0], self._keepout_n[1]])
        nn = np.linalg.norm(n_xy)
        if nn < 1e-6:
            return q_dot_base
        n_xy = n_xy / nn

        d = float(np.dot(p_base[:2] - self._keepout_p[:2], n_xy))
        v_n = float(np.dot(q_dot_base[:2], n_xy))
        # v_n < 0 = indo em direção ao plano (lado inseguro)
        if d <= self._keepout_d and v_n < 0.0:
            q_dot_base = q_dot_base.copy()
            q_dot_base[:2] = q_dot_base[:2] - v_n * n_xy
            rospy.loginfo_throttle(2.0,
                '[fuzzy_wb] restrição de aproximação ativa: base a %.2f m '
                '(mín %.2f) — componente de %.3f m/s removida',
                d, self._keepout_d, -v_n)
        return q_dot_base

    def _cb_servo_tooltip(self, msg):
        if msg.data != self._servo_tooltip:
            rospy.loginfo('[fuzzy_wb_ctrl] alvo interpretado como %s',
                          'PONTA DA FERRAMENTA' if msg.data else 'T265')
        self._servo_tooltip = msg.data

    def _cb_wb_enable(self, msg):
        if msg.data != self._wb_enabled:
            rospy.loginfo('[fuzzy_wb_ctrl] %s /cmd_vel',
                          'assumindo' if msg.data else 'liberando (stand-down)')
        self._wb_enabled = msg.data
        if not msg.data:
            # Zera o braço ao sair: navegação acontece com o braço parado.
            self._publish_arm_vel(np.zeros(5), rospy.Time.now())

    def _cb_target(self, msg):
        self._target   = _pose_stamped_to_matrix(msg)
        self._enabled  = True
        self._reached  = False
        self._err_prev = None   # reset derivada ao mudar alvo
        rospy.loginfo('[fuzzy_wb_ctrl] novo alvo: pos=(%.3f, %.3f, %.3f)',
                      msg.pose.position.x,
                      msg.pose.position.y,
                      msg.pose.position.z)

    # ------------------------------------------------------------------
    def _update_delta_err(self, err_total_norm, now):
        """Estima d(||err||)/dt via diferença de primeira ordem."""
        if self._err_prev is None or self._t_prev is None:
            self._delta_err = 0.0
        else:
            dt = (now - self._t_prev).to_sec()
            if dt > 1e-6:
                self._delta_err = (err_total_norm - self._err_prev) / dt
        self._err_prev = err_total_norm
        self._t_prev   = now

    def _get_ee_matrix(self, state):
        """PoseStamped do EE → 4×4."""
        p = state.ee_pose.pose.position
        o = state.ee_pose.pose.orientation
        T = quaternion_matrix([o.x, o.y, o.z, o.w])
        T[:3, 3] = [p.x, p.y, p.z]
        return T

    def _project_nonholo(self, q_dot_base, theta):
        """Projeta ẋ, ẏ na direção de heading do Pioneer."""
        v_fwd = np.cos(theta) * q_dot_base[0] + np.sin(theta) * q_dot_base[1]
        omega  = q_dot_base[2]
        return v_fwd, omega

    # ------------------------------------------------------------------
    def _publish_cmd_vel(self, q_dot_base, theta):
        msg = Twist()
        if self._nonholo:
            v, w = self._project_nonholo(q_dot_base, theta)
            v, w = _saturate_base(v, w)
            msg.linear.x  = _lin_floor(v)
            msg.angular.z = w
        else:
            msg.linear.x  = _lin_floor(float(np.clip(q_dot_base[0], -MAX_BASE_LIN, MAX_BASE_LIN)))
            msg.linear.y  = _lin_floor(float(np.clip(q_dot_base[1], -MAX_BASE_LIN, MAX_BASE_LIN)))
            msg.angular.z = float(np.clip(q_dot_base[2], -MAX_BASE_ANG, MAX_BASE_ANG))
        self._pub_cmdvel.publish(msg)

    def _publish_arm_vel(self, q_dot_arm, stamp):
        vmax = (MAX_ARM_VEL_LOCKED
                if (self._base_locked or self._servo_tooltip) else MAX_ARM_VEL)
        # Satura preservando a DIREÇÃO do movimento: clipar junta a junta
        # distorce a trajetória Cartesiana justamente quando o limite
        # morde (várias juntas saturam juntas na largada).
        n = float(np.max(np.abs(q_dot_arm)))
        if n > vmax:
            q_dot_arm = q_dot_arm * (vmax / n)
        msg          = JointState()
        msg.header.stamp = stamp
        msg.name     = JOINT_NAMES
        msg.velocity = q_dot_arm.tolist()
        self._pub_armvel.publish(msg)

    def _publish_gains(self, k_pos, k_orient, lam):
        msg      = Float64MultiArray()
        msg.data = [k_pos, k_orient, lam]
        self._pub_gains.publish(msg)

    # ------------------------------------------------------------------
    def _update_maneuver_state(self, base_xy, target_xy, theta, now):
        """Decide ALIGN vs ADVANCE com histerese.

        Usa o rumo (bearing) da base até a posição XY do alvo como
        referência — não a direção crua da solução holonômica do DLS.
        A primeira versão usava essa direção do DLS, mas ela some com o
        próprio braço rígido preso na base: com o braço parado durante o
        ALIGN, girar a base faz o EE orbitar em torno do centro de giro,
        deslocando a direção "ótima" a cada ciclo — o controlador perseguia
        um alvo que se movia por causa do próprio giro e nunca convergia
        (oscilação observada em Gazebo, heading_err indo a ±π sem cair). O
        rumo até a posição XY do alvo é fixo enquanto a base só gira no
        lugar (v=0), então não sofre desse acoplamento.

        base_xy, target_xy : (x, y) mundial, metros.
        """
        if not (self._nonholo and self._maneuver_enable):
            self._maneuver_state = 'ADVANCE'
            return 0.0

        dx, dy = target_xy[0] - base_xy[0], target_xy[1] - base_xy[1]
        planar_dist = float(np.hypot(dx, dy))
        if planar_dist < self._align_min_planar_dist:
            # Base já está sobre a posição XY do alvo (erro é só em z ou
            # orientação) — rumo indefinido/ruidoso, não força giro.
            heading_error = 0.0
        else:
            desired_theta = float(np.arctan2(dy, dx))
            heading_error = float(np.arctan2(np.sin(desired_theta - theta),
                                             np.cos(desired_theta - theta)))

        if self._maneuver_state == 'ADVANCE':
            if abs(heading_error) > self._heading_engage_tol:
                self._maneuver_state = 'ALIGN'
                self._align_t_start  = now
                rospy.loginfo('[fuzzy_wb_ctrl] manobra: ADVANCE -> ALIGN '
                              '(heading_err=%.3frad)', heading_error)
        else:
            elapsed = (now - self._align_t_start).to_sec() if self._align_t_start else 0.0
            if abs(heading_error) < self._heading_exit_tol:
                self._maneuver_state = 'ADVANCE'
                rospy.loginfo('[fuzzy_wb_ctrl] manobra: ALIGN -> ADVANCE')
            elif elapsed > self._align_max_duration:
                self._maneuver_state = 'ADVANCE'
                rospy.logwarn('[fuzzy_wb_ctrl] manobra: ALIGN excedeu %.1fs sem '
                              'convergir (heading_err=%.3frad) — desistindo, '
                              'provável deslizamento da base durante o giro. '
                              'ADVANCE segue com o melhor esforço.',
                              self._align_max_duration, heading_error)

        return heading_error

    def _run_align(self, heading_error, stamp, k_pos, k_orient, lam):
        """Fase ALIGN: gira em torno do próprio eixo, braço parado.

        Não calcula J_wb (evita o custo do DLS quando só se está girando),
        então não publica /b166er/wb_jacobian nesta fase.
        """
        omega = float(np.clip(self._align_k * heading_error,
                              -MAX_BASE_ANG, MAX_BASE_ANG))
        twist = Twist()
        twist.angular.z = omega
        self._pub_cmdvel.publish(twist)
        self._publish_arm_vel(np.zeros(5), stamp)
        self._publish_gains(k_pos, k_orient, lam)
        rospy.loginfo_throttle(1.0,
            '[fuzzy_wb] ALIGN heading_err=%.3frad omega=%.3f',
            heading_error, omega)

    def _publish_jacobian(self, J):
        msg = Float64MultiArray()
        d0  = MultiArrayDimension(label='rows', size=J.shape[0],
                                  stride=J.shape[0] * J.shape[1])
        d1  = MultiArrayDimension(label='cols', size=J.shape[1],
                                  stride=J.shape[1])
        msg.layout.dim = [d0, d1]
        msg.data       = J.flatten().tolist()
        self._pub_jacobian.publish(msg)

    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(self._rate_hz)

        while not rospy.is_shutdown():
            now = rospy.Time.now()

            if not self._wb_enabled:
                # Stand-down: outro nó é o dono de /cmd_vel agora (ex.:
                # chave_mission navegando a base). Publicar nem que seja
                # zero aqui brigaria com ele — os dois comandos chegariam
                # intercalados a 20 Hz no Gazebo e a base andaria aos
                # trancos. Silêncio total é o handoff correto.
                rate.sleep()
                continue

            if not self._enabled or self._robot_state is None:
                self._pub_cmdvel.publish(Twist())
                rate.sleep()
                continue

            state = self._robot_state

            # ---- 1. Erro de pose do EE --------------------------------
            T_cur    = self._get_ee_matrix(state)
            err_EE   = pose_error(T_cur, self._target)     # (6,)

            err_pos_norm    = float(np.linalg.norm(err_EE[:3]))
            err_orient_norm = float(np.linalg.norm(err_EE[3:]))
            err_total_norm  = err_pos_norm + 0.5 * err_orient_norm

            self._update_delta_err(err_total_norm, now)

            # Critério de parada com histerese: entra em "alcançado" dentro
            # da tolerância e só reativa se o erro crescer 50% além dela —
            # evita chaveamento 0 ↔ MIN_BASE_LIN quando o erro paira na borda.
            if self._servo_tooltip or self._base_locked:
                # Só posição, e medida na ponta quando servo_tooltip —
                # ver etapa 4a. O teste de alcance real acontece lá; aqui
                # só evita declarar "alcançado" cedo demais.
                if self._servo_tooltip:
                    r_tool = T_cur[:3, :3] @ T_T265_TOOLTIP[:3, 3]
                    err_tip = float(np.linalg.norm(
                        self._target[:3, 3] - (T_cur[:3, 3] + r_tool)))
                    if err_tip < self._tol_pos:
                        self._reached = True
                elif err_pos_norm < self._tol_pos:
                    self._reached = True
            elif err_pos_norm < self._tol_pos and err_orient_norm < self._tol_orient:
                self._reached = True
            elif (err_pos_norm > 1.5 * self._tol_pos
                  or err_orient_norm > 1.5 * self._tol_orient):
                self._reached = False
            if self._reached:
                rospy.loginfo_throttle(2.0,
                    '[fuzzy_wb_ctrl] alvo alcançado — pos=%.4fm ori=%.4frad',
                    err_pos_norm, err_orient_norm)
                self._pub_cmdvel.publish(Twist())
                rate.sleep()
                continue

            # ---- 2. Ganho Fuzzy ----------------------------------------
            k_pos, k_orient, lam = self._fuzzy.compute(
                err_pos_norm, err_orient_norm, self._delta_err)

            # Vetor de velocidade cartesiana desejada
            xdot_d = np.concatenate([
                k_pos    * err_EE[:3],
                k_orient * err_EE[3:],
            ])

            # ---- 3. Pose da base + manobra girar-avançar-girar ----------
            T_world_base = _odom_to_matrix(state.base_odom)
            q_arm        = np.array(state.q_arm)
            theta        = float(np.arctan2(T_world_base[1, 0], T_world_base[0, 0]))

            # A manobra ALIGN só faz sentido com a base LIVRE — ela gira
            # o chassi e zera o braço. Com a base travada isso trava a
            # tarefa inteira: observado ao vivo em 2026-08-13, a missão
            # ficou 60 s no "engage" sem o braço sair do lugar (erro
            # constante de 0,82 m) porque o ALIGN engatava a cada ciclo,
            # girava a base e mandava velocidade zero para as juntas.
            if not (self._base_locked or self._servo_tooltip):
                # ALIGN também fica FORA da manipulação fina com base
                # livre: ali a base participa com deslocamentos
                # pequenos e coordenados: parar tudo para girar o chassi
                # arrancaria o gancho do olhal.
                #
                # Se a base ainda não aponta na direção geral do alvo,
                # para tudo e gira antes de deixar o DLS/projeção
                # não-holonômica tentar (e falhar) corrigir erro lateral
                # com a base andando.
                heading_error = self._update_maneuver_state(
                    T_world_base[:2, 3], self._target[:2, 3], theta, now)
                if self._maneuver_state == 'ALIGN':
                    self._run_align(heading_error, now, k_pos, k_orient, lam)
                    rate.sleep()
                    continue

            # ---- 4a. Servo da PONTA DA FERRAMENTA (posição) -------------
            # Vale com a base travada (5 DOF) E com a base livre (8 DOF).
            # Com a base livre + restrição de aproximação é o modo
            # normal da manipulação: whole-body de verdade, que é o que
            # evita saturar as juntas do braço (ver _apply_keepout).
            if self._servo_tooltip:
                # NÃO AGIR SOBRE ESTIMATIVA RUIM: o braço é controlado a
                # partir de q estimado por IK (sem encoders). Em
                # movimento essa IK às vezes não converge, e usar esse q
                # monta uma Jacobiana errada — o controlador empurra na
                # direção errada e passa a caçar o alvo.
                if not state.ik_converged:
                    self._publish_arm_vel(np.zeros(5), now)
                    self._pub_cmdvel.publish(Twist())
                    rospy.logwarn_throttle(2.0,
                        '[fuzzy_wb] IK não convergiu (res=%.3fm) — parado até '
                        'a estimativa reassentar', state.ik_residual_pos)
                    rate.sleep()
                    continue

                # Jacobiana: whole-body (6×8) ou só braço (6×5).
                if self._base_locked:
                    J6 = arm_jacobian_world(q_arm, T_world_base @ T_BASELINK_ARM)
                else:
                    J6 = whole_body_jacobian(q_arm, T_world_base)

                # Transporta a Jacobiana do T265 para a PONTA (ponto
                # rigidamente preso): J_lin_ponta = J_lin − [r]× J_ang.
                p_t265 = T_cur[:3, 3]
                r_tool = T_cur[:3, :3] @ T_T265_TOOLTIP[:3, 3]
                r_skew = np.array([[0.0, -r_tool[2],  r_tool[1]],
                                   [r_tool[2], 0.0, -r_tool[0]],
                                   [-r_tool[1], r_tool[0], 0.0]])
                J_pos = J6[:3, :] - r_skew @ J6[3:, :]

                # POSIÇÃO APENAS: 5 DOF não fecham pose 6D, e a tarefa da
                # chave especifica direções de força, não orientação de
                # ferramenta (ver chave_seccionadora_task.md).
                err_lin = self._target[:3, 3] - (p_t265 + r_tool)
                err_pos_norm = float(np.linalg.norm(err_lin))

                xdot = k_pos * err_lin
                sp = float(np.linalg.norm(xdot))
                if sp > MAX_CART_VEL_LOCKED:
                    xdot = xdot * (MAX_CART_VEL_LOCKED / sp)

                if self._base_locked:
                    J_pin = dls_pseudoinverse(J_pos, lam)
                    q_dot = J_pin @ xdot
                else:
                    # DLS PONDERADO: q̇ = W⁻¹Jᵀ(JW⁻¹Jᵀ + λ²I)⁻¹ ẋ
                    # W penaliza a base (ver BASE_WEIGHT).
                    w = np.ones(J_pos.shape[1])
                    w[:3] = BASE_WEIGHT
                    Winv = np.diag(1.0 / w)
                    JW = J_pos @ Winv
                    q_dot = Winv @ J_pos.T @ np.linalg.solve(
                        JW @ J_pos.T + lam**2 * np.eye(3), xdot)
                # Sem termo de espaço nulo: ele empurra as juntas para o
                # meio da faixa e briga com a configuração escolhida pela
                # IK do sequenciador (em 2026-08-13 arrastou J3 de +58°
                # até o batente oposto).

                if self._base_locked:
                    self._pub_cmdvel.publish(Twist())
                    self._publish_arm_vel(q_dot, now)
                else:
                    q_dot_base = self._apply_keepout(q_dot[:3], T_world_base[:3, 3], theta)
                    self._publish_cmd_vel(q_dot_base, theta)
                    self._publish_arm_vel(q_dot[3:], now)

                self._publish_gains(k_pos, k_orient, lam)
                self._publish_jacobian(J_pos)
                if err_pos_norm < self._tol_pos:
                    self._reached = True
                rospy.loginfo_throttle(1.0,
                    '[fuzzy_wb] PONTA (%s) err=%.4f | q_dot=%s',
                    'base travada' if self._base_locked else 'whole-body 8-DOF',
                    err_pos_norm, q_dot.round(3))
                rate.sleep()
                continue

            # ---- 4. Jacobiana whole-body e DLS ---------------------------
            J_wb  = whole_body_jacobian(q_arm, T_world_base)
            J_pin = dls_pseudoinverse(J_wb, lam)
            N     = null_space_projector(J_wb, J_pin)

            q_dot_task = J_pin @ xdot_d
            q_dot_null = N @ np.concatenate([
                np.zeros(3),
                self._ns_gain * joint_limit_gradient(q_arm),
            ])

            q_dot_wb = q_dot_task + q_dot_null   # (8,)

            # ---- 5. Publica comandos ------------------------------------
            self._publish_cmd_vel(q_dot_wb[:3], theta)
            self._publish_arm_vel(q_dot_wb[3:], now)
            self._publish_gains(k_pos, k_orient, lam)
            self._publish_jacobian(J_wb)

            rospy.loginfo_throttle(1.0,
                '[fuzzy_wb] err_p=%.4fm err_o=%.4frad δerr=%.4f | '
                'k=[%.2f %.2f] λ=%.3f | '
                'base=[%.2f %.2f] arm=[%.2f %.2f %.2f %.2f %.2f]',
                err_pos_norm, err_orient_norm, self._delta_err,
                k_pos, k_orient, lam,
                *q_dot_wb[:2].round(3), *q_dot_wb[3:].round(3))

            rate.sleep()


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        FuzzyWBController().spin()
    except rospy.ROSInterruptException:
        pass
