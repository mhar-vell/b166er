#!/usr/bin/env python3
"""
arm_joint_servo — executa posturas e as velocidades do Fuzzy num braço SEM
encoders, em malha aberta de junta (modo braco:=malha_aberta).

Substitui, com a MESMA interface, a gazebo_arm_bridge (simulação) e o
arm_vel_integrator (hardware): um só nó para os dois modos, porque o que
muda entre eles é só quem recebe /setpoints — o firmware emulado
(arm_openloop_sim.py) ou o firmware de velocidade nos Arduinos.

Entradas
  /b166er/arm_vel_cmd     (JointState.velocity, rad/s) — velocidades do Fuzzy
  /b166er/arm_posture_cmd (JointState.position, rad)   — postura pedida
  /b166er/robot_state     (RobotState) — q_arm ESTIMADO (T265 → IK) e pose
                          do efetuador medida pela T265; nunca /joint_states
  /b166er/tilt_critical   (Bool) — para tudo
  /b166er/arm_resync      (JointState) — cancela postura ativa (reset)
  /b166er/arm_limit_switch (JointState, −1/0/+1 por junta) — da placa
  /b166er/arm_home_cmd    (Bool) — refaz o homing por switch
Saídas
  /setpoints                   (movemaster_msg/setpoint) set_1..5 em GRAUS/S
  /b166er/arm_posture_reached  (Bool, latched)
  /b166er/arm_posture_ok       (Bool, latched) — True alcançada, False TIMEOUT/BLOQUEADA
  /b166er/arm_homed            (Bool, latched) — homing concluído sem timeout
  /b166er/arm_posture_target   (JointState, latched) — semente do estimador

Como uma postura é executada sem encoder: resolved-rate sobre a pose
MEDIDA pelo T265, v = J⁺(q_est)·[kp_pos·Δp; kp_ori·Δθ] (DLS), saturada
em ~ramp_velocity — a malha fecha no sensor que o robô tem, e perto do
cotovelo reto (onde a IK alterna de ramo) a lei não depende do ramo. Sem
medida da ponta, reserva: v = kp·(q_alvo − q_estimado). "Chegou" é julgado pela medida
que o robô tem: a posição do efetuador medida pela T265 a menos de
~tol_ee da FK(q_alvo), sustentada por ~estavel_s; o erro de junta
ESTIMADO abaixo de ~tol_q só decide quando não há medida da ponta. Passado ~timeout
a postura fecha com aviso e resíduo no log — a missão compensa medindo a
ponta, como sempre fez (ver _reach_by_iterative_ik).

Enquanto uma postura está ativa, /b166er/arm_vel_cmd é ignorado (mesma
regra da ponte antiga). Sem comando novo do Fuzzy por ~vel_timeout, zero.
"""
import math
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf.transformations import quaternion_matrix
from movemaster_msg.msg import setpoint as SetpointMsg
from b166er_whole_body_control.msg import RobotState
from b166er_whole_body_control.kinematics import (
    JOINT_NAMES, JOINT_LOWER, JOINT_UPPER, T_BASELINK_ARM, fk_arm,
    arm_jacobian_world, dls_pseudoinverse, pose_error)


class ArmJointServo:
    def __init__(self):
        rospy.init_node('arm_joint_servo')
        self._rate_hz   = float(rospy.get_param('~rate', 20.0))
        self._kp        = float(rospy.get_param('~kp', 1.5))          # 1/s
        self._v_max     = float(rospy.get_param('/arm_postures/ramp_velocity', 0.3))
        self._tol_q     = float(rospy.get_param('~tol_q', 0.03))      # rad
        # 6 → 3 mm (2026-10-08, abortos em malha aberta): com a postura
        # fechando ao ENTRAR em 6 mm, a ponta parava do lado de onde vinha
        # — viés de +3 mm em profundidade e em altura em 30 atravessas —
        # contra tolerâncias de fase de 6/5 mm; 5 de 30 atravessas não
        # fecharam em 5 iterações. A lei de tarefa chega a 1–3 mm.
        self._tol_ee    = float(rospy.get_param('~tol_ee', 0.003))    # m (era 0,03 → 0,006 → 0,003)
        self._estavel_s = float(rospy.get_param('~estavel_s', 0.5))
        self._timeout   = float(rospy.get_param('~timeout', 20.0))   # era 30; stow<->deploy leva 11-14 s
        self._vel_to    = float(rospy.get_param('~vel_timeout', 0.5))
        # PISO DE VELOCIDADE (2026-10-07, 1ª missão em malha aberta). Na fase
        # destrava o Fuzzy pedia 1–2 °/s por junta: abaixo da zona morta do
        # firmware (PWM mínimo que vence o atrito do redutor) nada se move, e
        # a fase morreu por timeout a 12 mm do alvo. O braço de posição
        # integrava essas velocidades miúdas; um motor real não. Quando o
        # vetor de velocidades do Fuzzy é não nulo mas pequeno, ele é
        # ESCALADO para que a maior componente chegue a ~v_piso_deg
        # (mantém a direção; a T265 fecha o resto). Tem de ser > V_DEAD do
        # firmware (1,5 °/s).
        self._v_piso    = math.radians(float(rospy.get_param('~v_piso_deg', 3.0)))
        # CRITÉRIO DE CHEGADA PELA PONTA (2026-10-07, bateria malha_aberta
        # 3/5). As 22 posturas da bateria fecharam todas por "alcançada",
        # mas com a ponta a 7–21 mm da FK do alvo e resíduos de junta de
        # até 10° (J3/J4 se compensando): tol_ee de 30 mm é mais grosso que
        # as tolerâncias das fases no frame da parede (5–8 mm em altura e
        # profundidade), e as runs 3 e 5 morreram em "5 iterações sem
        # fechar" com alt +12/+14 mm. Agora, quando a T265 está disponível,
        # a postura só fecha com a ponta a menos de ~tol_ee (6 mm) da FK do
        # alvo; o critério de junta (~tol_q) fica como reserva para quando
        # não há medida da ponta. Para a ponta chegar lá sem encoder, o laço
        # de postura também recebe o PISO de velocidade (abaixo de V_DEAD
        # o firmware não move) e uma zona morta FINA por junta (~tol_q_fina)
        # para não bater em torno do alvo.
        self._tol_q_fina = float(rospy.get_param('~tol_q_fina', 0.005))  # rad (0,3°)
        # LEI NO ESPAÇO DA TAREFA (2026-10-07, bateria montagem_90_servo
        # 2/5, 21 posturas por TIMEOUT). A lei de junta v = kp·(q_alvo −
        # q_est) depende de q_est estar no ramo certo — e nas posturas da
        # tarefa o cotovelo fica quase reto (J3 ≈ 0), onde os dois ramos da
        # IK são vizinhos (J3 ±13°) e a estimativa alterna entre eles a cada
        # ciclo; a lei de junta então empurrava J2/J3/J4 em direções que não
        # reduziam o erro da ponta e a postura morria por timeout. O que o
        # robô MEDE é a pose do T265: a lei passa a ser resolved-rate sobre
        # o erro de pose medido, v = J⁺(q_est)·[kp_p·Δp; kp_o·Δθ], com DLS
        # — perto do cotovelo reto a coluna de J3 é pequena e o ramo deixa
        # de importar. A lei de junta fica como reserva sem medida da ponta.
        self._kp_p      = float(rospy.get_param('~kp_pos', 2.0))      # 1/s
        self._kp_o      = float(rospy.get_param('~kp_ori', 1.5))      # 1/s
        self._tol_ang   = float(rospy.get_param('~tol_ang', 0.05))    # rad (~3°)
        self._v_fina    = math.radians(float(rospy.get_param('~v_fina_deg', 0.25)))   # 0,5 → 0,25 (08/10: com tol 3 mm a ponta parava a 5 mm com todas as juntas em repouso)
        self._dls_lam   = float(rospy.get_param('~dls_lambda', 0.05))
        # Duas etapas (teste de 11:42: resolved-rate direto do stow para o
        # deploy fechou "BLOQUEADA" — com 1,6 rad de erro angular as linhas
        # angulares, em rad, dominam as de posição, em m, e a DLS leva a
        # ponta para longe). LONGE do alvo vale a lei de junta (robusta para
        # movimentos grandes; o ramo só confunde perto do cotovelo reto e
        # com erro pequeno); PERTO (~d_tarefa_m / ~ang_tarefa) entra a lei
        # da tarefa, com as linhas angulares pesadas por ~w_ang (m por rad).
        self._d_tarefa  = float(rospy.get_param('~d_tarefa_m', 0.04))
        self._ang_tarefa = float(rospy.get_param('~ang_tarefa', 0.35))
        self._w_ang     = float(rospy.get_param('~w_ang', 0.1))
        # JUNTA BLOQUEADA: se há comando e a ponta não se aproxima do alvo
        # por ~bloqueio_s (contato — o firmware aplica PWM e nada se move),
        # a postura fecha com ok=False em vez de esperar o timeout; a
        # missão já trata "ponta não avança" com recuo por contato.
        self._bloqueio_s   = float(rospy.get_param('~bloqueio_s', 3.0))
        self._bloqueio_dmin = float(rospy.get_param('~bloqueio_progresso_m', 0.001))
        self._hist_dp = []   # (t, ‖Δp‖) da postura ativa
        # HISTERESE das zonas mortas finas (bateria 4): uma junta na borda
        # da zona morta alternava 0 / ±piso a cada ciclo; o freio emulado
        # (e o atrito estático real) engole os pulsos, a junta quase não
        # anda, mas o dead reckoning integra o piso — foi assim que q_dr de
        # J1 derivou −25° → −75°. Em repouso a junta só volta a andar quando
        # o erro passa de ~hist× a zona morta.
        self._hist_fator = float(rospy.get_param('~histerese', 2.0))   # 3 → 2 (08/10)
        self._repouso = np.zeros(5, dtype=bool)
        self._goto_home = bool(rospy.get_param('~goto_home_on_start', True))
        home = rospy.get_param('/arm_postures/stow_home', [0.0, 1.10, -1.04, -1.8, 0.0])
        self._q_home = np.array(home, dtype=float)

        self._state = None
        self._q_est = None
        self._p_ee  = None
        self._T_ee  = None
        self._T_wb  = None
        self._v_fuzzy = np.zeros(5)
        self._t_fuzzy = None
        self._tilt = False
        self._q_alvo = None
        self._t_alvo = None
        self._t_ok = None
        self._home_feito = False
        # HOMING POR SWITCH (2026-10-07, pedido do Marco: "simular os switchs
        # de cada junta"). Como o controlador original do RV-M2: ao ligar,
        # cada junta de ~/arm_switches/home_ordem anda devagar para o lado
        # home_side até o seu switch fechar (/b166er/arm_limit_switch, que
        # vem da placa); o estimador ancora a junta no ângulo do switch. É a
        # única referência ABSOLUTA de junta do braço sem encoder — sem ela
        # a estimativa depende de o braço ter partido do stow. Depois do
        # último switch, a postura recolhida como sempre. Enquanto o homing
        # está ativo, posturas e Fuzzy são ignorados.
        self._homing_on_start = bool(rospy.get_param('~homing_on_start', False))
        # Depois do homing o braço FICA nos switches — é a posição de HOME
        # do laboratório, de onde a missão parte para o SEARCH (Marco,
        # 08/10). 'stow' recolhe como antes.
        self._apos_homing = rospy.get_param('~apos_homing', 'home')
        # HOME POSITION (09/10): depois do último switch o braço vem para
        # /arm_postures/home (cintura e rolagem em zero, J2/J3/J4 onde os
        # switches fecham) e só então /b166er/arm_homed é publicado.
        # 'switches' deixa o braço nos fins de curso (comportamento de 08/10).
        self._q_home_pos = np.array(rospy.get_param('/arm_postures/home', [0.0, 1.108, -1.021, -1.894, 0.0]), dtype=float)
        self._homing_para_home = False
        self._home_side  = np.array(rospy.get_param('/arm_switches/home_side', [0, 0, 0, 0, 0]), dtype=float)
        self._home_ordem = [int(j) - 1 for j in rospy.get_param('/arm_switches/home_ordem', [])]
        self._home_v     = np.radians(np.array(rospy.get_param('/arm_switches/home_v_deg', [5.0] * 5), dtype=float))
        self._home_to    = float(rospy.get_param('/arm_switches/home_timeout_s', 40.0))
        self._ls = np.zeros(5)
        # J2 só para trás (Marco, 07/10): limite de curso por software a
        # partir do switch de trás — ver arm_switches.yaml (j2_curso_deg).
        # O limite vem da cinemática (B166ER_J2_MIN_DEG, padrão 0°): a IK, o
        # servo e o Fuzzy enxergam o MESMO J2 mínimo.
        self._j2_min = float(JOINT_LOWER[1])
        self._j2_max = float(JOINT_UPPER[1])
        rospy.loginfo('[arm_joint_servo] J2 limitado por software a %.1f° … %.1f° '
                      '(B166ER_J2_MIN_DEG / B166ER_J2_MAX_DEG); homing do J2 só para trás',
                      math.degrees(self._j2_min), math.degrees(self._j2_max))
        self._homing = []          # fila de índices de junta ainda por fazer
        self._homing_t0 = None
        self._homing_resultado = {}
        # "IR PARA HOME" de qualquer lugar (Marco, 08/10: "deve existir um
        # processo para ir à HOME, pois nada garante que o robô esteja na
        # posição que precisamos entre uma missão e outra"). Sem encoder o
        # único sinal de que a junta ANDA é a T265: se, comandando a junta,
        # a ponta não se desloca ~homing_bloqueio_mm em ~homing_bloqueio_s,
        # a junta está presa (contato, batente, obstáculo) — recua
        # ~homing_recuo_s no sentido oposto e tenta de novo UMA vez; na
        # segunda, a junta fica 'BLOQUEADA' e o homing falha (a missão
        # trata). Junta já no switch passa direto.
        self._hb_mm   = float(rospy.get_param('~homing_bloqueio_mm', 3.0))
        self._hb_s    = float(rospy.get_param('~homing_bloqueio_s', 2.0))
        self._hrec_s  = float(rospy.get_param('~homing_recuo_s', 1.5))
        self._h_fase  = 'avanca'     # avanca | recua
        self._h_tent  = 0
        self._h_hist  = []           # (t, p_ee) durante o avanço da junta atual
        self._h_t_rec = None
        self._pub_homed = rospy.Publisher('/b166er/arm_homed', Bool, queue_size=1, latch=True)
        rospy.Subscriber('/b166er/arm_limit_switch', JointState, self._cb_ls, queue_size=1)
        rospy.Subscriber('/b166er/arm_home_cmd', Bool, self._cb_home_cmd, queue_size=1)

        self._pub_sp = rospy.Publisher('/setpoints', SetpointMsg, queue_size=1)
        self._pub_reached = rospy.Publisher('/b166er/arm_posture_reached', Bool,
                                            queue_size=1, latch=True)
        # True = alcançada de verdade, False = fechou por TIMEOUT. O estimador
        # só reancora o dead reckoning no alvo quando é True.
        self._pub_ok = rospy.Publisher('/b166er/arm_posture_ok', Bool,
                                       queue_size=1, latch=True)
        self._pub_target = rospy.Publisher('/b166er/arm_posture_target', JointState,
                                           queue_size=1, latch=True)
        rospy.Subscriber('/b166er/robot_state', RobotState, self._cb_state, queue_size=1)
        rospy.Subscriber('/b166er/arm_vel_cmd', JointState, self._cb_vel, queue_size=1)
        rospy.Subscriber('/b166er/arm_posture_cmd', JointState, self._cb_posture, queue_size=1)
        rospy.Subscriber('/b166er/arm_resync', JointState, self._cb_resync, queue_size=1)
        rospy.Subscriber('/b166er/tilt_critical', Bool, self._cb_tilt)
        rospy.loginfo('[arm_joint_servo] malha aberta de junta: kp %.2f, v_max %.2f rad/s, '
                      'tol_q %.3f rad, tol_ee %.3f m, timeout %.0f s',
                      self._kp, self._v_max, self._tol_q, self._tol_ee, self._timeout)

    # ------------------------------------------------------------ callbacks
    def _cb_state(self, m):
        self._state = m
        if len(m.q_arm) == 5:
            self._q_est = np.array(m.q_arm, dtype=float)
        p = m.ee_pose.pose.position
        self._p_ee = np.array([p.x, p.y, p.z])
        o = m.ee_pose.pose.orientation
        Te = quaternion_matrix([o.x, o.y, o.z, o.w])
        Te[:3, 3] = self._p_ee
        self._T_ee = Te
        b = m.base_odom.pose.pose
        T = quaternion_matrix([b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w])
        T[:3, 3] = [b.position.x, b.position.y, b.position.z]
        self._T_wb = T

    def _cb_vel(self, m):
        if len(m.velocity) == 5:
            self._v_fuzzy = np.array(m.velocity, dtype=float)
            self._t_fuzzy = rospy.Time.now()

    def _cb_posture(self, m):
        if self._tilt:
            rospy.logwarn_throttle(5.0, '[arm_joint_servo] postura ignorada: inclinação crítica')
            return
        if len(m.position) == 5:
            self._iniciar_postura(np.clip(np.array(m.position, dtype=float),
                                          JOINT_LOWER, JOINT_UPPER), 'comando externo')

    def _cb_resync(self, _m):
        # Reset da simulação: as juntas foram teleportadas; não há estado
        # integrado aqui para reancorar — só cancelar a postura ativa.
        if self._q_alvo is not None:
            rospy.logwarn('[arm_joint_servo] resync: postura ativa cancelada')
        self._q_alvo = None

    def _cb_ls(self, m):
        if len(m.position) == 5:
            self._ls = np.array(m.position, dtype=float)

    def _cb_home_cmd(self, m):
        if m.data:
            self._iniciar_homing('comando externo')

    # ------------------------------------------------------------ homing
    def _iniciar_homing(self, why):
        if self._home_side[1] < 0:
            rospy.logerr('[arm_joint_servo] homing: J2 para BAIXO é proibido (risco; Marco 07/10) — J2 fica fora do homing')
            self._home_side[1] = 0.0
        fila = [j for j in self._home_ordem if 0 <= j < 5 and self._home_side[j] != 0]
        if not fila:
            rospy.logwarn('[arm_joint_servo] homing (%s): nenhuma junta configurada em /arm_switches', why)
            return
        self._q_alvo = None
        self._homing = fila
        self._homing_t0 = None
        self._homing_resultado = {}
        self._h_fase, self._h_tent, self._h_hist, self._h_t_rec = 'avanca', 0, [], None
        # NÃO publica False aqui: /b166er/arm_homed só carrega o RESULTADO
        # (True/False) — a missão (estado HOME) espera a próxima mensagem
        # depois de pedir o homing, e um False no início era lido como falha.
        rospy.loginfo('[arm_joint_servo] homing (%s): juntas %s para os switches %s',
                      why, [JOINT_NAMES[j] for j in fila], [int(self._home_side[j]) for j in fila])

    def _passo_homing(self):
        """Devolve v (rad/s) do homing, ou None se não há homing ativo."""
        if not self._homing:
            return None
        j = self._homing[0]
        agora = rospy.Time.now()
        if self._homing_t0 is None:
            self._homing_t0 = agora
        lado = self._home_side[j]
        t_j = (agora - self._homing_t0).to_sec()
        def _proxima(resultado):
            self._homing_resultado[JOINT_NAMES[j]] = resultado
            self._homing.pop(0); self._homing_t0 = None
            self._h_fase, self._h_tent, self._h_hist, self._h_t_rec = 'avanca', 0, [], None
        if self._h_fase == 'recua':
            if (agora - self._h_t_rec).to_sec() >= self._hrec_s:
                self._h_fase, self._h_hist = 'avanca', []
                rospy.logwarn('[arm_joint_servo] homing: %s — recuou, tentando o switch de novo (%d/2)',
                              JOINT_NAMES[j], self._h_tent + 1)
            else:
                v = np.zeros(5); v[j] = -lado * max(self._home_v[j], self._v_piso)
                return v
        elif self._ls[j] == lado:
            _proxima('switch %+d em %.1f s' % (int(lado), t_j))
            rospy.loginfo('[arm_joint_servo] homing: %s no switch %+d (%.1f s)', JOINT_NAMES[j], int(lado), t_j)
        elif t_j > self._home_to:
            _proxima('TIMEOUT')
            rospy.logwarn('[arm_joint_servo] homing: %s não chegou ao switch %+d em %.0f s — seguindo',
                          JOINT_NAMES[j], int(lado), self._home_to)
        else:
            # bloqueio: a ponta (T265) não se desloca enquanto a junta é comandada
            if self._p_ee is not None:
                self._h_hist.append((agora.to_sec(), self._p_ee.copy()))
                self._h_hist = [h for h in self._h_hist if agora.to_sec() - h[0] <= self._hb_s + 0.05]
                if (len(self._h_hist) > 3 and agora.to_sec() - self._h_hist[0][0] >= self._hb_s
                        and max(float(np.linalg.norm(h[1] - self._h_hist[0][1])) for h in self._h_hist[1:]) < self._hb_mm / 1000.0):
                    self._h_tent += 1
                    if self._h_tent >= 2:
                        rospy.logerr('[arm_joint_servo] homing: %s BLOQUEADA — a ponta não se move comandando o switch %+d (2 tentativas)',
                                     JOINT_NAMES[j], int(lado))
                        _proxima('BLOQUEADA')
                    else:
                        rospy.logwarn('[arm_joint_servo] homing: %s presa (ponta parada %.1f s) — recuando %.1f s antes de tentar de novo',
                                      JOINT_NAMES[j], self._hb_s, self._hrec_s)
                        self._h_fase, self._h_t_rec, self._h_hist = 'recua', agora, []
                        v = np.zeros(5); v[j] = -lado * max(self._home_v[j], self._v_piso)
                        return v
        if not self._homing:
            ok = all(r not in ('TIMEOUT', 'BLOQUEADA') for r in self._homing_resultado.values())
            rospy.loginfo('[arm_joint_servo] homing concluído %s: %s', 'OK' if ok else 'COM FALHA', self._homing_resultado)
            if not ok or self._apos_homing == 'switches':
                self._pub_homed.publish(Bool(data=ok))
                rospy.loginfo('[arm_joint_servo] homing: braço fica nos fins de curso')
            elif self._apos_homing == 'stow':
                self._pub_homed.publish(Bool(data=ok))
                self._iniciar_postura(self._q_home, 'recolhido após homing')
            else:
                self._homing_para_home = True
                self._iniciar_postura(self._q_home_pos, 'HOME POSITION após os switches')
            return np.zeros(5)
        v = np.zeros(5)
        j = self._homing[0]
        v[j] = self._home_side[j] * max(self._home_v[j], self._v_piso)
        return v

    def _cb_tilt(self, m):
        self._tilt = bool(m.data)
        if self._tilt and self._q_alvo is not None:
            rospy.logwarn('[arm_joint_servo] inclinação crítica: postura cancelada')
            self._q_alvo = None

    # ------------------------------------------------------------ postura
    def _iniciar_postura(self, q, why):
        if q[1] < self._j2_min or q[1] > self._j2_max:
            rospy.logwarn('[arm_joint_servo] postura (%s): J2 %.1f° fora do limite de software %.1f° … %.1f° — cortada',
                          why, math.degrees(q[1]), math.degrees(self._j2_min), math.degrees(self._j2_max))
            q = q.copy(); q[1] = min(max(q[1], self._j2_min), self._j2_max)
        self._q_alvo = q
        self._t_alvo = rospy.Time.now()
        self._t_ok = None
        self._hist_dp = []
        self._repouso[:] = False
        self._pub_reached.publish(Bool(data=False))
        tgt = JointState()
        tgt.header.stamp = rospy.Time.now()
        tgt.name = JOINT_NAMES
        tgt.position = q.tolist()
        self._pub_target.publish(tgt)
        rospy.loginfo('[arm_joint_servo] postura (%s): %s rad', why, np.round(q, 3))

    def _erro_pose(self):
        """Erro 6-vetor [Δp, Δθ] (mundo) entre a pose MEDIDA do T265 e a FK
        da postura-alvo, ou None sem medida."""
        if self._q_alvo is None or self._T_ee is None or self._T_wb is None:
            return None
        T_alvo = self._T_wb @ T_BASELINK_ARM @ fk_arm(self._q_alvo.tolist())
        return pose_error(self._T_ee, T_alvo)

    def _zona_morta(self, x, limiar):
        """Zona morta fina com histerese por junta: entra em repouso abaixo
        de `limiar`, sai só acima de histerese×limiar."""
        self._repouso = np.where(self._repouso, np.abs(x) < self._hist_fator * limiar,
                                 np.abs(x) < limiar)
        return ~self._repouso

    def _lei_junta(self, e):
        """Longe do alvo: lei de junta por junta, consciente da placa."""
        v = self._kp * e
        v = np.sign(v) * np.maximum(np.abs(v), self._v_piso)
        v = np.where(self._zona_morta(e, self._tol_q_fina), v, 0.0)
        return np.clip(v, -self._v_max, self._v_max)

    def _lei_tarefa(self, e6):
        """Resolved-rate sobre o erro de pose medido, com DLS."""
        J = arm_jacobian_world(self._q_est.tolist(), self._T_wb @ T_BASELINK_ARM)
        J = np.vstack([J[:3], self._w_ang * J[3:]])          # rad -> m-equivalente
        ref = np.concatenate([self._kp_p * e6[:3], self._kp_o * self._w_ang * e6[3:]])
        v = dls_pseudoinverse(J, self._dls_lam) @ ref
        v = np.clip(v, -self._v_max, self._v_max)
        # consciente da placa: junta com pedido miúdo descansa (o freio
        # segura); as outras andam a pelo menos v_piso (zona morta do PWM)
        v_fino = np.where(self._zona_morta(v, self._v_fina),
                          np.sign(v) * np.maximum(np.abs(v), self._v_piso), 0.0)
        # Se a zona morta fina deixou TODAS as juntas em repouso com a ponta
        # ainda fora da tolerância (bateria 3: posturas fechadas "BLOQUEADA"
        # a 7–9 mm sem ninguém se mexer), a junta que mais ajuda anda no piso.
        if not np.any(v_fino) and np.any(v):
            i = int(np.argmax(np.abs(v)))
            v_fino[i] = np.sign(v[i]) * self._v_piso
        return v_fino

    def _passo_postura(self):
        """Devolve v (rad/s) para a postura ativa, ou None se não há postura."""
        if self._q_alvo is None:
            return None
        if self._q_est is None:
            return np.zeros(5)
        e = self._q_alvo - self._q_est
        e6 = self._erro_pose()
        agora = rospy.Time.now()
        if e6 is not None:
            dp, dth = float(np.linalg.norm(e6[:3])), float(np.linalg.norm(e6[3:]))
            perto = dp < self._d_tarefa and dth < self._ang_tarefa
            v = self._lei_tarefa(e6) if perto else self._lei_junta(e)
            chegou = dp < self._tol_ee and dth < self._tol_ang
            # bloqueio (só perto, onde há contato): comando não nulo e a
            # ponta não se aproxima
            if perto:
                self._hist_dp.append((agora.to_sec(), dp))
            else:
                self._hist_dp = []
            self._hist_dp = [h for h in self._hist_dp if agora.to_sec() - h[0] <= self._bloqueio_s + 0.05]
            if (perto and not chegou and np.any(v != 0.0) and len(self._hist_dp) > 3
                    and agora.to_sec() - self._hist_dp[0][0] >= self._bloqueio_s
                    and self._hist_dp[0][1] - min(h[1] for h in self._hist_dp[1:]) < self._bloqueio_dmin):
                self._fechar_postura(e, dp, 'BLOQUEADA')
                return np.zeros(5)
        else:
            v = self._lei_junta(e)
            dp = None
            chegou = bool(np.all(np.abs(e) < self._tol_q))
        if chegou:
            if self._t_ok is None:
                self._t_ok = agora
            if (agora - self._t_ok).to_sec() >= self._estavel_s:
                self._fechar_postura(e, dp, 'alcançada')
                return np.zeros(5)
        else:
            self._t_ok = None
        if (agora - self._t_alvo).to_sec() > self._timeout:
            self._fechar_postura(e, dp, 'TIMEOUT')
            return np.zeros(5)
        return v

    def _fechar_postura(self, e, e_ee, como):
        msg = ('[arm_joint_servo] postura %s: erro estimado %s°, ponta a %s da FK do alvo'
               % (como, np.degrees(e).round(1).tolist(),
                  ('%.3f m' % e_ee) if e_ee is not None else 'n/d'))
        (rospy.loginfo if como == 'alcançada' else rospy.logwarn)(msg)
        self._q_alvo = None
        self._pub_ok.publish(Bool(data=(como == 'alcançada')))   # antes do reached (latched)
        self._pub_reached.publish(Bool(data=True))
        if self._homing_para_home:
            self._homing_para_home = False
            ok = como in ('alcançada', 'BLOQUEADA')   # bloqueada = parou a poucos mm, ainda é HOME
            rospy.loginfo('[arm_joint_servo] HOME POSITION %s (%s)', 'alcançada' if ok else 'NÃO alcançada', como)
            self._pub_homed.publish(Bool(data=ok))

    # ------------------------------------------------------------ laço
    def spin(self):
        rate = rospy.Rate(self._rate_hz)
        while not rospy.is_shutdown():
            if self._state is None:
                rate.sleep()
                continue
            if not self._home_feito:
                self._home_feito = True
                if self._homing_on_start:
                    self._iniciar_homing('ao ligar')
                elif self._goto_home:
                    self._iniciar_postura(self._q_home, 'home inicial')
            v = self._passo_homing()
            if v is None:
                v = self._passo_postura()
            if v is None:
                # sem postura: velocidades do Fuzzy, com watchdog
                if (self._t_fuzzy is not None
                        and (rospy.Time.now() - self._t_fuzzy).to_sec() <= self._vel_to):
                    v = self._v_fuzzy.copy()
                    vmax = float(np.max(np.abs(v)))
                    if 1e-6 < vmax < self._v_piso:
                        v = v * (self._v_piso / vmax)
                else:
                    v = np.zeros(5)
            if self._tilt:
                v = np.zeros(5)
            # limite de software do J2 (vale para posturas, homing e Fuzzy):
            # abaixo de j2_min nunca se comanda para baixo
            if self._q_est is not None and self._q_est[1] <= self._j2_min and v[1] < 0.0:
                v[1] = 0.0
            if self._q_est is not None and self._q_est[1] >= self._j2_max and v[1] > 0.0 and not self._homing:
                v[1] = 0.0      # (no homing o J2 vai até o switch, que fecha exatamente aqui)
            m = SetpointMsg()
            vd = np.degrees(v)
            m.set_1, m.set_2, m.set_3, m.set_4, m.set_5 = (float(x) for x in vd)
            m.set_GRIP = False
            m.emergency_stop = bool(self._tilt)
            m.GoHome = 0
            self._pub_sp.publish(m)
            rate.sleep()


if __name__ == '__main__':
    try:
        ArmJointServo().spin()
    except rospy.ROSInterruptException:
        pass
