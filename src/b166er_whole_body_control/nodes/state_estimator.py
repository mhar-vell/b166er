#!/usr/bin/env python3
"""
state_estimator — whole-body state for b166er without joint encoders.

Fuses:
  • T265 odometry  → EE pose in world frame
  • Pioneer odom   → base pose (x, y, θ) in world frame

Estimates arm joint angles via numerical IK so the whole-body controller
can reason about the full 8-DOF chain (3 base + 5 arm) as a single entity.
"""

import rospy
import math
import numpy as np
from collections import deque
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf.transformations import quaternion_matrix

from b166er_whole_body_control.msg import RobotState
from b166er_whole_body_control.kinematics import (
    JOINT_NAMES, JOINT_LOWER, JOINT_UPPER, T_BASELINK_ARM,
    ik_arm,
    arm_jacobian_world)
from movemaster_msg.msg import setpoint as SetpointMsg

# Mesma HOME_Q do gazebo_arm_bridge: pose não-singular para warm-start do IK
_HOME_Q = np.array([0.0, -0.5, 0.8, 0.0, 0.0])


def _odom_to_matrix(odom):
    """nav_msgs/Odometry → 4×4 homogeneous transform."""
    p = odom.pose.pose.position
    o = odom.pose.pose.orientation
    T = quaternion_matrix([o.x, o.y, o.z, o.w])
    T[:3, 3] = [p.x, p.y, p.z]
    return T


class StateEstimator:

    def __init__(self):
        rospy.init_node('state_estimator')

        self._t265_topic    = rospy.get_param('~t265_odom_topic',    '/t265/odom/sample')
        self._pioneer_topic = rospy.get_param('~pioneer_odom_topic', '/pioneer/pose')
        self._world_frame   = rospy.get_param('~world_frame',        'odom')
        self._pub_rate      = rospy.get_param('~rate', 20.0)

        # Atalho de conveniência (Gazebo apenas): usa /joint_states real como
        # seed do IK. NÃO é fiel ao robô sem encoder — nem em Gazebo (o braço
        # real não tem essa informação) nem em modo hardware, onde o mesmo
        # tópico hoje existe mas carrega ruído do pino de encoder desconectado
        # (movemaster_control/state_publisher, ver ROADMAP Fase 4). Default
        # false: a estimativa recursiva (self._q_arm do ciclo anterior) é o
        # único seed condizente com o hardware real. Ligar só para demos de
        # tracking puro em Gazebo que não dizem respeito ao tuning do Fuzzy.
        self._use_joint_states_seed = rospy.get_param('~use_joint_states_seed', False)
        self._ik_retry_min_res = rospy.get_param('~ik_retry_min_residual', 0.02)  # m
        # Teto de iterações por solve (2026-09-11, RELATORIO17 adendo 2): o
        # laço é de tempo real e parte da estimativa anterior; quando
        # converge, converge em poucas iterações. Sem teto, um ciclo que
        # não converge custava 300 it × 11 FKs (1,6 s de simulação no NUC)
        # e a pose da BASE — que nem depende da IK — saía junto no atraso.
        self._ik_max_iter = rospy.get_param('~ik_max_iter', 40)
        # Sincronização base ↔ T265 pelo stamp: com a base andando, o
        # base_odom mais RECENTE e o T265 são de instantes diferentes e o
        # alvo da IK do braço sai inconsistente (medido: até 6 mm a
        # 0,15 m/s; com o stamp mais próximo, 0,1 mm). Vale também no
        # robô real (RosAria a 10 Hz × T265 a 200 Hz).
        self._base_sync_max_dt = rospy.get_param('~base_sync_max_dt', 0.2)  # s
        self._base_buf = deque(maxlen=400)   # (stamp_secs, Odometry)
        self._base_lock = threading.Lock()   # callback (thread do rospy) × spin

        self._base_odom  = None
        self._t265_odom  = None
        self._q_arm      = _HOME_Q.copy()   # evita singularidade q=0 no primeiro ciclo
        self._q_arm_prev = _HOME_Q.copy()
        self._dq_arm     = np.zeros(5)
        # DEAD RECKONING DOS COMANDOS (2026-10-06, modo braco:=malha_aberta).
        # A pose da T265 (posição + orientação do punho) NÃO distingue o
        # cotovelo para cima do cotovelo para baixo: J2, J3 e J4 têm eixos
        # paralelos, e as duas configurações dão o mesmo punho. Observado na
        # primeira subida em malha aberta: a IK convergiu (resíduo < 1 mm)
        # no ramo espelhado, o executor acreditou e levou o J4 ao lugar
        # errado. O que o robô real sabe e a T265 não diz é o que ele
        # COMANDOU: integrar as velocidades enviadas ao firmware (/setpoints,
        # graus/s) dá uma estimativa grosseira mas no ramo certo. Ela serve
        # só para escolher o ramo: quando a solução da IK se afasta dela
        # mais que ~dr_desacordo_rad, resolve-se de novo a partir dela e
        # fica a solução mais próxima do que foi comandado. Depois a
        # própria estimativa reancora o dead reckoning (sem acumular deriva).
        # dr_seed DESLIGADO por padrão (2026-10-07, bateria 5): o dead
        # reckoning nunca ficou confiável (piso, histerese, contato — ver
        # RELATORIO25) e a re-IK a partir dele é o que TROCAVA um ramo certo
        # por um errado (bateria 5 run4: braço levado ao batente de J3 e
        # estimativa presa no espelho). O ramo passa a ser seguido por
        # CONTINUIDADE da estimativa desde o stow conhecido.
        self._dr_seed      = bool(rospy.get_param('~dr_seed', False))
        self._seed_gt      = bool(rospy.get_param('~seed_ground_truth', False))
        # RAMO PELA VELOCIDADE MEDIDA (2026-10-07). A pose do T265 não separa
        # cotovelo para cima de cotovelo para baixo (mesma ponta, mesmo
        # punho), e seguir só por continuidade prende a estimativa no
        # espelho na primeira passagem pelo cotovelo reto (teste 13:20:
        # J3 estimado −60° com a verdade em +60°). O que separa os ramos é
        # a DIREÇÃO em que a ponta anda para o mesmo comando de junta:
        # J(q_a)·v ≠ J(q_b)·v. A cada ciclo com comando e movimento, os dois
        # candidatos (atual e espelhado) preveem a velocidade da ponta; a
        # medida (diferença finita da posição do T265 em ~rv_janela_s) é
        # comparada com as duas e o erro acumula com esquecimento ~rv_tau_s.
        # Troca-se de ramo quando o espelho explica a medida com menos de
        # ~rv_fator do erro do atual, com ~rv_nmin amostras.
        self._ramo_vel     = bool(rospy.get_param('~ramo_vel', False))
        # RAMO PELO SINAL DE J3 NA PASSAGEM PELO COTOVELO RETO (2026-10-07,
        # 13:30). O teste pela velocidade escolheu o espelho (as duas
        # soluções da IK estavam no batente). Regra mais simples e que o
        # robô real sabe: o ramo só muda quando o cotovelo passa por zero, e
        # nesse instante o sinal de J3 depois da passagem é o sinal da
        # VELOCIDADE COMANDADA em J3. Perto de zero (|J3| < ~j3_zona) guarda-
        # se sign(v_cmd J3) como sinal esperado; fora da zona, se a estimativa
        # tem o sinal contrário, a IK é resolvida do espelho e, se ele tem o
        # sinal certo com resíduo igual ou melhor, substitui. Postura
        # alcançada reancora o sinal esperado no alvo.
        self._ramo_sinal   = bool(rospy.get_param('~ramo_sinal', True))
        self._j3_zona      = math.radians(float(rospy.get_param('~j3_zona_deg', 8.0)))
        self._j3_sinal     = float(np.sign(_HOME_Q[2])) if abs(_HOME_Q[2]) > 1e-3 else None
        # FINS DE CURSO como referência absoluta (2026-10-07, bateria 6):
        # /b166er/arm_limit_switch (−1/0/+1 por junta) vem das placas (pinos
        # LS; emulado em arm_openloop_sim). Junta no switch está num ângulo
        # CONHECIDO: a semente da IK recebe o limite, e J3 no switch fixa o
        # sinal do ramo — é a única medida absoluta de junta que o braço
        # tem, e foi o que faltou quando a estimativa ficou presa no espelho
        # com o braço de verdade no batente.
        self._ls = np.zeros(5)
        rospy.Subscriber('/b166er/arm_limit_switch', JointState, self._cb_ls, queue_size=1)
        self._rv_j3_min    = math.radians(float(rospy.get_param('~rv_j3_min_deg', 8.0)))
        self._rv_vmin      = float(rospy.get_param('~rv_vmin', 0.005))     # m/s
        self._rv_janela    = float(rospy.get_param('~rv_janela_s', 0.25))
        self._rv_tau       = float(rospy.get_param('~rv_tau_s', 1.0))
        self._rv_nmin      = float(rospy.get_param('~rv_nmin', 4.0))
        self._rv_fator     = float(rospy.get_param('~rv_fator', 0.5))
        self._rv_hist      = deque(maxlen=40)   # (t, p_T265 no frame do braço)
        self._rv_Ea = self._rv_Eb = self._rv_n = 0.0
        self._rv_t_prev    = None
        self._dr_desacordo = float(rospy.get_param('~dr_desacordo_rad', 0.35))
        self._q_dr         = np.array(rospy.get_param('~q_inicial', [0.0, 0.0, 0.0, 0.0, 0.0]),
                                      dtype=float)
        self._v_cmd        = np.zeros(5)
        self._t_v_cmd      = None
        self._t_dr         = None
        # O dead reckoning integra o que o FIRMWARE EXECUTA, não o que foi
        # pedido (2026-10-07, run1 montagem_90_servo): o servo mandava 0,5–
        # 1,4 °/s em J1/J4, abaixo da zona morta da placa (V_DEAD 1,5 °/s em
        # arm_openloop_sim e nos Joints*_vel.ino) — o motor não se mexia, mas
        # q_dr integrava e derivou +20° em J4 e +20° em J1 em um minuto; a
        # escolha do ramo passou a ser feita a partir de lixo e oscilou a
        # cada ciclo. Mesma zona morta e saturação da placa, por junta.
        self._dr_v_dead    = math.radians(float(rospy.get_param('~dr_v_dead_deg', 1.5)))
        self._dr_v_max     = math.radians(float(rospy.get_param('~dr_v_max_deg', 60.0)))
        # Reancoragem SUAVE dentro do ramo (2026-10-07, bateria montagem_90_
        # servo): com a ponta em contato (atravessa/captura) o firmware aplica
        # PWM e a junta não anda — q_dr integrava +3 °/s em J1 por um minuto
        # (J1 −19° → +70°) e a escolha do ramo virou lixo. Enquanto a IK
        # concorda com q_dr (desacordo ≤ ~dr_desacordo_rad), q_dr é puxado
        # para a estimativa com constante ~dr_tau_s; um salto de ramo (> gate)
        # continua NÃO sendo seguido — que era o defeito da 1ª versão.
        self._dr_tau       = float(rospy.get_param('~dr_tau_s', 2.0))
        # Reancorar no alvo SÓ quando a postura foi de fato alcançada: o
        # arm_joint_servo também publica reached=True ao fechar por TIMEOUT,
        # e aí o alvo não é onde o braço está. /b166er/arm_posture_ok
        # (latched) diz qual foi o caso; as pontes antigas não o publicam e
        # ficam no comportamento anterior (sempre reancora).
        self._posture_ok   = True
        self._t_prev     = None
        self._q_joints     = None    # ground truth mais recente (None em hardware)
        # Buffer (stamp_secs, q) para sincronizar seed do IK com o stamp do T265.
        # sensor_sim publica T265 com stamp = stamp do /joint_states que gerou o FK,
        # então o q com stamp mais próximo é exatamente a solução → IK converge em 0 iter.
        self._q_joints_buf = deque(maxlen=50)

        self._pub_state  = rospy.Publisher('/b166er/robot_state',
                                           RobotState, queue_size=5)
        self._pub_joints = rospy.Publisher('/b166er/estimated_joint_states',
                                           JointState, queue_size=5)

        rospy.Subscriber(self._pioneer_topic, Odometry, self._cb_pioneer)
        rospy.Subscriber(self._t265_topic,    Odometry, self._cb_t265)
        if self._use_joint_states_seed:
            rospy.Subscriber('/joint_states', JointState,
                             self._cb_joint_states, queue_size=1)
            rospy.logwarn('[state_estimator] use_joint_states_seed=true — '
                          'seed via ground truth, NÃO fiel ao robô sem encoder')

        # Re-seed por postura comandada (2026-08-13): posturas dobradas
        # (ex.: home recolhido) têm múltiplas soluções de IK, e o DLS
        # recursivo pode cair no ramo espelhado do cotovelo e ESTABILIZAR
        # lá (observado ao vivo: real J2=+61°/J4=-103°, estimado
        # J2=+18°/J4=+110°, resíduo constante de 4,9cm sem nunca
        # convergir). Quando o dono do estado de juntas (gazebo_arm_bridge
        # em simulação; arm_vel_integrator no hardware, futuramente)
        # conclui uma rampa de postura, o setpoint comandado é o melhor
        # prior disponível — informação de COMANDO que o robô real sem
        # encoders também tem, não ground truth de simulador. Usamos esse
        # setpoint como seed one-shot do próximo ciclo do IK.
        self._posture_target_q = None
        rospy.Subscriber('/setpoints', SetpointMsg, self._cb_setpoints, queue_size=1)
        rospy.Subscriber('/b166er/arm_posture_target', JointState,
                         self._cb_posture_target, queue_size=1)
        rospy.Subscriber('/b166er/arm_posture_reached', Bool,
                         self._cb_posture_reached, queue_size=1)
        rospy.Subscriber('/b166er/arm_posture_ok', Bool, self._cb_posture_ok, queue_size=1)

        rospy.loginfo('[state_estimator] pronto')
        rospy.loginfo('  pioneer: %s', self._pioneer_topic)
        rospy.loginfo('  t265:    %s', self._t265_topic)

    def _cb_pioneer(self, msg):
        self._base_odom = msg
        with self._base_lock:
            self._base_buf.append((msg.header.stamp.to_sec(), msg))

    def _base_odom_sync(self):
        """base_odom de stamp mais próximo do T265 (ou o mais recente,
        se não houver nenhum a menos de base_sync_max_dt).

        Cópia sob lock: o callback roda em outra thread e um deque
        alterado durante a iteração levanta RuntimeError — foi assim que
        o estimador morreu no NUC (exit 1) na primeira execução desta
        sincronização, 2026-09-11.
        """
        if self._t265_odom is None:
            return self._base_odom
        with self._base_lock:
            buf = list(self._base_buf)
        if not buf:
            return self._base_odom
        t = self._t265_odom.header.stamp.to_sec()
        st, msg = min(buf, key=lambda x: abs(x[0] - t))
        return msg if abs(st - t) <= self._base_sync_max_dt else self._base_odom
    def _cb_t265(self, msg):    self._t265_odom = msg

    def _cb_posture_target(self, msg):
        if len(msg.position) == 5:
            self._posture_target_q = np.array(msg.position, dtype=float)

    def _cb_setpoints(self, m):
        # Velocidades comandadas ao firmware (graus/s) — dead reckoning.
        v = np.radians([m.set_1, m.set_2, m.set_3, m.set_4, m.set_5])
        v = np.where(np.abs(v) < self._dr_v_dead, 0.0, v)          # zona morta da placa
        self._v_cmd = np.clip(v, -self._dr_v_max, self._dr_v_max)   # saturação da placa
        self._t_v_cmd = rospy.Time.now()

    def _cb_posture_ok(self, msg):
        self._posture_ok = bool(msg.data)

    def _cb_posture_reached(self, msg):
        if msg.data and self._posture_target_q is not None and self._posture_ok:
            self._q_dr = self._posture_target_q.copy()
            # Seed imediato ao concluir a rampa. NÃO basta sozinho: a
            # rampa se declara concluída quando o q integrado do dono do
            # estado chega ao alvo, mas o braço FÍSICO ainda está
            # assentando (droop do PID) — o alvo de IK desse instante
            # corresponde a uma pose intermediária, e o seed bom pode
            # cair numa bacia ruim mesmo assim. O retry em _solve_ik é
            # que garante a recuperação depois que tudo assenta.
            self._q_arm = self._posture_target_q.copy()
            if abs(self._posture_target_q[2]) > self._j3_zona:
                self._j3_sinal = float(np.sign(self._posture_target_q[2]))
            rospy.loginfo('[state_estimator] re-seed por postura concluída: %s rad',
                          np.round(self._q_arm, 3))

    def _solve_ik(self, T_target, q_seed):
        """IK com retry a partir da última postura comandada.

        O DLS é local: uma vez que a recursão cai num mínimo local (o
        caso clássico é o ramo espelhado do cotovelo com uma junta
        grudada no limite), ele vira um ponto fixo — cada ciclo parte do
        resultado ruim do ciclo anterior e devolve exatamente o mesmo
        resultado ruim, para sempre (observado ao vivo: q travado em
        (37.7°, 22.3°, −110°) com resíduo constante de 0,213m). Sem um
        seed alternativo, o estimador nunca se recupera sozinho.

        A postura comandada é o prior disponível também no robô real
        (é o setpoint que o firmware executou, não ground truth de
        simulador), então serve de segunda tentativa. Fica com a melhor
        das duas soluções pelo resíduo de posição.
        """
        q, conv, rp, ro = ik_arm(T_target, q_init=q_seed, max_iter=self._ik_max_iter)
        if conv or self._posture_target_q is None:
            return q, conv, rp, ro
        # O retry existe para o MÍNIMO LOCAL (resíduo de centímetros, ver
        # docstring). Um resíduo de milímetros que só não fechou a
        # tolerância não é isso: o segundo seed devolve a mesma solução
        # e custa outra solve inteira — no NUC, com a IK do stow parando
        # em 3–5 mm, isso dobrava o buraco de publicação do estado
        # (RELATORIO17, RETURN cego). Abaixo do limiar fica a primeira.
        if rp < self._ik_retry_min_res:
            return q, conv, rp, ro

        q2, conv2, rp2, ro2 = ik_arm(T_target,
                                     q_init=self._posture_target_q.copy(),
                                     max_iter=self._ik_max_iter)
        if conv2 or rp2 < rp:
            rospy.loginfo_throttle(
                5.0, '[state_estimator] IK recuperada via seed de postura '
                     '(resíduo %.4f → %.4f m)', rp, rp2)
            return q2, conv2, rp2, ro2
        return q, conv, rp, ro

    def _cb_ls(self, m):
        if len(m.position) == 5:
            self._ls = np.array(m.position, dtype=float)
            if self._ls[2] != 0.0:
                self._j3_sinal = float(self._ls[2])

    def _semente_com_fins_de_curso(self, q_seed):
        q_seed = np.array(q_seed, dtype=float).copy()
        for j in range(5):
            if self._ls[j] > 0:   q_seed[j] = JOINT_UPPER[j]
            elif self._ls[j] < 0: q_seed[j] = JOINT_LOWER[j]
        return q_seed

    def _ramo_por_sinal(self, now, T_target, q, conv, rp, ro):
        """Ramo pelo sinal de J3 esperado desde a última passagem pelo
        cotovelo reto; ver o comentário em __init__."""
        fresh = self._t_v_cmd is not None and (now - self._t_v_cmd).to_sec() < 0.5
        if abs(q[2]) < self._j3_zona:
            if fresh and abs(self._v_cmd[2]) > 1e-6:
                self._j3_sinal = float(np.sign(self._v_cmd[2]))
            return q, conv, rp, ro
        if self._j3_sinal is None or np.sign(q[2]) == self._j3_sinal:
            return q, conv, rp, ro
        seed = np.array([q[0], q[1] + q[2], -q[2], q[3] + q[2], q[4]])
        q2, conv2, rp2, ro2 = ik_arm(T_target, q_init=np.clip(seed, JOINT_LOWER, JOINT_UPPER),
                                     max_iter=self._ik_max_iter)
        if conv2 and abs(q2[2]) >= self._j3_zona and np.sign(q2[2]) == self._j3_sinal and rp2 <= rp + 0.003:
            rospy.logwarn_throttle(2.0, '[state_estimator] ramo pelo sinal de J3 (%+d): %s -> %s (resíduo %.4f -> %.4f)',
                                   int(self._j3_sinal), np.degrees(q).round(1).tolist(),
                                   np.degrees(q2).round(1).tolist(), rp, rp2)
            return q2, conv2, rp2, ro2
        return q, conv, rp, ro

    def _ramo_por_velocidade(self, now, T_target, q, conv, rp, ro):
        """Escolhe entre a solução atual e a espelhada (cotovelo) pela
        velocidade MEDIDA da ponta; ver o comentário em __init__."""
        t = now.to_sec()
        p = T_target[:3, 3].copy()
        self._rv_hist.append((t, p))
        dt = t - self._rv_t_prev if self._rv_t_prev else 0.0
        self._rv_t_prev = t
        antigo = next((h for h in self._rv_hist if t - h[0] <= self._rv_janela), None)
        if antigo is None or t - antigo[0] < 0.5 * self._rv_janela or not (0.0 < dt < 0.5):
            return q, conv, rp, ro
        fresh = self._t_v_cmd is not None and (now - self._t_v_cmd).to_sec() < 0.5
        if not fresh or not np.any(self._v_cmd) or abs(q[2]) < self._rv_j3_min:
            return q, conv, rp, ro
        seed = np.array([q[0], q[1] + q[2], -q[2], q[3] + q[2], q[4]])
        q2, conv2, rp2, ro2 = ik_arm(T_target, q_init=np.clip(seed, JOINT_LOWER, JOINT_UPPER),
                                     max_iter=self._ik_max_iter)
        if not conv2 or abs(q2[2] - q[2]) < self._rv_j3_min or rp2 > rp + 0.003:
            return q, conv, rp, ro
        v_meas = (p - antigo[1]) / (t - antigo[0])
        va = arm_jacobian_world(q.tolist(), np.eye(4))[:3] @ self._v_cmd
        vb = arm_jacobian_world(q2.tolist(), np.eye(4))[:3] @ self._v_cmd
        if max(np.linalg.norm(va), np.linalg.norm(vb)) < self._rv_vmin:
            return q, conv, rp, ro
        lam = math.exp(-dt / self._rv_tau)
        self._rv_Ea = lam * self._rv_Ea + float(np.sum((v_meas - va) ** 2))
        self._rv_Eb = lam * self._rv_Eb + float(np.sum((v_meas - vb) ** 2))
        self._rv_n  = lam * self._rv_n + 1.0
        if self._rv_n >= self._rv_nmin and self._rv_Eb < self._rv_fator * self._rv_Ea:
            rospy.logwarn('[state_estimator] ramo pela velocidade: %s -> %s (erro atual %.4f, espelho %.4f, %.0f amostras)',
                          np.degrees(q).round(1).tolist(), np.degrees(q2).round(1).tolist(),
                          self._rv_Ea, self._rv_Eb, self._rv_n)
            self._rv_Ea = self._rv_Eb = self._rv_n = 0.0
            return q2, conv2, rp2, ro2
        return q, conv, rp, ro

    def _cb_joint_states(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(n in name_to_pos for n in JOINT_NAMES):
            q = np.array([name_to_pos[n] for n in JOINT_NAMES])
            self._q_joints = q
            self._q_joints_buf.append((msg.header.stamp.to_sec(), q))

    def _compute_T_target(self):
        """
        T_ArmBase_t265 = (T_world_base × T_BASELINK_ARM)^{-1} × T_world_t265
        """
        T_world_base     = _odom_to_matrix(self._base_odom_sync())
        T_world_t265     = _odom_to_matrix(self._t265_odom)
        T_world_arm_base = T_world_base @ T_BASELINK_ARM
        return np.linalg.inv(T_world_arm_base) @ T_world_t265

    def _publish_joint_states(self, q, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name     = JOINT_NAMES
        msg.position = q.tolist()
        msg.velocity = self._dq_arm.tolist()
        self._pub_joints.publish(msg)

    def _publish_state(self, q, converged, res_p, res_o, stamp):
        msg = RobotState()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._world_frame
        msg.base_odom       = self._base_odom
        msg.base_odom_valid = True

        ee = PoseStamped()
        ee.header.stamp    = stamp
        ee.header.frame_id = self._world_frame
        ee.pose            = self._t265_odom.pose.pose
        msg.ee_pose        = ee
        msg.t265_valid     = True

        msg.q_arm              = q.tolist()
        msg.dq_arm             = self._dq_arm.tolist()
        msg.ik_converged       = converged
        msg.ik_residual_pos    = float(res_p)
        msg.ik_residual_orient = float(res_o)
        self._pub_state.publish(msg)

    def spin(self):
        rate = rospy.Rate(self._pub_rate)

        while not rospy.is_shutdown():
            now = rospy.Time.now()

            if self._base_odom is None or self._t265_odom is None:
                rate.sleep()
                continue

            T_target = self._compute_T_target()

            # Seed sincronizado com o stamp do T265: o sensor_sim publica T265
            # com stamp = stamp do /joint_states que usou para o FK, portanto o q
            # com timestamp mais próximo é exatamente a solução → IK converge em 0 iter.
            # SEMENTE = ESTIMATIVA ANTERIOR por padrão (2026-10-07): é o que
            # o robô real tem (sem encoders) — continuidade desde o stow
            # conhecido (_HOME_Q / resync). Semear com a verdade do Gazebo
            # (~seed_ground_truth) escondia o problema do ramo na simulação
            # e só serve para diagnóstico.
            if self._seed_gt and self._q_joints_buf:
                t265_t = self._t265_odom.header.stamp.to_sec()
                q_seed = min(self._q_joints_buf,
                             key=lambda x: abs(x[0] - t265_t))[1]
            elif self._seed_gt and self._q_joints is not None:
                q_seed = self._q_joints
            else:
                q_seed = self._q_arm   # continuidade (hardware e simulação honesta)

            if np.any(self._ls != 0.0):
                q_seed = self._semente_com_fins_de_curso(q_seed)
            q, conv, res_p, res_o = self._solve_ik(T_target, q_seed)
            if self._ramo_vel:
                q, conv, res_p, res_o = self._ramo_por_velocidade(now, T_target, q, conv, res_p, res_o)
            if self._ramo_sinal:
                q, conv, res_p, res_o = self._ramo_por_sinal(now, T_target, q, conv, res_p, res_o)
            if self._dr_seed:
                # integra os comandos (só com mensagem fresca)
                if self._t_v_cmd is not None and (now - self._t_v_cmd).to_sec() < 0.5:
                    dt_dr = (now - self._t_dr).to_sec() if self._t_dr else 0.0
                    if 0.0 < dt_dr < 0.5:
                        self._q_dr = np.clip(self._q_dr + self._v_cmd * dt_dr,
                                             JOINT_LOWER, JOINT_UPPER)
                dt_anc = (now - self._t_dr).to_sec() if self._t_dr else 0.0
                self._t_dr = now
                desacordo = float(np.max(np.abs(q - self._q_dr)))
                if desacordo > self._dr_desacordo:
                    q2, conv2, rp2, ro2 = ik_arm(T_target, q_init=self._q_dr.copy(),
                                                 max_iter=self._ik_max_iter)
                    d2 = float(np.max(np.abs(q2 - self._q_dr)))
                    rospy.logwarn_throttle(
                        2.0, '[state_estimator] ramo: IK %s | dead reck. %s | re-IK %s '
                             '(conv %s, desacordos %.1f° / %.1f°)',
                        np.degrees(q).round(1).tolist(), np.degrees(self._q_dr).round(1).tolist(),
                        np.degrees(q2).round(1).tolist(), conv2,
                        np.degrees(desacordo), np.degrees(d2))
                    # Bateria 4 (07/10): q_dr corrompido levava a re-IK a
                    # soluções ENCOSTADAS no batente (J3 ±60°, J4 110°) e o
                    # estimador alternava entre elas e a verdadeira a cada
                    # ciclo — a lei de postura invertia o sinal do comando
                    # a cada ciclo e o braço não saía do lugar. A solução do
                    # ramo só substitui a atual se não está no batente e
                    # não piora o resíduo de posição.
                    no_batente = bool(np.any(np.minimum(q2 - JOINT_LOWER, JOINT_UPPER - q2) < 0.035))
                    if conv2 and d2 + 1e-6 < desacordo and not no_batente and rp2 <= res_p + 0.002:
                        q, conv, res_p, res_o = q2, conv2, rp2, ro2
                        desacordo = d2
                if desacordo <= self._dr_desacordo and 0.0 < dt_anc < 0.5 and self._dr_tau > 0:
                    self._q_dr = self._q_dr + (dt_anc / self._dr_tau) * (q - self._q_dr)
                # NÃO reancorar na estimativa a cada ciclo: foi o erro da
                # primeira versão (q_dr virava cópia de q_est e seguia o ramo
                # errado junto). O dead reckoning só reancora em eventos em
                # que a postura é conhecida: postura concluída (alvo) e
                # reset/resync. A deriva por ganho de execução entre esses
                # eventos (segundos) é pequena diante dos 50–70° que separam
                # os ramos.
                rospy.loginfo_throttle(
                    10.0, '[state_estimator] dr: q_est %s | q_dr %s | v_cmd %s°/s',
                    np.degrees(q).round(1).tolist(), np.degrees(self._q_dr).round(1).tolist(),
                    np.degrees(self._v_cmd).round(1).tolist())

            dt = (now - self._t_prev).to_sec() if self._t_prev else None
            if dt and dt > 0:
                self._dq_arm = (q - self._q_arm_prev) / dt
            self._q_arm_prev = q.copy()
            self._q_arm      = q
            self._t_prev     = now

            self._publish_state(q, conv, res_p, res_o, now)
            self._publish_joint_states(q, now)

            if not conv:
                rospy.logwarn_throttle(5.0,
                    '[state_estimator] IK não convergiu — pos=%.4fm orient=%.4frad',
                    res_p, res_o)
            rate.sleep()


if __name__ == '__main__':
    try:
        StateEstimator().spin()
    except rospy.ROSInterruptException:
        pass
