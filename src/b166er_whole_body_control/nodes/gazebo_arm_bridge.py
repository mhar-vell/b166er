#!/usr/bin/env python3
"""
gazebo_arm_bridge — integra /b166er/arm_vel_cmd → position controllers do Gazebo.

Equivalente ao arm_vel_integrator.py mas para simulação:
  - Integra ẋ_arm (rad/s) → q_arm (rad) via Euler a 20 Hz
  - Publica Float64 individuais para /Jx_position_controller/command

Feedforward de gravidade ativo desde o primeiro ciclo (usando q_spawn)
para que os controladores nunca vejam erro estático e o braço não caia
ao inicializar. Após o warm-start com /joint_states, usa q_arm real.

Posturas (2026-08-13): este nó é o DONO do estado integrado de juntas —
qualquer publicação externa direta nos Jx_position_controller/command é
sobrescrita a 20 Hz (comportamento confirmado ao vivo). Por isso a
interface de postura mora aqui: /b166er/arm_posture_cmd (JointState com
5 positions) inicia uma rampa em espaço de juntas até a postura pedida,
a ~posture_ramp_vel rad/s — nunca por degrau de setpoint, que com
P=300 (J2) já chutou o chassi no passado (ver b166er_gazebo.launch).
Enquanto uma postura está ativa, /b166er/arm_vel_cmd é ignorado
(movimento de postura é deliberado, o controlador Cartesiano não deve
interferir no meio). Ao concluir, publica True em
/b166er/arm_posture_reached (latched).

Home recolhido: com ~goto_home_on_start=true (default), após o
warm-start o braço rampa sozinho até /arm_postures/stow_home — postura
recolhida definida pelo Marco (J2 pra cima, J3 pra baixo, punho
dobrado) para manter o CG do conjunto compacto; o braço esticado do
antigo q=0 já tombou o robô duas vezes em teste (2026-08-12). O spawn
continua em q=0 (determinístico, sem a race do -J documentada no
b166er_gazebo.launch) — o recolhimento é pós-spawn, suave.
"""

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Bool

from b166er_whole_body_control.kinematics import (
    JOINT_NAMES, JOINT_LOWER, JOINT_UPPER, gravity_torque_arm)

_VEL_TIMEOUT = 0.5   # s sem comando → zera velocidade

# Ganhos P dos controladores (arm_controllers.yaml) — usados no feedforward
_P_GAINS = np.array([100.0, 300.0, 200.0, 100.0, 50.0])


class GazeboArmBridge:

    def __init__(self):
        rospy.init_node('gazebo_arm_bridge')

        self._rate_hz = rospy.get_param('~rate', 20.0)
        self._k_ff    = rospy.get_param('~k_gravity_ff', 1.0)

        # Posição de spawn (deve bater com o spawn_model — sem -J, q=0).
        # Usada para calcular o feedforward antes do warm-start real.
        q_spawn_list  = rospy.get_param('~q_spawn', [0.0, 0.0, 0.0, 0.0, 0.0])
        self._q_spawn = np.clip(np.array(q_spawn_list, dtype=float),
                                JOINT_LOWER, JOINT_UPPER)

        self._q_arm      = None          # rad — None até warm-start real
        self._dq_cmd     = np.zeros(5)
        self._t_last_cmd = None

        # ── Posturas (rampa em espaço de juntas) ──
        self._ramp_vel    = rospy.get_param('/arm_postures/ramp_velocity', 0.3)
        self._reached_tol = rospy.get_param('/arm_postures/reached_tolerance', 0.02)
        self._goto_home   = rospy.get_param('~goto_home_on_start', True)
        home_list = rospy.get_param('/arm_postures/stow_home',
                                    [0.0, 1.1, -1.0, -1.8, 0.0])
        self._q_home = np.clip(np.array(home_list, dtype=float),
                               JOINT_LOWER, JOINT_UPPER)
        self._q_posture_target = None    # None = sem postura ativa

        self._pubs = [
            rospy.Publisher(f'/{n}_position_controller/command',
                            Float64, queue_size=1)
            for n in JOINT_NAMES
        ]
        self._pub_posture_reached = rospy.Publisher(
            '/b166er/arm_posture_reached', Bool, queue_size=1, latch=True)
        # Alvo da rampa corrente (latched) — consumido pelo state_estimator
        # como re-seed do IK ao concluir a postura (a postura comandada é
        # informação que o robô real sem encoders também tem: é o setpoint
        # que o firmware acabou de executar, não ground truth de simulador).
        self._pub_posture_target = rospy.Publisher(
            '/b166er/arm_posture_target', JointState, queue_size=1, latch=True)

        rospy.Subscriber('/joint_states', JointState,
                         self._cb_joint_states, queue_size=1)
        rospy.Subscriber('/b166er/arm_vel_cmd', JointState,
                         self._cb_vel_cmd, queue_size=1)
        rospy.Subscriber('/b166er/arm_posture_cmd', JointState,
                         self._cb_posture_cmd, queue_size=1)
        # SEGURANÇA DE TOMBAMENTO: congela o braço onde está. Mexer um
        # braço de 28 kg estendido com o chassi tombando só piora a
        # situação — e a rampa de postura em curso é justamente o tipo
        # de comando que continuaria rodando alheio ao acidente. A
        # reação NÃO é congelar e sim recolher: ver o bloco em _run().
        self._tilt_critical = False
        # Fração de ramp_velocity usada na retração de emergência.
        self._tilt_retract_scale = rospy.get_param('~tilt_retract_scale', 0.6)
        rospy.Subscriber('/b166er/tilt_critical', Bool, self._cb_tilt_critical)

        rospy.loginfo('[gazebo_arm_bridge] q_spawn=%s  aguardando warm-start...',
                      np.round(self._q_spawn, 3))

    # ------------------------------------------------------------------
    def _q_pub_for(self, q_arm):
        """Posição comandada ao PID: q_arm + feedforward de gravidade."""
        tau_g = gravity_torque_arm(q_arm)
        q_ff  = -self._k_ff * tau_g / _P_GAINS
        return np.clip(q_arm + q_ff, JOINT_LOWER, JOINT_UPPER)

    def _cb_joint_states(self, msg):
        if self._q_arm is not None:
            return   # warm-start one-shot
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(n in name_to_pos for n in JOINT_NAMES):
            q_init = np.array([name_to_pos[n] for n in JOINT_NAMES])
            self._q_arm = np.clip(q_init, JOINT_LOWER, JOINT_UPPER)
            rospy.loginfo('[gazebo_arm_bridge] warm-start q=%s rad',
                          np.round(self._q_arm, 3))
            if self._goto_home:
                self._start_posture(self._q_home, 'home recolhido (boot)')

    def _cb_vel_cmd(self, msg):
        if len(msg.velocity) == 5:
            self._dq_cmd     = np.array(msg.velocity, dtype=float)
            self._t_last_cmd = rospy.Time.now()

    def _cb_tilt_critical(self, msg):
        if msg.data and not self._tilt_critical:
            rospy.logerr('[gazebo_arm_bridge] INCLINAÇÃO CRÍTICA — abortando '
                         'a tarefa e RECOLHENDO o braço para stow_home')
            self._q_posture_target = None   # cancela rampa da tarefa
            self._dq_cmd = np.zeros(5)
        elif not msg.data and self._tilt_critical:
            rospy.logwarn('[gazebo_arm_bridge] inclinação normalizada — '
                          'operação liberada')
        self._tilt_critical = msg.data

    def _cb_posture_cmd(self, msg):
        if self._tilt_critical:
            rospy.logwarn_throttle(5.0, '[gazebo_arm_bridge] comando de postura '
                                        'ignorado: inclinação crítica')
            return
        if len(msg.position) == 5:
            q_target = np.clip(np.array(msg.position, dtype=float),
                               JOINT_LOWER, JOINT_UPPER)
            self._start_posture(q_target, 'comando externo')

    def _start_posture(self, q_target, why):
        self._q_posture_target = q_target
        self._pub_posture_reached.publish(Bool(data=False))
        tgt = JointState()
        tgt.header.stamp = rospy.Time.now()
        tgt.name     = JOINT_NAMES
        tgt.position = q_target.tolist()
        self._pub_posture_target.publish(tgt)
        rospy.loginfo('[gazebo_arm_bridge] rampa de postura (%s): %s rad',
                      why, np.round(q_target, 3))

    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(self._rate_hz)
        dt   = 1.0 / self._rate_hz

        # Feedforward pré-calculado para q_spawn: evita queda do braço
        # antes do warm-start real. Com este comando, o equilíbrio do
        # PID coincide com q_spawn → nenhuma força de reação na base.
        q_pub_spawn = self._q_pub_for(self._q_spawn)

        while not rospy.is_shutdown():
            if self._q_arm is None:
                # Publica feedforward de spawn para segurar o braço
                # enquanto aguarda /joint_states.
                for i, pub in enumerate(self._pubs):
                    pub.publish(Float64(data=float(q_pub_spawn[i])))
                rate.sleep()
                continue

            if self._tilt_critical:
                # RECOLHER, não congelar (corrigido em 2026-08-24).
                #
                # A primeira versão congelava o braço (dq = 0). Errado
                # por física e por consequência:
                #
                # Física — quem derruba o robô é o momento do braço de
                # 28 kg estendido. Congelar mantém exatamente o braço
                # de alavanca que causou o tombamento; recolher para
                # junto do corpo reduz o momento e é sempre a direção
                # segura, mesmo com o chassi já em movimento.
                #
                # Consequência — o congelamento criou um DEADLOCK real,
                # observado numa bateria inteira (8/8 abortadas em
                # 2026-08-24): robô tomba com o braço estendido → tilt
                # crítico congela o braço → o reset põe o chassi em pé,
                # mas o braço continua estendido → tomba de novo em
                # menos de 2 s → nunca cumpre o tempo de nivelamento
                # para limpar o estado → braço segue congelado. O
                # sistema não conseguia se recuperar sozinho, e a
                # postura que impedia a recuperação era justamente a
                # que a trava protegia.
                #
                # Velocidade reduzida: é manobra de segurança com o
                # chassi instável, não movimento de tarefa.
                err = self._q_home - self._q_arm
                if np.max(np.abs(err)) < self._reached_tol:
                    dq = np.zeros(5)
                else:
                    v = self._ramp_vel * self._tilt_retract_scale
                    dq = np.clip(err / dt, -v, v)
                    rospy.logwarn_throttle(
                        2.0, '[gazebo_arm_bridge] recolhendo por inclinação '
                             'crítica (falta %.2f rad)', np.max(np.abs(err)))
            elif self._q_posture_target is not None:
                # Rampa de postura: move q_arm em direção ao alvo a
                # ~ramp_velocity, ignorando arm_vel_cmd (movimento
                # deliberado em espaço de juntas, ver docstring).
                err = self._q_posture_target - self._q_arm
                if np.max(np.abs(err)) < self._reached_tol:
                    self._q_arm = self._q_posture_target.copy()
                    self._q_posture_target = None
                    self._pub_posture_reached.publish(Bool(data=True))
                    rospy.loginfo('[gazebo_arm_bridge] postura atingida: %s rad',
                                  np.round(self._q_arm, 3))
                    dq = np.zeros(5)
                else:
                    dq = np.clip(err / dt, -self._ramp_vel, self._ramp_vel)
            elif self._t_last_cmd is not None:
                age = (rospy.Time.now() - self._t_last_cmd).to_sec()
                dq = self._dq_cmd if age < _VEL_TIMEOUT else np.zeros(5)
            else:
                dq = np.zeros(5)

            self._q_arm = np.clip(self._q_arm + dq * dt,
                                  JOINT_LOWER, JOINT_UPPER)

            q_pub = self._q_pub_for(self._q_arm)
            for i, pub in enumerate(self._pubs):
                pub.publish(Float64(data=float(q_pub[i])))

            # Instrumentação (2026-08-13): rastrear se a ponte está de
            # fato integrando arm_vel_cmd. Sintoma investigado: o
            # controlador comandava 0,1 rad/s por 60 s e o braço não
            # saía do lugar — precisa distinguir "não recebeu",
            # "recebeu mas está em modo postura" e "integrou mas o PID
            # não seguiu".
            age = ((rospy.Time.now() - self._t_last_cmd).to_sec()
                   if self._t_last_cmd else -1.0)
            rospy.loginfo_throttle(2.0,
                '[bridge] postura=%s | dq_cmd=%s idade=%.2fs | dq_aplicado=%s | '
                'q_arm=%s',
                'ATIVA' if self._q_posture_target is not None else 'nao',
                np.round(self._dq_cmd, 3), age, np.round(dq, 3),
                np.round(self._q_arm, 3))

            rate.sleep()


if __name__ == '__main__':
    try:
        GazeboArmBridge().spin()
    except rospy.ROSInterruptException:
        pass
