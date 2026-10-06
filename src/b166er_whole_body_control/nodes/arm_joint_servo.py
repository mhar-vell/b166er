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
Saídas
  /setpoints                   (movemaster_msg/setpoint) set_1..5 em GRAUS/S
  /b166er/arm_posture_reached  (Bool, latched)
  /b166er/arm_posture_target   (JointState, latched) — semente do estimador

Como uma postura é executada sem encoder: v = kp·(q_alvo − q_estimado),
saturada em ~ramp_velocity. A estimativa vem da T265 pelo estimador, logo
a malha fecha no sensor que o robô tem. "Chegou" é julgado por duas
medidas observáveis: o erro de junta ESTIMADO abaixo de ~tol_q em todas
as juntas, ou a posição do efetuador medida pela T265 a menos de ~tol_ee
da FK(q_alvo); qualquer uma, sustentada por ~estavel_s. Passado ~timeout
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
    JOINT_NAMES, JOINT_LOWER, JOINT_UPPER, T_BASELINK_ARM, fk_arm)


class ArmJointServo:
    def __init__(self):
        rospy.init_node('arm_joint_servo')
        self._rate_hz   = float(rospy.get_param('~rate', 20.0))
        self._kp        = float(rospy.get_param('~kp', 1.5))          # 1/s
        self._v_max     = float(rospy.get_param('/arm_postures/ramp_velocity', 0.3))
        self._tol_q     = float(rospy.get_param('~tol_q', 0.03))      # rad
        self._tol_ee    = float(rospy.get_param('~tol_ee', 0.03))     # m
        self._estavel_s = float(rospy.get_param('~estavel_s', 0.5))
        self._timeout   = float(rospy.get_param('~timeout', 30.0))
        self._vel_to    = float(rospy.get_param('~vel_timeout', 0.5))
        self._goto_home = bool(rospy.get_param('~goto_home_on_start', True))
        home = rospy.get_param('/arm_postures/stow_home', [0.0, 1.13, -1.04, -1.8, 0.0])
        self._q_home = np.array(home, dtype=float)

        self._state = None
        self._q_est = None
        self._p_ee  = None
        self._T_wb  = None
        self._v_fuzzy = np.zeros(5)
        self._t_fuzzy = None
        self._tilt = False
        self._q_alvo = None
        self._t_alvo = None
        self._t_ok = None
        self._home_feito = False

        self._pub_sp = rospy.Publisher('/setpoints', SetpointMsg, queue_size=1)
        self._pub_reached = rospy.Publisher('/b166er/arm_posture_reached', Bool,
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

    def _cb_tilt(self, m):
        self._tilt = bool(m.data)
        if self._tilt and self._q_alvo is not None:
            rospy.logwarn('[arm_joint_servo] inclinação crítica: postura cancelada')
            self._q_alvo = None

    # ------------------------------------------------------------ postura
    def _iniciar_postura(self, q, why):
        self._q_alvo = q
        self._t_alvo = rospy.Time.now()
        self._t_ok = None
        self._pub_reached.publish(Bool(data=False))
        tgt = JointState()
        tgt.header.stamp = rospy.Time.now()
        tgt.name = JOINT_NAMES
        tgt.position = q.tolist()
        self._pub_target.publish(tgt)
        rospy.loginfo('[arm_joint_servo] postura (%s): %s rad', why, np.round(q, 3))

    def _erro_ee(self):
        """Distância entre a FK da postura-alvo e a posição medida pela T265."""
        if self._q_alvo is None or self._p_ee is None or self._T_wb is None:
            return None
        T = self._T_wb @ T_BASELINK_ARM @ fk_arm(self._q_alvo.tolist())
        return float(np.linalg.norm(T[:3, 3] - self._p_ee))

    def _passo_postura(self):
        """Devolve v (rad/s) para a postura ativa, ou None se não há postura."""
        if self._q_alvo is None:
            return None
        if self._q_est is None:
            return np.zeros(5)
        e = self._q_alvo - self._q_est
        v = np.clip(self._kp * e, -self._v_max, self._v_max)
        e_ee = self._erro_ee()
        chegou = bool(np.all(np.abs(e) < self._tol_q)) or (e_ee is not None and e_ee < self._tol_ee)
        agora = rospy.Time.now()
        if chegou:
            if self._t_ok is None:
                self._t_ok = agora
            if (agora - self._t_ok).to_sec() >= self._estavel_s:
                self._fechar_postura(e, e_ee, 'alcançada')
                return np.zeros(5)
        else:
            self._t_ok = None
        if (agora - self._t_alvo).to_sec() > self._timeout:
            self._fechar_postura(e, e_ee, 'TIMEOUT')
            return np.zeros(5)
        return v

    def _fechar_postura(self, e, e_ee, como):
        msg = ('[arm_joint_servo] postura %s: erro estimado %s°, ponta a %s da FK do alvo'
               % (como, np.degrees(e).round(1).tolist(),
                  ('%.3f m' % e_ee) if e_ee is not None else 'n/d'))
        (rospy.logwarn if como == 'TIMEOUT' else rospy.loginfo)(msg)
        self._q_alvo = None
        self._pub_reached.publish(Bool(data=True))

    # ------------------------------------------------------------ laço
    def spin(self):
        rate = rospy.Rate(self._rate_hz)
        while not rospy.is_shutdown():
            if self._state is None:
                rate.sleep()
                continue
            if self._goto_home and not self._home_feito:
                self._home_feito = True
                self._iniciar_postura(self._q_home, 'home inicial')
            v = self._passo_postura()
            if v is None:
                # sem postura: velocidades do Fuzzy, com watchdog
                if (self._t_fuzzy is not None
                        and (rospy.Time.now() - self._t_fuzzy).to_sec() <= self._vel_to):
                    v = self._v_fuzzy
                else:
                    v = np.zeros(5)
            if self._tilt:
                v = np.zeros(5)
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
