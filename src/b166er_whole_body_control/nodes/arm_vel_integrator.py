#!/usr/bin/env python3
"""
arm_vel_integrator — integra /b166er/arm_vel_cmd (rad/s) → /setpoints (graus).

Hardware path: converte os comandos de velocidade do fuzzy_wb_controller
em posições angulares (graus) para o controlador PID de cada junta via
rosserial/Arduino.

Fluxo:
  /b166er/arm_vel_cmd (JointState.velocity, rad/s)
    → integração Euler (dt = 1/rate)
    → clip em JOINT_LOWER / JOINT_UPPER (rad)
    → rad × (180/π) → graus
    → /setpoints (movemaster_msg/setpoint)
"""

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from movemaster_msg.msg import setpoint as SetpointMsg

from b166er_whole_body_control.kinematics import JOINT_LOWER, JOINT_UPPER

_R2D = 180.0 / np.pi

# Se nenhum arm_vel_cmd chegar neste intervalo, trata velocidade como zero
_VEL_TIMEOUT = 0.5   # s

# Posição de repouso segura (aproximação da home do RV-M2, em rad)
_Q_HOME = np.array([0.0, 0.0, 0.0, 0.0, 0.0])


class ArmVelIntegrator:

    def __init__(self):
        rospy.init_node('arm_vel_integrator')

        self._rate_hz    = rospy.get_param('~rate',        20.0)
        self._goto_home  = rospy.get_param('~goto_home',   False)

        self._q_arm      = None          # rad — preenchido pelo state_estimator
        self._dq_cmd     = np.zeros(5)   # rad/s — último comando recebido
        self._t_last_cmd = None          # stamp do último arm_vel_cmd

        self._pub = rospy.Publisher('/setpoints', SetpointMsg, queue_size=1)

        rospy.Subscriber('/b166er/estimated_joint_states', JointState,
                         self._cb_joint_states, queue_size=1)
        rospy.Subscriber('/b166er/arm_vel_cmd', JointState,
                         self._cb_vel_cmd, queue_size=1)

        rospy.loginfo('[arm_vel_integrator] pronto — aguardando estado inicial...')

    # ------------------------------------------------------------------
    def _cb_joint_states(self, msg):
        if self._q_arm is None and len(msg.position) == 5:
            self._q_arm = np.array(msg.position, dtype=float)
            rospy.loginfo('[arm_vel_integrator] warm-start q=%s rad',
                          np.round(self._q_arm, 3))

    def _cb_vel_cmd(self, msg):
        if len(msg.velocity) == 5:
            self._dq_cmd    = np.array(msg.velocity, dtype=float)
            self._t_last_cmd = rospy.Time.now()

    # ------------------------------------------------------------------
    def _publish(self, q_deg):
        msg = SetpointMsg()
        msg.set_1          = float(q_deg[0])
        msg.set_2          = float(q_deg[1])
        msg.set_3          = float(q_deg[2])
        msg.set_4          = float(q_deg[3])
        msg.set_5          = float(q_deg[4])
        msg.set_GRIP       = False
        msg.emergency_stop = False
        msg.GoHome         = 0
        self._pub.publish(msg)

    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(self._rate_hz)
        dt   = 1.0 / self._rate_hz

        if self._goto_home:
            rospy.loginfo('[arm_vel_integrator] publicando HOME antes de iniciar')
            self._q_arm = _Q_HOME.copy()
            self._publish(self._q_arm * _R2D)
            rospy.sleep(2.0)

        while not rospy.is_shutdown():
            if self._q_arm is None:
                rate.sleep()
                continue

            # velocidade com timeout de segurança
            if self._t_last_cmd is not None:
                age = (rospy.Time.now() - self._t_last_cmd).to_sec()
                dq = self._dq_cmd if age < _VEL_TIMEOUT else np.zeros(5)
            else:
                dq = np.zeros(5)

            # integração Euler + saturação nos limites de junta
            self._q_arm = np.clip(self._q_arm + dq * dt,
                                  JOINT_LOWER, JOINT_UPPER)

            self._publish(self._q_arm * _R2D)

            rospy.logdebug_throttle(1.0,
                '[arm_vel_integrator] q=%s° dq=%s rad/s',
                np.round(self._q_arm * _R2D, 1),
                np.round(dq, 3))

            rate.sleep()


if __name__ == '__main__':
    try:
        ArmVelIntegrator().spin()
    except rospy.ROSInterruptException:
        pass
