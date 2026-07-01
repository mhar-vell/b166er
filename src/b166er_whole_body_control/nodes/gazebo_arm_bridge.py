#!/usr/bin/env python3
"""
gazebo_arm_bridge — integra /b166er/arm_vel_cmd → position controllers do Gazebo.

Equivalente ao arm_vel_integrator.py mas para simulação:
  - Integra ẋ_arm (rad/s) → q_arm (rad) via Euler a 20 Hz
  - Publica Float64 individuais para /Jx_position_controller/command

Usa /joint_states do Gazebo como warm-start da posição atual.
"""

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from b166er_whole_body_control.kinematics import JOINT_NAMES, JOINT_LOWER, JOINT_UPPER

_VEL_TIMEOUT = 0.5   # s sem comando → zera velocidade


class GazeboArmBridge:

    def __init__(self):
        rospy.init_node('gazebo_arm_bridge')

        self._rate_hz    = rospy.get_param('~rate', 20.0)
        self._q_arm      = None          # rad — warm-start do /joint_states
        self._dq_cmd     = np.zeros(5)   # rad/s
        self._t_last_cmd = None

        # publicadores para cada controlador de posição
        self._pubs = [
            rospy.Publisher(f'/{n}_position_controller/command',
                            Float64, queue_size=1)
            for n in JOINT_NAMES
        ]

        rospy.Subscriber('/joint_states', JointState,
                         self._cb_joint_states, queue_size=1)
        rospy.Subscriber('/b166er/arm_vel_cmd', JointState,
                         self._cb_vel_cmd, queue_size=1)

        rospy.loginfo('[gazebo_arm_bridge] aguardando /joint_states para warm-start...')

    # ------------------------------------------------------------------
    def _cb_joint_states(self, msg):
        if self._q_arm is None:
            name_to_pos = dict(zip(msg.name, msg.position))
            if all(n in name_to_pos for n in JOINT_NAMES):
                self._q_arm = np.array([name_to_pos[n] for n in JOINT_NAMES])
                rospy.loginfo('[gazebo_arm_bridge] warm-start q=%s rad',
                              np.round(self._q_arm, 3))

    def _cb_vel_cmd(self, msg):
        if len(msg.velocity) == 5:
            self._dq_cmd    = np.array(msg.velocity, dtype=float)
            self._t_last_cmd = rospy.Time.now()

    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(self._rate_hz)
        dt   = 1.0 / self._rate_hz

        while not rospy.is_shutdown():
            if self._q_arm is None:
                rate.sleep()
                continue

            if self._t_last_cmd is not None:
                age = (rospy.Time.now() - self._t_last_cmd).to_sec()
                dq = self._dq_cmd if age < _VEL_TIMEOUT else np.zeros(5)
            else:
                dq = np.zeros(5)

            self._q_arm = np.clip(self._q_arm + dq * dt,
                                  JOINT_LOWER, JOINT_UPPER)

            for i, pub in enumerate(self._pubs):
                pub.publish(Float64(data=float(self._q_arm[i])))

            rate.sleep()


if __name__ == '__main__':
    try:
        GazeboArmBridge().spin()
    except rospy.ROSInterruptException:
        pass
