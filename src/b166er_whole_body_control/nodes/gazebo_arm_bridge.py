#!/usr/bin/env python3
"""
gazebo_arm_bridge — encaminha /b166er/arm_vel_cmd para controladores Gazebo.

Decompõe o JointState com 5 velocidades em 5 tópicos individuais
Float64 que os velocity_controllers do ros_control esperam.

  /b166er/arm_vel_cmd (JointState.velocity[0..4], rad/s)
    → /J1_velocity_controller/command (Float64)
    → /J2_velocity_controller/command (Float64)
    ...
    → /J5_velocity_controller/command (Float64)
"""

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from b166er_whole_body_control.kinematics import JOINT_NAMES


class GazeboArmBridge:

    def __init__(self):
        rospy.init_node('gazebo_arm_bridge')

        self._pubs = [
            rospy.Publisher(f'/{n}_velocity_controller/command',
                            Float64, queue_size=1)
            for n in JOINT_NAMES
        ]

        rospy.Subscriber('/b166er/arm_vel_cmd', JointState,
                         self._cb_vel_cmd, queue_size=1)

        rospy.loginfo('[gazebo_arm_bridge] encaminhando para %s',
                      [f'/{n}_velocity_controller/command' for n in JOINT_NAMES])

    def _cb_vel_cmd(self, msg):
        if len(msg.velocity) < 5:
            return
        for i, pub in enumerate(self._pubs):
            pub.publish(Float64(data=float(msg.velocity[i])))


if __name__ == '__main__':
    try:
        GazeboArmBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
