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
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_matrix

from b166er_whole_body_control.msg import RobotState
from b166er_whole_body_control.kinematics import (
    JOINT_NAMES, T_BASELINK_ARM,
    ik_arm,
)


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

        self._base_odom  = None
        self._t265_odom  = None
        self._q_arm      = np.zeros(5)
        self._q_arm_prev = np.zeros(5)
        self._dq_arm     = np.zeros(5)
        self._t_prev     = None

        self._pub_state  = rospy.Publisher('/b166er/robot_state',
                                           RobotState, queue_size=5)
        self._pub_joints = rospy.Publisher('/b166er/estimated_joint_states',
                                           JointState, queue_size=5)

        rospy.Subscriber(self._pioneer_topic, Odometry, self._cb_pioneer)
        rospy.Subscriber(self._t265_topic,    Odometry, self._cb_t265)

        rospy.loginfo('[state_estimator] pronto')
        rospy.loginfo('  pioneer: %s', self._pioneer_topic)
        rospy.loginfo('  t265:    %s', self._t265_topic)

    def _cb_pioneer(self, msg): self._base_odom = msg
    def _cb_t265(self, msg):    self._t265_odom = msg

    def _compute_T_target(self):
        """
        T_ArmBase_t265 = (T_world_base × T_BASELINK_ARM)^{-1} × T_world_t265
        """
        T_world_base     = _odom_to_matrix(self._base_odom)
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
            q, conv, res_p, res_o = ik_arm(T_target, q_init=self._q_arm)

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
