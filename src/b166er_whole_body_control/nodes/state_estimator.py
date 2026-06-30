#!/usr/bin/env python3
"""
state_estimator — whole-body state for b166er without joint encoders.

Fuses:
  • T265 odometry  → EE pose in world frame
  • Pioneer odom   → base pose (x, y, θ) in world frame

Estimates arm joint angles via numerical IK so the whole-body controller
can reason about the full 8-DOF chain (3 base + 5 arm) as a single entity.

Kinematic chain (Base frame → t265_link):
  Base → J1(RotZ) → L1 → J2(RotZ) → L2 → J3(RotZ) → L3
       → J4(RotZ) → L4 → J5(RotZ) → L5 → CameraSupport → t265_link
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import (quaternion_matrix, quaternion_from_matrix,
                                 euler_from_matrix)

from b166er_whole_body_control.msg import RobotState


# ---------------------------------------------------------------------------
# RV-M2 joint limits [lower, upper] in radians (from URDF / manual)
# ---------------------------------------------------------------------------
JOINT_NAMES  = ['J1', 'J2', 'J3', 'J4', 'J5']
JOINT_LOWER  = np.array([-2.61799, -1.13446, -1.04720, -1.91986, -3.14159])
JOINT_UPPER  = np.array([ 2.61799,  1.13446,  1.04720,  1.91986,  3.14159])

# IK tuning
IK_MAX_ITER   = 150
IK_TOL_POS    = 1e-4   # metres
IK_TOL_ORIENT = 1e-3   # radians
IK_LAMBDA     = 0.05   # DLS damping
IK_DQ_FINITE  = 1e-6   # Jacobian step size


# ---------------------------------------------------------------------------
# Pure-numpy homogeneous transform helpers
# ---------------------------------------------------------------------------

def _rotx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]], dtype=float)

def _roty(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]], dtype=float)

def _rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]], dtype=float)

def _trans(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T

def _tf(xyz, rpy):
    """Combined translation + RPY rotation (extrinsic: Rz*Ry*Rx)."""
    T = np.eye(4)
    T[:3, :3] = (  _rotz(rpy[2]) @ _roty(rpy[1]) @ _rotx(rpy[0])  )[:3, :3]
    T[:3,  3] = xyz
    return T


# Fixed transforms (from URDF)
#   base_link → top_plate : xyz=(0.003, 0, 0.274)
#   top_plate → Base (arm): xyz=(0,     0, 0.02)
#   combined:
T_BASE_LINK_TO_ARM_BASE = _tf([0.003, 0, 0.294], [0, 0, 0])

# Precomputed constant suffix: L5 → t265_link
# JCam: xyz=(0,0,-0.08), fixed
# t265_mount: xyz=(0,0.075,-0.07), rpy=(0, 2.1, 1.57)
_T_L5_T265 = _trans(0, 0, -0.08) @ _tf([0, 0.075, -0.07], [0, 2.1, 1.57])


def fk_arm(q):
    """
    Forward kinematics: Base → t265_link.

    Parameters
    ----------
    q : array-like, shape (5,)
        Joint angles [q1, q2, q3, q4, q5] in radians.

    Returns
    -------
    T : ndarray, shape (4, 4)
        Homogeneous transform from arm Base frame to t265_link.
    """
    q1, q2, q3, q4, q5 = q

    T = (
        _trans(0, 0, 0.4)    @ _rotz(q1)              # J1
        @ _trans(0.12, 0, 0) @ _rotx(1.5708) @ _rotz(q2)  # J2
        @ _trans(0.25, 0, 0) @ _rotz(q3)               # J3
        @ _trans(0.2,  0, 0) @ _rotz(q4)               # J4
        @ _roty(-1.5708)     @ _rotz(q5)               # J5
        @ _T_L5_T265                                    # fixed suffix
    )
    return T


def _T_to_6vec(T):
    """4×4 transform → 6-vector [x, y, z, wx, wy, wz] for Jacobian computation.
    Orientation uses the skew-symmetric extraction (≈ axis×sin(angle))."""
    pos = T[:3, 3]
    R   = T[:3, :3]
    dw  = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]]) * 0.5
    return np.concatenate([pos, dw])


def _pose_error(T_current, T_target):
    """
    6-vector error [dp (3), dω (3)] for DLS IK.
    dp  = position error.
    dω  = axis-angle of orientation error R_err = R_target × R_current^T.
    """
    dp    = T_target[:3, 3] - T_current[:3, 3]
    R_err = T_target[:3, :3] @ T_current[:3, :3].T
    dω    = np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ]) * 0.5
    return np.concatenate([dp, dω])


def ik_arm(T_target, q_init=None):
    """
    Numerical IK for the RV-M2 arm (Base → t265_link).

    Uses damped least-squares (DLS) with a numerical Jacobian.

    Parameters
    ----------
    T_target : ndarray (4, 4)
        Desired Base → t265_link transform.
    q_init : array-like (5,) or None
        Initial joint angles. Defaults to zeros.

    Returns
    -------
    q : ndarray (5,)
        Estimated joint angles.
    converged : bool
    residual_pos : float    metres
    residual_orient : float radians
    """
    q = np.zeros(5) if q_init is None else np.array(q_init, dtype=float)

    for _ in range(IK_MAX_ITER):
        T_cur = fk_arm(q)
        err   = _pose_error(T_cur, T_target)

        res_pos    = np.linalg.norm(err[:3])
        res_orient = np.linalg.norm(err[3:])

        if res_pos < IK_TOL_POS and res_orient < IK_TOL_ORIENT:
            return q, True, res_pos, res_orient

        # Numerical Jacobian of FK (6×5): J = d(pose)/d(q_i)
        # Must use FK derivative, NOT error derivative (different sign).
        J = np.zeros((6, 5))
        for i in range(5):
            dq_i      = np.zeros(5)
            dq_i[i]   = IK_DQ_FINITE
            J[:, i]   = (_T_to_6vec(fk_arm(q + dq_i))
                         - _T_to_6vec(fk_arm(q - dq_i))) / (2 * IK_DQ_FINITE)

        # DLS: dq = J^T (J J^T + λ²I)^{-1} err
        JJT = J @ J.T
        dq  = J.T @ np.linalg.solve(JJT + IK_LAMBDA**2 * np.eye(6), err)

        q  = np.clip(q + dq, JOINT_LOWER, JOINT_UPPER)

    T_cur = fk_arm(q)
    err   = _pose_error(T_cur, T_target)
    return q, False, np.linalg.norm(err[:3]), np.linalg.norm(err[3:])


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class StateEstimator:

    def __init__(self):
        rospy.init_node('state_estimator')

        # Parameters
        self._t265_topic    = rospy.get_param('~t265_odom_topic',    '/camera/odom/sample')
        self._pioneer_topic = rospy.get_param('~pioneer_odom_topic', '/RosAria/pose')
        self._world_frame   = rospy.get_param('~world_frame',        'odom')
        self._arm_base_frame = rospy.get_param('~arm_base_frame',    'Base')
        self._pub_rate      = rospy.get_param('~rate', 20.0)

        # State
        self._base_odom  = None
        self._t265_odom  = None
        self._q_arm      = np.zeros(5)
        self._dq_arm     = np.zeros(5)
        self._q_arm_prev = np.zeros(5)
        self._t_prev     = None

        # TF
        self._tf_buf      = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Publishers
        self._pub_state = rospy.Publisher(
            '/b166er/robot_state', RobotState, queue_size=5)
        self._pub_joints = rospy.Publisher(
            '/b166er/estimated_joint_states', JointState, queue_size=5)

        # Subscribers
        rospy.Subscriber(self._pioneer_topic, Odometry, self._cb_pioneer)
        rospy.Subscriber(self._t265_topic,    Odometry, self._cb_t265)

        rospy.loginfo('[state_estimator] ready')
        rospy.loginfo('  pioneer: %s', self._pioneer_topic)
        rospy.loginfo('  t265:    %s', self._t265_topic)

    # ------------------------------------------------------------------
    def _cb_pioneer(self, msg):
        self._base_odom = msg

    def _cb_t265(self, msg):
        self._t265_odom = msg

    # ------------------------------------------------------------------
    def _odom_to_matrix(self, odom):
        """nav_msgs/Odometry → 4×4 homogeneous transform."""
        p = odom.pose.pose.position
        o = odom.pose.pose.orientation
        T = quaternion_matrix([o.x, o.y, o.z, o.w])
        T[:3, 3] = [p.x, p.y, p.z]
        return T

    def _compute_T_target(self):
        """
        Compute T_ArmBase_t265 from Pioneer odom + T265 odom.

        T_ArmBase_t265 = T_world_ArmBase^{-1} × T_world_t265

        T_world_ArmBase = T_world_baselink × T_BASELINK_ARM_BASE
        T_world_t265    = from T265 odom (already in world/odom frame)
        """
        T_world_base = self._odom_to_matrix(self._base_odom)
        T_world_t265 = self._odom_to_matrix(self._t265_odom)

        T_world_arm_base = T_world_base @ T_BASE_LINK_TO_ARM_BASE
        T_arm_base_t265  = np.linalg.inv(T_world_arm_base) @ T_world_t265
        return T_arm_base_t265

    # ------------------------------------------------------------------
    def _publish_joint_states(self, q, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name     = JOINT_NAMES
        msg.position = q.tolist()
        msg.velocity = self._dq_arm.tolist()
        self._pub_joints.publish(msg)

    def _publish_state(self, T_target, q, converged, res_p, res_o, stamp):
        msg = RobotState()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._world_frame

        msg.base_odom      = self._base_odom
        msg.base_odom_valid = True

        # EE pose in world frame (from T265 directly)
        ee = PoseStamped()
        ee.header.stamp    = stamp
        ee.header.frame_id = self._world_frame
        p = self._t265_odom.pose.pose.position
        o = self._t265_odom.pose.pose.orientation
        ee.pose.position    = p
        ee.pose.orientation = o
        msg.ee_pose      = ee
        msg.t265_valid   = True

        msg.q_arm            = q.tolist()
        msg.dq_arm           = self._dq_arm.tolist()
        msg.ik_converged     = converged
        msg.ik_residual_pos  = float(res_p)
        msg.ik_residual_orient = float(res_o)

        self._pub_state.publish(msg)

    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(self._pub_rate)

        while not rospy.is_shutdown():
            now = rospy.Time.now()

            if self._base_odom is None or self._t265_odom is None:
                rate.sleep()
                continue

            # IK
            T_target = self._compute_T_target()
            q, conv, res_p, res_o = ik_arm(T_target, q_init=self._q_arm)

            # Joint velocity estimate
            dt = (now - self._t_prev).to_sec() if self._t_prev else None
            if dt and dt > 0:
                self._dq_arm = (q - self._q_arm_prev) / dt
            self._q_arm_prev = q.copy()
            self._q_arm      = q
            self._t_prev     = now

            self._publish_state(T_target, q, conv, res_p, res_o, now)
            self._publish_joint_states(q, now)

            if not conv:
                rospy.logwarn_throttle(5.0,
                    '[state_estimator] IK não convergiu — pos=%.4fm orient=%.4frad',
                    res_p, res_o)

            rate.sleep()


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        StateEstimator().spin()
    except rospy.ROSInterruptException:
        pass
