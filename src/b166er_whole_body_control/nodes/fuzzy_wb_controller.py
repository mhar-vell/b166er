#!/usr/bin/env python3
"""
fuzzy_wb_controller — controlador whole-body com ganho Fuzzy adaptativo.

Substitui o ganho fixo K do whole_body_planner por um schedulador Mamdani
que ajusta (k_pos, k_orient, lambda_dls) em função do erro e sua variação.

Lei de controle:
  q̇_wb = J_wb†(λ) · K_fuzzy · err_EE  +  (I − J_wb† J_wb) · q̇₀

  K_fuzzy = diag(k_pos, k_pos, k_pos, k_orient, k_orient, k_orient)
  λ       = lambda_dls adaptativo

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
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from tf.transformations import quaternion_matrix

from b166er_whole_body_control.msg import RobotState
from b166er_whole_body_control.kinematics import (
    JOINT_NAMES, JOINT_LOWER, JOINT_UPPER,
    whole_body_jacobian,
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
K_NULL       = 0.3    # ganho do objetivo secundário (espaço nulo)


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

        # Estado
        self._robot_state = None
        self._target      = None
        self._enabled     = False

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

        # Subscritores
        rospy.Subscriber('/b166er/robot_state', RobotState,   self._cb_state)
        rospy.Subscriber('/b166er/ee_target',   PoseStamped,  self._cb_target)

        rospy.loginfo('[fuzzy_wb_controller] pronto — aguardando /b166er/ee_target')

    # ------------------------------------------------------------------
    def _cb_state(self, msg):
        self._robot_state = msg

    def _cb_target(self, msg):
        self._target   = _pose_stamped_to_matrix(msg)
        self._enabled  = True
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
            msg.linear.x  = v
            msg.angular.z = w
        else:
            msg.linear.x  = float(np.clip(q_dot_base[0], -MAX_BASE_LIN, MAX_BASE_LIN))
            msg.linear.y  = float(np.clip(q_dot_base[1], -MAX_BASE_LIN, MAX_BASE_LIN))
            msg.angular.z = float(np.clip(q_dot_base[2], -MAX_BASE_ANG, MAX_BASE_ANG))
        self._pub_cmdvel.publish(msg)

    def _publish_arm_vel(self, q_dot_arm, stamp):
        msg          = JointState()
        msg.header.stamp = stamp
        msg.name     = JOINT_NAMES
        msg.velocity = np.clip(q_dot_arm, -MAX_ARM_VEL, MAX_ARM_VEL).tolist()
        self._pub_armvel.publish(msg)

    def _publish_gains(self, k_pos, k_orient, lam):
        msg      = Float64MultiArray()
        msg.data = [k_pos, k_orient, lam]
        self._pub_gains.publish(msg)

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

            # Critério de parada
            if err_pos_norm < self._tol_pos and err_orient_norm < self._tol_orient:
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

            # ---- 3. Jacobiana whole-body e DLS --------------------------
            T_world_base = _odom_to_matrix(state.base_odom)
            q_arm        = np.array(state.q_arm)

            J_wb  = whole_body_jacobian(q_arm, T_world_base)
            J_pin = dls_pseudoinverse(J_wb, lam)
            N     = null_space_projector(J_wb, J_pin)

            # ---- 4. Tarefa principal + espaço nulo ----------------------
            q_dot_task = J_pin @ xdot_d

            q_dot_null = N @ np.concatenate([
                np.zeros(3),
                self._ns_gain * joint_limit_gradient(q_arm),
            ])

            q_dot_wb = q_dot_task + q_dot_null   # (8,)

            # ---- 5. Publica comandos ------------------------------------
            theta = float(np.arctan2(T_world_base[1, 0], T_world_base[0, 0]))

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
