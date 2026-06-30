#!/usr/bin/env python3
"""
whole_body_planner — controlador whole-body unificado do b166er.

O sistema é tratado como uma única entidade de 8-DOF:
  q_wb = [x_b, y_b, θ_b,  q1, q2, q3, q4, q5]
          ←  base  →       ←— braço RV-M2 —→

Path planning é global: Pioneer e RV-M2 se movem SIMULTANEAMENTE.

Lei de controle (PBVS whole-body com DLS + espaço nulo):

  q̇_wb = J_wb† · K · err_EE  +  (I − J_wb† J_wb) · q̇₀

  onde:
    J_wb    = Jacobiana whole-body 6×8 [J_base | J_arm]
    K       = ganho (substituído pelo Fuzzy na próxima fase)
    err_EE  = erro 6D entre EE atual (T265) e alvo
    q̇₀     = objetivo secundário (evitar limites de junta)

Tópicos
-------
  Subscreve:
    /b166er/robot_state    (RobotState)    — estado completo do robô
    /b166er/ee_target      (PoseStamped)   — pose alvo do EE no frame mundial
  Publica:
    /cmd_vel               (Twist)         — velocidade da base Pioneer
    /b166er/arm_vel_cmd    (JointState)    — velocidade das juntas do braço
    /b166er/wb_jacobian    (Float64MultiArray) — J_wb para debug/logging
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
    T_BASELINK_ARM,
    fk_arm_joint_frames,
    whole_body_jacobian,
    dls_pseudoinverse, null_space_projector,
    joint_limit_gradient,
    pose_error,
)

# ---------------------------------------------------------------------------
# Parâmetros de controle
# ---------------------------------------------------------------------------

# Ganho proporcional (será substituído pelo Fuzzy)
K_POS    = 0.8   # m/s por metro de erro
K_ORIENT = 0.6   # rad/s por rad de erro

# Saturação de velocidade
MAX_BASE_LIN  = 0.3    # m/s
MAX_BASE_ANG  = 0.5    # rad/s
MAX_ARM_VEL   = 0.8    # rad/s por junta

# Ganho do objetivo secundário (evitar limites de junta)
K_NULL   = 0.3

# DLS — será substituído por λ adaptativo do Fuzzy
DLS_LAMBDA = 0.05


# ---------------------------------------------------------------------------
# Helpers de conversão
# ---------------------------------------------------------------------------

def _odom_to_matrix(odom):
    """nav_msgs/Odometry → 4×4."""
    p = odom.pose.pose.position
    o = odom.pose.pose.orientation
    T = quaternion_matrix([o.x, o.y, o.z, o.w])
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _pose_stamped_to_matrix(ps):
    """geometry_msgs/PoseStamped → 4×4."""
    p = ps.pose.position
    o = ps.pose.orientation
    T = quaternion_matrix([o.x, o.y, o.z, o.w])
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _saturate(v, vmax):
    """Escala o vetor se a norma exceder vmax."""
    n = np.linalg.norm(v)
    return v if n <= vmax else v * (vmax / n)


# ---------------------------------------------------------------------------
# Nó principal
# ---------------------------------------------------------------------------

class WholeBodyPlanner:

    def __init__(self):
        rospy.init_node('whole_body_planner')

        # Parâmetros
        self._rate_hz  = rospy.get_param('~rate', 20.0)
        self._nonholo  = rospy.get_param('~nonholonomic', True)
        self._ns_gain  = rospy.get_param('~null_space_gain', K_NULL)

        # Estado
        self._robot_state = None
        self._target      = None   # 4×4 pose alvo do EE
        self._enabled     = False

        # Publicadores
        self._pub_cmdvel  = rospy.Publisher('/cmd_vel',            Twist,              queue_size=1)
        self._pub_armvel  = rospy.Publisher('/b166er/arm_vel_cmd', JointState,         queue_size=1)
        self._pub_jacobian = rospy.Publisher('/b166er/wb_jacobian', Float64MultiArray, queue_size=1)

        # Subscritores
        rospy.Subscriber('/b166er/robot_state', RobotState,    self._cb_state)
        rospy.Subscriber('/b166er/ee_target',   PoseStamped,   self._cb_target)

        rospy.loginfo('[whole_body_planner] pronto — aguardando /b166er/ee_target')

    # ------------------------------------------------------------------
    def _cb_state(self, msg):
        self._robot_state = msg

    def _cb_target(self, msg):
        self._target  = _pose_stamped_to_matrix(msg)
        self._enabled = True
        rospy.loginfo('[whole_body_planner] novo alvo recebido: '
                      'pos=(%.3f, %.3f, %.3f)',
                      msg.pose.position.x,
                      msg.pose.position.y,
                      msg.pose.position.z)

    # ------------------------------------------------------------------
    def _compute_wb_jacobian(self, state):
        """Monta J_wb (6×8) a partir do RobotState."""
        T_world_base = _odom_to_matrix(state.base_odom)
        q_arm = np.array(state.q_arm)
        return whole_body_jacobian(q_arm, T_world_base)

    def _compute_error(self, state, T_target):
        """Erro 6D entre EE atual (T265) e pose alvo."""
        p = state.ee_pose.pose.position
        o = state.ee_pose.pose.orientation
        T_cur = quaternion_matrix([o.x, o.y, o.z, o.w])
        T_cur[:3, 3] = [p.x, p.y, p.z]
        return pose_error(T_cur, T_target)

    def _apply_gain(self, err):
        """Ganho proporcional fixo (fase 1 — será Fuzzy na fase 2)."""
        v = np.zeros(6)
        v[:3] = K_POS    * err[:3]
        v[3:] = K_ORIENT * err[3:]
        return v

    def _null_space_objective(self, q_arm):
        """
        Objetivo secundário whole-body (8-DOF):
        [0, 0, 0,  ∇limit(q_arm)]
        — base não tem objetivo secundário aqui (sem preferência de pose).
        """
        grad_arm = joint_limit_gradient(q_arm)
        return np.concatenate([np.zeros(3), self._ns_gain * grad_arm])

    def _project_nonholonomic(self, q_dot_base, theta):
        """
        Projeta q̇_base = [ẋ, ẏ, θ̇] na constraint não-holonômica do Pioneer:
        velocidade lateral (frame do corpo) = 0.

        linear.x  = componente do ẋ,ẏ na direção de heading
        angular.z = θ̇
        """
        v_forward = np.cos(theta) * q_dot_base[0] + np.sin(theta) * q_dot_base[1]
        omega     = q_dot_base[2]
        return v_forward, omega

    def _publish_cmd_vel(self, q_dot_base, theta):
        msg = Twist()
        if self._nonholo:
            v_fwd, omega = self._project_nonholonomic(q_dot_base, theta)
            v_fwd  = float(np.clip(v_fwd,  -MAX_BASE_LIN, MAX_BASE_LIN))
            omega  = float(np.clip(omega,  -MAX_BASE_ANG, MAX_BASE_ANG))
            msg.linear.x  = v_fwd
            msg.angular.z = omega
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

    def _publish_jacobian(self, J):
        msg = Float64MultiArray()
        d0  = MultiArrayDimension(label='rows', size=J.shape[0], stride=J.shape[0]*J.shape[1])
        d1  = MultiArrayDimension(label='cols', size=J.shape[1], stride=J.shape[1])
        msg.layout.dim = [d0, d1]
        msg.data       = J.flatten().tolist()
        self._pub_jacobian.publish(msg)

    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(self._rate_hz)

        while not rospy.is_shutdown():
            if not self._enabled or self._robot_state is None:
                self._pub_cmdvel.publish(Twist())
                rate.sleep()
                continue

            state  = self._robot_state
            now    = rospy.Time.now()

            # Erro EE
            err_EE = self._compute_error(state, self._target)
            err_norm_pos    = np.linalg.norm(err_EE[:3])
            err_norm_orient = np.linalg.norm(err_EE[3:])

            # Para quando converge
            if err_norm_pos < 0.005 and err_norm_orient < 0.02:
                rospy.loginfo_throttle(2.0, '[whole_body_planner] alvo alcançado!')
                self._pub_cmdvel.publish(Twist())
                rate.sleep()
                continue

            # Jacobiana whole-body
            J_wb  = self._compute_wb_jacobian(state)
            J_pin = dls_pseudoinverse(J_wb, DLS_LAMBDA)
            N     = null_space_projector(J_wb, J_pin)

            # Velocidade cartesiana desejada (ganho fixo — fase 1)
            xdot_d = self._apply_gain(err_EE)

            # Tarefa principal
            q_dot_task = J_pin @ xdot_d

            # Objetivo secundário no espaço nulo
            q_arm = np.array(state.q_arm)
            q_dot_null = N @ self._null_space_objective(q_arm)

            # Comando whole-body
            q_dot_wb = q_dot_task + q_dot_null   # (8,)

            q_dot_base = q_dot_wb[:3]
            q_dot_arm  = q_dot_wb[3:]

            # Theta do Pioneer para projeção não-holonômica
            T_world_base = _odom_to_matrix(state.base_odom)
            theta = np.arctan2(T_world_base[1, 0], T_world_base[0, 0])

            # Publica
            self._publish_cmd_vel(q_dot_base, theta)
            self._publish_arm_vel(q_dot_arm, now)
            self._publish_jacobian(J_wb)

            rospy.loginfo_throttle(1.0,
                '[wb_planner] err_pos=%.4fm err_ori=%.4frad | '
                'base=[%.2f %.2f %.2f] arm=[%.2f %.2f %.2f %.2f %.2f]',
                err_norm_pos, err_norm_orient,
                *q_dot_base.round(3), *q_dot_arm.round(3))

            rate.sleep()


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        WholeBodyPlanner().spin()
    except rospy.ROSInterruptException:
        pass
