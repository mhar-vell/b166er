#!/usr/bin/env python3
"""
gazebo_sensor_sim — ponte de sensores para validação no Gazebo.

Substitui hardware real por dados sintéticos derivados do simulador:

  • /joint_states (Gazebo) → FK do braço → /t265/odom/sample (Odometry)
  • /odom (Pioneer diff-drive) → /pioneer/pose (Odometry)

Permite rodar o stack completo (state_estimator + fuzzy_wb_controller)
sem modificar nenhum nó de controle: apenas os tópicos de fonte mudam.

Atenção: /joint_states do Gazebo pode conter as juntas de rodas do Pioneer
além das juntas do braço (J1-J5). Este nó filtra pelo nome.
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_matrix

from b166er_whole_body_control.kinematics import (
    JOINT_NAMES, T_BASELINK_ARM, fk_arm,
)


def _matrix_to_odometry(T, frame_id, child_frame_id, stamp):
    """4×4 homogeneous matrix → nav_msgs/Odometry."""
    msg = Odometry()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id  = child_frame_id

    msg.pose.pose.position.x = T[0, 3]
    msg.pose.pose.position.y = T[1, 3]
    msg.pose.pose.position.z = T[2, 3]

    # quaternion a partir da sub-matriz de rotação
    q = quaternion_from_matrix(T)          # [x, y, z, w]
    msg.pose.pose.orientation = Quaternion(*q)
    return msg


class GazeboSensorSim:

    def __init__(self):
        rospy.init_node('gazebo_sensor_sim')

        self._world_frame = rospy.get_param('~world_frame', 'odom')

        # Publicadores
        self._pub_t265   = rospy.Publisher('/t265/odom/sample',
                                           Odometry, queue_size=5)
        self._pub_pioneer = rospy.Publisher('/pioneer/pose',
                                            Odometry, queue_size=5)

        # Subscritores
        rospy.Subscriber('/joint_states', JointState,
                         self._cb_joints, queue_size=5)
        rospy.Subscriber('/odom', Odometry,
                         self._cb_odom, queue_size=5)

        # Estado da base (do diff-drive) para montar T_world_base
        self._base_odom = None

        rospy.loginfo('[gazebo_sensor_sim] pronto')

    # ------------------------------------------------------------------
    def _cb_odom(self, msg):
        """Republica /odom como /pioneer/pose (sem alteração)."""
        self._base_odom = msg
        self._pub_pioneer.publish(msg)

    def _cb_joints(self, msg):
        """Computa FK do braço e publica pose do T265 como Odometry."""
        # extrai apenas J1-J5 pelo nome
        name_to_pos = dict(zip(msg.name, msg.position))
        if not all(n in name_to_pos for n in JOINT_NAMES):
            return          # juntas do braço ainda não chegaram

        q = np.array([name_to_pos[n] for n in JOINT_NAMES])

        # FK: Base → t265_link (frame montado no EE)
        T_arm_t265 = fk_arm(q)           # em frame da base do braço

        if self._base_odom is not None:
            # calcula T_world_arm_base a partir do odom da base
            p = self._base_odom.pose.pose.position
            o = self._base_odom.pose.pose.orientation
            from tf.transformations import quaternion_matrix
            T_wb = quaternion_matrix([o.x, o.y, o.z, o.w])
            T_wb[:3, 3] = [p.x, p.y, p.z]
            T_world_arm_base = T_wb @ T_BASELINK_ARM
        else:
            # base ainda desconhecida → assume identidade
            T_world_arm_base = T_BASELINK_ARM.copy()

        T_world_t265 = T_world_arm_base @ T_arm_t265

        odom_t265 = _matrix_to_odometry(
            T_world_t265,
            frame_id=self._world_frame,
            child_frame_id='t265_pose_frame',
            stamp=msg.header.stamp,
        )
        self._pub_t265.publish(odom_t265)


if __name__ == '__main__':
    try:
        GazeboSensorSim()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
