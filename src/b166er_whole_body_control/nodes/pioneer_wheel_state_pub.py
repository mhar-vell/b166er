#!/usr/bin/env python3
"""Publica joint states das rodas do Pioneer a posição zero.

Em mode:=gazebo, o joint_state_controller só publica J1-J5 (braço).
As 4 rodas do Pioneer não têm <transmission> no URDF e não passam pelo
ros_control, então nunca aparecem em /joint_states. Sem esse estado,
robot_state_publisher não calcula o TF das rodas e o RViz não exibe o
modelo completo do Pioneer.

Este nó publica as 4 rodas a posição=0 para fechar a árvore TF sem
interferir nos estados reais do braço.
"""
import rospy
from sensor_msgs.msg import JointState

WHEEL_JOINTS = [
    'p3at_front_left_wheel_joint',
    'p3at_front_right_wheel_joint',
    'p3at_back_left_wheel_joint',
    'p3at_back_right_wheel_joint',
]


def main():
    rospy.init_node('pioneer_wheel_state_pub')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

    # Aguarda /clock não-zero para não inundar TF com stamp=0 antes do Gazebo iniciar.
    # rospy.WallDuration.sleep() usa tempo real — seguro antes do /clock chegar.
    rospy.loginfo('[pioneer_wheel_state_pub] aguardando /clock...')
    while not rospy.is_shutdown() and rospy.Time.now().is_zero():
        rospy.WallDuration(0.05).sleep()

    rate = rospy.Rate(10)   # 10 Hz é suficiente para rodas estáticas
    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name     = WHEEL_JOINTS
        msg.position = [0.0] * len(WHEEL_JOINTS)
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
