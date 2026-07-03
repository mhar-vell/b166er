#!/usr/bin/env python3
"""Publica joint states das rodas do Pioneer a posição zero.

Em mode:=gazebo, o joint_state_controller só publica J1-J5 (braço).
As 4 rodas do Pioneer não têm <transmission> no URDF e não passam pelo
ros_control, então nunca aparecem em /joint_states. Sem esse estado,
robot_state_publisher não calcula o TF das rodas e o RViz não exibe o
modelo completo do Pioneer.

Este nó publica as 4 rodas a posição=0 para fechar a árvore TF sem
interferir nos estados reais do braço.

Problema clássico de "roda branca no centro":
    O joint_state_controller publica J1-J5 no 1.º tick do Gazebo (~1ms
    após o unpause). Se o roda-pub demora mais que isso para publicar as
    rodas, robot_state_publisher desconhece a posição das rodas e o RViz
    renderiza os 4 links de roda na origem do fixed_frame — aparece como
    um disco branco no centro embaixo do Pioneer.

    Solução: durante a fase pausada fazemos polling rápido (2ms) do
    clock. Assim que o relógio avança (unpause), publicamos imediatamente
    — garantindo que robot_state_publisher já conhece as rodas antes de
    processar o 1.º joint_state do braço.
"""
import time
import rospy
from sensor_msgs.msg import JointState

WHEEL_JOINTS = [
    'p3at_front_left_wheel_joint',
    'p3at_front_right_wheel_joint',
    'p3at_back_left_wheel_joint',
    'p3at_back_right_wheel_joint',
]


def _publish(pub, stamp):
    msg = JointState()
    msg.header.stamp = stamp
    msg.name = WHEEL_JOINTS
    msg.position = [0.0] * len(WHEEL_JOINTS)
    pub.publish(msg)


def main():
    rospy.init_node('pioneer_wheel_state_pub')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

    # ── Fase 1: polling rápido até o clock avançar (Gazebo despausado) ──
    # tf2 rejeita transforms com stamp=0 (Gazebo ainda pausado).
    # Polling a 2ms minimiza a janela sem TF das rodas após o unpause.
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        if not stamp.is_zero():
            _publish(pub, stamp)
            rospy.loginfo('[wheel_pub] clock ativo (%.3f s) — rodas publicadas',
                          stamp.to_sec())
            break
        time.sleep(0.002)   # 2ms — independente de sim_time

    # ── Fase 2: manutenção a 50 Hz em tempo de parede ──────────────────
    # rospy não tem WallRate; time.sleep é o equivalente independente de
    # sim_time (rospy.Rate bloquearia se o Gazebo pausasse de novo).
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        if not stamp.is_zero():
            _publish(pub, stamp)
        time.sleep(0.02)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
