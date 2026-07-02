#!/usr/bin/env python3
"""
arm_controller_loader — carrega controladores de posição sem esperar por sim_time.

Substitui o spawner padrão que fica preso aguardando /clock em modo gazebo.
Aguarda o serviço controller_manager/load_controller usando tempo real,
depois carrega e inicia os controladores.
"""

import time
import rospy
from controller_manager_msgs.srv import (
    LoadController, SwitchController, SwitchControllerRequest
)
from std_srvs.srv import Empty

CONTROLLERS = [
    'joint_state_controller',
    'J1_position_controller',
    'J2_position_controller',
    'J3_position_controller',
    'J4_position_controller',
    'J5_position_controller',
]

WAIT_TIMEOUT = 60   # s em tempo real


def wait_service(name):
    """Aguarda serviço usando tempo real (não sim_time)."""
    import socket
    import xmlrpc.client
    deadline = time.time() + WAIT_TIMEOUT
    while time.time() < deadline:
        try:
            rospy.wait_for_service(name, timeout=2.0)
            return True
        except rospy.ROSException:
            rospy.loginfo_throttle(5.0, '[arm_loader] aguardando %s...', name)
    return False


def main():
    # init SEM esperar por sim_time — desativa internamente para não bloquear
    import os
    os.environ['ROS_TIME_REAL'] = '1'   # hint; nem sempre funciona em noetic
    rospy.init_node('arm_controller_loader', disable_rostime=True)

    rospy.loginfo('[arm_controller_loader] aguardando controller_manager...')
    if not wait_service('/controller_manager/load_controller'):
        rospy.logerr('[arm_loader] timeout: controller_manager não encontrado')
        return

    load_svc    = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
    switch_svc  = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

    loaded = []
    for ctrl in CONTROLLERS:
        try:
            resp = load_svc(name=ctrl)
            if resp.ok:
                rospy.loginfo('[arm_loader] carregado: %s', ctrl)
                loaded.append(ctrl)
            else:
                rospy.logwarn('[arm_loader] falha ao carregar: %s', ctrl)
        except rospy.ServiceException as e:
            rospy.logerr('[arm_loader] erro ao carregar %s: %s', ctrl, e)

    if loaded:
        req = SwitchControllerRequest()
        req.start_controllers = loaded
        req.stop_controllers  = []
        req.strictness        = SwitchControllerRequest.BEST_EFFORT
        resp = switch_svc(req)
        if resp.ok:
            rospy.loginfo('[arm_loader] controladores iniciados: %s', loaded)
        else:
            rospy.logwarn('[arm_loader] switch_controller retornou False')
    else:
        rospy.logerr('[arm_loader] nenhum controlador foi carregado')

    # Despausa o Gazebo. A simulação foi iniciada pausada para garantir que
    # o braço não caia sob gravidade antes dos controladores estarem prontos.
    try:
        rospy.wait_for_service('/gazebo/unpause_physics', timeout=5.0)
        unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause()
        rospy.loginfo('[arm_loader] Gazebo despausado.')
    except Exception as e:
        rospy.logwarn('[arm_loader] não foi possível despauser: %s', e)


if __name__ == '__main__':
    main()
