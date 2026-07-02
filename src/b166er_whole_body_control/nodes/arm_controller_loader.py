#!/usr/bin/env python3
"""
arm_controller_loader — carrega controladores de posição sem esperar por sim_time.

Sequência crítica:
  1. load_controller  (com Gazebo pausado — não depende de física)
  2. unpause_physics  (física começa a rodar)
  3. switch_controller (depois do unpause — switch_controller bloqueia
                        internamente aguardando o loop de física do
                        gazebo_ros_control; chamar enquanto pausado → deadlock)

O Gazebo é iniciado pausado (paused=true no launch) para garantir que o
braço não caia sob gravidade antes dos controladores estarem prontos.
"""

import time
import os
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


def wait_service_real(name, timeout=WAIT_TIMEOUT):
    """Aguarda serviço usando tempo real (não sim_time)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            rospy.wait_for_service(name, timeout=2.0)
            return True
        except rospy.ROSException:
            rospy.loginfo_throttle(5.0, '[arm_loader] aguardando %s...', name)
    return False


def unpause_gazebo():
    """Despausa o Gazebo."""
    try:
        if wait_service_real('/gazebo/unpause_physics', timeout=10.0):
            rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
            rospy.loginfo('[arm_loader] Gazebo despausado.')
        else:
            rospy.logerr('[arm_loader] timeout ao aguardar /gazebo/unpause_physics')
    except Exception as e:
        rospy.logerr('[arm_loader] falha ao despauser Gazebo: %s', e)


def main():
    os.environ['ROS_TIME_REAL'] = '1'
    rospy.init_node('arm_controller_loader', disable_rostime=True)

    rospy.loginfo('[arm_controller_loader] aguardando controller_manager...')

    loaded = []

    # ── Fase 1: load com Gazebo pausado ────────────────────────────────
    # load_controller chama init() no plugin, que lê parâmetros e registra
    # handles de hardware. Não precisa de física rodando.
    try:
        if not wait_service_real('/controller_manager/load_controller'):
            rospy.logerr('[arm_loader] timeout: controller_manager não encontrado')
            return

        load_svc = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)

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

        if not loaded:
            rospy.logerr('[arm_loader] nenhum controlador foi carregado')

    finally:
        # ── Fase 2: unpause ANTES do switch ────────────────────────────
        # CRÍTICO: switch_controller bloqueia no loop de física do
        # gazebo_ros_control. Com Gazebo pausado esse loop nunca roda
        # e o switch nunca retorna → deadlock. O unpause deve vir antes.
        unpause_gazebo()

    # ── Fase 3: switch com física rodando ──────────────────────────────
    if not loaded:
        return

    switch_svc = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    req = SwitchControllerRequest()
    req.start_controllers = loaded
    req.stop_controllers  = []
    req.strictness        = SwitchControllerRequest.BEST_EFFORT
    try:
        if switch_svc(req).ok:
            rospy.loginfo('[arm_loader] controladores iniciados: %s', loaded)
        else:
            rospy.logwarn('[arm_loader] switch_controller retornou False')
    except rospy.ServiceException as e:
        rospy.logerr('[arm_loader] erro no switch_controller: %s', e)


if __name__ == '__main__':
    main()
