#!/usr/bin/env python3
"""Reset completo do robô entre execuções da bateria.

Existe porque `set_model_state` sozinho NÃO basta: ele repõe a pose do
chassi mas deixa as juntas do braço onde estavam. Numa bateria de
2026-08-24 isso custou 8 execuções — o robô tombou com o braço
estendido, a trava de inclinação congelou o braço, e cada reset punha o
chassi em pé com o braço ainda estendido, que tombava de novo em menos
de 2 s. O estado crítico nunca limpava e as 8 execuções morreram no
primeiro comando de postura.

Ordem que funciona (verificada ao vivo): pausar a física, configurar as
juntas, repor a pose, despausar. Pausar importa — configurar juntas com
a física correndo deixa velocidades residuais.

Sai com código 1 se o robô não estabilizar nivelado, para a bateria
parar em vez de gerar execuções inúteis.
"""

import math
import sys
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelConfiguration, SetModelState
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion

STOW = [0.0, 1.13, -1.04, -1.8, 0.0]
JOINTS = ['J1', 'J2', 'J3', 'J4', 'J5']
LEVEL_TOL = 0.10      # rad — nivelado o bastante para começar
SETTLE_S = 12.0       # tempo máximo esperando estabilizar


def main():
    rospy.init_node('reset_sim', anonymous=True, disable_signals=True)
    for s in ('/gazebo/set_model_state', '/gazebo/set_model_configuration',
              '/gazebo/pause_physics', '/gazebo/unpause_physics',
              '/gazebo/get_model_state'):
        rospy.wait_for_service(s, timeout=15)

    get = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    set_cfg = rospy.ServiceProxy('/gazebo/set_model_configuration',
                                 SetModelConfiguration)
    pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

    crit = {'v': None}
    rospy.Subscriber('/b166er/tilt_critical', Bool,
                     lambda m: crit.update(v=m.data))

    pause()
    set_cfg('b166er', 'robot_description', JOINTS, STOW)
    ms = ModelState()
    ms.model_name = 'b166er'
    ms.pose.position.z = 0.25
    ms.pose.orientation.w = 1.0
    ms.reference_frame = 'world'
    set_state(ms)
    time.sleep(0.5)
    unpause()

    # Chave de volta a fechada (modelo separado, sem física acoplada).
    try:
        set_cfg('chave_seccionadora_fixture', 'chave_fixture_description',
                ['chave_blade_joint'], [0.0])
    except rospy.ServiceException as exc:
        print('aviso: chave não resetada (%s)' % exc)

    t0 = time.time()
    tilt = None
    while time.time() - t0 < SETTLE_S:
        time.sleep(1.0)
        s = get('b166er', 'world')
        o = s.pose.orientation
        r, p, _ = euler_from_quaternion([o.x, o.y, o.z, o.w])
        tilt = math.hypot(r, p)
        if tilt < LEVEL_TOL and crit['v'] is False:
            break

    try:
        js = rospy.wait_for_message('/b166er/estimated_joint_states',
                                    JointState, 5)
        q = [round(math.degrees(v), 1) for v in js.position]
    except rospy.ROSException:
        q = None

    ok = tilt is not None and tilt < LEVEL_TOL and crit['v'] is False
    print('reset: tilt=%.3f rad  critico=%s  q=%s  -> %s'
          % (tilt if tilt is not None else -1, crit['v'], q,
             'OK' if ok else 'FALHOU'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
