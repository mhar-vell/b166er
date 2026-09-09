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

import argparse
import math
import sys
import time

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState, SetModelConfiguration, SetModelState
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion

STOW = [0.0, 1.13, -1.04, -1.8, 0.0]
JOINTS = ['J1', 'J2', 'J3', 'J4', 'J5']
LEVEL_TOL = 0.10      # rad — nivelado o bastante para começar
SETTLE_S = 12.0       # tempo máximo esperando estabilizar
RECOLHE_S = 12.0      # tempo máximo esperando a ponte recolher o braço

# Pose de partida — TEM QUE CASAR com os args x/y/yaw de
# b166er_gazebo.launch. Ficam duplicados porque o reset roda fora do
# launch (é um script solto chamado entre execuções da bateria) e não
# tem como ler os args de um launch que já terminou.
#
# 2 m da parede (parede em y=3,0). Antes era y=0, ou seja 3,0 m, onde a
# tag de 132 mm não é detectável — o SEARCH girava 90 s e abortava.
START_X = 0.0
START_Y = 1.0


# Tolerância de postura no reset. Folgada de propósito: o PID assenta
# com alguns graus de erro estacionário e isso não atrapalha nada — o
# que precisa ser pego é o braço em OUTRA postura (dezenas de graus),
# não o resíduo do controlador.
POSTURA_TOL = math.radians(8.0)


def _wrap(a):
    """Desembrulha para (-pi, pi]. J4 é contínua no Gazebo e acumula
    voltas: a mesma postura física já foi lida como 423° e 257,7°."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def _juntas_reais():
    """Juntas medidas no Gazebo (/joint_states), desembrulhadas.

    Ground truth da simulação. O estimated_joint_states depende do
    estimador; para conferir um RESET queremos a medida mais crua
    disponível.
    """
    try:
        js = rospy.wait_for_message('/joint_states', JointState, 5)
    except rospy.ROSException:
        return None
    z = dict(zip(js.name, js.position))
    if not all(j in z for j in JOINTS):
        return None
    return [_wrap(z[j]) for j in JOINTS]


def _erro_postura():
    """Maior desvio absoluto em relação ao stow, em rad."""
    q = _juntas_reais()
    if q is None:
        return None
    return max(abs(_wrap(a - b)) for a, b in zip(q, STOW))


def _args():
    # Pose de partida por linha de comando (bateria de poses, 2026-09-04):
    # o padrão continua sendo a pose do launch; a bateria passa outras
    # para medir se a missão depende de onde o robô começa.
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--x', type=float, default=START_X, help='m (padrão %(default)s)')
    ap.add_argument('--y', type=float, default=START_Y, help='m (padrão %(default)s)')
    ap.add_argument('--yaw', type=float, default=0.0, help='graus (padrão 0)')
    ap.add_argument('--sem-recolher', action='store_true',
                    help='pula o recolhimento do braço pela ponte antes do teleporte')
    return ap.parse_args(rospy.myargv(sys.argv)[1:])


def main():
    args = _args()
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

    # Ressincronizar a ponte FAZ PARTE do reset. Ela integra q_arm em
    # malha aberta e nunca relê /joint_states, então o teleporte das
    # juntas passa despercebido: ao despausar, ela comanda os
    # controladores de volta para a postura antiga e o braço salta.
    # Medido em 2026-08-24: reset com o braço estendido levava a
    # inclinação de 0,007 para 1,114 rad — o robô "nascia caindo".
    pub_resync = rospy.Publisher('/b166er/arm_resync', JointState,
                                 queue_size=1, latch=True)
    time.sleep(1.0)   # deixa a conexão assentar antes do primeiro publish

    # RECOLHER O BRAÇO PELA PONTE ANTES DE TELEPORTAR (2026-09-08).
    #
    # Ficou claro nas baterias do teto por postura: o reset só falhava
    # depois de execuções que terminavam com o braço ESTENDIDO E EM
    # MOVIMENTO (aproximação whole-body em search/deploy, cortada por
    # timeout) — 4 de 4 caíam na primeira tentativa, e o retry passava
    # porque então o braço já estava em stow. Depois de missões, que
    # recolhem o braço antes de terminar, nunca falhou. O teleporte de
    # juntas (set_model_configuration) muda posição, não velocidade: o
    # braço chega ao stow com a velocidade residual da execução e o PID
    # tem que segurar isso no primeiro passo de física, com o chassi
    # ainda caindo do z=0,25 do set_model_state. Recolher antes, com a
    # física correndo, deixa o braço parado e no lugar; o teleporte vira
    # só um ajuste fino. No hardware o equivalente é o homing antes de
    # reposicionar o robô.
    pub_stop = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    pub_post = rospy.Publisher('/b166er/arm_posture_cmd', JointState,
                               queue_size=1, latch=True)
    chegou = {'t': None}
    rospy.Subscriber('/b166er/arm_posture_reached', Bool,
                     lambda m: chegou.update(t=time.time()) if m.data else None)
    time.sleep(0.5)
    pub_stop.publish(Twist())
    if not args.sem_recolher:
        erro_antes = _erro_postura()
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = JOINTS
        js.position = STOW
        t_cmd = time.time()
        pub_post.publish(js)
        # arm_posture_reached é latched: só vale um True DEPOIS do comando.
        while time.time() - t_cmd < RECOLHE_S:
            time.sleep(0.2)
            if chegou['t'] is not None and chegou['t'] > t_cmd + 0.3:
                break
        e = _erro_postura()
        print('reset: braço recolhido pela ponte em %.1f s (erro %s -> %s)'
              % (time.time() - t_cmd,
                 ('%.1f°' % math.degrees(erro_antes)) if erro_antes is not None else '?',
                 ('%.1f°' % math.degrees(e)) if e is not None else '?'))
        time.sleep(0.5)   # velocidades residuais assentam

    pause()
    set_cfg('b166er', 'robot_description', JOINTS, STOW)
    # Ainda PAUSADO: a ponte precisa saber da nova postura antes do
    # primeiro passo de física, senão os controladores disparam o braço
    # para o setpoint antigo.
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = JOINTS
    js.position = STOW
    pub_resync.publish(js)
    time.sleep(0.5)
    ms = ModelState()
    ms.model_name = 'b166er'
    ms.pose.position.x = args.x
    ms.pose.position.y = args.y
    ms.pose.position.z = 0.25
    ms.pose.orientation.z = math.sin(math.radians(args.yaw) / 2.0)
    ms.pose.orientation.w = math.cos(math.radians(args.yaw) / 2.0)
    ms.reference_frame = 'world'
    set_state(ms)
    time.sleep(0.5)
    unpause()
    # Depois de despausar: o callback precisa de /joint_states fluindo
    # com a postura NOVA para reancorar no valor certo.
    time.sleep(1.0)

    # Chave de volta a fechada (modelo separado, sem física acoplada).
    # A lingueta (gatilho, 2026-09-03) volta sozinha pela mola, mas
    # zerá-la junto garante a aba DENTRO do laço antes de a lâmina ser
    # posta em 0°: na ordem inversa a aba subiria contra os batentes.
    try:
        set_cfg('chave_seccionadora_fixture', 'chave_fixture_description',
                ['chave_lingueta_joint', 'chave_blade_joint'], [0.0, 0.0])
    except rospy.ServiceException as exc:
        print('aviso: chave não resetada (%s)' % exc)

    t0 = time.time()
    tilt = None
    erro_q = None
    while time.time() - t0 < SETTLE_S:
        time.sleep(1.0)
        s = get('b166er', 'world')
        o = s.pose.orientation
        r, p, _ = euler_from_quaternion([o.x, o.y, o.z, o.w])
        tilt = math.hypot(r, p)
        erro_q = _erro_postura()
        if (tilt < LEVEL_TOL and crit['v'] is False
                and erro_q is not None and erro_q < POSTURA_TOL):
            break

    # A POSTURA ENTRA NO CRITÉRIO, não só a inclinação.
    #
    # A versão anterior só olhava tilt e tilt_critical, e por isso
    # aprovava resets que entregavam o braço na postura errada. Flagrado
    # em 2026-08-24: "reset: tilt=0.002 critico=False q=[0.0, 31.7, -0.1,
    # 110.0, -0.0] -> OK" — J4 no batente de +110° em vez dos -103° do
    # stow. A missão começava com o braço estendido e a medição seguinte
    # já nascia contaminada. Um verificador que aprova o estado errado é
    # pior que não ter verificador: dá confiança onde não há.
    ok = (tilt is not None and tilt < LEVEL_TOL and crit['v'] is False
          and erro_q is not None and erro_q < POSTURA_TOL)
    q_real = _juntas_reais()
    print('reset: pose=(%.2f, %.2f, %.0f°)  tilt=%.3f rad  critico=%s  erro_postura=%s  q=%s  -> %s'
          % (args.x, args.y, args.yaw, tilt if tilt is not None else -1, crit['v'],
             ('%.1f°' % math.degrees(erro_q)) if erro_q is not None else '?',
             [round(math.degrees(v), 1) for v in q_real] if q_real else None,
             'OK' if ok else 'FALHOU'))
    if not ok and erro_q is not None and erro_q >= POSTURA_TOL:
        print('  causa: braço fora do stow (esperado %s)'
              % [round(math.degrees(v), 1) for v in STOW])
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
