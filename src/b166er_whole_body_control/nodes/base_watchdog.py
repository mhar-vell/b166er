#!/usr/bin/env python3
"""base_watchdog — para a base quando os comandos de velocidade cessam.

POR QUE EXISTE. O `libgazebo_ros_skid_steer_drive` não implementa
`commandTimeout`: ele aplica o último `/cmd_vel` recebido
INDEFINIDAMENTE. Se quem comanda morre no meio de um avanço, a base
continua rolando até esbarrar em algo.

Foi exatamente o que aconteceu em 2026-08-27, e o Marco viu ao vivo:
a bateria mata a missão com `pkill -9` (que não dá chance de
desligamento ordenado), a missão morreu durante um avanço, e o robô
seguiu andando ~420 mm depois do fim da execução até prensar a
ferramenta contra a lâmina da chave — 2607 contatos
`tool_rod x chave_blade` medidos com o `/cmd_vel` já em silêncio.

Antes disso o mesmo mecanismo já tinha deixado o robô com a borda a
56 mm da parede, e eu atribuí aquilo só ao filtro do `laser_safety`.
A correção do laser era necessária e está certa, mas não era a causa
desta parte.

POR QUE UM NÓ, E NÃO SÓ UM `commandTimeout` NO PLUGIN. Duas razões:
· o plugin usado não tem esse parâmetro (o `diff_drive` tem; o
  `skid_steer` não), e trocar de plugin mexe na tração de 4 rodas que
  hoje funciona;
· o mesmo risco existe na BANCADA. Um nó que morre — travamento,
  Ctrl-C, queda de rede — não deve deixar 28 kg de robô rolando. Este
  nó vale nos dois mundos.

COMO NÃO BRIGA COM QUEM COMANDA. Só publica zero quando o ÚLTIMO
comando recebido foi não-nulo E já se passou `~timeout` sem nada novo.
Depois de publicar, o próprio zero volta pelo subscribe e o nó se
aquieta — não há realimentação. Enquanto alguém comanda normalmente,
este nó fica calado.
"""

import rospy
from geometry_msgs.msg import Twist


def _nulo(t, eps=1e-4):
    return (abs(t.linear.x) < eps and abs(t.linear.y) < eps
            and abs(t.linear.z) < eps and abs(t.angular.x) < eps
            and abs(t.angular.y) < eps and abs(t.angular.z) < eps)


class BaseWatchdog(object):

    def __init__(self):
        rospy.init_node('base_watchdog')

        # 0,5 s: folgado para qualquer publicador em regime (a missão
        # comanda a 10-20 Hz) e curto o bastante para a base andar
        # poucos centímetros antes de parar. A 0,25 m/s são 12 cm.
        self._timeout = rospy.get_param('~timeout', 0.5)
        self._topic = rospy.get_param('~cmd_topic', '/cmd_vel')

        self._ultimo = None        # último Twist recebido
        self._t_ultimo = None      # quando chegou
        self._parou = False        # já publicamos o zero deste silêncio?

        self._pub = rospy.Publisher(self._topic, Twist, queue_size=1)
        rospy.Subscriber(self._topic, Twist, self._cb, queue_size=1)

        rospy.loginfo('[base_watchdog] vigiando %s — para a base após '
                      '%.2f s sem comando', self._topic, self._timeout)

    def _cb(self, msg):
        self._ultimo = msg
        self._t_ultimo = rospy.Time.now()
        if not _nulo(msg):
            # Comando de movimento novo: rearma o watchdog.
            self._parou = False

    def spin(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            rate.sleep()
            if self._t_ultimo is None or self._ultimo is None:
                continue
            if self._parou or _nulo(self._ultimo):
                continue
            idade = (rospy.Time.now() - self._t_ultimo).to_sec()
            if idade < self._timeout:
                continue
            self._pub.publish(Twist())
            self._parou = True
            rospy.logwarn('[base_watchdog] %.2f s sem comando com a base '
                          'em movimento (v=%.2f m/s, w=%.2f rad/s) — '
                          'PARANDO', idade, self._ultimo.linear.x,
                          self._ultimo.angular.z)


if __name__ == '__main__':
    try:
        BaseWatchdog().spin()
    except rospy.ROSInterruptException:
        pass
