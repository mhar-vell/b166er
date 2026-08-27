#!/usr/bin/env python3
"""
laser_safety — mede a distância livre à frente pelo Hokuyo e sinaliza
proximidade de obstáculo.

Existe porque o laser estava publicando o tempo todo
(/pioneer3at/laser_hokuyo/scan) e nenhum nó assinava — o Marco
apontou: "há um sensor para identificar obstáculos à frente: Hokuyo.
Nós estamos usando ele?". Não estávamos. A aproximação da parede vinha
sendo contida por um plano de exclusão calculado a partir da tag, ou
seja, por inferência, quando havia medida direta disponível.

Publica:
  /b166er/front_clearance (Float64) — menor distância livre no setor
                            frontal, JÁ CORRIGIDA para o centro da
                            base (o laser fica ~21,5 cm à frente dele)
  /b166er/obstacle_close  (Bool)    — clearance abaixo do limiar

Calibração feita em 2026-08-13 contra o ground truth do Gazebo, com o
robô a três distâncias conhecidas da parede: o setor frontal (±20°)
mediu 1,768 / 0,966 / 0,559 m para esperados 1,78 / 0,98 / 0,58 —
erro de ~1,5 cm, sem retorno espúrio.

Auto-detecção: fora do setor frontal o laser vê a própria estrutura do
robô (retornos em ~0,063 m, no mínimo do sensor). Por isso o setor é
estreito e há um piso de distância (~self_ignore) abaixo do qual o
retorno é descartado como sendo o próprio robô. Ampliar o setor exige
rever esse filtro.
"""

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64


class LaserSafety(object):

    def __init__(self):
        rospy.init_node('laser_safety')

        self._scan_topic = rospy.get_param('~scan_topic',
                                           '/pioneer3at/laser_hokuyo/scan')
        # Setor frontal considerado (rad, ±). Estreito de propósito: o
        # robô aparece no próprio scan fora dele.
        self._sector = rospy.get_param('~front_sector', 0.35)   # ±20°
        # Retornos mais próximos que isto são o próprio robô.
        self._self_ignore = rospy.get_param('~self_ignore', 0.25)
        # Deslocamento do laser à frente do CENTRO DA BASE (base_link).
        #
        # 0,3189 = 0,003 (base_link -> top_plate) + 0,3159 (x do laser no
        # frame do top_plate, ver b166er.urdf.xacro). Era 0,215, medido
        # sobre a plataforma de fábrica; a plataforma foi alongada 91,8 mm
        # à frente e o Hokuyo ficou a 3 cm da nova borda. Manter o valor
        # antigo fazia o nó reportar a parede ~10 cm mais longe do que
        # está — exatamente o sintoma de o robô encostar demais.
        self._laser_offset = rospy.get_param('~laser_offset_x', 0.3189)
        self._close_thresh = rospy.get_param('~close_threshold', 0.55)

        self._pub_clear = rospy.Publisher('/b166er/front_clearance', Float64,
                                          queue_size=1)
        self._pub_close = rospy.Publisher('/b166er/obstacle_close', Bool,
                                          queue_size=1, latch=True)
        self._was_close = None

        rospy.Subscriber(self._scan_topic, LaserScan, self._cb_scan, queue_size=1)
        rospy.loginfo('[laser_safety] usando %s — setor ±%.2f rad, limiar '
                      '%.2f m (dist. medida a partir do CENTRO da base)',
                      self._scan_topic, self._sector, self._close_thresh)

    def _cb_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=float)
        ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment

        valid = (np.isfinite(r) & (r > max(msg.range_min, self._self_ignore))
                 & (r < msg.range_max) & (np.abs(ang) <= self._sector))
        if not valid.any():
            # Sem retorno no setor = caminho livre até o alcance do
            # sensor. Publicar o alcance máximo é mais útil que silêncio.
            clearance = msg.range_max + self._laser_offset
        else:
            clearance = float(r[valid].min()) + self._laser_offset

        self._pub_clear.publish(Float64(data=clearance))

        close = clearance < self._close_thresh
        if close != self._was_close:
            self._pub_close.publish(Bool(data=close))
            if close:
                rospy.logwarn('[laser_safety] obstáculo a %.2f m do centro da '
                              'base (limiar %.2f)', clearance, self._close_thresh)
            else:
                rospy.loginfo('[laser_safety] caminho livre (%.2f m)', clearance)
            self._was_close = close


if __name__ == '__main__':
    try:
        LaserSafety()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
