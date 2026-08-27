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

Auto-detecção: havia aqui a afirmação de que, fora do setor frontal, o
laser via a própria estrutura do robô (retornos em ~0,063 m). MEDIDO em
2026-08-27 com o robô em campo aberto: **zero retornos em todo o arco de
±90°**, com o braço recolhido E na postura de busca estendida. O setor
estreito e o piso de 0,25 m saíram dessa afirmação não verificada, e
juntos produziram uma inversão de segurança — ver as notas em
`front_sector` e `self_ignore`.

Regra que ficou: distinguir SEMPRE "nenhum eco" (livre) de "ecos, mas
todos colados no sensor" (obstáculo encostado). Fundir os dois é o que
fez este nó anunciar 5,32 m de folga com a parede a 56 mm da borda.
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
        # Setor frontal considerado (rad, ±).
        #
        # Era ±20°, "estreito de propósito porque o robô aparece no
        # próprio scan fora dele". MEDIDO em 2026-08-27, com o robô em
        # campo aberto: ZERO retornos em todo o arco de ±90°, tanto com
        # o braço recolhido quanto na postura de busca estendida. A
        # justificativa não valia para este modelo.
        #
        # O setor estreito tinha custo real: aproximando-se da parede em
        # diagonal, ela fica fora do setor. Medido na execução de
        # 2026-08-27, a parede passou de 1,17 m para 0,89 m ao longo de
        # 6 s enquanto ficava 58°-71° fora do setor e o nó reportava
        # 4,97 m.
        #
        # Continua parâmetro: na BANCADA a estrutura real (cabos, botão
        # de emergência) pode aparecer, e aí o certo é MASCARAR ÂNGULOS,
        # não filtrar por distância — ver a nota do self_ignore.
        self._sector = rospy.get_param('~front_sector', 1.05)   # ±60°

        # Piso de distância abaixo do qual o retorno é tratado como
        # estrutura do próprio robô.
        #
        # ERA 0,25 m, E ISSO INVERTIA A SEGURANÇA. Com a parede a 0,088 m
        # do laser, TODOS os retornos dela caíam abaixo do piso, eram
        # descartados como "o próprio robô", o setor ficava vazio e o nó
        # publicava range_max — ou seja, "caminho livre". Quanto mais
        # perto o obstáculo, mais convicto o nó ficava de que não havia
        # nada. Em 2026-08-27 o robô parou com a borda a 56 mm da parede
        # com este nó anunciando 5,32 m de folga.
        #
        # Agora 0,07 m, logo acima do mínimo do sensor (0,06), só para
        # rejeitar o artefato de alcance mínimo. E o caso "havia
        # retornos, mas todos abaixo do piso" deixou de ser tratado como
        # o caso "não havia retorno nenhum" — ver _cb_scan.
        self._self_ignore = rospy.get_param('~self_ignore', 0.07)
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

        no_setor = np.isfinite(r) & (r < msg.range_max) & (np.abs(ang) <= self._sector)
        # Duas situações OPOSTAS que o código antigo fundia numa só:
        #   uteis = ecos a uma distância que conta como obstáculo
        #   perto = ecos abaixo do piso de auto-filtro
        # "Nenhum eco" significa caminho livre. "Ecos, mas todos coladas
        # no sensor" significa que há algo encostado — nunca livre.
        uteis = no_setor & (r > max(msg.range_min, self._self_ignore))
        perto = no_setor & (r <= max(msg.range_min, self._self_ignore))

        if uteis.any():
            clearance = float(r[uteis].min()) + self._laser_offset
        elif perto.any():
            # Antes isto virava "range_max = livre". É o caso perigoso:
            # publicar a MEDIDA, não o alcance do sensor.
            clearance = float(r[perto].min()) + self._laser_offset
            rospy.logwarn_throttle(
                1.0, '[laser_safety] %d retornos abaixo do piso de '
                     'auto-filtro (%.3f m) — tratando como OBSTÁCULO '
                     'COLADO a %.3f m do centro da base, não como livre',
                int(perto.sum()), self._self_ignore, clearance)
        else:
            # Sem eco nenhum no setor: caminho livre até o alcance.
            clearance = msg.range_max + self._laser_offset

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
