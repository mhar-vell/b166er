#!/usr/bin/env python3
"""fisheye_info_fixer — publica o camera_info correto da fisheye simulada.

Existe porque o Gazebo RENDERIZA a wideanglecamera com lente equidistante
mas PUBLICA o camera_info como se fosse pinhole, sem coeficientes de
distorção:

    distortion_model: "plumb_bob"
    D: [0, 0, 0, 0, 0]
    fx: 87.0          <- não corresponde a nenhum dos dois modelos

Medido em 2026-08-26: para 848 px cobrindo 157° com r = f·θ, o fx correto
é 309,9 — o Gazebo publica 0,28x disso. Com essa intrínseca o solvePnP
devolve distância errada por um fator ~3,5, e a simulação mediria um
alcance de detecção que não é o do robô.

O driver real da RealSense publica o modelo certo (Kannala-Brandt, que o
ROS chama de "equidistant"). Este nó existe só para a simulação chegar ao
mesmo lugar, de modo que o apriltag_localizer seja IDÊNTICO nos dois
modos — ele lê camera_info e não sabe se está no Gazebo ou na bancada.

Alternativa descartada: dar ao localizador um parâmetro de intrínseca
para simulação. Funcionaria, mas faria a simulação divergir do hardware
exatamente no ponto que estamos tentando aproximar.
"""

import math

import rospy
from sensor_msgs.msg import CameraInfo, Image


class FisheyeInfoFixer(object):

    def __init__(self):
        rospy.init_node('fisheye_info_fixer')

        self._width = rospy.get_param('~width', 848)
        self._height = rospy.get_param('~height', 800)

        # ── INTRÍNSECA MEDIDA NA T265 REAL (2026-08-26) ──
        #
        # Lida de /t265/fisheye1/camera_info com o driver da RealSense na
        # bancada, número de série 925122110468. Substitui a intrínseca
        # que eu CALCULAVA a partir do FOV nominal, e que errava:
        #
        #            calculado por mim     real
        #   fx            309,86          285,32   (-8,6%)
        #   cx            423,50          417,13   (-6,4 px)
        #   cy            399,50          393,87   (-5,6 px)
        #   D           [0,0,0,0]     calibração de fábrica
        #
        # O centro óptico NÃO é o centro do sensor, e a lente tem
        # distorção real além do modelo equidistante. Publicar a
        # intrínseca verdadeira é o que faz a simulação exercitar o mesmo
        # PnP que roda na bancada — com a calculada, a estimativa de
        # distância errava proporcionalmente à distância.
        fx = rospy.get_param('~fx', 285.3222961425781)
        fy = rospy.get_param('~fy', 285.4494934082031)
        cx = rospy.get_param('~cx', 417.1328125)
        cy = rospy.get_param('~cy', 393.86590576171875)
        # k1..k4 do modelo Kannala-Brandt, calibração de fábrica.
        d = rospy.get_param('~D', [-0.007719085086137056,
                                   0.044336311519145966,
                                   -0.04106656834483147,
                                   0.007214618846774101])

        info = CameraInfo()
        info.width = self._width
        info.height = self._height
        info.distortion_model = 'equidistant'
        info.D = list(d)
        info.K = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.P = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self._info = info
        self._fx = fx

        self._pub = rospy.Publisher('/camera/fisheye1/camera_info',
                                    CameraInfo, queue_size=1)
        # Espelha o stamp e o frame da IMAGEM, não do camera_info do
        # Gazebo: quem consome sincroniza os dois, e um stamp defasado
        # faria o par ser descartado.
        rospy.Subscriber('/camera/fisheye1/image_raw', Image, self._cb_image,
                         queue_size=1)

        rospy.loginfo('[fisheye_info_fixer] intrínseca REAL da T265: '
                      'equidistant fx=%.2f cx=%.1f cy=%.1f (%dx%d)',
                      fx, cx, cy, self._width, self._height)

    def _cb_image(self, msg):
        self._info.header = msg.header
        self._pub.publish(self._info)


if __name__ == '__main__':
    try:
        FisheyeInfoFixer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
