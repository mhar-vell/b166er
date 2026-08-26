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
        # FOV horizontal do sensor no URDF (t265_link / t265_fisheye1).
        self._hfov = rospy.get_param('~horizontal_fov', 2.7367)

        # Projeção equidistante: r = f · θ. Meia-largura em pixels
        # corresponde a meio-FOV em radianos, então f = (w/2) / (hfov/2).
        f = (self._width / 2.0) / (self._hfov / 2.0)

        info = CameraInfo()
        info.width = self._width
        info.height = self._height
        info.distortion_model = 'equidistant'
        # Sem distorção ADICIONAL além do modelo: a lente do Gazebo é
        # equidistante ideal. A T265 real traz coeficientes k1..k4 da
        # calibração de fábrica, e o driver os publica.
        info.D = [0.0, 0.0, 0.0, 0.0]
        info.K = [f, 0.0, self._width / 2.0 - 0.5,
                  0.0, f, self._height / 2.0 - 0.5,
                  0.0, 0.0, 1.0]
        info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.P = [f, 0.0, self._width / 2.0 - 0.5, 0.0,
                  0.0, f, self._height / 2.0 - 0.5, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self._info = info

        self._pub = rospy.Publisher('/camera/fisheye1/camera_info',
                                    CameraInfo, queue_size=1)
        # Espelha o stamp e o frame da IMAGEM, não do camera_info do
        # Gazebo: quem consome sincroniza os dois, e um stamp defasado
        # faria o par ser descartado.
        rospy.Subscriber('/camera/fisheye1/image_raw', Image, self._cb_image,
                         queue_size=1)

        rospy.loginfo('[fisheye_info_fixer] publicando equidistante '
                      'f=%.1f px para %dx%d (%.1f° de FOV)',
                      f, self._width, self._height, math.degrees(self._hfov))

    def _cb_image(self, msg):
        self._info.header = msg.header
        self._pub.publish(self._info)


if __name__ == '__main__':
    try:
        FisheyeInfoFixer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
