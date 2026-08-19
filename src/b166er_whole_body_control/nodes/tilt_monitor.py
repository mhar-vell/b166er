#!/usr/bin/env python3
"""
tilt_monitor — vigia a inclinação do chassi pela IMU e sinaliza risco
de tombamento.

Existe porque o robô tombou várias vezes durante o desenvolvimento da
missão da chave (2026-08-12/13) tendo uma IMU a bordo que ninguém
estava usando — o Marco apontou isso: "há um IMU para justamente dar
informação para uma possível queda (parece que nós não estamos
usando)". A AHRS-8 estava no URDF só como geometria, sem sensor no
Gazebo e sem nenhum consumidor.

Lê /imu/data e publica:
  /b166er/tilt          (Float64) — inclinação em rad, √(roll² + pitch²)
  /b166er/tilt_warning  (Bool)    — passou do limiar de alerta
  /b166er/tilt_critical (Bool)    — passou do limiar crítico (latched)

Quem consome: chave_mission aborta a missão em tilt_critical, indo para
ABORT_SAFE (para a base e recolhe o braço). O sinal é latched de
propósito: uma vez que o robô tombou, o estado não deve ser esquecido
por um pico de ruído voltar abaixo do limiar.

Por que roll/pitch da IMU do BRAÇO servem para o chassi: a AHRS-8 é
filha de L1, e L1 gira em relação à base apenas em torno de Z (J1 tem
eixo Z e origem sem rpy; a montagem da IMU só acrescenta yaw). Roll e
pitch passam inalterados — ver comentário no movemaster.urdf.xacro.

A lógica de fator de estabilidade veio de
b166er_robot/scripts/stability_controller.py, um protótipo órfão que
nunca foi integrado (ele filtrava /cmd_vel_raw → /cmd_vel_filtered,
encanamento que não corresponde a este stack).
"""

import math

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64
from tf.transformations import euler_from_quaternion


class TiltMonitor(object):

    def __init__(self):
        rospy.init_node('tilt_monitor')

        # 0,25 rad ≈ 14° — o Pioneer 3-AT ainda se recupera; serve de
        # alerta antecipado. 0,45 rad ≈ 26° — passou disso nos
        # tombamentos observados, não volta sozinho.
        self._warn_tilt = rospy.get_param('~warn_tilt', 0.25)
        self._crit_tilt = rospy.get_param('~critical_tilt', 0.45)

        self._pub_tilt = rospy.Publisher('/b166er/tilt', Float64, queue_size=1)
        self._pub_warn = rospy.Publisher('/b166er/tilt_warning', Bool,
                                         queue_size=1, latch=True)
        self._pub_crit = rospy.Publisher('/b166er/tilt_critical', Bool,
                                         queue_size=1, latch=True)

        self._warned = False
        self._critical = False
        self._pub_warn.publish(Bool(data=False))
        self._pub_crit.publish(Bool(data=False))

        rospy.Subscriber('/imu/data', Imu, self._cb_imu, queue_size=5)
        rospy.loginfo('[tilt_monitor] vigiando /imu/data — alerta %.2f rad '
                      '(%.0f°), crítico %.2f rad (%.0f°)',
                      self._warn_tilt, math.degrees(self._warn_tilt),
                      self._crit_tilt, math.degrees(self._crit_tilt))

    def _cb_imu(self, msg):
        o = msg.orientation
        roll, pitch, _ = euler_from_quaternion([o.x, o.y, o.z, o.w])
        tilt = math.hypot(roll, pitch)
        self._pub_tilt.publish(Float64(data=tilt))

        if tilt >= self._crit_tilt and not self._critical:
            self._critical = True
            self._pub_crit.publish(Bool(data=True))
            rospy.logerr('[tilt_monitor] INCLINAÇÃO CRÍTICA: %.2f rad (%.0f°) '
                         '— roll=%.2f pitch=%.2f. Risco de tombamento.',
                         tilt, math.degrees(tilt), roll, pitch)
        elif tilt >= self._warn_tilt and not self._warned:
            self._warned = True
            self._pub_warn.publish(Bool(data=True))
            rospy.logwarn('[tilt_monitor] inclinação de alerta: %.2f rad (%.0f°)',
                          tilt, math.degrees(tilt))
        elif tilt < self._warn_tilt * 0.7 and self._warned and not self._critical:
            # Histerese na volta — só o alerta se recupera. O crítico é
            # latched: se o robô chegou a tombar, o operador precisa
            # saber, mesmo que a leitura oscile de volta.
            self._warned = False
            self._pub_warn.publish(Bool(data=False))


if __name__ == '__main__':
    try:
        TiltMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
