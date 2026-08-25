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
  /b166er/tilt_critical (Bool)    — passou do limiar crítico

Quem consome (três, e isso importa):
  · fuzzy_wb_controller — zera comando de base e braço
  · gazebo_arm_bridge   — cancela a rampa da tarefa e RECOLHE o braço
                          para stow_home (recolher reduz o momento que
                          derruba o robô; congelar o mantinha)
  · chave_mission       — aborta a missão, indo para ABORT_SAFE

A primeira versão era ouvida SÓ pela máquina de estados, e o Marco
pegou a falha: o robô tombou fora de uma missão, a IMU avisou
corretamente e `rostopic info` mostrava "Subscribers: None". Detectar
não é reagir — corte de comando tem que valer sempre, não só enquanto
uma tarefa está em execução.

O sinal NÃO é permanente: limpa sozinho depois que o robô fica
nivelado de forma sustentada (~recover_time). Latch eterno travava o
braço mesmo após reposicionar o robô, e numa bateria automatizada um
único tombo inviabilizava todas as execuções seguintes. O evento
continua registrado em ERROR — o que muda é poder recuperar.

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

        # Tempo que o robô precisa ficar nivelado para o estado crítico
        # ser limpo. O latch original nunca limpava ("se tombou, o
        # operador precisa saber"), mas isso trava o sistema para
        # sempre: o braço fica congelado mesmo depois do robô ser
        # reposicionado, e numa bateria automatizada um tombo
        # inviabilizava todas as execuções seguintes. O evento continua
        # sendo registrado em ERROR — o que muda é só a possibilidade
        # de recuperar.
        self._recover_time = rospy.get_param('~recover_time', 2.0)
        self._recover_tilt = rospy.get_param('~recover_tilt',
                                             self._warn_tilt * 0.5)
        self._level_since = None

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
            self._warned = False
            self._pub_warn.publish(Bool(data=False))

        # Recuperação do estado crítico: exige o robô NIVELADO de forma
        # sustentada, não só um instante abaixo do limiar — leitura
        # oscilando durante um tombamento cruza o limiar várias vezes.
        if self._critical:
            if tilt < self._recover_tilt:
                if self._level_since is None:
                    self._level_since = rospy.Time.now()
                elif (rospy.Time.now() - self._level_since).to_sec() >= self._recover_time:
                    self._critical = False
                    self._warned = False
                    self._level_since = None
                    self._pub_crit.publish(Bool(data=False))
                    self._pub_warn.publish(Bool(data=False))
                    rospy.logwarn('[tilt_monitor] robô nivelado por %.1fs '
                                  '(%.2f rad) — estado crítico LIMPO, comandos '
                                  'liberados', self._recover_time, tilt)
            else:
                self._level_since = None


if __name__ == '__main__':
    try:
        TiltMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
