#!/usr/bin/env python3
"""
arm_openloop_sim — "firmware emulado" do braço SEM encoders (modo braco:=malha_aberta).

O RV-M2 real não tem encoder funcional em nenhuma junta (orientador,
2026-10-06). O que o firmware de velocidade (Joints*_vel.ino) consegue fazer é
receber, por junta, uma velocidade pedida em graus/s e aplicar sentido + PWM
por um período, sem nenhuma malha de posição na placa. Este nó faz EXATAMENTE
isso na simulação, para que a missão seja testada contra o que o robô real
tem — e não contra os controladores de posição do Gazebo, que seguram a
junta onde se pede por construção.

Entrada :  /setpoints (movemaster_msg/setpoint) — set_1..set_5 em GRAUS/S
           (mesma mensagem e mesma convenção que o firmware de velocidade
           vai receber do arm_joint_servo.py). emergency_stop para tudo.
Saída   :  /J<k>_velocity_controller/command (rad/s) — os controladores de
           velocidade do gazebo_ros_control (arm_controllers_vel.yaml).

O que é emulado de propósito, por ser o que o hardware faz:
  · zona morta (~v_min_deg): PWM abaixo do atrito do redutor não move;
  · saturação (~v_max_deg): PWM máximo;
  · ganho de execução por junta (~ganho): o motor real não entrega a
    velocidade pedida com exatidão — 1,0 é nominal; use 0,8 ou 1,2 numa
    bateria para ver se a malha da T265 absorve o erro;
  · watchdog (~watchdog_s): sem mensagem nova a placa para os motores;
  · inclinação crítica: para tudo (a placa real recebe emergency_stop).

Nada aqui lê /joint_states. A única realimentação do braço continua sendo
a T265, no estimador e no Fuzzy.
"""
import numpy as np
import rospy
from std_msgs.msg import Float64, Bool
from movemaster_msg.msg import setpoint as SetpointMsg

JOINTS = ['J1', 'J2', 'J3', 'J4', 'J5']


class FirmwareEmulado:
    def __init__(self):
        rospy.init_node('arm_openloop_sim')
        self._rate_hz   = float(rospy.get_param('~rate', 50.0))
        self._v_max     = float(rospy.get_param('~v_max_deg', 60.0))
        self._v_min     = float(rospy.get_param('~v_min_deg', 1.5))
        self._ganho     = np.array(rospy.get_param('~ganho', [1.0] * 5), dtype=float)
        self._watchdog  = float(rospy.get_param('~watchdog_s', 0.5))
        topico          = rospy.get_param('~topic_setpoints', '/setpoints')

        self._v_cmd   = np.zeros(5)      # graus/s pedidos
        self._t_cmd   = None
        self._parado  = False            # emergency_stop / tilt
        self._pubs = [rospy.Publisher('/%s_velocity_controller/command' % j,
                                      Float64, queue_size=1) for j in JOINTS]
        rospy.Subscriber(topico, SetpointMsg, self._cb_setpoint, queue_size=1)
        rospy.Subscriber('/b166er/tilt_critical', Bool, self._cb_tilt)
        rospy.loginfo('[arm_openloop_sim] firmware emulado: %s em graus/s, zona morta '
                      '%.1f, máx %.0f, ganho %s, watchdog %.2f s', topico,
                      self._v_min, self._v_max, self._ganho.round(2).tolist(),
                      self._watchdog)

    def _cb_setpoint(self, m):
        if m.emergency_stop:
            self._parado = True
            self._v_cmd = np.zeros(5)
            rospy.logwarn_throttle(2.0, '[arm_openloop_sim] emergency_stop: motores parados')
            return
        self._parado = False
        self._v_cmd = np.array([m.set_1, m.set_2, m.set_3, m.set_4, m.set_5], dtype=float)
        self._t_cmd = rospy.Time.now()

    def _cb_tilt(self, m):
        if m.data and not self._parado:
            self._parado = True
            self._v_cmd = np.zeros(5)
            rospy.logwarn('[arm_openloop_sim] inclinação crítica: motores parados')

    def spin(self):
        rate = rospy.Rate(self._rate_hz)
        avisou = False
        while not rospy.is_shutdown():
            v = self._v_cmd.copy()
            if self._parado or self._t_cmd is None:
                v[:] = 0.0
            elif (rospy.Time.now() - self._t_cmd).to_sec() > self._watchdog:
                if not avisou:
                    rospy.logwarn('[arm_openloop_sim] watchdog: sem comando há %.1f s — '
                                  'motores parados', self._watchdog)
                    avisou = True
                v[:] = 0.0
            else:
                avisou = False
            # zona morta, saturação, ganho de execução
            v = np.where(np.abs(v) < self._v_min, 0.0, v)
            v = np.clip(v, -self._v_max, self._v_max) * self._ganho
            for i, pub in enumerate(self._pubs):
                pub.publish(Float64(data=float(np.radians(v[i]))))
            rate.sleep()


if __name__ == '__main__':
    try:
        FirmwareEmulado().spin()
    except rospy.ROSInterruptException:
        pass
