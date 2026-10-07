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

FREIO / AUTOTRAVAMENTO (2026-10-06, primeira subida): com velocidade zero
os controladores de velocidade do Gazebo NÃO seguram a junta — o braço caiu
de J2 = +65° para −65° em segundos depois de "chegar" à postura recolhida.
O RV-M2 real não faz isso: J2 e J3 têm freio eletromagnético e os redutores
harmônicos (110:1 a 161:1) praticamente não retro-acionam. Este nó emula
essa MECÂNICA: quando o comando de uma junta é zero, ela é segurada na
posição em que parou. Para isso lê /joint_states (verdade do Gazebo) —
e SÓ para isso. Não é realimentação para o controle: nenhum comando de
movimento usa essa leitura; é o equivalente do freio físico, que também
"sabe" onde a junta está porque a trava mecanicamente.

Fora o freio, nada aqui lê /joint_states. A única realimentação do braço
para o controle continua sendo a T265, no estimador e no Fuzzy.
"""
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Bool
from movemaster_msg.msg import setpoint as SetpointMsg
from b166er_whole_body_control.kinematics import JOINT_LOWER, JOINT_UPPER

JOINTS = ['J1', 'J2', 'J3', 'J4', 'J5']


class FirmwareEmulado:
    def __init__(self):
        rospy.init_node('arm_openloop_sim')
        self._rate_hz   = float(rospy.get_param('~rate', 50.0))
        self._v_max     = float(rospy.get_param('~v_max_deg', 60.0))
        self._v_min     = float(rospy.get_param('~v_min_deg', 1.5))
        self._ganho     = np.array(rospy.get_param('~ganho', [1.0] * 5), dtype=float)
        self._watchdog  = float(rospy.get_param('~watchdog_s', 0.5))
        self._freio_kp  = float(rospy.get_param('~freio_kp', 6.0))      # 1/s
        self._freio_vmax = float(rospy.get_param('~freio_vmax_deg', 40.0))
        # FINS DE CURSO (2026-10-07, bateria 6). As placas reais leem os
        # switches LS_xA/LS_xB (ativos em HIGH) e cortam o sentido que
        # encosta neles (driveJx nos Joints*_vel.ino). Sem isso aqui, uma
        # estimativa presa no espelho fez o servo empurrar J2/J3 até o
        # batente e GIRAR o J4 por 681° (o batente do ODE cede). Emulado:
        # switch ativo a < ~ls_margem_deg do limite; corta o sentido que
        # empurra contra ele e PUBLICA o estado em /b166er/arm_limit_switch
        # (JointState, position = −1/0/+1 por junta) — é um sensor que o
        # robô real TEM (as placas devem publicar o mesmo a partir dos pinos
        # LS) e que o estimador usa como referência absoluta da junta.
        self._ls_margem  = np.radians(float(rospy.get_param('~ls_margem_deg', 0.2)))
        self._pub_ls = rospy.Publisher('/b166er/arm_limit_switch', JointState, queue_size=1)
        topico          = rospy.get_param('~topic_setpoints', '/setpoints')
        self._q_true = None
        self._q_hold = [None] * 5
        rospy.Subscriber('/joint_states', JointState, self._cb_js, queue_size=1)

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

    def _cb_js(self, m):
        # Verdade do Gazebo, usada SÓ pelo freio emulado (ver docstring).
        if 'J1' in m.name:
            q = dict(zip(m.name, m.position))
            self._q_true = np.array([q[j] for j in JOINTS], dtype=float)

    def _freio(self, i, v_i):
        """Junta i com comando zero: segura onde parou (freio/autotravamento)."""
        if self._q_true is None:
            return 0.0
        if self._q_hold[i] is None:
            self._q_hold[i] = float(self._q_true[i])
        e = self._q_hold[i] - float(self._q_true[i])
        return float(np.clip(np.degrees(self._freio_kp * e), -self._freio_vmax, self._freio_vmax))

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
            # fins de curso: corta o sentido que empurra contra o switch
            ls = np.zeros(5)
            if self._q_true is not None:
                ls = np.where(self._q_true > JOINT_UPPER - self._ls_margem, 1.0,
                              np.where(self._q_true < JOINT_LOWER + self._ls_margem, -1.0, 0.0))
                v = np.where((ls > 0) & (v > 0), 0.0, v)
                v = np.where((ls < 0) & (v < 0), 0.0, v)
                m_ls = JointState(); m_ls.header.stamp = rospy.Time.now()
                m_ls.name = JOINTS; m_ls.position = ls.tolist()
                self._pub_ls.publish(m_ls)
            # freio: junta sem comando segura onde parou; com comando, solta
            for i in range(5):
                if v[i] == 0.0:
                    v[i] = self._freio(i, v[i])
                else:
                    self._q_hold[i] = None
            for i, pub in enumerate(self._pubs):
                pub.publish(Float64(data=float(np.radians(v[i]))))
            rate.sleep()


if __name__ == '__main__':
    try:
        FirmwareEmulado().spin()
    except rospy.ROSInterruptException:
        pass
