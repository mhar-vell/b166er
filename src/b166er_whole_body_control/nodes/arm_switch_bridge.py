#!/usr/bin/env python3
"""Junta os fins de curso das três placas do RV-M2 em /b166er/arm_limit_switch.

Cada placa (Joints12_vel, Joints34_vel, Joints56_vel) publica um
std_msgs/Int8MultiArray com −1/0/+1 por junta sua (arm_switch_12,
arm_switch_34, arm_switch_5), já no sinal do MODELO (DIR_SIGN aplicado no
firmware). Este nó monta o JointState de cinco juntas que arm_joint_servo
(HOMING), state_estimator (âncora) e fuzzy_wb_controller (saturação)
consomem — o mesmo formato que arm_openloop_sim publica na simulação.
Placa muda por mais de ~stale_s: as juntas dela vão a 0 e um aviso sai a
cada 5 s (um switch que a placa não reporta não pode segurar o homing).
Só hardware: na simulação quem publica é o firmware emulado.
"""
import rospy
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import JointState

JOINTS = ['J1', 'J2', 'J3', 'J4', 'J5']
FONTES = {'arm_switch_12': (0, 2), 'arm_switch_34': (2, 2), 'arm_switch_5': (4, 1)}


class Bridge:
    def __init__(self):
        self.sw = [0.0] * 5
        self.t = {k: None for k in FONTES}
        self.stale = float(rospy.get_param('~stale_s', 1.0))
        self.pub = rospy.Publisher(rospy.get_param('~topic_out', '/b166er/arm_limit_switch'),
                                   JointState, queue_size=1)
        for topic, (i0, n) in FONTES.items():
            rospy.Subscriber(topic, Int8MultiArray, self._cb, callback_args=(topic, i0, n), queue_size=1)

    def _cb(self, msg, args):
        topic, i0, n = args
        d = list(msg.data)
        for k in range(min(n, len(d))):
            self.sw[i0 + k] = float(max(-1, min(1, int(d[k]))))
        self.t[topic] = rospy.Time.now()

    def spin(self):
        rate = rospy.Rate(float(rospy.get_param('~rate_hz', 20.0)))
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            for topic, (i0, n) in FONTES.items():
                t = self.t[topic]
                if t is None or (now - t).to_sec() > self.stale:
                    for k in range(n):
                        self.sw[i0 + k] = 0.0
                    rospy.logwarn_throttle(5.0, '[arm_switch_bridge] %s mudo há %s — fins de curso %s em 0',
                                           topic, 'sempre' if t is None else '%.1f s' % (now - t).to_sec(),
                                           JOINTS[i0:i0 + n])
            m = JointState()
            m.header.stamp = now
            m.name = JOINTS
            m.position = list(self.sw)
            self.pub.publish(m)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('arm_switch_bridge')
    Bridge().spin()
