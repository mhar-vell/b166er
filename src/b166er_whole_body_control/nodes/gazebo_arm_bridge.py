#!/usr/bin/env python3
"""
gazebo_arm_bridge — integra /b166er/arm_vel_cmd → position controllers do Gazebo.

Equivalente ao arm_vel_integrator.py mas para simulação:
  - Integra ẋ_arm (rad/s) → q_arm (rad) via Euler a 20 Hz
  - Publica Float64 individuais para /Jx_position_controller/command

Feedforward de gravidade ativo desde o primeiro ciclo (usando q_spawn)
para que os controladores nunca vejam erro estático e o braço não caia
ao inicializar. Após o warm-start com /joint_states, usa q_arm real.
"""

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from b166er_whole_body_control.kinematics import (
    JOINT_NAMES, JOINT_LOWER, JOINT_UPPER, gravity_torque_arm)

_VEL_TIMEOUT = 0.5   # s sem comando → zera velocidade

# Ganhos P dos controladores (arm_controllers.yaml) — usados no feedforward
_P_GAINS = np.array([100.0, 300.0, 200.0, 100.0, 50.0])


class GazeboArmBridge:

    def __init__(self):
        rospy.init_node('gazebo_arm_bridge')

        self._rate_hz = rospy.get_param('~rate', 20.0)
        self._k_ff    = rospy.get_param('~k_gravity_ff', 1.0)

        # Posição de spawn (deve bater com -J no spawn_model).
        # Usada para calcular o feedforward antes do warm-start real.
        q_spawn_list  = rospy.get_param('~q_spawn', [0.0, 0.5, 0.0, 0.0, 0.0])
        self._q_spawn = np.clip(np.array(q_spawn_list, dtype=float),
                                JOINT_LOWER, JOINT_UPPER)

        self._q_arm      = None          # rad — None até warm-start real
        self._dq_cmd     = np.zeros(5)
        self._t_last_cmd = None

        self._pubs = [
            rospy.Publisher(f'/{n}_position_controller/command',
                            Float64, queue_size=1)
            for n in JOINT_NAMES
        ]

        rospy.Subscriber('/joint_states', JointState,
                         self._cb_joint_states, queue_size=1)
        rospy.Subscriber('/b166er/arm_vel_cmd', JointState,
                         self._cb_vel_cmd, queue_size=1)

        rospy.loginfo('[gazebo_arm_bridge] q_spawn=%s  aguardando warm-start...',
                      np.round(self._q_spawn, 3))

    # ------------------------------------------------------------------
    def _q_pub_for(self, q_arm):
        """Posição comandada ao PID: q_arm + feedforward de gravidade."""
        tau_g = gravity_torque_arm(q_arm)
        q_ff  = -self._k_ff * tau_g / _P_GAINS
        return np.clip(q_arm + q_ff, JOINT_LOWER, JOINT_UPPER)

    def _cb_joint_states(self, msg):
        if self._q_arm is not None:
            return   # warm-start one-shot
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(n in name_to_pos for n in JOINT_NAMES):
            q_init = np.array([name_to_pos[n] for n in JOINT_NAMES])
            self._q_arm = np.clip(q_init, JOINT_LOWER, JOINT_UPPER)
            rospy.loginfo('[gazebo_arm_bridge] warm-start q=%s rad',
                          np.round(self._q_arm, 3))

    def _cb_vel_cmd(self, msg):
        if len(msg.velocity) == 5:
            self._dq_cmd     = np.array(msg.velocity, dtype=float)
            self._t_last_cmd = rospy.Time.now()

    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(self._rate_hz)
        dt   = 1.0 / self._rate_hz

        # Feedforward pré-calculado para q_spawn: evita queda do braço
        # antes do warm-start real. Com este comando, o equilíbrio do
        # PID coincide com q_spawn → nenhuma força de reação na base.
        q_pub_spawn = self._q_pub_for(self._q_spawn)

        while not rospy.is_shutdown():
            if self._q_arm is None:
                # Publica feedforward de spawn para segurar o braço
                # enquanto aguarda /joint_states.
                for i, pub in enumerate(self._pubs):
                    pub.publish(Float64(data=float(q_pub_spawn[i])))
                rate.sleep()
                continue

            if self._t_last_cmd is not None:
                age = (rospy.Time.now() - self._t_last_cmd).to_sec()
                dq = self._dq_cmd if age < _VEL_TIMEOUT else np.zeros(5)
            else:
                dq = np.zeros(5)

            self._q_arm = np.clip(self._q_arm + dq * dt,
                                  JOINT_LOWER, JOINT_UPPER)

            q_pub = self._q_pub_for(self._q_arm)
            for i, pub in enumerate(self._pubs):
                pub.publish(Float64(data=float(q_pub[i])))

            rate.sleep()


if __name__ == '__main__':
    try:
        GazeboArmBridge().spin()
    except rospy.ROSInterruptException:
        pass
