#!/usr/bin/env python3
"""handeye_collect — coleta pares (base->flange, câmera->tag) para a
calibração hand-eye eye-in-hand (T265 no flange do RV-M2), problema
clássico AX=XB (Tsai-Lenz / Park-Martin).

Este nó só COLETA os pares — não resolve a calibração. A solução (via
cv2.calibrateHandEye) fica pra um script separado, numa etapa seguinte.

Pré-requisito: bancada_fase4.launch já rodando (braço + T265 fisheye +
apriltag_detector) e o tag fixo e visível pela câmera, com o braço
posicionado manualmente numa pose inicial que já enxergue o tag.

Fluxo por pose:
  1. publica /setpoints com o alvo em graus
  2. espera IsDone em /status_1..5 (+ dwell de assentamento mecânico)
  3. confere se /b166er/tag_pose está fresco (tag visível)
  4. lê a FK Base->L5 (TF) e a pose câmera->tag
  5. grava o par

As poses de calibração são a pose ATUAL do braço (lida em /joint_states
no início, usada como seed) mais uma tabela de pequenas perturbações por
junta — assim a pose fica sempre perto de onde o operador já confirmou
que o tag está visível, em vez de varrer o espaço de juntas às cegas.

Parâmetros (ver config/handeye_collect.yaml).

Segurança: braço real. Por padrão (~confirm_each_move) pede confirmação
manual antes de CADA movimento — é a única rede de segurança do script.
Rodar com `rosrun`, não `roslaunch`: o nó lê Enter do stdin, que o
roslaunch não repassa de forma confiável.
"""

import os
import time
from typing import Dict, List, Optional

import numpy as np
import rospy
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import PoseStamped, Transform
from movemaster_msg.msg import setpoint as SetpointMsg
from movemaster_msg.msg import status as StatusMsg
from sensor_msgs.msg import JointState

from b166er_whole_body_control.kinematics import (
    JOINT_LOWER,
    JOINT_NAMES,
    JOINT_UPPER,
)


def _pose_to_matrix(pose) -> np.ndarray:
    T = tft.quaternion_matrix(
        [pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w]
    )
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T


def _transform_to_matrix(tr: Transform) -> np.ndarray:
    T = tft.quaternion_matrix(
        [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]
    )
    T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    return T


def _build_pose_table(delta_rad: np.ndarray) -> List[np.ndarray]:
    """Deltas (rad) em torno do seed: 1 (seed) + 2 por junta + 4 combinadas."""
    poses = [np.zeros(5)]
    for i in range(5):
        for sign in (1.0, -1.0):
            d = np.zeros(5)
            d[i] = sign * delta_rad[i]
            poses.append(d)

    combo_pairs = [(0, 4), (0, 4), (1, 3), (1, 3)]
    combo_signs = [(1.0, 1.0), (-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0)]
    for (i, j), (si, sj) in zip(combo_pairs, combo_signs):
        d = np.zeros(5)
        d[i] = si * delta_rad[i] * 0.7
        d[j] = sj * delta_rad[j] * 0.7
        poses.append(d)
    return poses


class HandEyeCollector:

    def __init__(self) -> None:
        rospy.init_node('handeye_collect')

        delta_deg = rospy.get_param('~delta_deg', [15.0, 10.0, 10.0, 15.0, 25.0])
        self._delta_rad = np.radians(np.array(delta_deg, dtype=float))

        self._base_frame = rospy.get_param('~base_frame', 'Base')
        self._flange_frame = rospy.get_param('~flange_frame', 'L5')

        self._settle_timeout = float(rospy.get_param('~settle_timeout', 8.0))
        self._settle_dwell = float(rospy.get_param('~settle_dwell', 0.6))
        self._tag_timeout = float(rospy.get_param('~tag_timeout', 1.5))
        self._tag_retry_wait = float(rospy.get_param('~tag_retry_wait', 2.0))

        self._confirm_each_move = bool(rospy.get_param('~confirm_each_move', True))
        self._return_to_seed = bool(rospy.get_param('~return_to_seed', True))
        self._output_dir = os.path.expanduser(
            rospy.get_param('~output_dir', '~/b166er_handeye')
        )

        self._joint_pos: Dict[str, float] = {}
        self._status: Dict[int, dict] = {}
        self._tag_pose: Optional[PoseStamped] = None
        self._tag_stamp: Optional[rospy.Time] = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._pub_setpoint = rospy.Publisher('/setpoints', SetpointMsg, queue_size=1)

        rospy.Subscriber('/joint_states', JointState, self._cb_joint_states, queue_size=1)
        for i in range(1, 6):
            rospy.Subscriber(f'/status_{i}', StatusMsg, self._make_status_cb(i), queue_size=1)
        rospy.Subscriber('/b166er/tag_pose', PoseStamped, self._cb_tag_pose, queue_size=1)

    # ------------------------------------------------------------------
    def _make_status_cb(self, idx: int):
        def _cb(msg: StatusMsg) -> None:
            self._status[idx] = {'IsDone': msg.IsDone, 'error': msg.error}
        return _cb

    def _cb_joint_states(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self._joint_pos[name] = pos

    def _cb_tag_pose(self, msg: PoseStamped) -> None:
        self._tag_pose = msg
        self._tag_stamp = rospy.Time.now()

    # ------------------------------------------------------------------
    def _wait_ready(self, timeout: float = 10.0) -> bool:
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(10)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            if all(n in self._joint_pos for n in JOINT_NAMES) and len(self._status) == 5:
                return True
            rate.sleep()
        return False

    def _get_seed_q(self) -> np.ndarray:
        return np.array([self._joint_pos[n] for n in JOINT_NAMES])

    def _publish_setpoint(self, q_rad: np.ndarray) -> None:
        deg = np.degrees(q_rad)
        msg = SetpointMsg()
        (msg.set_1, msg.set_2, msg.set_3,
         msg.set_4, msg.set_5) = [float(x) for x in deg]
        msg.set_GRIP = False
        msg.emergency_stop = False
        msg.GoHome = 0
        self._pub_setpoint.publish(msg)

    def _wait_settle(self) -> bool:
        deadline = rospy.Time.now() + rospy.Duration(self._settle_timeout)
        rate = rospy.Rate(10)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            if len(self._status) == 5 and all(s['IsDone'] for s in self._status.values()):
                rospy.sleep(self._settle_dwell)
                return True
            rate.sleep()
        return False

    def _try_capture(self, q_rad: np.ndarray) -> Optional[dict]:
        if self._tag_pose is None or self._tag_stamp is None:
            return None
        if (rospy.Time.now() - self._tag_stamp).to_sec() > self._tag_timeout:
            return None
        try:
            tf_flange = self._tf_buffer.lookup_transform(
                self._base_frame, self._flange_frame, rospy.Time(0), rospy.Duration(1.0)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('[handeye_collect] TF %s->%s indisponível: %s',
                          self._base_frame, self._flange_frame, e)
            return None
        return {
            'T_base_flange': _transform_to_matrix(tf_flange.transform),
            'T_cam_tag': _pose_to_matrix(self._tag_pose.pose),
            'camera_frame': self._tag_pose.header.frame_id,
            'stamp': self._tag_pose.header.stamp.to_sec(),
            'q_deg': np.degrees(q_rad).tolist(),
        }

    # ------------------------------------------------------------------
    def run(self) -> None:
        if not self._wait_ready():
            rospy.logerr('[handeye_collect] timeout esperando /joint_states + /status_1..5. '
                         'A bancada (bancada_fase4.launch) está rodando?')
            return

        q_seed = self._get_seed_q()
        rospy.loginfo('[handeye_collect] seed (pose atual, graus): %s',
                      np.round(np.degrees(q_seed), 1))

        deltas = _build_pose_table(self._delta_rad)
        poses = [np.clip(q_seed + d, JOINT_LOWER, JOINT_UPPER) for d in deltas]

        rospy.loginfo('[handeye_collect] %d poses planejadas:', len(poses))
        for i, q in enumerate(poses):
            rospy.loginfo('  [%2d] %s deg', i, np.round(np.degrees(q), 1))

        if self._confirm_each_move:
            ans = input('\nConfirma a coleta? O braço vai se mover — tag deve '
                        'estar fixo e visível, mão perto do E-stop. [s/N] ')
            if ans.strip().lower() not in ('s', 'y', 'sim', 'yes'):
                rospy.logwarn('[handeye_collect] cancelado pelo operador.')
                return

        samples = []
        for i, q in enumerate(poses):
            if self._confirm_each_move:
                ans = input(f'[{i + 1}/{len(poses)}] mover p/ {np.round(np.degrees(q), 1)} deg? '
                            f'[Enter=mover, p=pular, q=abortar] ')
                if ans.strip().lower() == 'q':
                    rospy.logwarn('[handeye_collect] abortado pelo operador.')
                    break
                if ans.strip().lower() == 'p':
                    continue

            self._publish_setpoint(q)
            if not self._wait_settle():
                rospy.logwarn('[handeye_collect] pose %d: timeout esperando IsDone, pulando.', i)
                continue

            sample = self._try_capture(q)
            if sample is None:
                rospy.loginfo('[handeye_collect] pose %d: tag não visível, aguardando '
                              '%.1fs e tentando de novo...', i, self._tag_retry_wait)
                rospy.sleep(self._tag_retry_wait)
                sample = self._try_capture(q)

            if sample is None:
                rospy.logwarn('[handeye_collect] pose %d: tag não visível, amostra descartada.', i)
                continue

            samples.append(sample)
            rospy.loginfo('[handeye_collect] pose %d: amostra %d/%d capturada.',
                          i, len(samples), len(poses))

        if self._return_to_seed:
            rospy.loginfo('[handeye_collect] retornando à pose inicial...')
            self._publish_setpoint(q_seed)
            self._wait_settle()

        if not samples:
            rospy.logerr('[handeye_collect] nenhuma amostra capturada, nada salvo.')
            return

        self._save(samples)

    # ------------------------------------------------------------------
    def _save(self, samples: List[dict]) -> None:
        os.makedirs(self._output_dir, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self._output_dir, f'handeye_{stamp}.npz')
        np.savez(
            path,
            T_base_flange=np.stack([s['T_base_flange'] for s in samples]),
            T_cam_tag=np.stack([s['T_cam_tag'] for s in samples]),
            q_deg=np.array([s['q_deg'] for s in samples]),
            stamp=np.array([s['stamp'] for s in samples]),
            camera_frame=samples[-1]['camera_frame'],
        )
        rospy.loginfo('[handeye_collect] %d amostras salvas em %s', len(samples), path)
        if len(samples) < 10:
            rospy.logwarn('[handeye_collect] só %d amostras — Tsai-Lenz recomenda >=10 '
                          'com boa diversidade rotacional.', len(samples))


if __name__ == '__main__':
    try:
        HandEyeCollector().run()
    except rospy.ROSInterruptException:
        pass
