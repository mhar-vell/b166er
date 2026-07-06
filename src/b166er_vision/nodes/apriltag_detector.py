#!/usr/bin/env python3
"""apriltag_detector — pose de AprilTag a partir das fisheye da T265.

Assina a fisheye crua da T265, retifica (Kannala-Brandt → pinhole
virtual) e detecta tags 36h11, publicando:

  • /b166er/tag_pose   (PoseStamped)  — pose do tag ALVO no frame óptico
                                        da câmera (frame_id do header da
                                        imagem), para a servovisão.
  • TF                                — <frame da câmera> → tag_<id>,
                                        para todos os tags detectados.
  • ~debug_image       (Image, mono8) — retificada com tags desenhados
                                        (só se ~publish_debug_image).

Parâmetros (ver config/apriltag_detector.yaml):
  ~image_topic, ~camera_info_topic, ~tag_size, ~target_tag_id,
  ~pinhole_zoom, ~publish_debug_image, ~max_reproj_error_px

A conversão Image↔numpy é manual (mono8) para não depender do cv_bridge.
"""

from typing import Optional

import cv2
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image

from b166er_vision.fisheye_apriltag import (
    AprilTagDetector,
    FisheyeRectifier,
    TagDetection,
    select_tag,
)


def _rotation_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Matriz 3×3 → quaternion [x, y, z, w] (método de Shepperd)."""
    q = np.empty(4)
    trace = np.trace(R)
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        q[3] = 0.25 * s
        q[0] = (R[2, 1] - R[1, 2]) / s
        q[1] = (R[0, 2] - R[2, 0]) / s
        q[2] = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0) * 2.0
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s
    return q / np.linalg.norm(q)


def _image_to_mono8(msg: Image) -> Optional[np.ndarray]:
    """sensor_msgs/Image → np.ndarray mono8 (sem cv_bridge)."""
    if msg.encoding in ('mono8', '8UC1'):
        img = np.frombuffer(msg.data, dtype=np.uint8)
        return img.reshape(msg.height, msg.step)[:, : msg.width]
    if msg.encoding in ('bgr8', 'rgb8'):
        img = np.frombuffer(msg.data, dtype=np.uint8)
        img = img.reshape(msg.height, msg.step // 3, 3)[:, : msg.width, :]
        code = cv2.COLOR_BGR2GRAY if msg.encoding == 'bgr8' else cv2.COLOR_RGB2GRAY
        return cv2.cvtColor(img, code)
    rospy.logwarn_throttle(
        10.0, '[apriltag] encoding não suportado: %s', msg.encoding
    )
    return None


class AprilTagNode:

    def __init__(self) -> None:
        rospy.init_node('apriltag_detector')

        image_topic = rospy.get_param(
            '~image_topic', '/t265/fisheye1/image_raw'
        )
        info_topic = rospy.get_param(
            '~camera_info_topic', '/t265/fisheye1/camera_info'
        )
        self._tag_size = float(rospy.get_param('~tag_size', 0.10))
        self._target_id = int(rospy.get_param('~target_tag_id', -1))
        self._zoom = float(rospy.get_param('~pinhole_zoom', 1.0))
        self._debug = bool(rospy.get_param('~publish_debug_image', False))
        self._max_reproj = float(rospy.get_param('~max_reproj_error_px', 2.0))

        self._rectifier: Optional[FisheyeRectifier] = None
        self._detector: Optional[AprilTagDetector] = None

        self._pub_pose = rospy.Publisher(
            '/b166er/tag_pose', PoseStamped, queue_size=1
        )
        self._pub_debug = (
            rospy.Publisher('~debug_image', Image, queue_size=1)
            if self._debug else None
        )
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.Subscriber(info_topic, CameraInfo, self._cb_info, queue_size=1)
        rospy.Subscriber(
            image_topic, Image, self._cb_image,
            queue_size=1, buff_size=2 ** 22,
        )

        rospy.loginfo(
            '[apriltag] aguardando %s (tag_size=%.3f m, alvo=%s)',
            info_topic, self._tag_size,
            self._target_id if self._target_id >= 0 else 'qualquer',
        )

    # ------------------------------------------------------------------
    def _cb_info(self, msg: CameraInfo) -> None:
        """One-shot: monta retificador + detector a partir do CameraInfo."""
        if self._rectifier is not None:
            return
        if msg.distortion_model not in ('equidistant', 'fisheye'):
            rospy.logwarn(
                '[apriltag] modelo de distorção "%s" (esperado equidistant); '
                'retificando mesmo assim com D[:4]', msg.distortion_model,
            )
        K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.D[:4], dtype=np.float64)
        self._rectifier = FisheyeRectifier(
            K, D, (msg.width, msg.height), zoom=self._zoom
        )
        self._detector = AprilTagDetector(
            self._rectifier.pinhole_K, self._tag_size
        )
        rospy.loginfo(
            '[apriltag] retificador pronto (%dx%d, fx_pinhole=%.1f)',
            msg.width, msg.height, self._rectifier.pinhole_K[0, 0],
        )

    # ------------------------------------------------------------------
    def _cb_image(self, msg: Image) -> None:
        if self._rectifier is None or self._detector is None:
            return   # ainda sem CameraInfo

        gray = _image_to_mono8(msg)
        if gray is None:
            return

        rectified = self._rectifier.rectify(gray)
        detections = [
            d for d in self._detector.detect(rectified)
            if d.reproj_error_px <= self._max_reproj
        ]

        for det in detections:
            self._broadcast_tf(det, msg)

        target = select_tag(detections, self._target_id)
        if target is not None:
            self._pub_pose.publish(self._to_pose_msg(target, msg))

        if self._pub_debug is not None:
            self._publish_debug(rectified, detections, msg)

    # ------------------------------------------------------------------
    def _to_pose_msg(self, det: TagDetection, img_msg: Image) -> PoseStamped:
        q = _rotation_to_quaternion(det.R_cam_tag)
        pose = PoseStamped()
        pose.header.stamp = img_msg.header.stamp
        pose.header.frame_id = img_msg.header.frame_id
        pose.pose.position.x = det.t_cam_tag[0]
        pose.pose.position.y = det.t_cam_tag[1]
        pose.pose.position.z = det.t_cam_tag[2]
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = q
        return pose

    def _broadcast_tf(self, det: TagDetection, img_msg: Image) -> None:
        q = _rotation_to_quaternion(det.R_cam_tag)
        t = TransformStamped()
        t.header.stamp = img_msg.header.stamp
        t.header.frame_id = img_msg.header.frame_id
        t.child_frame_id = f'tag_{det.tag_id}'
        t.transform.translation.x = det.t_cam_tag[0]
        t.transform.translation.y = det.t_cam_tag[1]
        t.transform.translation.z = det.t_cam_tag[2]
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        self._tf_broadcaster.sendTransform(t)

    def _publish_debug(self, rectified, detections, img_msg: Image) -> None:
        canvas = cv2.cvtColor(rectified, cv2.COLOR_GRAY2BGR)
        for det in detections:
            pts = det.corners_px.astype(np.int32)
            cv2.polylines(canvas, [pts], True, (0, 255, 0), 2)
            cv2.putText(
                canvas, f'id={det.tag_id} z={det.t_cam_tag[2]:.2f}m',
                tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )
        out = Image()
        out.header = img_msg.header
        out.height, out.width = canvas.shape[:2]
        out.encoding = 'bgr8'
        out.step = out.width * 3
        out.data = canvas.tobytes()
        self._pub_debug.publish(out)


if __name__ == '__main__':
    try:
        AprilTagNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
