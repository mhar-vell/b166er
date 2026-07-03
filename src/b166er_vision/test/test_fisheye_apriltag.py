"""Testes end-to-end do pipeline fisheye → pinhole → AprilTag → pose.

Sem hardware e sem ROS: renderiza um tag 36h11 numa pose 3D conhecida,
sintetiza a imagem FISHEYE correspondente (modelo Kannala-Brandt com
coeficientes típicos da T265) e verifica que o pipeline completo
recupera a pose com erro pequeno.
"""

from typing import Tuple

import cv2
import numpy as np
import pytest

from b166er_vision.fisheye_apriltag import (
    AprilTagDetector,
    FisheyeRectifier,
    select_tag,
)

# ── Câmera sintética no espírito da T265 ───────────────────────────────
SIZE: Tuple[int, int] = (848, 800)
K_FISHEYE = np.array(
    [[285.0, 0.0, 424.0],
     [0.0, 285.0, 400.0],
     [0.0, 0.0, 1.0]]
)
D_FISHEYE = np.array([-0.008, 0.045, -0.043, 0.008])   # ordem de grandeza T265

TAG_SIZE = 0.10   # m
TAG_ID = 3


def _render_tag_pinhole(
    pinhole_K: np.ndarray, rvec: np.ndarray, tvec: np.ndarray
) -> np.ndarray:
    """Renderiza o tag (com borda branca) visto por uma câmera pinhole."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker_px = 300
    marker = cv2.aruco.generateImageMarker(dictionary, TAG_ID, marker_px)

    # Borda branca de 1 módulo (36h11 tem 8 módulos de lado no quadrado preto)
    module = marker_px // 8
    marker = cv2.copyMakeBorder(
        marker, module, module, module, module,
        cv2.BORDER_CONSTANT, value=255,
    )
    border_m = TAG_SIZE / 8.0

    # Cantos 3D do marcador COM borda, no frame do tag (ordem TL, TR, BR, BL
    # igual à imagem gerada: y para baixo na imagem = -y no frame do tag)
    half = TAG_SIZE / 2.0 + border_m
    obj = np.array(
        [[-half,  half, 0.0],
         [ half,  half, 0.0],
         [ half, -half, 0.0],
         [-half, -half, 0.0]],
        dtype=np.float64,
    )
    img_pts, _ = cv2.projectPoints(obj, rvec, tvec, pinhole_K, None)
    img_pts = img_pts.reshape(4, 2).astype(np.float32)

    src = np.array(
        [[0, 0], [marker.shape[1], 0],
         [marker.shape[1], marker.shape[0]], [0, marker.shape[0]]],
        dtype=np.float32,
    )
    H = cv2.getPerspectiveTransform(src, img_pts)
    canvas = np.full((SIZE[1], SIZE[0]), 128, dtype=np.uint8)
    warped = cv2.warpPerspective(
        marker, H, SIZE, canvas,
        borderMode=cv2.BORDER_TRANSPARENT,
    )
    return warped


def _distort_to_fisheye(
    pinhole_img: np.ndarray, pinhole_K: np.ndarray
) -> np.ndarray:
    """Sintetiza a imagem fisheye a partir da imagem pinhole.

    Para cada pixel da imagem fisheye de saída, encontra a coordenada
    correspondente na imagem pinhole (undistortPoints com P=pinhole_K)
    e faz remap — o inverso exato do que o FisheyeRectifier faz.
    """
    w, h = SIZE
    xs, ys = np.meshgrid(np.arange(w, dtype=np.float32),
                         np.arange(h, dtype=np.float32))
    fisheye_px = np.stack([xs, ys], axis=-1).reshape(-1, 1, 2)

    pinhole_px = cv2.fisheye.undistortPoints(
        fisheye_px, K_FISHEYE, D_FISHEYE.reshape(4, 1),
        R=np.eye(3), P=pinhole_K,
    ).reshape(h, w, 2)

    return cv2.remap(
        pinhole_img,
        pinhole_px[..., 0], pinhole_px[..., 1],
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT, borderValue=128,
    )


@pytest.fixture(scope='module')
def rectifier() -> FisheyeRectifier:
    return FisheyeRectifier(K_FISHEYE, D_FISHEYE, SIZE, zoom=1.0)


def _rvec_flip_and_tilt(tilt_y_deg: float, tilt_x_deg: float = 0.0) -> np.ndarray:
    """rvec do tag de frente pra câmera (flip x) + inclinação decisiva.

    Inclinação >= 15° separa bem os dois ramos do IPPE (ambiguidade de
    pose planar); com o tag quase frontal os ramos colapsam e a
    orientação deixa de ser testável de forma determinística.
    """
    flip, _ = cv2.Rodrigues(np.array([np.pi, 0.0, 0.0]))
    tilt, _ = cv2.Rodrigues(
        np.array([np.radians(tilt_x_deg), np.radians(tilt_y_deg), 0.0])
    )
    rvec, _ = cv2.Rodrigues(flip @ tilt)
    return rvec.flatten()


@pytest.mark.parametrize(
    'tvec_true, rvec_true',
    [
        (np.array([0.00, 0.00, 0.45]), _rvec_flip_and_tilt(15.0)),
        (np.array([0.10, -0.05, 0.55]), _rvec_flip_and_tilt(-20.0)),
        (np.array([-0.08, 0.06, 0.50]), _rvec_flip_and_tilt(25.0, 10.0)),
    ],
    ids=['centro-15deg', 'deslocado-20deg', 'canto-25deg'],
)
def test_pipeline_recupera_pose(
    rectifier: FisheyeRectifier,
    tvec_true: np.ndarray,
    rvec_true: np.ndarray,
) -> None:
    """fisheye sintética → rectify → detect → pose ≈ ground truth."""
    pinhole = _render_tag_pinhole(rectifier.pinhole_K, rvec_true, tvec_true)
    fisheye = _distort_to_fisheye(pinhole, rectifier.pinhole_K)

    rectified = rectifier.rectify(fisheye)
    detector = AprilTagDetector(rectifier.pinhole_K, TAG_SIZE)
    detections = detector.detect(rectified)

    assert detections, 'nenhum tag detectado na imagem retificada'
    det = select_tag(detections, TAG_ID)
    assert det is not None, f'tag {TAG_ID} não encontrado'

    # Posição: erro < 2% da distância
    dist = float(np.linalg.norm(tvec_true))
    pos_err = float(np.linalg.norm(det.t_cam_tag - tvec_true))
    assert pos_err < 0.02 * dist, (
        f'erro de posição {pos_err*1000:.1f} mm a {dist:.2f} m'
    )

    # Orientação: ângulo do erro rotacional < 3°
    R_true, _ = cv2.Rodrigues(rvec_true)
    R_err = det.R_cam_tag.T @ R_true
    angle = float(np.degrees(np.arccos(
        np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    )))
    assert angle < 3.0, f'erro de orientação {angle:.2f}°'

    assert det.reproj_error_px < 1.0
    # Inclinação decisiva → ramo correto do IPPE vence com folga
    assert det.pose_ambiguity < 0.8, (
        f'pose ambígua (razão {det.pose_ambiguity:.2f}) em pose inclinada'
    )


def test_frontal_posicao_boa_mesmo_com_ambiguidade(
    rectifier: FisheyeRectifier,
) -> None:
    """Tag quase frontal: posição confiável; ambiguidade é reportada.

    Documenta o comportamento que o consumidor (servovisão) deve
    conhecer: de frente, a ORIENTAÇÃO pode vir do ramo errado do IPPE,
    mas a POSIÇÃO permanece precisa — filtrar orientação por
    ``pose_ambiguity``.
    """
    tvec_true = np.array([0.0, 0.0, 0.5])
    rvec_true = np.array([np.pi, 0.0, 0.0])

    pinhole = _render_tag_pinhole(rectifier.pinhole_K, rvec_true, tvec_true)
    fisheye = _distort_to_fisheye(pinhole, rectifier.pinhole_K)
    detector = AprilTagDetector(rectifier.pinhole_K, TAG_SIZE)
    det = select_tag(detector.detect(rectifier.rectify(fisheye)), TAG_ID)

    assert det is not None
    pos_err = float(np.linalg.norm(det.t_cam_tag - tvec_true))
    assert pos_err < 0.01, f'erro de posição {pos_err*1000:.1f} mm'
    assert 0.0 <= det.pose_ambiguity <= 1.0


def test_select_tag_prefere_id_alvo(rectifier: FisheyeRectifier) -> None:
    """select_tag com id >= 0 retorna só o alvo; ausente → None."""
    pinhole = _render_tag_pinhole(
        rectifier.pinhole_K,
        np.array([np.pi, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.5]),
    )
    detector = AprilTagDetector(rectifier.pinhole_K, TAG_SIZE)
    detections = detector.detect(pinhole)
    assert detections

    assert select_tag(detections, TAG_ID) is not None
    assert select_tag(detections, TAG_ID + 1) is None
    assert select_tag([], -1) is None


def test_tag_size_invalido(rectifier: FisheyeRectifier) -> None:
    with pytest.raises(ValueError):
        AprilTagDetector(rectifier.pinhole_K, 0.0)
