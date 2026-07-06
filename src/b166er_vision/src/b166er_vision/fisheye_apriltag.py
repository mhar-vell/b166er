"""Pipeline de visão para servovisão: fisheye T265 → pinhole virtual → AprilTag.

As fisheye da T265 (OV9282, 848×800 mono, FOV ~163°) usam o modelo de
distorção Kannala-Brandt ("equidistant" no CameraInfo), que os detectores
de marcadores não entendem. O pipeline:

    1. FisheyeRectifier: retifica a imagem para uma câmera pinhole virtual
       (cv2.fisheye) — os mapas são pré-computados uma única vez.
    2. AprilTagDetector: detecta tags 36h11 (cv2.aruco, sem dependências
       novas) na imagem retificada e estima pose 6-DOF via solvePnP
       (IPPE_SQUARE) com a intrínseca pinhole virtual.

Módulo puro (NumPy + OpenCV): sem ROS, testável com pytest.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class TagDetection:
    """Uma detecção de AprilTag com pose no frame da câmera.

    Convenção: eixo z da câmera aponta para fora da lente; ``t_cam_tag``
    é a posição do centro do tag no frame da câmera (m); ``R_cam_tag``
    leva vetores do frame do tag para o frame da câmera.

    ``pose_ambiguity`` ∈ [0, 1]: razão erro_melhor/erro_segunda das duas
    soluções do IPPE (ambiguidade de pose planar). Próximo de 1 = as duas
    soluções explicam os cantos igualmente bem → a ORIENTAÇÃO não é
    confiável (a posição permanece boa). Consumidores que dependem da
    orientação devem filtrar por este campo.
    """

    tag_id: int
    corners_px: np.ndarray          # (4, 2) cantos na imagem retificada
    t_cam_tag: np.ndarray           # (3,) m
    R_cam_tag: np.ndarray           # (3, 3)
    reproj_error_px: float
    pose_ambiguity: float


class FisheyeRectifier:
    """Retifica imagens fisheye (Kannala-Brandt) para pinhole virtual.

    A pinhole virtual é construída EXPLICITAMENTE a partir do fx do
    fisheye (mesma resolução angular central), não via
    ``estimateNewCameraMatrixForUndistortRectify`` — com FOV muito largo
    (T265 ≈ 163°) essa função degenera e devolve focal minúscula
    (fx 285 → 58), encolhendo os marcadores até ficarem indetectáveis.

    Com fx≈285 e 848 px de largura, a retificada cobre
    2·atan(424/285) ≈ 112° de FOV útil — suficiente para servoing.

    Args:
        K: intrínseca fisheye 3×3 (do CameraInfo.K).
        D: 4 coeficientes Kannala-Brandt (do CameraInfo.D[:4]).
        size: (largura, altura) da imagem de entrada.
        zoom: escala da focal virtual (1.0 = mesma resolução angular
            central do fisheye; >1 aproxima, reduzindo o FOV coberto).
    """

    def __init__(
        self,
        K: np.ndarray,
        D: np.ndarray,
        size: Tuple[int, int],
        zoom: float = 1.0,
    ) -> None:
        if zoom <= 0.0:
            raise ValueError(f'zoom deve ser positivo, veio {zoom}')
        self._size = size
        K = np.asarray(K, dtype=np.float64).reshape(3, 3)
        D = np.asarray(D, dtype=np.float64).reshape(4, 1)

        f_virtual = K[0, 0] * zoom
        self.pinhole_K: np.ndarray = np.array(
            [[f_virtual, 0.0, K[0, 2]],
             [0.0, f_virtual, K[1, 2]],
             [0.0, 0.0, 1.0]]
        )
        self._map1, self._map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), self.pinhole_K, size, cv2.CV_16SC2
        )

    def rectify(self, image: np.ndarray) -> np.ndarray:
        """Imagem fisheye → imagem pinhole virtual (mesma resolução)."""
        return cv2.remap(
            image, self._map1, self._map2, interpolation=cv2.INTER_LINEAR
        )


class AprilTagDetector:
    """Detecta AprilTags 36h11 e estima pose com solvePnP IPPE_SQUARE.

    Args:
        pinhole_K: intrínseca 3×3 da imagem retificada (sem distorção).
        tag_size: lado do quadrado PRETO externo do tag, em metros.
    """

    def __init__(self, pinhole_K: np.ndarray, tag_size: float) -> None:
        if tag_size <= 0.0:
            raise ValueError(f'tag_size deve ser positivo, veio {tag_size}')
        self._K = np.asarray(pinhole_K, dtype=np.float64).reshape(3, 3)
        self._tag_size = tag_size

        dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11
        )
        params = cv2.aruco.DetectorParameters()
        # Refino subpixel dos cantos melhora bastante a pose a distância
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(dictionary, params)

        # Cantos 3D do tag no frame do próprio tag (z=0, ordem do aruco:
        # top-left, top-right, bottom-right, bottom-left)
        half = tag_size / 2.0
        self._obj_pts = np.array(
            [
                [-half,  half, 0.0],
                [ half,  half, 0.0],
                [ half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )

    def detect(self, gray: np.ndarray) -> List[TagDetection]:
        """Detecta todos os tags 36h11 numa imagem retificada (mono8)."""
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None:
            return []

        detections: List[TagDetection] = []
        for tag_corners, tag_id in zip(corners, ids.flatten()):
            img_pts = tag_corners.reshape(4, 2).astype(np.float64)
            # IPPE tem 2 soluções (ambiguidade de pose planar). O solvePnP
            # simples pode devolver o ramo errado; o Generic devolve ambas
            # com os erros — escolhemos a melhor e medimos a ambiguidade.
            n_sol, rvecs, tvecs, errs = cv2.solvePnPGeneric(
                self._obj_pts,
                img_pts,
                self._K,
                None,   # imagem retificada: sem distorção
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if n_sol < 1:
                continue

            errs = np.asarray(errs, dtype=np.float64).flatten()
            best = int(np.argmin(errs))
            if n_sol > 1:
                second = float(np.partition(errs, 1)[1])
                ambiguity = (
                    float(errs[best]) / second if second > 1e-12 else 1.0
                )
            else:
                ambiguity = 0.0

            R, _ = cv2.Rodrigues(rvecs[best])
            detections.append(
                TagDetection(
                    tag_id=int(tag_id),
                    corners_px=img_pts,
                    t_cam_tag=tvecs[best].flatten(),
                    R_cam_tag=R,
                    reproj_error_px=float(errs[best]),
                    pose_ambiguity=ambiguity,
                )
            )
        return detections


def select_tag(
    detections: List[TagDetection], target_id: int
) -> Optional[TagDetection]:
    """Seleciona o tag alvo (target_id < 0 → o de menor erro de reprojeção)."""
    if not detections:
        return None
    if target_id >= 0:
        for det in detections:
            if det.tag_id == target_id:
                return det
        return None
    return min(detections, key=lambda d: d.reproj_error_px)
