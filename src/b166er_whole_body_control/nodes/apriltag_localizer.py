#!/usr/bin/env python3
"""
apriltag_localizer — detecta a AprilTag 36h11 (id configurável, default
0 — mesma tag física de 132mm já validada em hardware real, ver memória
project_fase4_apriltag_validacao) na imagem de task_camera_link e
publica a pose estimada de wall_link no mundo.

Substitui a "trapaça" de ground-truth que task_sequencer.py usava
antes (/gazebo/get_link_state) — pedido explícito do Marco (2026-08-12):
"a tag deve ser utilizada para aproximação do robot até a chave" /
"porque vc não faz o cálculo com a ajuda da tag?". Com este nó, a
localização da chave vem de um pipeline de percepção de verdade
(detecção de imagem + PnP + composição com a pose do T265), não de uma
consulta direta ao simulador.

Pipeline
--------
  1. Detecta a tag na imagem (cv2.aruco, dicionário DICT_APRILTAG_36H11
     — suportado nativamente pelo OpenCV ≥4.7, sem depender de
     apriltag_ros/pacote apriltag externo, nenhum dos dois instalados
     neste ambiente).
  2. solvePnP com os 4 cantos detectados + tag_size conhecido (132mm,
     o QUADRADO PRETO — mesma medida da tag física) + intrínsecos de
     /task_camera/camera_info → pose da tag no frame ÓPTICO da câmera
     (T_optical_tag).
  3. T_world_camera = T_world_t265 (de /b166er/robot_state.ee_pose) @
     T_T265_TASKCAMERA (offset fixo conhecido, kinematics.py — mesma
     lógica de T_T265_TOOLTIP, câmera e T265 pendurados rigidamente no
     mesmo CameraSupport).
  4. T_world_tag = T_world_camera @ R_LINK_OPTICAL @ T_optical_tag
     (a rotação no meio converte do frame do link, X-forward, para o
     frame óptico do OpenCV, Z-forward).
  5. wall_link = tag_plate menos o offset fixo conhecido do fixture
     (TAG_OFFSET_FROM_WALL_LINK — mesmos tag_x_offset/tag_mount_z de
     chave_com_tag.urdf.xacro).

VALIDADO COM DETECÇÃO AO VIVO em 2026-08-13, depois de três correções
que só a execução real revelou (a versão de 2026-08-12 estava
documentada como não-validada, e de fato tinha os três problemas):

1. FRAME ÓPTICO ≠ FRAME DO LINK: o sensor de câmera do Gazebo classic
   olha ao longo do +X do link, mas o solvePnP devolve a pose no frame
   óptico OpenCV (Z-forward, X-right, Y-down). Faltava a rotação
   padrão link→óptico (R_LINK_OPTICAL abaixo) na composição — sem ela,
   a pose da tag saía num lugar completamente errado do mundo.
2. ESCALA DA TAG: a textura tem quiet zone branca — o quadrado preto
   (o que o detector mede) ocupa 80% da placa. Com a placa antiga de
   132mm, o preto renderizava 105,6mm e o PnP superestimava a
   distância em exatamente 25% (medido: fator 1,25 no raio
   câmera→tag). Corrigido no fixture (placa de 165mm, preto = 132mm,
   igual à tag física validada) — o ~tag_size daqui segue 0.132 e vale
   para simulação E hardware.
3. ORIENTAÇÃO DA TEXTURA NA PLACA (_R_PNP_TO_FIXTURE): medida
   empiricamente comparando R_world_tag do PnP com a orientação
   conhecida do fixture (yaw=π): X_pnp = −X_fixture (textura
   "espelhada" em relação ao chute original), Y_pnp = +Z_fixture (up),
   Z_pnp = +Y_fixture (normal saindo da parede). O chute de
   2026-08-12 tinha X e Z com sinais errados.

Validação final (2026-08-13): wall_link estimado por visão vs
/gazebo/get_link_state — ver números no commit correspondente.

Tópicos
-------
  Subscreve:
    /task_camera/image_raw   (sensor_msgs/Image)
    /task_camera/camera_info (sensor_msgs/CameraInfo)
    /b166er/robot_state      (RobotState) — pose do T265 no mundo
  Publica:
    /b166er/wall_pose        (PoseStamped) — wall_link estimado, no
                              frame mundial ("odom") — última detecção
                              válida, republicada a cada frame com tag
                              visível.
"""

import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from tf.transformations import quaternion_matrix, quaternion_from_matrix

from b166er_whole_body_control.msg import RobotState
from b166er_whole_body_control.kinematics import (T_T265_FISHEYE1,
                                                  T_T265_TASKCAMERA)
from b166er_whole_body_control.chave_task import TAG_OFFSET_FROM_WALL_LINK

TAG_SIZE = 0.132  # m — quadrado preto.
#
# VOLTOU DE 220 PARA 132 mm em 2026-08-27. Os 220 mm existiam para a
# tag ser legível no SEARCH a ~3 m, mas a medição da bancada
# (olhal-dimension.png) põe a tag a 17 cm do olhal, e nessa separação a
# placa de 220 mm (27,5 cm de largura) invade a placa da chave. 132 mm é
# o que cabe — e é a tag que o Marco tem impressa e caracterizou contra
# a T265 real.
#
# O preço está medido e é real: com 132 mm a T265 detecta até ~1,8 m e
# só MEDE bem até ~1,2 m. O SEARCH da missão acontece a 2,8 m, onde esta
# tag não aparece. Ou a busca desce, ou a bancada afasta a tag do olhal
# para caber a de 220 mm — decisão do Marco, não deste arquivo.

# Offset de wall_link até a tag: IMPORTADO de chave_task, NÃO copiado.
#
# Aqui havia uma cópia local com 0.20 fixo. Quando a tag foi movida para
# o outro lado da chave (-0.60, 2026-08-13), chave_task foi atualizado e
# esta cópia não — o localizador passou a estimar a parede 0,795 m fora
# do lugar. Pior: a missão rodou inteira e terminou em MISSION_OK,
# porque tudo depois disso é relativo à estimativa errada; o robô
# "abriu a chave" a 80 cm de onde ela está. O Marco pegou o risco antes
# do teste: "ao mudar o lado da tag, tem q alterar as direções no
# código também".
#
# Duplicar constante de geometria é exatamente o que chave_task existe
# para evitar. Importar fecha a porta para esse erro se repetir.

# Rotação padrão link-da-câmera (X-forward, convenção Gazebo/ROS para
# corpos) → frame óptico (Z-forward, X-right, Y-down, convenção
# OpenCV/solvePnP). Colunas = eixos ópticos expressos no frame do link:
# x_opt = -Y_link, y_opt = -Z_link, z_opt = +X_link.
R_LINK_OPTICAL = np.array([
    [0.0,  0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])

# Permutação frame-da-tag-no-PnP → frame do fixture, MEDIDA
# empiricamente em 2026-08-13 (ver item 3 do histórico de validação na
# docstring). Colunas = eixos do PnP expressos no frame do fixture:
# X_pnp = -X_fixture, Y_pnp = +Z_fixture, Z_pnp = +Y_fixture.
_R_PNP_TO_FIXTURE = np.array([
    [-1.0, 0.0, 0.0],
    [0.0,  0.0, 1.0],
    [0.0,  1.0, 0.0],
])


def _to_imgmsg(bgr):
    """BGR (ndarray) → sensor_msgs/Image, sem passar pelo cv_bridge.

    O cv_bridge desta instalação (ROS Noetic sobre Python 3.12) ainda
    chama ndarray.tostring(), removido no NumPy 2.0 — cv2_to_imgmsg
    quebra com AttributeError. A conversão é trivial o bastante para
    fazer à mão e evita prender o nó a essa incompatibilidade.
    (imgmsg_to_cv2, o caminho inverso, funciona normalmente.)
    """
    msg = Image()
    msg.height, msg.width = bgr.shape[0], bgr.shape[1]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = 3 * bgr.shape[1]
    msg.data = bgr.tobytes()
    return msg


def _draw_text(img, txt, y, color):
    cv2.putText(img, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)


def _quat_to_matrix(o):
    return quaternion_matrix([o.x, o.y, o.z, o.w])


class AprilTagLocalizer:

    def __init__(self):
        rospy.init_node('apriltag_localizer')

        self._tag_id   = rospy.get_param('~tag_id', 0)
        self._tag_size = rospy.get_param('~tag_size', TAG_SIZE)
        self._world_frame = rospy.get_param('~world_frame', 'odom')

        # ── CÂMERA CONFIGURÁVEL (2026-08-25, preparando a bancada) ──
        #
        # Os tópicos eram fixos em /task_camera/*, que só existe na
        # simulação: é uma câmera que eu acrescentei ao URDF e que o robô
        # real NÃO tem. No laboratório a única câmera disponível é a
        # T265, cujas duas fisheye o realsense2_camera publica.
        #
        # A troca não é só de tópico. A T265 usa modelo equidistante
        # (Kannala-Brandt), e o solvePnP com K + coeficientes pinhole
        # devolve lixo nesse modelo — daí o parâmetro ~fisheye, que
        # desprojeta os cantos com cv2.fisheye antes do PnP.
        self._img_topic  = rospy.get_param('~image_topic',
                                           '/task_camera/image_raw')
        self._info_topic = rospy.get_param('~camera_info_topic',
                                           '/task_camera/camera_info')
        # Extrínseca T265 -> câmera. Na simulação é a da câmera de
        # tarefa; com a fisheye da própria T265 vem do TF do driver.
        self._fisheye = rospy.get_param('~fisheye', False)
        # Quociente mínimo entre o erro de reprojeção da solução
        # descartada e o da escolhida. Abaixo disso as duas são
        # indistinguíveis e a medida é descartada — devolver uma pose
        # ambígua com cara de boa é o modo de falha que queremos evitar.
        self._pnp_razao_min = rospy.get_param('~pnp_razao_minima', 2.0)
        self._pnp_razao = float('inf')
        # Default: a fisheye da T265, que é a câmera do robô real. A
        # câmera de tarefa (T_T265_TASKCAMERA) só existe na simulação e
        # fica disponível por parâmetro para comparação.
        self._T_t265_cam = np.array(
            rospy.get_param('~T_t265_camera', T_T265_FISHEYE1.tolist()),
            dtype=float)
        if self._T_t265_cam.shape != (4, 4):
            rospy.logwarn('[apriltag] ~T_t265_camera não é 4x4 — usando a '
                          'transformada da câmera de tarefa')
            self._T_t265_cam = T_T265_FISHEYE1.copy()

        self._bridge = CvBridge()
        self._K = None
        self._dist = None
        self._t265_pose = None

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
        aruco_params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        half = self._tag_size / 2.0
        self._obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)

        self._pub_wall_pose = rospy.Publisher('/b166er/wall_pose', PoseStamped,
                                              queue_size=1)
        # Offset da tag no QUADRO, normalizado em [-1, 1] (x: horizontal,
        # y: vertical; z = lado do quadrado em px, proxy de distância).
        # Serve ao rastreamento tipo gimbal ("pescoço de galinha") que
        # mantém a tag centrada enquanto a base navega — sem isso a tag
        # sai de quadro na aproximação e a estimativa degrada justamente
        # quando mais importa.
        self._pub_tag_pixel = rospy.Publisher('/b166er/tag_pixel', Point,
                                              queue_size=1)
        # Imagem ANOTADA com o resultado da detecção. Existe porque não
        # havia como ver, ao vivo, se a câmera estava achando a tag —
        # durante a depuração isso só era visível gerando prints sob
        # demanda (Marco: "não consigo visualizar se a câmera está
        # realmente encontrando a tag"). Publica sempre, com ou sem
        # detecção, para o silêncio também ser informativo.
        #   rosrun image_view image_view image:=/b166er/tag_debug_image
        self._pub_debug = rospy.Publisher('/b166er/tag_debug_image', Image,
                                          queue_size=1)

        rospy.Subscriber(self._info_topic, CameraInfo, self._cb_camera_info)
        rospy.Subscriber('/b166er/robot_state', RobotState, self._cb_state)
        rospy.Subscriber(self._img_topic, Image, self._cb_image, queue_size=1)
        rospy.loginfo('[apriltag] câmera: %s (fisheye=%s), tag %d de %.3f m',
                      self._img_topic, self._fisheye, self._tag_id, self._tag_size)

        rospy.loginfo('[apriltag_localizer] pronto — procurando tag id=%d (%.0fmm)',
                      self._tag_id, self._tag_size * 1000)

    # ------------------------------------------------------------------
    def _publish_debug(self, frame, corners, ids, pnp):
        """Publica a imagem com o resultado da detecção desenhado.

        Mostra também a regra prática de alcance da tag (lado ≥ d/11,
        validada em hardware na Fase 4): a distância estimada aparece em
        verde quando está dentro da faixa confiável e em vermelho fora
        dela — foi justamente detecção fora de faixa, a ~3 m, que fez a
        primeira versão da missão calcular o standoff errado e ir para
        cima da parede.
        """
        # SEM guarda de assinantes. Havia aqui um
        # `if get_num_connections() == 0: return` para poupar CPU, mas o
        # efeito prático era o tópico parecer morto justamente quando
        # alguém ia olhar: image_view abre, o publisher ainda não
        # registrou a conexão, nada chega, e a impressão é de que a
        # câmera não funciona (o Marco relatou isso mais de uma vez).
        # Publicar sempre custa uma conversão de imagem a 15 Hz e
        # garante que o tópico esteja vivo quando precisar.
        out = frame.copy()
        if corners is not None and ids is not None:
            cv2.aruco.drawDetectedMarkers(out, corners, ids)

        if pnp is None:
            if ids is None:
                _draw_text(out, 'NENHUMA TAG DETECTADA', 24, (0, 0, 255))
            else:
                _draw_text(out, 'tag(s) %s — id %d NAO encontrada'
                           % (list(ids), self._tag_id), 24, (0, 165, 255))
        else:
            rvec, tvec, side_px = pnp
            d = float(np.linalg.norm(tvec))
            d_max = self._tag_size * 11.0
            ok = d <= d_max
            cv2.drawFrameAxes(out, self._K, self._dist, rvec, tvec,
                              self._tag_size * 0.7)
            _draw_text(out, 'tag id=%d  lado=%.0f px  dist=%.2f m'
                       % (self._tag_id, side_px, d), 24, (0, 255, 0))
            _draw_text(out, 'faixa confiavel (d/11): ate %.2f m  ->  %s'
                       % (d_max, 'OK' if ok else 'FORA DE FAIXA'), 48,
                       (0, 255, 0) if ok else (0, 0, 255))
        self._pub_debug.publish(_to_imgmsg(out))

    def _cb_camera_info(self, msg):
        self._K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self._dist = np.array(msg.D, dtype=np.float64) if msg.D else np.zeros(5)

    def _cb_state(self, msg):
        self._t265_pose = msg.ee_pose.pose

    def _resolve_pnp(self, img_points):
        """PnP respeitando o modelo de lente.

        A lente pinhole e a equidistante divergem MUITO fora do centro do
        quadro, que é justamente onde a tag aparece quando o robô se
        aproxima. Rodar solvePnP com K pinhole sobre pontos de uma
        fisheye não é uma aproximação ruim: é uma pose errada com cara de
        boa — exatamente o tipo de falha silenciosa que já custou caro
        neste projeto.

        Com ~fisheye, os cantos são desprojetados para raios normalizados
        pelo modelo certo e o PnP roda contra uma câmera ideal
        (K = identidade, distorção nula), o que é equivalente e evita
        desdistorcer a imagem inteira a cada quadro.
        """
        if self._fisheye:
            # Desprojeta pelo modelo equidistante e resolve contra uma
            # câmera ideal — equivalente a desdistorcer a imagem inteira,
            # e muito mais barato.
            d = self._dist.flatten()
            d4 = np.zeros((4, 1))
            d4[:min(4, len(d)), 0] = d[:4]
            pts = img_points.reshape(-1, 1, 2).astype(np.float64)
            pontos = cv2.fisheye.undistortPoints(pts, self._K,
                                                 d4).astype(np.float32)
            K, D = np.eye(3), np.zeros(5)
        else:
            pontos, K, D = img_points, self._K, self._dist

        # IPPE_SQUARE COM DESEMPATE POR ERRO DE REPROJEÇÃO (2026-08-27).
        #
        # O solvePnP comum devolve UMA solução, e para um alvo PLANAR
        # existem duas quase igualmente boas — a tag e sua imagem
        # espelhada em relação ao plano da câmera. Vista de frente e com
        # muitos pixels, a escolha é óbvia; vista de esquina e com poucos
        # pixels, o solver escolhe errado com frequência.
        #
        # Medido na simulação com a fisheye, quatro poses do robô:
        # o erro de orientação da parede foi 9,3° / 21,0° / 24,7° e
        # 155,4° — o último é o flip clássico, e os demais crescem com o
        # ângulo de visada. Todos com posição errada por centenas de
        # milímetros. Na bancada isso não apareceu porque lá medimos
        # DISTÂNCIA (|tvec|), que é imune à ambiguidade.
        #
        # IPPE_SQUARE é específico para alvos planares quadrados e
        # devolve AS DUAS soluções com seus erros de reprojeção. Escolher
        # a de menor erro resolve a ambiguidade quando ela é
        # distinguível, e o quociente entre os dois erros diz quando NÃO
        # é — caso em que é melhor descartar a medida do que arriscar.
        ok, rvecs, tvecs, erros = cv2.solvePnPGeneric(
            self._obj_points, pontos, K, D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or not len(rvecs):
            return False, None, None

        e = np.asarray(erros).flatten()
        i = int(np.argmin(e))
        self._pnp_razao = float(e[1 - i] / e[i]) if len(e) > 1 and e[i] > 1e-9 else float('inf')
        if self._pnp_razao < self._pnp_razao_min:
            # As duas soluções são igualmente plausíveis: a pose é
            # ambígua e qualquer escolha é chute. Descartar.
            rospy.logwarn_throttle(
                2.0, '[apriltag] pose ambígua (erros de reprojeção %.3f vs '
                     '%.3f, razão %.2f) — descartando', e[i], e[1 - i],
                self._pnp_razao)
            return False, None, None
        return True, rvecs[i], tvecs[i]

    def _cb_image(self, msg):
        if self._K is None or self._t265_pose is None:
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        if ids is None:
            self._publish_debug(frame, None, None, None)
            return
        ids = ids.flatten()
        if self._tag_id not in ids:
            self._publish_debug(frame, corners, ids, None)
            return

        idx = int(np.where(ids == self._tag_id)[0][0])
        img_points = corners[idx].reshape(4, 2).astype(np.float32)

        # Offset no quadro, para o rastreamento gimbal.
        h, w = gray.shape[:2]
        cx, cy = img_points.mean(axis=0)
        side_px = float(np.mean([np.linalg.norm(img_points[i] - img_points[(i + 1) % 4])
                                 for i in range(4)]))
        self._pub_tag_pixel.publish(Point(
            x=float((cx - w / 2.0) / (w / 2.0)),
            y=float((cy - h / 2.0) / (h / 2.0)),
            z=side_px))

        ok, rvec, tvec = self._resolve_pnp(img_points)
        if not ok:
            return

        # solvePnP devolve a pose no frame ÓPTICO (Z-forward); a cadeia
        # cinemática (T_T265_TASKCAMERA) termina no frame do LINK
        # (X-forward, convenção Gazebo). R_LINK_OPTICAL faz a ponte —
        # sem ela a tag "aparece" num lugar completamente errado do
        # mundo (bug nº 1 da validação de 2026-08-13, ver docstring).
        R_opt_tag, _ = cv2.Rodrigues(rvec)
        T_optical_tag = np.eye(4)
        T_optical_tag[:3, :3] = R_opt_tag
        T_optical_tag[:3, 3]  = tvec.flatten()

        T_link_optical = np.eye(4)
        T_link_optical[:3, :3] = R_LINK_OPTICAL

        T_world_t265   = _quat_to_matrix(self._t265_pose.orientation)
        T_world_t265[:3, 3] = [self._t265_pose.position.x,
                               self._t265_pose.position.y,
                               self._t265_pose.position.z]

        T_world_camera = T_world_t265 @ self._T_t265_cam
        T_world_tag    = T_world_camera @ T_link_optical @ T_optical_tag

        # Reorienta do frame PnP (X-direita,Y-cima,Z-saindo-da-tag) para
        # o frame do fixture (X,Y,Z conforme chave_com_tag.urdf.xacro).
        R_world_tagplate = T_world_tag[:3, :3] @ _R_PNP_TO_FIXTURE
        p_world_tagplate = T_world_tag[:3, 3]

        p_world_wall = p_world_tagplate - R_world_tagplate @ TAG_OFFSET_FROM_WALL_LINK

        out = PoseStamped()
        out.header.frame_id = self._world_frame
        out.header.stamp    = msg.header.stamp
        out.pose.position.x, out.pose.position.y, out.pose.position.z = p_world_wall
        T_out = np.eye(4)
        T_out[:3, :3] = R_world_tagplate
        qx, qy, qz, qw = quaternion_from_matrix(T_out)
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        self._pub_wall_pose.publish(out)

        self._publish_debug(frame, corners, ids, (rvec, tvec, side_px))

        rospy.loginfo_throttle(2.0,
            '[apriltag_localizer] tag id=%d detectada — wall_link estimado em '
            '(%.3f, %.3f, %.3f)', self._tag_id, *p_world_wall)


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        AprilTagLocalizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
