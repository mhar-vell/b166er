#!/usr/bin/env python3
"""
chave_mission — missão completa de operação da chave seccionadora,
orquestrada por máquina de estados (SMACH).

Pedida pelo Marco em 2026-08-13: "realizar um movimento completo de
procura da chave através da apriltag e realizar uma aproximação
adequada, simulando o movimento de abertura da chave e o robot
retornando para o ponto de partida".

Sequência
---------
  STOW_INIT  → braço recolhido (config/arm_postures.yaml: stow_home).
               Nada se move com o braço estendido: a postura estendida
               já tombou o robô contra a parede em 2026-08-12.
  SEARCH     → postura "search" (câmera apontando à frente) e giro no
               próprio eixo até o apriltag_localizer publicar a pose da
               parede em /b166er/wall_pose. É o único estado que
               descobre ONDE está a chave — nada aqui consulta o
               simulador.
  APPROACH   → aproximação em etapas: vai até ~1,30 m (dentro da faixa
               confiável da tag), remede parado, e só então navega até o
               standoff final. Braço recolhido, câmera rastreando a tag.
  REFINE     → já parado e perto, remede a parede pela tag e substitui a
               estimativa grosseira do SEARCH. Sem isso o erro angular
               da detecção distante vira dezenas de cm no alvo.
  DEPLOY     → resolve a IK da ponta para o primeiro alvo e
               pré-posiciona o braço nela (não uma postura fixa: o ramo
               errado trava o controlador num batente de junta).
  MANIPULATE → as 4 fases Cartesianas (engage → release → pos1 → pos2)
               via /b166er/ee_target, conduzidas pelo controlador
               whole-body, exatamente como no task_sequencer.
  RETRACT    → braço de volta ao recolhido, antes de qualquer
               deslocamento.
  RETURN     → base volta à pose de partida (registrada no STOW_INIT).
  DONE

Falhas
------
Cada estado tem timeout próprio e sai por 'failed'; a máquina inteira
termina em ABORT_SAFE, que para a base e recolhe o braço — nunca deixa
o robô parado com o braço estendido, que é a condição de tombamento.

Navegação da base
-----------------
Girar-avançar-girar explícito (mesma filosofia da manobra já existente
no fuzzy_wb_controller): a base é skid-steer e não faz correção
lateral. Aqui o controle é de BASE pura (/cmd_vel direto), não
whole-body — durante deslocamento o braço fica recolhido e parado.

Tópicos
-------
  Subscreve: /b166er/robot_state, /b166er/wall_pose,
             /b166er/arm_posture_reached
  Publica:   /cmd_vel, /b166er/ee_target, /b166er/arm_posture_cmd
"""

import math

import numpy as np
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from tf.transformations import (quaternion_matrix, euler_from_quaternion,
                                quaternion_from_matrix)

from gazebo_msgs.srv import SetModelConfiguration
from b166er_whole_body_control.msg import RobotState
from b166er_whole_body_control.kinematics import (
    JOINT_NAMES, pose_error, T_T265_TOOLTIP, T_BASELINK_ARM,
    ik_tooltip_position)
from b166er_whole_body_control import chave_task

# pre_engage primeiro: aproxima por um ponto afastado da parede e só
# então entra reto no olhal — o controlador Cartesiano não desvia de
# obstáculo sozinho (ver nota no YAML).
PHASE_ORDER = ['pre_engage', 'engage', 'release', 'pos1', 'pos2']


def _yaw_of(quat):
    return euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]


def _ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class MissionContext(object):
    """Estado compartilhado + interface ROS, injetado em cada estado SMACH."""

    def __init__(self):
        self.rate_hz       = rospy.get_param('~rate', 20.0)
        self.standoff_dist = rospy.get_param('~standoff_distance', 0.80)
        self.search_omega  = rospy.get_param('~search_omega', 0.35)
        self.search_timeout   = rospy.get_param('~search_timeout', 90.0)
        self.approach_timeout = rospy.get_param('~approach_timeout', 120.0)
        self.posture_timeout  = rospy.get_param('~posture_timeout', 30.0)
        self.phase_timeout    = rospy.get_param('~phase_timeout', 60.0)
        self.refine_samples   = rospy.get_param('~refine_samples', 10)
        # Nome do modelo da fixture no Gazebo — usado para girar a
        # lâmina da chave (set_blade_angle).
        self.fixture_model    = rospy.get_param('~fixture_model_name',
                                                'chave_seccionadora_fixture')
        # Distância intermediária de aproximação: dentro da faixa
        # confiável da tag (132 mm → ~1,45 m pela regra d/11).
        self.coarse_distance  = rospy.get_param('~coarse_distance', 1.30)
        self.deploy_ik_tol    = rospy.get_param('~deploy_ik_tol', 0.02)
        self.refine_timeout   = rospy.get_param('~refine_timeout', 15.0)

        # Navegação (girar-avançar-girar)
        self.nav_v         = rospy.get_param('~nav_linear_vel', 0.15)
        self.nav_w         = rospy.get_param('~nav_angular_vel', 0.4)
        self.nav_pos_tol   = rospy.get_param('~nav_pos_tol', 0.06)
        self.nav_yaw_tol   = rospy.get_param('~nav_yaw_tol', 0.09)
        # Distância mínima (centro da base → obstáculo) pelo laser.
        self.min_clearance = rospy.get_param('~min_clearance', 0.55)

        # Tolerância Cartesiana da manipulação: 20 mm, NÃO os 5 mm que
        # o fuzzy_wb_controller usa por padrão.
        #
        # Razão: não faz sentido exigir do controle uma precisão maior
        # que a da MEDIÇÃO do alvo. A posição do olhal vem da detecção
        # da tag, que mede com ~9 mm de erro (validado contra ground
        # truth em 2026-08-13, três poses). Perseguir 5 mm de erro de
        # EE contra um alvo conhecido a ±9 mm é perseguir ruído — e na
        # prática o sistema estacionava em 12 mm com o ganho Fuzzy já
        # reduzido perto do alvo, gastando o timeout inteiro sem
        # fechar. 20 mm é compatível com o engate: o vão do gancho tem
        # ~20 mm e o anel do olhal 28 mm de raio.
        self.tol_pos    = rospy.get_param('~tol_pos', 0.020)
        self.tol_orient = rospy.get_param('~tol_orient', 0.02)

        self.postures = rospy.get_param('/arm_postures')
        self.phases   = rospy.get_param('/chave_seccionadora_task')

        # Gimbal ("pescoço de galinha", ideia do Marco em 2026-08-13):
        # manter a tag centrada no quadro girando J1 enquanto a base
        # navega. Sem isso a tag sai do campo de visão na aproximação e a
        # estimativa degrada justamente quando mais precisa ser boa.
        self.gimbal_gain    = rospy.get_param('~gimbal_gain', 0.6)
        self.gimbal_deadband = rospy.get_param('~gimbal_deadband', 0.06)
        self.gimbal_period  = rospy.get_param('~gimbal_period', 0.4)   # s
        self.tag_pixel      = None
        self.tag_pixel_t    = None
        self._j1_track      = None   # J1 corrente do rastreamento

        self.robot_state  = None
        self.wall_pose    = None
        self.posture_done = False

        self.start_pose    = None   # (x, y, yaw) registrado no STOW_INIT
        self.wall_pos      = None   # np(3,) estimado por visão
        self.wall_R        = None   # np(3,3)
        self.ee_orientation = None  # orientação do EE fixada na manipulação

        self.pub_cmdvel  = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_target  = rospy.Publisher('/b166er/ee_target', PoseStamped,
                                           queue_size=1, latch=True)
        self.pub_posture = rospy.Publisher('/b166er/arm_posture_cmd', JointState,
                                           queue_size=1, latch=True)
        # Arbitragem de /cmd_vel entre a missão (navegação de base) e o
        # fuzzy_wb_controller (manipulação whole-body). Sem isso os dois
        # publicam intercalado a 20 Hz e a base anda aos trancos.
        self.pub_wb_enable = rospy.Publisher('/b166er/wb_enable', Bool,
                                             queue_size=1, latch=True)
        # Trava de base durante a manipulação: o whole-body não conhece a
        # parede e já dirigiu a base contra ela tentando alcançar o olhal
        # (2026-08-13, robô tombado). Depois do standoff, só o braço serve
        # o alvo.
        self.pub_base_lock = rospy.Publisher('/b166er/base_lock', Bool,
                                             queue_size=1, latch=True)
        # Plano de exclusão da base (substitui a trava total). Ver
        # _apply_keepout no fuzzy_wb_controller.
        self.pub_keepout   = rospy.Publisher('/b166er/base_keepout', PoseStamped,
                                             queue_size=1, latch=True)
        # Alvo publicado como posição da PONTA da ferramenta (não do
        # T265) durante a manipulação — ver fuzzy_wb_controller, 4a.
        self.pub_servo_tip = rospy.Publisher('/b166er/servo_tooltip', Bool,
                                             queue_size=1, latch=True)
        self.pub_markers   = rospy.Publisher('/b166er/mission_markers',
                                             MarkerArray, queue_size=1, latch=True)

        rospy.Subscriber('/b166er/robot_state', RobotState, self._cb_state)
        rospy.Subscriber('/b166er/wall_pose', PoseStamped, self._cb_wall)
        rospy.Subscriber('/b166er/arm_posture_reached', Bool, self._cb_posture)
        rospy.Subscriber('/b166er/tag_pixel', Point, self._cb_tag_pixel)
        # Segurança: tombamento detectado pela IMU aborta a missão.
        # Antes disso o robô tombou várias vezes e a máquina de estados
        # seguia tentando, alheia — a IMU existia e não era usada.
        self.tilt_critical = False
        rospy.Subscriber('/b166er/tilt_critical', Bool, self._cb_tilt)
        # Distância livre medida pelo Hokuyo — usada na navegação para
        # parar antes de encostar, sem depender da estimativa da tag.
        self.front_clearance = None
        rospy.Subscriber('/b166er/front_clearance', Float64,
                         self._cb_clearance)

    # ------------------------------------------------------------------
    def _cb_state(self, msg):
        self.robot_state = msg

    def _cb_wall(self, msg):
        self.wall_pose = msg

    def _cb_posture(self, msg):
        if msg.data:
            self.posture_done = True

    def _cb_clearance(self, msg):
        self.front_clearance = msg.data

    def _cb_tilt(self, msg):
        if msg.data and not self.tilt_critical:
            rospy.logerr('[mission] IMU reportou inclinação crítica — '
                         'abortando a missão')
        self.tilt_critical = msg.data

    def _cb_tag_pixel(self, msg):
        self.tag_pixel   = msg
        self.tag_pixel_t = rospy.Time.now()

    # ------------------------------------------------------------------
    def gimbal_track(self):
        """Ajusta J1 para recentrar a tag no quadro (rastreamento gimbal).

        Chamado periodicamente durante a navegação. Feedback PURO DE
        PIXEL: não depende da estimativa de mundo (que é justamente o
        que está ruim quando o robô está longe), só de onde a tag
        aparece na imagem. Deadband evita ficar caçando ruído.
        """
        if self.tag_pixel is None or self.tag_pixel_t is None:
            return
        if (rospy.Time.now() - self.tag_pixel_t).to_sec() > 1.0:
            return   # detecção velha: tag sumiu, não persegue fantasma
        off_x = self.tag_pixel.x
        if abs(off_x) < self.gimbal_deadband:
            return

        base = list(self.postures['search'])
        if self._j1_track is None:
            self._j1_track = base[0]
        # offset positivo = tag à direita no quadro; o eixo óptico é +X
        # do link e J1 gira em torno de Z, então recentrar pede J1
        # DIMINUINDO. Sinal confirmado ao vivo.
        self._j1_track = float(np.clip(self._j1_track - self.gimbal_gain * off_x,
                                       -2.6, 2.6))
        q = list(base)
        q[0] = self._j1_track
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name     = JOINT_NAMES
        msg.position = q
        self.pub_posture.publish(msg)   # sem esperar: rastreamento é contínuo
        rospy.loginfo_throttle(2.0,
            '[mission] gimbal: tag em x=%+.2f do centro → J1=%.3f rad',
            off_x, self._j1_track)

    def set_blade_angle(self, deg):
        """Gira a lâmina da chave para o ângulo dado (graus).

        A chave só ganhou junta revoluta em 2026-08-13 — antes era um
        bloco rígido e não abria quando o robô puxava, como o Marco
        observou. Aqui o ângulo é IMPOSTO cinematicamente, acompanhando
        o waypoint que a ferramenta está executando, em vez de emergir
        do contato gancho-anel.

        Por que não por contato: puxar um anel com um gancho é um
        problema de contato fino (duas superfícies curvas, engate
        geométrico), que o ODE com colisões primitivas não resolve de
        forma confiável — a ferramenta atravessaria ou escorregaria. O
        acoplamento cinemático dá a demonstração correta do movimento
        de abertura, que é o que a missão precisa mostrar. Simular o
        contato de verdade exigiria malhas de colisão e um solver mais
        caro, e não muda nada no que está sendo validado aqui (a
        navegação, a percepção e o alcance do braço).
        """
        try:
            rospy.wait_for_service('/gazebo/set_model_configuration', timeout=2.0)
            srv = rospy.ServiceProxy('/gazebo/set_model_configuration',
                                     SetModelConfiguration)
            srv(model_name=self.fixture_model,
                urdf_param_name='chave_fixture_description',
                joint_names=['chave_blade_joint'],
                joint_positions=[math.radians(deg)])
            rospy.loginfo('[mission] chave girada para %.0f° (%s)', deg,
                          'aberta' if deg > 25 else
                          'fechada' if deg < 5 else 'parcial')
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn('[mission] não consegui girar a lâmina: %s', e)

    def settle_gimbal(self):
        """Encerra o rastreamento gimbal de forma limpa.

        Reenvia a postura corrente como alvo e espera 'reached', para
        garantir que a ponte de braço não fique com uma rampa pendente
        — enquanto houver, ela ignora /b166er/arm_vel_cmd e o
        controlador Cartesiano não move nada.
        """
        if self._j1_track is None:
            return
        q = list(self.postures['search'])
        q[0] = self._j1_track
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name     = JOINT_NAMES
        msg.position = q
        self.posture_done = False
        self.pub_posture.publish(msg)
        t0 = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.posture_done:
            if (rospy.Time.now() - t0).to_sec() > 5.0:
                rospy.logwarn('[mission] gimbal não assentou em 5s')
                return
            rate.sleep()

    # ------------------------------------------------------------------
    def publish_markers(self, goal=None):
        """Marcadores para o RViz: alvo de navegação, olhal e caminho.

        Antes disso o RViz não mostrava NADA sobre o deslocamento (o
        Marco perguntou por quê): a missão navega com controlador
        próprio escrevendo direto em /cmd_vel, sem move_base e sem
        publicar Path/Marker — não havia o que desenhar.
        """
        arr = MarkerArray()
        now = rospy.Time.now()

        def _mk(mid, mtype, scale, color):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = now
            m.ns = 'chave_mission'
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = color
            m.pose.orientation.w = 1.0
            return m

        if self.wall_pos is not None and self.wall_R is not None:
            olhal = chave_task.olhal_position(self.wall_pos, self.wall_R)
            m = _mk(0, Marker.SPHERE, (0.08, 0.08, 0.08), (1.0, 0.4, 0.0, 0.9))
            m.pose.position.x, m.pose.position.y, m.pose.position.z = olhal
            arr.markers.append(m)

        if goal is not None and self.robot_state is not None:
            gx, gy, _ = goal
            m = _mk(1, Marker.CYLINDER, (0.25, 0.25, 0.02), (0.0, 0.8, 1.0, 0.8))
            m.pose.position.x, m.pose.position.y, m.pose.position.z = gx, gy, 0.01
            arr.markers.append(m)

            x, y, _ = self.base_pose()
            line = _mk(2, Marker.LINE_STRIP, (0.03, 0.0, 0.0), (0.0, 0.8, 1.0, 0.7))
            line.points = [Point(x=x, y=y, z=0.05), Point(x=gx, y=gy, z=0.05)]
            arr.markers.append(line)

        if arr.markers:
            self.pub_markers.publish(arr)

    # ------------------------------------------------------------------
    def wait_for_state(self, timeout=10.0):
        t0 = rospy.Time.now()
        while self.robot_state is None and not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > timeout:
                return False
            rospy.sleep(0.1)
        return self.robot_state is not None

    def base_pose(self):
        """(x, y, yaw) da base, do /b166er/robot_state."""
        p = self.robot_state.base_odom.pose.pose
        return p.position.x, p.position.y, _yaw_of(p.orientation)

    def stop_base(self):
        self.pub_cmdvel.publish(Twist())

    def send_posture(self, name):
        q = self.postures[name]
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name     = JOINT_NAMES
        msg.position = list(q)
        self.posture_done = False
        self.pub_posture.publish(msg)
        rospy.loginfo('[mission] postura "%s" comandada: %s', name, q)

    def drive(self, v, w):
        t = Twist()
        t.linear.x  = float(np.clip(v, -self.nav_v, self.nav_v))
        t.angular.z = float(np.clip(w, -self.nav_w, self.nav_w))
        self.pub_cmdvel.publish(t)

    def update_wall_from(self, pose_msg):
        """Atualiza a estimativa da parede a partir de um PoseStamped.

        Orientação achatada para yaw puro — ver chave_task.flatten_wall_R
        para o porquê (tilt espúrio do PnP vira erro grande na posição
        calculada do olhal).
        """
        p = pose_msg.pose
        self.wall_pos = np.array([p.position.x, p.position.y, p.position.z])
        R = quaternion_matrix([p.orientation.x, p.orientation.y,
                               p.orientation.z, p.orientation.w])[:3, :3]
        self.wall_R = chave_task.flatten_wall_R(R)

    def take_base(self):
        """Missão assume /cmd_vel (navegação); fuzzy_wb_controller cala."""
        self.pub_wb_enable.publish(Bool(data=False))
        rospy.sleep(0.2)   # deixa o stand-down chegar antes de dirigir

    def publish_keepout(self):
        """Publica o plano que a base não pode cruzar: a face da parede.

        Substitui a trava total de base. A trava evitava a colisão mas
        tirava a base da solução — sobravam 5 juntas para a tarefa e o
        DLS saturava J2/J3 nos batentes, congelando o erro em ~10 cm
        (medido em 2026-08-13). Com a restrição, a base volta a
        participar dos 8 DOF e só perde a liberdade de avançar além do
        limite.

        Convenção esperada pelo controlador: position = ponto no plano,
        eixo X da orientação = normal apontando para o LADO SEGURO.
        """
        n = chave_task.wall_front_normal(self.wall_R)   # aponta para o robô
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.wall_pos

        # Monta uma rotação cujo eixo X seja a normal do lado seguro.
        x = n / max(np.linalg.norm(n), 1e-9)
        z = np.array([0.0, 0.0, 1.0])
        y = np.cross(z, x); y /= max(np.linalg.norm(y), 1e-9)
        z = np.cross(x, y)
        T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z
        qx, qy, qz, qw = quaternion_from_matrix(T)
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self.pub_keepout.publish(msg)
        rospy.loginfo('[mission] plano de exclusão publicado: parede em %s, '
                      'normal segura %s', self.wall_pos.round(3), x.round(3))

    def release_base(self):
        """Devolve /cmd_vel ao controlador, com a base LIVRE mas restrita.

        Antes isto travava a base por completo. A trava evitava a
        colisão com a parede mas eliminava a redundância: com 5 juntas
        só, o DLS saturava J2/J3 nos batentes e o erro congelava.
        Agora a base participa dos 8 DOF, contida pelo plano de
        exclusão — que é a resposta certa para whole-body.
        """
        self.stop_base()
        self.publish_keepout()
        self.pub_base_lock.publish(Bool(data=False))
        self.pub_servo_tip.publish(Bool(data=True))
        self.pub_wb_enable.publish(Bool(data=True))
        rospy.sleep(0.3)

    def unlock_base(self):
        """Destrava a base e volta a mirar o T265 (navegação)."""
        self.pub_servo_tip.publish(Bool(data=False))
        self.pub_base_lock.publish(Bool(data=False))
        rospy.sleep(0.2)


# ═══════════════════════════════════════════════════════════════════════
# Estados
# ═══════════════════════════════════════════════════════════════════════

class StowInit(smach.State):
    """Recolhe o braço e registra a pose de partida (para o retorno)."""

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['ok', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx = self.ctx
        if not ctx.wait_for_state():
            rospy.logerr('[mission] sem /b166er/robot_state — stack whole-body está rodando?')
            return 'failed'

        ctx.take_base()
        ctx.start_pose = ctx.base_pose()
        rospy.loginfo('[mission] pose de partida registrada: (%.2f, %.2f, %.2f rad)',
                      *ctx.start_pose)

        ctx.send_posture('stow_home')
        return 'ok' if _wait_posture(ctx) else 'failed'


class Search(smach.State):
    """Gira no próprio eixo até a tag ser detectada."""

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['found', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx = self.ctx
        ctx.send_posture('search')
        if not _wait_posture(ctx):
            return 'failed'

        # Descarta detecções anteriores ao início da busca: a missão
        # precisa achar a tag AGORA, não reaproveitar uma pose estimada
        # antes do braço estar na postura de busca.
        ctx.wall_pose = None

        rospy.loginfo('[mission] SEARCH — girando à procura da tag...')
        rate = rospy.Rate(ctx.rate_hz)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            if ctx.wall_pose is not None:
                ctx.stop_base()
                rospy.loginfo('[mission] tag avistada — parando para medir')
                rospy.sleep(1.0)   # deixa a base assentar antes de amostrar
                if not _sample_wall(ctx, ctx.refine_samples,
                                    ctx.refine_timeout, 'SEARCH'):
                    # Avistou de relance mas não conseguiu medir parada:
                    # segue girando, a tag volta ao quadro.
                    rospy.logwarn('[mission] SEARCH: medição falhou, continuando busca')
                    ctx.wall_pose = None
                    continue
                rospy.loginfo('[mission] parede localizada em (%.3f, %.3f, %.3f)',
                              *ctx.wall_pos)
                return 'found'

            if (rospy.Time.now() - t0).to_sec() > ctx.search_timeout:
                ctx.stop_base()
                rospy.logerr('[mission] SEARCH: tag não encontrada em %.0fs',
                             ctx.search_timeout)
                return 'failed'

            ctx.drive(0.0, ctx.search_omega)
            rate.sleep()
        return 'failed'


class Approach(smach.State):
    """Aproximação em ETAPAS, remedindo a parede entre elas.

    Uma etapa só não funciona: a estimativa do SEARCH vem de ~3 m, e a
    tag de 132 mm só é confiável até ~1,45 m pela regra tag ≥ d/11 (a
    mesma medida em hardware real na Fase 4). Em 2026-08-13 isso deu
    erro de 0,53 m na pose da parede — o standoff calculado caiu a 9 cm
    da parede e o robô foi direto para cima dela.

    Então: aproxima até uma distância intermediária que caia DENTRO da
    faixa confiável, remede parado, e só então calcula o standoff final.
    Cada etapa recalcula o alvo com a melhor estimativa disponível.
    """

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['arrived', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx = self.ctx
        # Etapas de distância ao olhal: a intermediária só entra se o
        # robô ainda estiver mais longe que ela.
        stages = [d for d in (ctx.coarse_distance, ctx.standoff_dist)
                  if d > ctx.standoff_dist] + [ctx.standoff_dist]

        for i, dist in enumerate(stages):
            goal = chave_task.standoff_base_pose(ctx.wall_pos, ctx.wall_R, dist)
            rospy.loginfo('[mission] APPROACH etapa %d/%d — %.2f m do olhal, '
                          'alvo (%.2f, %.2f, %.2f rad)',
                          i + 1, len(stages), dist, *goal)
            ctx.publish_markers(goal)
            if not _navigate_to(ctx, goal, ctx.approach_timeout, 'APPROACH'):
                return 'failed'

            # Remede parado antes da próxima etapa (a última medição fina
            # fica a cargo do REFINE, já na pose de standoff).
            if i < len(stages) - 1:
                rospy.sleep(1.0)
                if not _sample_wall(ctx, ctx.refine_samples,
                                    ctx.refine_timeout, 'APPROACH/remedida'):
                    rospy.logerr('[mission] APPROACH: perdi a tag na etapa '
                                 'intermediária')
                    return 'failed'
        return 'arrived'


class Refine(smach.State):
    """Remede a parede de perto, já parado na pose de standoff.

    A estimativa do SEARCH vem de longe (~2,5 m), onde o erro angular
    chega a 7,5° — e o olhal fica a 0,86 m da origem de wall_link, então
    esse erro vira dezenas de centímetros no alvo de manipulação. Em
    2026-08-13 isso colocou a fase "engage" atrás da parede e derrubou
    o robô. A ~0,65 m o mesmo pipeline mede com erro de ~18 mm.

    Média de várias amostras para reduzir o ruído por frame; a posição
    é média aritmética simples e o yaw é médio circular.
    """

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['ok', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx = self.ctx
        n_wanted = ctx.refine_samples
        rospy.loginfo('[mission] REFINE — remedindo a parede de perto (%d amostras)',
                      n_wanted)

        antes = ctx.wall_pos.copy()
        if not _sample_wall(ctx, n_wanted, ctx.refine_timeout, 'REFINE'):
            rospy.logerr('[mission] REFINE: parede fora do campo de visão na '
                         'pose de standoff — estimativa do SEARCH era ruim?')
            return 'failed'
        rospy.loginfo('[mission] REFINE: deslocou %.3f m da estimativa anterior',
                      float(np.linalg.norm(ctx.wall_pos - antes)))
        rospy.loginfo('[mission] olhal estimado em (%.3f, %.3f, %.3f)',
                      *chave_task.olhal_position(ctx.wall_pos, ctx.wall_R))
        return 'ok'


class Deploy(smach.State):
    """Pré-posiciona o braço na SOLUÇÃO DE IK do primeiro alvo.

    Não usa mais uma postura fixa. O braço tem múltiplos ramos de
    solução e uma postura fixa cai na bacia errada com frequência: em
    2026-08-13 a missão convergia suave até 4,5 cm do olhal e travava
    lá, com J3 cravado em −60° (o batente), enquanto a solução do mesmo
    alvo pedia J3 = +31°. Resolver a IK aqui coloca o braço no ramo
    certo, e o controlador Cartesiano só fecha o resíduo.

    Fallback para a postura 'deploy' do YAML se a IK não achar solução
    (alvo fora de alcance a partir do standoff) — aí o MANIPULATE falha
    com diagnóstico claro em vez de o braço sair tentando às cegas.
    """

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['ok', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx = self.ctx
        # Mira o primeiro waypoint (pré-engate, afastado da parede) —
        # não o olhal: pré-posicionar direto no olhal levaria o braço a
        # atravessar a parede durante a rampa de postura.
        p_tip = chave_task.phase_target_position(
            ctx.wall_pos, ctx.wall_R, ctx.phases[PHASE_ORDER[0]]['offset_xyz_m'])

        x, y, yaw = ctx.base_pose()
        b = ctx.robot_state.base_odom.pose.pose
        T_wb = quaternion_matrix([b.orientation.x, b.orientation.y,
                                  b.orientation.z, b.orientation.w])
        T_wb[:3, 3] = [b.position.x, b.position.y, b.position.z]
        p_local = (np.linalg.inv(T_wb @ T_BASELINK_ARM) @ np.append(p_tip, 1))[:3]

        q_ik, err_ik = ik_tooltip_position(
            p_local, q_current=np.array(ctx.robot_state.q_arm))
        if err_ik < ctx.deploy_ik_tol:
            rospy.loginfo('[mission] DEPLOY: IK da ponta resolvida (resíduo '
                          '%.4f m) — pré-posicionando em %s graus',
                          err_ik, np.degrees(q_ik).round(1))
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name     = JOINT_NAMES
            msg.position = q_ik.tolist()
            ctx.posture_done = False
            ctx.pub_posture.publish(msg)
        else:
            rospy.logwarn('[mission] DEPLOY: IK da ponta não fechou (resíduo '
                          '%.3f m) — alvo provavelmente fora de alcance deste '
                          'standoff. Usando a postura fixa do YAML.', err_ik)
            ctx.send_posture('deploy')

        if not _wait_posture(ctx):
            return 'failed'
        # Handoff: daqui em diante quem dirige o braço é o controlador
        # whole-body (com a base travada), conduzido por /b166er/ee_target.
        ctx.release_base()
        return 'ok'


class Manipulate(smach.State):
    """As 4 fases Cartesianas da abertura, via controlador whole-body."""

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx = self.ctx
        # Orientação do EE fixada agora e mantida em todas as fases —
        # o documento da tarefa especifica direções de força, não uma
        # orientação de ferramenta (ver task_sequencer.py).
        ctx.ee_orientation = ctx.robot_state.ee_pose.pose.orientation
        R_ee = quaternion_matrix([ctx.ee_orientation.x, ctx.ee_orientation.y,
                                  ctx.ee_orientation.z, ctx.ee_orientation.w])[:3, :3]

        for phase in PHASE_ORDER:
            offset = ctx.phases[phase]['offset_xyz_m']
            p_tip  = chave_task.phase_target_position(ctx.wall_pos, ctx.wall_R, offset)

            # ── 1) Movimento em ESPAÇO DE JUNTAS até a solução de IK ──
            # Não peça ao DLS para vencer 18 cm sozinho: ele é local e
            # sistematicamente encalha num batente. Em 2026-08-13 o
            # DEPLOY posicionava J3 em torno de 0°/+58° (solução boa) e
            # o controlador Cartesiano arrastava a junta até -60°, onde
            # travava com 10-14 cm de erro residual. A IK enxerga a
            # postura inteira e respeita os limites; a rampa até ela é
            # lenta e previsível.
            b = ctx.robot_state.base_odom.pose.pose
            T_wb = quaternion_matrix([b.orientation.x, b.orientation.y,
                                      b.orientation.z, b.orientation.w])
            T_wb[:3, 3] = [b.position.x, b.position.y, b.position.z]
            p_local = (np.linalg.inv(T_wb @ T_BASELINK_ARM) @ np.append(p_tip, 1))[:3]
            # Passa a postura ATUAL: mantém os waypoints no mesmo ramo
            # de solução (ver ik_tooltip_position — sem isso a escolha
            # pulava de ramo entre execuções e o resultado não era
            # repetível).
            q_ik, err_ik = ik_tooltip_position(
                p_local, q_current=np.array(ctx.robot_state.q_arm))

            if err_ik < ctx.deploy_ik_tol:
                rospy.loginfo('[mission] fase "%s" — IK resolvida (%.4f m), '
                              'indo por espaço de juntas: %s graus',
                              phase, err_ik, np.degrees(q_ik).round(1))
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.name     = JOINT_NAMES
                msg.position = q_ik.tolist()
                ctx.posture_done = False
                ctx.pub_posture.publish(msg)
                if not _wait_posture(ctx):
                    return 'failed'
            else:
                rospy.logwarn('[mission] fase "%s": IK não fechou (%.3f m) — '
                              'alvo fora de alcance deste standoff', phase, err_ik)
                return 'failed'

            # ── 2) Ajuste fino CARTESIANO, fechando o resíduo ──
            # Aqui o DLS trabalha no que ele faz bem: correção local
            # pequena, medida na ponta pelo sensor, compensando o droop
            # do PID e o erro da própria estimativa de juntas.
            T_target = np.eye(4)
            T_target[:3, :3] = R_ee
            T_target[:3, 3]  = p_tip

            msg = PoseStamped()
            msg.header.frame_id = 'odom'
            msg.header.stamp    = rospy.Time.now()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p_tip
            msg.pose.orientation = ctx.ee_orientation
            ctx.pub_target.publish(msg)
            ctx.publish_markers()
            rospy.loginfo('[mission] fase "%s" — ajuste fino até (%.3f, %.3f, %.3f)',
                          phase, *p_tip)

            if not _wait_ee_convergence(ctx, T_target, phase):
                return 'failed'

            # A chave acompanha a ferramenta: o ângulo da lâmina segue o
            # waypoint alcançado (ver set_blade_angle sobre por que é
            # cinemático e não por contato).
            ang = ctx.phases[phase].get('blade_angle_deg')
            if ang is not None:
                ctx.set_blade_angle(float(ang))
            rospy.sleep(1.0)

        rospy.loginfo('[mission] MANIPULATE concluída — chave aberta')
        return 'done'


class Retract(smach.State):
    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['ok', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx = self.ctx
        # ORDEM IMPORTA: silencia o controlador ANTES de destravar. Ao
        # contrário, existe uma janela em que ele fica com a base livre
        # E ainda perseguindo o último alvo latched — observado ao vivo
        # em 2026-08-13, a base saiu andando ~0,7 m depois do abort.
        ctx.take_base()          # fuzzy cala, missão assume /cmd_vel
        ctx.unlock_base()        # só então destrava
        ctx.stop_base()
        ctx.send_posture('stow_home')
        return 'ok' if _wait_posture(ctx) else 'failed'


class Return(smach.State):
    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['home', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx = self.ctx
        rospy.loginfo('[mission] RETURN — voltando a (%.2f, %.2f, %.2f rad)',
                      *ctx.start_pose)
        ok = _navigate_to(ctx, ctx.start_pose, ctx.approach_timeout, 'RETURN')
        return 'home' if ok else 'failed'


class AbortSafe(smach.State):
    """Estado terminal de falha: para a base e recolhe o braço.

    Nunca deixa o robô parado com o braço estendido — é a condição que
    tombou o robô em 2026-08-12.
    """

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['aborted'])
        self.ctx = ctx

    def execute(self, _):
        rospy.logwarn('[mission] ABORT — parando base e recolhendo braço')
        self.ctx.take_base()     # silencia o controlador PRIMEIRO
        self.ctx.unlock_base()
        self.ctx.stop_base()
        self.ctx.send_posture('stow_home')
        _wait_posture(self.ctx)
        return 'aborted'


# ═══════════════════════════════════════════════════════════════════════
# Helpers compartilhados entre estados
# ═══════════════════════════════════════════════════════════════════════

def _sample_wall(ctx, n_wanted, timeout, tag):
    """Coleta N estimativas da parede com o robô PARADO e devolve a média.

    Amostrar parado importa: em 2026-08-13 o SEARCH aceitava a primeira
    detecção, capturada durante o giro, em ângulo rasante — erro de 46 cm
    na pose da parede, que jogou o standoff quase encostado nela e deixou
    a tag fora do campo de visão para o REFINE. Parado e de frente, o
    mesmo pipeline erra ~2 cm.

    Posição por média aritmética; yaw por média circular; roll/pitch
    descartados (ver chave_task.flatten_wall_R).
    """
    positions, yaws = [], []
    rate = rospy.Rate(10)
    t0 = rospy.Time.now()
    ctx.wall_pose = None
    while len(positions) < n_wanted and not rospy.is_shutdown():
        if (rospy.Time.now() - t0).to_sec() > timeout:
            break
        if ctx.wall_pose is not None:
            p = ctx.wall_pose.pose
            positions.append([p.position.x, p.position.y, p.position.z])
            R = quaternion_matrix([p.orientation.x, p.orientation.y,
                                   p.orientation.z, p.orientation.w])[:3, :3]
            yaws.append(math.atan2(R[1, 0], R[0, 0]))
            ctx.wall_pose = None
        rate.sleep()

    if len(positions) < 3:
        rospy.logerr('[mission] %s: só %d amostras da tag', tag, len(positions))
        return False

    pos_mean = np.mean(np.array(positions), axis=0)
    yaw_mean = math.atan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws)))
    c, s_ = math.cos(yaw_mean), math.sin(yaw_mean)
    ctx.wall_pos = pos_mean
    ctx.wall_R   = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
    rospy.loginfo('[mission] %s: %d amostras — parede (%.3f, %.3f, %.3f) yaw=%.3f',
                  tag, len(positions), *pos_mean, yaw_mean)
    return True


def _wait_posture(ctx):
    t0 = rospy.Time.now()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if ctx.posture_done:
            return True
        if (rospy.Time.now() - t0).to_sec() > ctx.posture_timeout:
            rospy.logerr('[mission] postura não atingida em %.0fs', ctx.posture_timeout)
            return False
        rate.sleep()
    return False


def _navigate_to(ctx, goal, timeout, tag):
    """Girar-avançar-girar até (x, y, yaw). Base pura, braço parado."""
    gx, gy, gyaw = goal
    rate = rospy.Rate(ctx.rate_hz)
    t0 = rospy.Time.now()
    phase = 'TURN_TO'

    t_gimbal = rospy.Time.now()
    while not rospy.is_shutdown():
        if (rospy.Time.now() - t0).to_sec() > timeout:
            ctx.stop_base()
            rospy.logerr('[mission] %s: timeout de %.0fs em %s', tag, timeout, phase)
            return False

        # Mantém a tag centrada enquanto anda, e alimenta o RViz.
        # ATENÇÃO: o gimbal publica comandos de POSTURA, e a ponte de
        # braço (gazebo_arm_bridge) ignora /b166er/arm_vel_cmd enquanto
        # uma rampa de postura está ativa. Se uma rampa do gimbal ficar
        # em voo quando a navegação terminar, ela sequestra o braço e o
        # controlador Cartesiano da manipulação não consegue mover nada
        # — foi o que travou o "engage" em 2026-08-13, com o
        # controlador comandando velocidade e o braço parado.
        if (rospy.Time.now() - t_gimbal).to_sec() > ctx.gimbal_period:
            ctx.gimbal_track()
            ctx.publish_markers(goal)
            t_gimbal = rospy.Time.now()

        x, y, yaw = ctx.base_pose()
        dx, dy = gx - x, gy - y
        dist    = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)

        if phase == 'TURN_TO':
            err = _ang_diff(bearing, yaw)
            if dist < ctx.nav_pos_tol or abs(err) < ctx.nav_yaw_tol:
                phase = 'DRIVE'
                rospy.loginfo('[mission] %s: alinhado, avançando (%.2f m)', tag, dist)
            else:
                ctx.drive(0.0, math.copysign(ctx.nav_w, err))

        elif phase == 'DRIVE':
            # Freio por MEDIDA: o laser vê o obstáculo real à frente.
            # Antes a única proteção era o alvo calculado a partir da
            # tag — se a estimativa errasse, o robô ia para cima da
            # parede (aconteceu em 2026-08-13, standoff calculado a 9 cm
            # da parede por erro de 53 cm na detecção distante).
            if (ctx.front_clearance is not None
                    and ctx.front_clearance < ctx.min_clearance):
                ctx.stop_base()
                rospy.logwarn('[mission] %s: laser reporta obstáculo a %.2f m '
                              '(mín %.2f) — parando avanço', tag,
                              ctx.front_clearance, ctx.min_clearance)
                phase = 'TURN_FINAL'
            elif dist < ctx.nav_pos_tol:
                ctx.stop_base()
                phase = 'TURN_FINAL'
                rospy.loginfo('[mission] %s: chegou, ajustando heading final', tag)
            else:
                # Correção proporcional de rumo enquanto avança.
                err = _ang_diff(bearing, yaw)
                ctx.drive(ctx.nav_v, 1.2 * err)

        else:  # TURN_FINAL
            err = _ang_diff(gyaw, yaw)
            if abs(err) < ctx.nav_yaw_tol:
                ctx.stop_base()
                # Fecha qualquer rampa de gimbal pendente: reenvia a
                # postura ATUAL do rastreamento como alvo e espera ela
                # assentar, para a ponte de braço sair do modo postura
                # antes de a manipulação começar.
                ctx.settle_gimbal()
                rospy.loginfo('[mission] %s: concluído', tag)
                return True
            ctx.drive(0.0, math.copysign(ctx.nav_w, err))

        rate.sleep()
    return False


def _wait_ee_convergence(ctx, T_target, phase):
    rate = rospy.Rate(10)
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        if ctx.tilt_critical:
            rospy.logerr('[mission] fase "%s": abortada por inclinação crítica',
                         phase)
            return False
        st = ctx.robot_state
        p, o = st.ee_pose.pose.position, st.ee_pose.pose.orientation
        T_cur = quaternion_matrix([o.x, o.y, o.z, o.w])
        T_cur[:3, 3] = [p.x, p.y, p.z]
        # Erro medido na PONTA DA FERRAMENTA (o T265 é só o sensor).
        p_tip_cur = T_cur[:3, 3] + T_cur[:3, :3] @ T_T265_TOOLTIP[:3, 3]
        err = pose_error(T_cur, T_target)
        rp = float(np.linalg.norm(T_target[:3, 3] - p_tip_cur))
        ro = float(np.linalg.norm(err[3:]))

        # Só posição: durante a manipulação a base fica travada e o
        # controlador serve apenas posição (5 DOF não fecham pose 6D —
        # ver fuzzy_wb_controller, etapa 4a). Exigir orientação aqui
        # seria esperar por um critério que ninguém está perseguindo.
        if rp < ctx.tol_pos:
            rospy.loginfo('[mission] fase "%s" convergiu (%.4f m; ori %.3f rad, '
                          'não controlada)', phase, rp, ro)
            return True

        if (rospy.Time.now() - t0).to_sec() > ctx.phase_timeout:
            rospy.logerr('[mission] fase "%s": timeout (%.4f m, %.4f rad)',
                         phase, rp, ro)
            return False
        rate.sleep()
    return False


# ═══════════════════════════════════════════════════════════════════════
def main():
    rospy.init_node('chave_mission')
    ctx = MissionContext()

    sm = smach.StateMachine(outcomes=['MISSION_OK', 'MISSION_ABORTED'])
    with sm:
        smach.StateMachine.add('STOW_INIT', StowInit(ctx),
                               transitions={'ok': 'SEARCH', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('SEARCH', Search(ctx),
                               transitions={'found': 'APPROACH', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('APPROACH', Approach(ctx),
                               transitions={'arrived': 'REFINE', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('REFINE', Refine(ctx),
                               transitions={'ok': 'DEPLOY', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('DEPLOY', Deploy(ctx),
                               transitions={'ok': 'MANIPULATE', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('MANIPULATE', Manipulate(ctx),
                               transitions={'done': 'RETRACT', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('RETRACT', Retract(ctx),
                               transitions={'ok': 'RETURN', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('RETURN', Return(ctx),
                               transitions={'home': 'MISSION_OK', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('ABORT_SAFE', AbortSafe(ctx),
                               transitions={'aborted': 'MISSION_ABORTED'})

    sis = smach_ros.IntrospectionServer('chave_mission', sm, '/CHAVE_MISSION')
    sis.start()
    outcome = sm.execute()
    rospy.loginfo('[mission] resultado: %s', outcome)
    sis.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
