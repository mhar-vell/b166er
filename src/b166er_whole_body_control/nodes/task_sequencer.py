#!/usr/bin/env python3
"""
task_sequencer — sequenciador de waypoints Cartesianos para a tarefa de
abertura da chave seccionadora (ver docs/chave_seccionadora_task.md e
config/chave_seccionadora_task.yaml).

Carrega os offsets de config/chave_seccionadora_task.yaml e publica os
alvos, em sequência, em /b166er/ee_target — avançando de fase quando o
controlador ativo (fuzzy_wb_controller.py, ou whole_body_planner.py em
testes mais simples — qualquer um que consuma /b166er/ee_target e
publique /b166er/robot_state) chega perto o bastante do alvo atual,
usando o mesmo critério de convergência dos dois controladores
(pose_error + tolerâncias tol_pos/tol_orient).

Fases (ver PHASE_ORDER): engage → release → pos1 → pos2.

Pose do olhal: vem de PERCEPÇÃO, não mais de ground-truth do Gazebo.
Versão anterior (até 2026-08-12) consultava /gazebo/get_link_state
direto — uma trapaça que nenhum robô real teria, e que também escondia
o problema real: sem noção nenhuma do ambiente, o whole-body controller
só perseguia um ponto cego e colidia com a parede (Marco pegou isso:
"parece que o robo não está fazendo uma leitura do ambiente antes de
se locomover"). Agora este nó assina /b166er/wall_pose, publicado por
apriltag_localizer.py a partir da detecção real da tag na câmera de
tarefa (ver esse nó para o pipeline completo: detecção → PnP →
composição com a pose do T265). ~use_ground_truth:=true bypassa isso e
volta a consultar /gazebo/get_link_state — só para debug/calibração
(comparar a estimativa por visão com a verdade do simulador), nunca
para uso normal.

Chave_olhal_link não existe como link consultável no Gazebo de todo
jeito (mesmo por get_link_state) — a fixture inteira (bracket→blade→
olhal, só juntas fixas) é fundida pelo Gazebo num único corpo rígido na
conversão URDF→SDF, só "wall_link" sobra. Por isso tanto a via de
percepção (que já publica a pose de wall_link, calculada a partir da
tag) quanto o fallback de debug aplicam por cima o mesmo offset FIXO
conhecido até o olhal (OLHAL_OFFSET_FROM_WALL_LINK, replicado da
geometria de chave_seccionadora_lf.urdf.xacro + chave_com_tag.urdf.xacro
— mais uma cópia dessa fórmula, inevitável). Os offsets do YAML
(pos1/pos2/release) estão no mesmo frame local do fixture (X horizontal
ao longo da parede, Z vertical); giramos tudo pela orientação de
wall_link (por percepção ou por get_link_state).

Orientação do alvo: o documento da tarefa descreve só DIREÇÕES DE
FORÇA (perpendicular/paralela ao eixo X), não uma orientação de
ferramenta para o engate. Este nó mantém fixa a orientação do EE
capturada no início da tarefa (primeiro /b166er/robot_state recebido)
em todas as fases — só a posição varia. Se o tooling do end-effector
precisar de uma orientação de engate específica, isso ainda precisa
ser adicionado (não está no escopo do documento original).

Ferramenta (vara + gancho, ver movemaster.urdf.xacro): os alvos das
fases (engage/release/pos1/pos2) são pensados para a PONTA DA
FERRAMENTA (tool_tip) alcançar o olhal — mas /b166er/ee_target e
/b166er/robot_state.ee_pose continuam sendo o T265 (único sensor de
pose do EE, arquitetura sem encoders; a ferramenta não é observada
diretamente). Como a orientação do EE fica fixa durante toda a tarefa
(ver acima), a conversão é só de POSIÇÃO — kinematics.
tooltip_position_to_t265_position (mantendo a orientação do T265
capturada) — e não kinematics.tooltip_target_to_t265_target (essa
outra é para quando a orientação DESEJADA é a da ponta da ferramenta,
não a do T265; não é o caso aqui). Convergência é checada no alvo do
T265 convertido, comparando com ee_pose (que é T265, não a
ferramenta).

Tópicos
-------
  Subscreve:
    /b166er/robot_state    (RobotState)   — para monitorar convergência
    /b166er/wall_pose      (PoseStamped)  — pose de wall_link por percepção
                             (apriltag_localizer.py); só se ~use_ground_truth=false
  Publica:
    /b166er/ee_target      (PoseStamped)  — alvo Cartesiano da fase atual
  Serviços:
    /gazebo/get_link_state — só com ~use_ground_truth:=true (debug/calibração)
"""

import math

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetLinkState
from tf.transformations import quaternion_matrix

from b166er_whole_body_control.msg import RobotState
from b166er_whole_body_control.kinematics import pose_error, tooltip_position_to_t265_position


PHASE_ORDER = ['engage', 'release', 'pos1', 'pos2']

# ── Geometria replicada de urdf/fixtures/chave_seccionadora_lf.urdf.xacro
#    e chave_com_tag.urdf.xacro (terceira cópia da fórmula do pivô — ver
#    docstring do módulo para o motivo). Se os parâmetros dimensionais da
#    chave mudarem lá, atualizar aqui também. ──
_CHAVE_X_OFFSET     = -0.20    # chave_x_offset, chave_com_tag.urdf.xacro
_OLHAL_HEIGHT       = 0.810    # olhal_height, spec do Marco
_BLADE_LENGTH       = 0.200    # blade_length
_BLADE_ANGLE_CLOSED = 0.349    # blade_angle, 20° em rad, posição fechada
_WALL_STANDOFF      = 0.03     # wall_standoff

_PIVOT_X = _BLADE_LENGTH * math.sin(_BLADE_ANGLE_CLOSED)

# Offset fixo de wall_link (origem = pose de spawn do fixture) até
# chave_olhal_link, no frame local do fixture (X horizontal ao longo
# da parede, Y profundidade/normal da parede, Z vertical). O Z é
# exatamente _OLHAL_HEIGHT porque wall_link nasce no chão (spawn z=0
# por padrão) e toda a cadeia bracket→blade→olhal foi construída, de
# propósito, para que o olhal caia exatamente nessa altura (ver nota
# em chave_seccionadora_lf.urdf.xacro) — os termos de bracket_height/
# pivot_z se cancelam por construção, não precisam ser repetidos aqui.
OLHAL_OFFSET_FROM_WALL_LINK = np.array([
    _CHAVE_X_OFFSET - _PIVOT_X,
    _WALL_STANDOFF,
    _OLHAL_HEIGHT,
])


def _quat_to_matrix(o):
    return quaternion_matrix([o.x, o.y, o.z, o.w])


class TaskSequencer:

    def __init__(self):
        rospy.init_node('task_sequencer')

        task_params = rospy.get_param('/chave_seccionadora_task')
        self._phases = [(name, task_params[name]) for name in PHASE_ORDER]

        # Link REALMENTE consultável no Gazebo (ver docstring do módulo —
        # a fixture inteira é fundida em wall_link na conversão URDF→SDF).
        # ~target_link permite sobrescrever para outra fixture/link no
        # futuro; não é mais o target_frame do YAML (esse continua
        # documentando o frame ao qual os offsets do YAML se referem,
        # não o link a consultar).
        self._link_name   = rospy.get_param('~target_link', 'wall_link')
        self._model_name  = rospy.get_param('~fixture_model_name',
                                             'chave_seccionadora_fixture')
        # false (default) = pose de wall_link vem da percepção
        # (apriltag_localizer.py); true = volta pro ground-truth do
        # Gazebo — só para debug/calibração, ver docstring do módulo.
        self._use_ground_truth = rospy.get_param('~use_ground_truth', False)
        self._wall_pose_timeout = rospy.get_param('~wall_pose_timeout', 30.0)  # s

        # DOIS frames diferentes, não confundir (bug encontrado ao vivo
        # em testes no Gazebo, 2026-08-12): /gazebo/get_link_state só
        # entende "world" como reference_frame (Gazebo não conhece o
        # nome "odom") — já o resto do stack (fuzzy_wb_controller,
        # state_estimator, /b166er/robot_state) usa "odom" como
        # frame_id. Numericamente são o mesmo frame aqui (gazebo_sensor_sim
        # publica /pioneer/pose direto do ground truth do Gazebo, sem
        # nenhuma transformação — ver gazebo_sensor_sim.py), mas os
        # NOMES não são intercambiáveis nas chamadas de serviço/mensagem.
        self._gazebo_world_frame = 'world'
        self._world_frame = rospy.get_param('~world_frame', 'odom')
        self._dwell_time  = rospy.get_param('~dwell_time', 1.0)      # s parado em cada waypoint
        self._phase_timeout = rospy.get_param('~phase_timeout', 30.0)  # s — só gera aviso, não aborta

        # Mesmas tolerâncias default do fuzzy_wb_controller/whole_body_planner
        self._tol_pos    = rospy.get_param('~tol_pos',    0.005)  # m
        self._tol_orient = rospy.get_param('~tol_orient', 0.02)   # rad

        self._pub_target = rospy.Publisher('/b166er/ee_target', PoseStamped,
                                           queue_size=1, latch=True)

        self._robot_state = None
        rospy.Subscriber('/b166er/robot_state', RobotState, self._cb_state)

        rospy.loginfo('[task_sequencer] aguardando /b166er/robot_state...')
        while self._robot_state is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Orientação do EE no início da tarefa, mantida fixa em todas as
        # fases (ver docstring do módulo).
        self._ee_orientation = self._robot_state.ee_pose.pose.orientation
        rospy.loginfo('[task_sequencer] orientação do EE capturada — '
                      'mantida fixa durante toda a tarefa')

        self._olhal_pos, self._olhal_R = self._query_olhal_pose()

    # ------------------------------------------------------------------
    def _cb_state(self, msg):
        self._robot_state = msg

    def _query_olhal_pose(self):
        if self._use_ground_truth:
            wall_pos, R = self._query_wall_pose_ground_truth()
        else:
            wall_pos, R = self._query_wall_pose_perception()

        # wall_link + offset fixo (girado pela orientação de wall_link,
        # que é a mesma orientação de toda a cadeia bracket→blade→olhal)
        # = pose real de chave_olhal_link.
        olhal_pos = wall_pos + R @ OLHAL_OFFSET_FROM_WALL_LINK
        rospy.loginfo('[task_sequencer] chave_olhal_link (calculado) em '
                      '(%.3f, %.3f, %.3f)', *olhal_pos)
        return olhal_pos, R

    def _query_wall_pose_perception(self):
        """wall_link por percepção — assina /b166er/wall_pose (apriltag_localizer.py).

        Bloqueia até a tag ser detectada pelo menos uma vez (ou até
        ~wall_pose_timeout, caso em que levanta erro — sem detecção não
        tem como saber onde manipular, não faz sentido seguir).
        """
        rospy.loginfo('[task_sequencer] aguardando detecção da tag em '
                      '/b166er/wall_pose (apriltag_localizer precisa estar '
                      'rodando e a tag visível pela task_camera)...')
        try:
            msg = rospy.wait_for_message('/b166er/wall_pose', PoseStamped,
                                         timeout=self._wall_pose_timeout)
        except rospy.ROSException:
            raise RuntimeError(
                '[task_sequencer] nenhuma detecção de tag em /b166er/wall_pose '
                'depois de {:.0f}s — apriltag_localizer.py está rodando? a tag '
                'está no campo de visão da task_camera?'.format(
                    self._wall_pose_timeout))

        p = msg.pose.position
        o = msg.pose.orientation
        wall_pos = np.array([p.x, p.y, p.z])
        R        = _quat_to_matrix(o)[:3, :3]
        rospy.loginfo('[task_sequencer] wall_link (por percepção) em '
                      '(%.3f, %.3f, %.3f)', *wall_pos)
        return wall_pos, R

    def _query_wall_pose_ground_truth(self):
        """wall_link via /gazebo/get_link_state — só debug/calibração
        (~use_ground_truth:=true), nunca o caminho normal."""
        rospy.logwarn('[task_sequencer] ~use_ground_truth=true — usando '
                      '/gazebo/get_link_state, NÃO a percepção. Só para '
                      'debug/calibração do apriltag_localizer.')
        rospy.wait_for_service('/gazebo/get_link_state')
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        resp = get_link_state(self._link_name, self._gazebo_world_frame)
        if not resp.success:
            # Gazebo às vezes só resolve com o nome com escopo do modelo
            scoped_name = '{}::{}'.format(self._model_name, self._link_name)
            resp = get_link_state(scoped_name, self._gazebo_world_frame)

        if not resp.success:
            raise RuntimeError(
                '[task_sequencer] não consegui obter a pose de {} via '
                '/gazebo/get_link_state (status_message="{}") — a fixture '
                '(spawn_chave_fixture.launch) está rodando?'.format(
                    self._link_name, resp.status_message))

        p = resp.link_state.pose.position
        o = resp.link_state.pose.orientation
        wall_pos = np.array([p.x, p.y, p.z])
        R        = _quat_to_matrix(o)[:3, :3]
        rospy.loginfo('[task_sequencer] %s (ground-truth) em (%.3f, %.3f, %.3f)',
                      self._link_name, *wall_pos)
        return wall_pos, R

    def _tooltip_target_matrix(self, phase_params):
        """Pose alvo da PONTA DA FERRAMENTA (não do T265) para a fase."""
        offset_local = np.array(phase_params['offset_xyz_m'], dtype=float)
        offset_world = self._olhal_R @ offset_local
        target_pos   = self._olhal_pos + offset_world

        T = _quat_to_matrix(self._ee_orientation)
        T[:3, 3] = target_pos
        return T

    def _publish_target(self, T_target):
        msg = PoseStamped()
        msg.header.frame_id = self._world_frame
        msg.header.stamp    = rospy.Time.now()
        msg.pose.position.x = T_target[0, 3]
        msg.pose.position.y = T_target[1, 3]
        msg.pose.position.z = T_target[2, 3]
        msg.pose.orientation = self._ee_orientation
        self._pub_target.publish(msg)

    def _get_ee_matrix(self, state):
        p = state.ee_pose.pose.position
        o = state.ee_pose.pose.orientation
        T = _quat_to_matrix(o)
        T[:3, 3] = [p.x, p.y, p.z]
        return T

    def _wait_convergence(self, phase_name, T_target):
        t_start = rospy.Time.now()
        rate    = rospy.Rate(10)
        warned  = False
        # self._robot_state já chegou pelo menos uma vez no __init__ e o
        # subscriber sempre guarda a última mensagem — err_pos/err_orient
        # só ficam None se, por algum motivo anômalo, ainda não tiverem
        # sido calculados no primeiro ciclo.
        err_pos, err_orient = None, None

        while not rospy.is_shutdown():
            state = self._robot_state
            if state is not None:
                T_cur = self._get_ee_matrix(state)
                err   = pose_error(T_cur, T_target)
                err_pos    = float(np.linalg.norm(err[:3]))
                err_orient = float(np.linalg.norm(err[3:]))

                if err_pos < self._tol_pos and err_orient < self._tol_orient:
                    rospy.loginfo('[task_sequencer] fase "%s" convergiu '
                                  '(err_pos=%.4fm err_ori=%.4frad)',
                                  phase_name, err_pos, err_orient)
                    return True

            elapsed = (rospy.Time.now() - t_start).to_sec()
            if elapsed > self._phase_timeout and not warned:
                rospy.logwarn('[task_sequencer] fase "%s" não convergiu em '
                              '%.1fs (err_pos=%s err_ori=%s) — continuando '
                              'a aguardar, sem abortar a tarefa',
                              phase_name, self._phase_timeout, err_pos, err_orient)
                warned = True

            rate.sleep()
        return False

    # ------------------------------------------------------------------
    def run(self):
        for phase_name, phase_params in self._phases:
            if rospy.is_shutdown():
                return

            T_tooltip_target = self._tooltip_target_matrix(phase_params)
            R_ee_fixed = T_tooltip_target[:3, :3]   # = self._ee_orientation, mantida fixa
            p_t265_target = tooltip_position_to_t265_position(
                T_tooltip_target[:3, 3], R_ee_fixed)

            T_t265_target = np.eye(4)
            T_t265_target[:3, :3] = R_ee_fixed
            T_t265_target[:3, 3]  = p_t265_target

            rospy.loginfo('[task_sequencer] fase "%s" — alvo ponta-da-ferramenta='
                          '(%.3f, %.3f, %.3f), alvo T265 equivalente=(%.3f, %.3f, %.3f)',
                          phase_name, *T_tooltip_target[:3, 3], *T_t265_target[:3, 3])
            self._publish_target(T_t265_target)

            # Convergência checada no alvo do T265 — ee_pose é o T265,
            # não a ponta da ferramenta (não observada diretamente).
            if not self._wait_convergence(phase_name, T_t265_target):
                return  # rospy.is_shutdown() durante a espera

            rospy.sleep(self._dwell_time)

        rospy.loginfo('[task_sequencer] sequência completa — chave aberta '
                      '(mantendo último alvo, "%s")', PHASE_ORDER[-1])
        rospy.spin()


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        TaskSequencer().run()
    except rospy.ROSInterruptException:
        pass
