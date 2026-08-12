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

Pose do olhal: consultada UMA VEZ, no startup, via
/gazebo/get_link_state (chave_olhal_link) — fonte única de verdade, em
vez de recalcular a geometria do pivô em Python (ela já está duplicada
entre o macro chave_seccionadora_lf.urdf.xacro e chave_com_tag.urdf.xacro;
não faz sentido duplicar de novo aqui). Os offsets do YAML estão no
frame local do fixture (X horizontal ao longo da parede, Z vertical —
ver docs/chave_seccionadora_task.md); giramos esse offset pela
orientação do link consultada (que é só o yaw de spawn do fixture,
já que nenhum joint na cadeia bracket→blade→olhal aplica rotação de
frame — blade_angle é só um offset visual, não gira o frame).

Orientação do alvo: o documento da tarefa descreve só DIREÇÕES DE
FORÇA (perpendicular/paralela ao eixo X), não uma orientação de
ferramenta para o engate. Este nó mantém fixa a orientação do EE
capturada no início da tarefa (primeiro /b166er/robot_state recebido)
em todas as fases — só a posição varia. Se o tooling do end-effector
precisar de uma orientação de engate específica, isso ainda precisa
ser adicionado (não está no escopo do documento original).

Tópicos
-------
  Subscreve:
    /b166er/robot_state    (RobotState)   — para monitorar convergência
  Publica:
    /b166er/ee_target      (PoseStamped)  — alvo Cartesiano da fase atual
  Serviços:
    /gazebo/get_link_state — consultado uma vez, no startup
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetLinkState
from tf.transformations import quaternion_matrix

from b166er_whole_body_control.msg import RobotState
from b166er_whole_body_control.kinematics import pose_error


PHASE_ORDER = ['engage', 'release', 'pos1', 'pos2']


def _quat_to_matrix(o):
    return quaternion_matrix([o.x, o.y, o.z, o.w])


class TaskSequencer:

    def __init__(self):
        rospy.init_node('task_sequencer')

        task_params = rospy.get_param('/chave_seccionadora_task')
        self._phases = [(name, task_params[name]) for name in PHASE_ORDER]

        # ~target_link sobrescreve o target_frame do YAML só se setado
        # explicitamente — o YAML é a fonte de verdade por padrão.
        self._link_name   = rospy.get_param('~target_link',
                                             task_params.get('target_frame',
                                                              'chave_olhal_link'))
        self._model_name  = rospy.get_param('~fixture_model_name',
                                             'chave_seccionadora_fixture')
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
        rospy.loginfo('[task_sequencer] aguardando /gazebo/get_link_state...')
        rospy.wait_for_service('/gazebo/get_link_state')
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        resp = get_link_state(self._link_name, self._world_frame)
        if not resp.success:
            # Gazebo às vezes só resolve com o nome com escopo do modelo
            scoped_name = '{}::{}'.format(self._model_name, self._link_name)
            resp = get_link_state(scoped_name, self._world_frame)

        if not resp.success:
            raise RuntimeError(
                '[task_sequencer] não consegui obter a pose de {} via '
                '/gazebo/get_link_state (status_message="{}") — a fixture '
                '(spawn_chave_fixture.launch) está rodando?'.format(
                    self._link_name, resp.status_message))

        p = resp.link_state.pose.position
        o = resp.link_state.pose.orientation
        pos = np.array([p.x, p.y, p.z])
        R   = _quat_to_matrix(o)[:3, :3]
        rospy.loginfo('[task_sequencer] %s em (%.3f, %.3f, %.3f)',
                      self._link_name, *pos)
        return pos, R

    def _phase_target_matrix(self, phase_params):
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

            T_target = self._phase_target_matrix(phase_params)
            rospy.loginfo('[task_sequencer] fase "%s" — alvo=(%.3f, %.3f, %.3f)',
                          phase_name, *T_target[:3, 3])
            self._publish_target(T_target)

            if not self._wait_convergence(phase_name, T_target):
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
