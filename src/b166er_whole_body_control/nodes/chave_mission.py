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
  APPROACH   → navega a base até a pose de standoff (0,65 m à frente do
               olhal, encarando a parede), braço ainda recolhido.
  DEPLOY     → braço para a postura de pré-manipulação.
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
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf.transformations import quaternion_matrix, euler_from_quaternion

from b166er_whole_body_control.msg import RobotState
from b166er_whole_body_control.kinematics import (
    JOINT_NAMES, pose_error, tooltip_position_to_t265_position)
from b166er_whole_body_control import chave_task

PHASE_ORDER = ['engage', 'release', 'pos1', 'pos2']


def _yaw_of(quat):
    return euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]


def _ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class MissionContext(object):
    """Estado compartilhado + interface ROS, injetado em cada estado SMACH."""

    def __init__(self):
        self.rate_hz       = rospy.get_param('~rate', 20.0)
        self.standoff_dist = rospy.get_param('~standoff_distance', 0.65)
        self.search_omega  = rospy.get_param('~search_omega', 0.35)
        self.search_timeout   = rospy.get_param('~search_timeout', 90.0)
        self.approach_timeout = rospy.get_param('~approach_timeout', 120.0)
        self.posture_timeout  = rospy.get_param('~posture_timeout', 30.0)
        self.phase_timeout    = rospy.get_param('~phase_timeout', 60.0)

        # Navegação (girar-avançar-girar)
        self.nav_v         = rospy.get_param('~nav_linear_vel', 0.15)
        self.nav_w         = rospy.get_param('~nav_angular_vel', 0.4)
        self.nav_pos_tol   = rospy.get_param('~nav_pos_tol', 0.06)
        self.nav_yaw_tol   = rospy.get_param('~nav_yaw_tol', 0.09)

        # Tolerâncias Cartesianas — mesmas do fuzzy_wb_controller
        self.tol_pos    = rospy.get_param('~tol_pos', 0.005)
        self.tol_orient = rospy.get_param('~tol_orient', 0.02)

        self.postures = rospy.get_param('/arm_postures')
        self.phases   = rospy.get_param('/chave_seccionadora_task')

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

        rospy.Subscriber('/b166er/robot_state', RobotState, self._cb_state)
        rospy.Subscriber('/b166er/wall_pose', PoseStamped, self._cb_wall)
        rospy.Subscriber('/b166er/arm_posture_reached', Bool, self._cb_posture)

    # ------------------------------------------------------------------
    def _cb_state(self, msg):
        self.robot_state = msg

    def _cb_wall(self, msg):
        self.wall_pose = msg

    def _cb_posture(self, msg):
        if msg.data:
            self.posture_done = True

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

    def take_base(self):
        """Missão assume /cmd_vel (navegação); fuzzy_wb_controller cala."""
        self.pub_wb_enable.publish(Bool(data=False))
        rospy.sleep(0.2)   # deixa o stand-down chegar antes de dirigir

    def release_base(self):
        """Devolve /cmd_vel ao fuzzy_wb_controller (manipulação)."""
        self.stop_base()
        self.pub_wb_enable.publish(Bool(data=True))
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
                p = ctx.wall_pose.pose
                ctx.wall_pos = np.array([p.position.x, p.position.y, p.position.z])
                ctx.wall_R   = quaternion_matrix(
                    [p.orientation.x, p.orientation.y,
                     p.orientation.z, p.orientation.w])[:3, :3]
                rospy.loginfo('[mission] tag encontrada — parede em (%.3f, %.3f, %.3f)',
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
    """Navega a base até a pose de standoff, de frente para a chave."""

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['arrived', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx = self.ctx
        goal = chave_task.standoff_base_pose(ctx.wall_pos, ctx.wall_R,
                                             ctx.standoff_dist)
        rospy.loginfo('[mission] APPROACH — standoff em (%.2f, %.2f, %.2f rad)', *goal)
        ok = _navigate_to(ctx, goal, ctx.approach_timeout, 'APPROACH')
        return 'arrived' if ok else 'failed'


class Deploy(smach.State):
    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['ok', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx = self.ctx
        ctx.send_posture('deploy')
        if not _wait_posture(ctx):
            return 'failed'
        # Handoff: daqui em diante quem dirige base E braço é o
        # controlador whole-body, conduzido por /b166er/ee_target.
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
            p_t265 = tooltip_position_to_t265_position(p_tip, R_ee)

            T_target = np.eye(4)
            T_target[:3, :3] = R_ee
            T_target[:3, 3]  = p_t265

            msg = PoseStamped()
            msg.header.frame_id = 'odom'
            msg.header.stamp    = rospy.Time.now()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p_t265
            msg.pose.orientation = ctx.ee_orientation
            ctx.pub_target.publish(msg)
            rospy.loginfo('[mission] fase "%s" — ponta em (%.3f, %.3f, %.3f)',
                          phase, *p_tip)

            if not _wait_ee_convergence(ctx, T_target, phase):
                return 'failed'
            rospy.sleep(1.0)

        rospy.loginfo('[mission] MANIPULATE concluída — chave aberta')
        return 'done'


class Retract(smach.State):
    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['ok', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx = self.ctx
        ctx.take_base()          # retoma /cmd_vel para navegar de volta
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
        self.ctx.take_base()
        self.ctx.stop_base()
        self.ctx.send_posture('stow_home')
        _wait_posture(self.ctx)
        return 'aborted'


# ═══════════════════════════════════════════════════════════════════════
# Helpers compartilhados entre estados
# ═══════════════════════════════════════════════════════════════════════

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

    while not rospy.is_shutdown():
        if (rospy.Time.now() - t0).to_sec() > timeout:
            ctx.stop_base()
            rospy.logerr('[mission] %s: timeout de %.0fs em %s', tag, timeout, phase)
            return False

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
            if dist < ctx.nav_pos_tol:
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
                rospy.loginfo('[mission] %s: concluído', tag)
                return True
            ctx.drive(0.0, math.copysign(ctx.nav_w, err))

        rate.sleep()
    return False


def _wait_ee_convergence(ctx, T_target, phase):
    rate = rospy.Rate(10)
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        st = ctx.robot_state
        p, o = st.ee_pose.pose.position, st.ee_pose.pose.orientation
        T_cur = quaternion_matrix([o.x, o.y, o.z, o.w])
        T_cur[:3, 3] = [p.x, p.y, p.z]
        err = pose_error(T_cur, T_target)
        rp, ro = np.linalg.norm(err[:3]), np.linalg.norm(err[3:])

        if rp < ctx.tol_pos and ro < ctx.tol_orient:
            rospy.loginfo('[mission] fase "%s" convergiu (%.4f m, %.4f rad)',
                          phase, rp, ro)
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
                               transitions={'arrived': 'DEPLOY', 'failed': 'ABORT_SAFE'})
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
