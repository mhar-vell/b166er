"""
chave_task — geometria compartilhada da tarefa da chave seccionadora.

Existe para que os nós que precisam da MESMA geometria não repitam as
constantes do fixture (urdf/fixtures/chave_seccionadora_lf.urdf.xacro e
chave_com_tag.urdf.xacro) cada um por conta própria — antes deste
módulo, task_sequencer.py já era a terceira cópia da fórmula do pivô, e
chave_mission.py seria a quarta.

O que continua duplicado, inevitavelmente: os NÚMEROS abaixo espelham
os xacro:property do fixture. O xacro não expõe suas propriedades para
fora, e o Gazebo funde a cadeia de juntas fixas num único corpo
(ver docstring de apriltag_localizer.py), então nem via TF nem via
/gazebo/get_link_state dá para consultar o olhal diretamente. Se os
parâmetros dimensionais mudarem no xacro, atualizar aqui também.
"""

import math

import numpy as np

# ── Espelha chave_seccionadora_lf.urdf.xacro + chave_com_tag.urdf.xacro ──
CHAVE_X_OFFSET     = -0.20    # chave_x_offset
OLHAL_HEIGHT       = 0.810    # olhal_height (spec do Marco)
BLADE_LENGTH       = 0.200    # blade_length
BLADE_ANGLE_CLOSED = 0.0      # fechada = lâmina VERTICAL (correção do Marco:
                              # os 20° da versão anterior eram a posição ABERTA)
BLADE_ANGLE_OPEN   = 0.524    # 30° — curso de abertura confirmado pelo Marco
WALL_STANDOFF      = 0.08     # wall_standoff da chave (mecanismo destacado da placa)
TAG_X_OFFSET       = -0.42    # tag_x_offset (outro lado da chave, 0,22 m dela)
TAG_MOUNT_Z        = 1.030    # tag_mount_z

_PIVOT_X = BLADE_LENGTH * math.sin(BLADE_ANGLE_CLOSED)

# Offset fixo de wall_link até chave_olhal_link, no frame local do
# fixture (X ao longo da parede, Y normal saindo da face, Z vertical).
# O Z é exatamente OLHAL_HEIGHT porque wall_link nasce no chão e a
# cadeia do fixture foi construída para o olhal cair nessa altura.
#
# Histórico da geometria (2026-08-13, duas correções do Marco no mesmo
# dia): primeiro a mecânica foi invertida (pivô passou para BAIXO,
# olhal para CIMA); depois a chave inteira foi rotacionada 90° no eixo
# Z, de modo que a lâmina inclina PARA FORA da parede (plano Y-Z) em
# vez de de lado, e a barra do olhal corre paralela à face — como na
# foto da bancada.
#
# E uma terceira: a posição FECHADA passou a ser a lâmina VERTICAL —
# os 20° que o modelo desenhava como "fechada" são, na verdade, a
# posição ABERTA ("a inclinação que está é a posição da chave aberta").
# Fechada e vertical, o olhal fica direto acima do pivô, sem componente
# de inclinação. A altura (810 mm) e o X não mudaram em nenhuma das
# três correções.
OLHAL_OFFSET_FROM_WALL_LINK = np.array([
    CHAVE_X_OFFSET,
    WALL_STANDOFF,        # lâmina vertical: olhal direto acima do pivô
    OLHAL_HEIGHT,
])

# Offset fixo de wall_link até o centro da placa da tag.
TAG_OFFSET_FROM_WALL_LINK = np.array([TAG_X_OFFSET, 0.0, TAG_MOUNT_Z])

# Normal da face frontal da parede (onde chave e tag estão montadas),
# no frame local do fixture — aponta para fora, na direção de onde o
# robô se aproxima.
WALL_FRONT_NORMAL_LOCAL = np.array([0.0, 1.0, 0.0])


def flatten_wall_R(wall_R):
    """
    Projeta a orientação estimada da parede em rotação PURA DE YAW.

    A parede é vertical por construção (assenta no chão, tag e chave
    montadas na face) — roll e pitch são zero de verdade, e o que a
    estimativa por PnP devolve nesses eixos é só ruído. Ruído que
    custa caro: o olhal fica a 0,86 m da origem de wall_link
    (majoritariamente em Z), então cada grau de tilt espúrio empurra a
    posição calculada do olhal em ~1,5 cm na horizontal.

    Foi exatamente esse o modo de falha em 2026-08-13: detecção a
    ~2,5 m com ~24° de tilt acumulado colocou o alvo da fase "engage"
    em y=3,357 — atrás da parede (que está em y=3,0) — e o
    controlador whole-body dirigiu o robô contra ela até tombar.

    Aplicar o vínculo de verticalidade aqui é usar informação que o
    robô realmente tem sobre a tarefa, não maquiar a medição: a
    incerteza de yaw (a única que importa para mirar) fica preservada.
    """
    wall_R = np.asarray(wall_R)
    yaw = math.atan2(wall_R[1, 0], wall_R[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]])


def olhal_position(wall_pos, wall_R):
    """Posição de chave_olhal_link no mundo, dada a pose de wall_link."""
    return np.asarray(wall_pos) + wall_R @ OLHAL_OFFSET_FROM_WALL_LINK


def wall_front_normal(wall_R):
    """Normal (unitária, no mundo) da face frontal da parede."""
    return wall_R @ WALL_FRONT_NORMAL_LOCAL


def standoff_base_pose(wall_pos, wall_R, distance):
    """
    Pose de aproximação da BASE: parada a `distance` metros à frente do
    olhal, sobre a normal da face da parede, encarando a parede.

    A distância é medida do olhal (não da parede) porque é o alcance do
    braço até o olhal que limita — alcance horizontal da ponta da
    ferramenta na altura do olhal é ~0,93 m (medido por varredura de
    FK), então valores em torno de 0,65 m deixam margem sem exigir o
    braço quase estendido (postura que já tombou o robô).

    Retorna (x, y, yaw) no frame do mundo.
    """
    olhal = olhal_position(wall_pos, wall_R)
    n     = wall_front_normal(wall_R)

    # Projeta no plano do chão: a base só navega em XY.
    n_xy = np.array([n[0], n[1]])
    norm = np.linalg.norm(n_xy)
    if norm < 1e-6:
        raise ValueError('normal da parede é vertical — fixture mal orientada?')
    n_xy = n_xy / norm

    pos_xy = np.array([olhal[0], olhal[1]]) + n_xy * distance
    # Encara a parede: heading é a normal invertida.
    yaw = math.atan2(-n_xy[1], -n_xy[0])
    return float(pos_xy[0]), float(pos_xy[1]), float(yaw)


def phase_target_position(wall_pos, wall_R, offset_xyz):
    """
    Posição alvo da PONTA DA FERRAMENTA para uma fase da tarefa
    (engage/release/pos1/pos2), dado o offset local do YAML
    (config/chave_seccionadora_task.yaml).
    """
    return olhal_position(wall_pos, wall_R) + wall_R @ np.asarray(offset_xyz)
