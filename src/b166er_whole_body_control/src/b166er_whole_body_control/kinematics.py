"""
kinematics.py — cinemática do b166er (Pioneer + RV-M2) em numpy puro.

Cadeia completa:
  world → base_link → top_plate → Base(braço) → J1 → L1 → J2 → L2
        → J3 → L3 → J4 → L4 → J5 → L5 → CameraSupport → t265_link

8-DOF whole-body: q_wb = [x_b, y_b, θ_b,  q1, q2, q3, q4, q5]
                          ← 3 base →       ←— 5 braço —→
"""

import math

import numpy as np

# ---------------------------------------------------------------------------
# Constantes
# ---------------------------------------------------------------------------

JOINT_NAMES   = ['J1', 'J2', 'J3', 'J4', 'J5']
JOINT_LOWER   = np.array([-2.61799, -1.13446, -1.04720, -1.91986, -3.14159])
JOINT_UPPER   = np.array([ 2.61799,  1.13446,  1.04720,  1.91986,  3.14159])

# IK
IK_MAX_ITER   = 300
IK_TOL_POS    = 3e-3   # m  — 3 mm suficiente para estimação de estado
IK_TOL_ORIENT = 5e-2   # rad — ~3°, suficiente para estimação de estado
IK_LAMBDA     = 0.02
IK_DQ_STEP    = 1e-6
IK_MAX_STEP   = 0.1    # rad/iteração — clamp de passo do DLS (ver ik_arm)

# Compensação de gravidade
_G           = 9.81                                # m/s²
_LINK_MASSES = np.array([6.3, 5.0, 4.2, 2.5, 1.7])  # L1..L5 kg (do URDF)


# ---------------------------------------------------------------------------
# Primitivas de transformação homogênea (4×4)
# ---------------------------------------------------------------------------

def _rotx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]], dtype=float)

def _roty(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]], dtype=float)

def _rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]], dtype=float)

def _trans(x, y, z):
    T = np.eye(4); T[:3, 3] = [x, y, z]; return T

def _tf(xyz, rpy):
    """Translação + RPY extrínseco (Rz·Ry·Rx)."""
    T = np.eye(4)
    T[:3, :3] = (_rotz(rpy[2]) @ _roty(rpy[1]) @ _rotx(rpy[0]))[:3, :3]
    T[:3,  3] = xyz
    return T

T_BASELINK_ARM = _tf([0.003, 0, 0.294], [0, 0, 0])
# Montagem da T265 no berço de CIMA do suporte (2026-09-02). Espelha
# t265_mount_joint no movemaster.urdf.xacro, onde estão a medição da
# malha e o porquê de cada número — mudar aqui sem mudar lá (ou o
# contrário) é o erro que já pôs a parede 0,795 m fora do lugar.
_T_L5_T265     = _trans(0, 0, -0.08) @ _tf([-0.0011, 0.0860, -0.0194],
                                           [1.5708, 1.5708, 0])

# DEDO FIXO (desenho do Marco, 2026-08-25) — cadeia rígida do L5 até a
# ponta, via GripCube (JCam + JGripCube + JTool + JToolTip, todas fixed).
#
# Substituiu a vara com gancho em J. Duas diferenças que importam para
# quem consome esta constante:
#
#   · a peça monta na posição de um DEDO do gripper (x = +30 mm), não no
#     eixo — a ponta deixou de ser coaxial com J5. Isso tira J5 do espaço
#     nulo: antes |∂ponta/∂J5| era exatamente zero em qualquer postura,
#     agora o rolamento do punho move a ponta.
#   · o comprimento caiu de 200 mm para 135 mm (25 garfo + 10 afunilamento
#     + 80 haste + 20 ponta alargada), então a ponta subiu 65 mm.
_T_L5_TOOLTIP = (_trans(0, 0, -0.08)     # JCam:      L5 → CameraSupport
                @ _trans(0, 0, -0.005)   # JGripCube: CameraSupport → GripCube
                @ _trans(0.03, 0, -0.08)  # JTool:     GripCube → dedo fixo (posição do dedo)
                @ _trans(0, 0, -0.115))  # JToolTip:  garfo + afunilamento + haste = 115 mm
# A origem de tool_tip é a base do DEGRAU — o ponto onde o arame do olhal
# repousa depois da descida de 5 mm. É esse ponto, e não a extremidade da
# peça, que a missão persegue: o degrau se estende 20 mm em −X a partir
# daqui, e o anel pode ficar em qualquer lugar ao longo dele.

# Offset FIXO e conhecido de t265_link até a ponta da ferramenta —
# ambos pendurados rigidamente no mesmo CameraSupport, então essa
# transformação não muda com a pose do braço. T265 continua sendo o
# único sensor de pose do EE (arquitetura sem encoders); quem precisa
# de um alvo Cartesiano para a PONTA DA FERRAMENTA (ex.: task_sequencer.py
# mirando chave_olhal_link) usa esta constante para converter esse alvo
# num alvo equivalente para o T265 — nunca o contrário.
T_T265_TOOLTIP = np.linalg.inv(_T_L5_T265) @ _T_L5_TOOLTIP

# Câmera de tarefa (detecção de AprilTag, ver movemaster.urdf.xacro,
# JTaskCamera) — mesma ideia do tool_tip acima: offset fixo e
# conhecido a partir do T265, pendurada no mesmo GripCube (JCam +
# JGripCube + JTaskCamera, todas fixed). NÃO é o T265 — é uma câmera
# de imagem separada, proxy de simulação para o stream que a câmera
# fisheye do T265 real também forneceria (ver apriltag_localizer.py).
_T_L5_TASKCAMERA = (_trans(0, 0, -0.08)    # JCam:      L5 → CameraSupport
                    @ _trans(0, 0, -0.005)  # JGripCube: CameraSupport → GripCube
                    @ _trans(0, 0, -0.08))  # JTaskCamera: GripCube → task_camera_link

T_T265_TASKCAMERA = np.linalg.inv(_T_L5_T265) @ _T_L5_TASKCAMERA

# ── FISHEYE1 DA T265 ──────────────────────────────────────────────────
# Extrínseca MEDIDA na bancada em 2026-08-26 (série 925122110468):
#
#   t265_link -> t265_fisheye1_optical_frame
#     translação [0, 0,032, 0]      -> 32 mm em Y
#     rotação    [-90°, 0°, -90°]   -> a convenção óptica padrão
#
# Aqui guardamos a transformada até o frame do LINK, não até o óptico:
# quem consome (apriltag_localizer) aplica R_LINK_OPTICAL em seguida, e
# incluir a rotação óptica nesta constante a duplicaria.
#
# O que importa e não estava modelado são os 32 mm: a câmera que VÊ não
# fica na origem do sensor que dá a POSE. Deslocamento fixo na origem do
# raio de projeção vira erro de pose que cresce com a distância angular à
# tag — suspeito do erro proporcional observado na simulação.
T_T265_FISHEYE1 = _trans(0.0, 0.032, 0.0)


def tooltip_target_to_t265_target(T_world_tooltip_target):
    """
    Converte um alvo Cartesiano de ORIENTAÇÃO EXPLÍCITA pensado para a
    ponta da ferramenta no alvo equivalente para o T265 — o frame que o
    whole-body controller realmente controla.

    ATENÇÃO — rotação, não só posição: a rotação de T_world_tooltip_target
    é tratada como a rotação DESEJADA DA PONTA DA FERRAMENTA, não do T265.
    Como T_T265_TOOLTIP tem uma parte rotacional não-trivial (o mount da
    ferramenta não é um alinhamento puro), a rotação resultante do T265
    (T_world_t265_target[:3,:3]) sai DIFERENTE da rotação de entrada — é
    a rotação de T265 necessária para que a FERRAMENTA atinja aquela
    orientação, não uma cópia dela. Use esta função só quando a tarefa
    realmente especifica a orientação da ferramenta.

    Para o caso mais comum neste código (manter a orientação do PRÓPRIO
    T265 fixa e só variar a posição-alvo da ponta da ferramenta — ver
    task_sequencer.py), use tooltip_position_to_t265_position abaixo, não
    esta função.

    T_world_tooltip_target : ndarray (4,4) — pose alvo desejada da
                              ponta da ferramenta, no frame mundial.

    Retorna T_world_t265_target (4,4).
    """
    return T_world_tooltip_target @ np.linalg.inv(T_T265_TOOLTIP)


def tooltip_position_to_t265_position(p_world_tooltip_target, R_world_t265_fixed):
    """
    Converte uma posição-alvo da ponta da ferramenta na posição-alvo
    equivalente do T265, MANTENDO a orientação do T265 fixa em
    R_world_t265_fixed (o caso comum: a tarefa não especifica
    orientação de ferramenta, só mantém o EE numa orientação
    constante — ver task_sequencer.py).

    p_world_tooltip_target : array (3,)   — posição alvo da ponta da
                              ferramenta, no frame mundial.
    R_world_t265_fixed     : ndarray (3,3) — orientação do T265 a
                              manter (mesma em toda a tarefa).

    Retorna p_world_t265_target (3,), tal que comandar o T265 para essa
    posição, nessa orientação, deixa a ponta da ferramenta exatamente
    em p_world_tooltip_target.
    """
    offset_world = R_world_t265_fixed @ T_T265_TOOLTIP[:3, 3]
    return np.asarray(p_world_tooltip_target) - offset_world


def _adj(R):
    """Adjunto 6×6 para transformar vetores de velocidade de frame."""
    A = np.zeros((6, 6))
    A[:3, :3] = R
    A[3:, 3:] = R
    return A


# ---------------------------------------------------------------------------
# Cinemática direta (FK)
# ---------------------------------------------------------------------------

def fk_arm(q):
    """
    FK completa: Base → t265_link.

    Parâmetros
    ----------
    q : array (5,)  — ângulos [q1..q5] em rad

    Retorna
    -------
    T : ndarray (4,4) — transform Base → t265_link
    """
    q1, q2, q3, q4, q5 = q
    return (
        _trans(0, 0, 0.4)    @ _rotz(q1)
        @ _trans(0.12, 0, 0) @ _rotx(1.5708) @ _rotz(q2)
        @ _trans(0.25, 0, 0) @ _rotz(q3)
        @ _trans(0.2,  0, 0) @ _rotz(q4)
        @ _roty(-1.5708)     @ _rotz(q5)
        @ _T_L5_T265
    )


def fk_arm_joint_frames(q):
    """
    FK intermediária: retorna o frame de cada junta (antes da rotação)
    e o frame do EE, todos no frame Base do braço.

    Retorna
    -------
    frames : list de 5 ndarray (4,4)  — T_Base_Ji_pre para i=1..5
    T_EE   : ndarray (4,4)            — T_Base_t265
    """
    q1, q2, q3, q4, q5 = q

    T_J1_pre = _trans(0, 0, 0.4)
    T_L1     = T_J1_pre @ _rotz(q1)

    T_J2_pre = T_L1 @ _trans(0.12, 0, 0) @ _rotx(1.5708)
    T_L2     = T_J2_pre @ _rotz(q2)

    T_J3_pre = T_L2 @ _trans(0.25, 0, 0)
    T_L3     = T_J3_pre @ _rotz(q3)

    T_J4_pre = T_L3 @ _trans(0.2, 0, 0)
    T_L4     = T_J4_pre @ _rotz(q4)

    T_J5_pre = T_L4 @ _roty(-1.5708)
    T_L5     = T_J5_pre @ _rotz(q5)

    T_EE     = T_L5 @ _T_L5_T265

    return [T_J1_pre, T_J2_pre, T_J3_pre, T_J4_pre, T_J5_pre], T_EE


# ---------------------------------------------------------------------------
# Jacobianas
# ---------------------------------------------------------------------------

def arm_jacobian_world(q, T_world_arm_base):
    """
    Jacobiana geométrica do braço no frame mundial (6×5).

    Para cada junta revoluta i:
      Velocidade linear : z_i × (p_EE - p_i)
      Velocidade angular: z_i
    onde z_i = eixo Z do frame pre-rotação de Ji, expresso no frame mundial.

    Parâmetros
    ----------
    q                : array (5,)    — ângulos das juntas
    T_world_arm_base : ndarray (4,4) — transform world → Base do braço
    """
    joint_frames, T_EE_base = fk_arm_joint_frames(q)

    R_wb = T_world_arm_base[:3, :3]

    # EE em frame mundial
    p_EE_w = (T_world_arm_base @ np.append(T_EE_base[:3, 3], 1))[:3]

    J = np.zeros((6, 5))
    for i, T_ji in enumerate(joint_frames):
        z_w = R_wb @ T_ji[:3, 2]                              # eixo Z em frame mundial
        p_i_w = (T_world_arm_base @ np.append(T_ji[:3, 3], 1))[:3]
        r = p_EE_w - p_i_w
        J[:3, i] = np.cross(z_w, r)
        J[3:, i] = z_w

    return J


def base_jacobian_world(T_world_base, p_EE_world):
    """
    Jacobiana da base Pioneer (x_b, y_b, θ_b) sobre o EE, frame mundial (6×3).

    ẋ_EE = ẋ_b − r_y·θ̇_b
    ẏ_EE = ẏ_b + r_x·θ̇_b
    ż_EE = 0
    ω_z  = θ̇_b

    onde r = p_EE − p_base  (frame mundial).
    """
    r = p_EE_world - T_world_base[:3, 3]

    J = np.zeros((6, 3))
    J[0, 0] =  1        # ẋ_EE / ẋ_base
    J[1, 1] =  1        # ẏ_EE / ẏ_base
    J[0, 2] = -r[1]     # ẋ_EE / θ̇_base
    J[1, 2] =  r[0]     # ẏ_EE / θ̇_base
    J[5, 2] =  1        # ω_z_EE / θ̇_base
    return J


def whole_body_jacobian(q, T_world_base):
    """
    Jacobiana whole-body J_wb (6×8) = [J_base | J_arm].

    q_wb = [x_b, y_b, θ_b, q1, q2, q3, q4, q5]

    Parâmetros
    ----------
    q            : array (5,) — ângulos das juntas do braço
    T_world_base : ndarray (4,4) — transform world → base_link do Pioneer
    """
    T_world_arm_base = T_world_base @ T_BASELINK_ARM

    joint_frames, T_EE_base = fk_arm_joint_frames(q)
    p_EE_world = (T_world_arm_base @ np.append(T_EE_base[:3, 3], 1))[:3]

    J_base = base_jacobian_world(T_world_base, p_EE_world)
    J_arm  = arm_jacobian_world(q, T_world_arm_base)

    return np.hstack([J_base, J_arm])   # (6×8)


# ---------------------------------------------------------------------------
# Controle: DLS pseudoinversa + projeção no espaço nulo
# ---------------------------------------------------------------------------

def dls_pseudoinverse(J, lam=IK_LAMBDA):
    """Pseudoinversa damped least-squares: J^T (J J^T + λ²I)^{-1}."""
    m = J.shape[0]
    return J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(m), np.eye(m))


def null_space_projector(J, J_pinv):
    """(I − J†J) — projeta para o espaço nulo de J."""
    n = J.shape[1]
    return np.eye(n) - J_pinv @ J


def joint_limit_gradient(q_arm):
    """
    Gradiente do custo de limite de junta para objetivo secundário.
    Empurra cada junta em direção ao centro de seu intervalo.
    """
    q_mid   = (JOINT_UPPER + JOINT_LOWER) / 2.0
    q_range = (JOINT_UPPER - JOINT_LOWER) / 2.0
    return -(q_arm - q_mid) / (q_range ** 2)


# ---------------------------------------------------------------------------
# IK do braço isolado (usado pelo state_estimator)
# ---------------------------------------------------------------------------

def _T_to_6vec(T):
    """4×4 → 6-vetor [pos, skew_sym(R)] para Jacobiano numérico."""
    pos = T[:3, 3]
    R   = T[:3, :3]
    dw  = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]]) * 0.5
    return np.concatenate([pos, dw])


def so3_log(R):
    """Logaritmo em SO(3): devolve θ·eixo, com θ ∈ [0, π] o ângulo REAL.

    Substitui a parte vetorial de R (2026-08-24). A versão anterior usava
    vee(R − Rᵀ)/2, que vale sin(θ)·eixo — e isso tem duas patologias:

      θ real     0°   30°   90°   150°   179°   180°
      relatado  0.00  0.50  1.00   0.50   0.02   0.00

    1) Em θ = 180° o erro relatado é EXATAMENTE ZERO: uma orientação
       completamente invertida é indistinguível de alinhamento perfeito,
       e a IK declara convergência.
    2) Pior: acima de 90° o valor DECRESCE, então o gradiente empurra
       para 180°. Não é um ponto cego, é um atrator espúrio.

    Consequência medida no b166er (2026-08-24): o state_estimator
    convergia com a posição da T265 certa a 0,6 mm e a orientação
    invertida 179,6°, reportando convergido. Como a ponta da ferramenta
    fica 0,228 m fora da origem da T265, ela ia parar 0,45 m longe do
    real. Toda a manipulação (que é baseada na ponta) passava a
    perseguir um alvo fantasma, o controlador estendia o braço para
    corrigir um erro inexistente e o robô tombava.

    Perto de θ = π, sin θ → 0 e a fórmula usual estoura; ali o eixo sai
    de (R + I)/2 = eixo·eixoᵀ. O sinal do eixo é ambíguo em π (θ·n e
    −θ·n são a mesma rotação) — qualquer um serve, ambos afastam da
    singularidade.
    """
    c = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    theta = math.acos(c)
    v = np.array([R[2, 1] - R[1, 2],
                  R[0, 2] - R[2, 0],
                  R[1, 0] - R[0, 1]])
    if theta < 1e-6:
        return 0.5 * v                      # sin θ ≈ θ
    if theta < math.pi - 1e-3:
        return (theta / (2.0 * math.sin(theta))) * v
    A = 0.5 * (R + np.eye(3))               # = eixo·eixoᵀ em θ = π
    i = int(np.argmax(np.diag(A)))
    d = math.sqrt(max(A[i, i], 0.0))
    if d < 1e-9:
        return np.zeros(3)
    eixo = A[:, i] / d
    return theta * (eixo / np.linalg.norm(eixo))


def pose_error(T_current, T_target):
    """
    Erro 6-vetor [Δp, Δω] de SE(3).
    Δp  = p_target − p_current
    Δω  = log de R_err = R_target · R_current^T  (ver so3_log)
    """
    dp    = T_target[:3, 3] - T_current[:3, 3]
    R_err = T_target[:3, :3] @ T_current[:3, :3].T
    return np.concatenate([dp, so3_log(R_err)])


def ik_arm(T_target, q_init=None):
    """
    IK numérica para o braço RV-M2 (Base → t265_link).
    DLS com Jacobiano central de diferenças finitas e clamp de passo.

    Retorna (q, converged, res_pos, res_orient).
    """
    q = np.zeros(5) if q_init is None else np.array(q_init, dtype=float)

    for _ in range(IK_MAX_ITER):
        T_cur  = fk_arm(q)
        err    = pose_error(T_cur, T_target)
        rp, ro = np.linalg.norm(err[:3]), np.linalg.norm(err[3:])

        if rp < IK_TOL_POS and ro < IK_TOL_ORIENT:
            return q, True, rp, ro

        # Jacobiano do ERRO (diferenças centrais de pose_error).
        #
        # BUG CORRIGIDO em 2026-08-13: a versão anterior derivava
        # _T_to_6vec(fk_arm(q)) — cuja parte rotacional é a antissimétrica
        # da orientação ABSOLUTA — enquanto o vetor conduzido (err) usa a
        # antissimétrica da rotação RELATIVA (R_target·R_curᵀ). São
        # parametrizações diferentes de rotação, então J não era a
        # derivada do erro que estava sendo minimizado: as 3 linhas
        # angulares apontavam numa direção inconsistente com as 3
        # lineares. Em poses estendidas o acoplamento é fraco e o DLS
        # ainda convergia (por isso passou despercebido desde a Fase 2);
        # em posturas dobradas, não — com o home recolhido o estimador
        # travava num mínimo local espelhado (real J2=+61°/J4=−103°,
        # estimado J2=+18°/J4=+110°, resíduo fixo de 4,9cm).
        #
        # Derivar pose_error diretamente mantém as duas metades
        # consistentes. Sinal negativo porque pose_error mede alvo −
        # atual: ∂err/∂q = −J_movimento.
        J = np.zeros((6, 5))
        for i in range(5):
            dq_i    = np.zeros(5); dq_i[i] = IK_DQ_STEP
            err_p   = pose_error(fk_arm(q + dq_i), T_target)
            err_m   = pose_error(fk_arm(q - dq_i), T_target)
            J[:, i] = -(err_p - err_m) / (2 * IK_DQ_STEP)

        dq = J.T @ np.linalg.solve(J @ J.T + IK_LAMBDA**2 * np.eye(6), err)

        # Clamp de passo: mesmo com o Jacobiano correto, posturas
        # dobradas deixam a Jacobiana mal-condicionada e o DLS com λ fixo
        # pode dar saltos que pulam para fora da bacia de atração do
        # seed. Com 300 iterações, o alcance total (30 rad) não limita
        # nenhum movimento real.
        step = np.max(np.abs(dq))
        if step > IK_MAX_STEP:
            dq *= IK_MAX_STEP / step

        q  = np.clip(q + dq, JOINT_LOWER, JOINT_UPPER)

    T_cur = fk_arm(q); err = pose_error(T_cur, T_target)
    return q, False, np.linalg.norm(err[:3]), np.linalg.norm(err[3:])


# ---------------------------------------------------------------------------
# Compensação de gravidade
# ---------------------------------------------------------------------------

def gravity_torque_arm(q):
    """
    Torque gravitacional em cada junta do RV-M2 (N·m).
    Assume frame Base do braço com eixo z alinhado com world up (Pioneer nivelado).
    CoM de cada elo aproximado na origem do frame da junta filha (consistente
    com os inertiais do URDF, todos em <origin xyz="0 0 0">).

    Retorna tau_grav (5,) — positivo = direção de aumento do ângulo da junta.
    """
    joint_frames, _ = fk_arm_joint_frames(q)
    p_joints = [T[:3, 3] for T in joint_frames]
    g_vec = np.array([0.0, 0.0, -_G])
    tau = np.zeros(5)
    for i in range(5):
        z_i = joint_frames[i][:3, 2]   # eixo de rotação da junta i no frame Base
        p_i = p_joints[i]
        for j in range(i + 1, 5):      # elos a jusante (CoM em p_joints[j])
            r = p_joints[j] - p_i
            tau[i] += _LINK_MASSES[j] * np.dot(g_vec, np.cross(z_i, r))
    return tau


def ik_tooltip_position(p_target_arm, q_seeds=None, max_iter=400, max_step=0.08,
                        q_current=None, continuity_weight=0.35,
                        ik_reach_tol=0.005):
    """
    IK de POSIÇÃO da ponta da ferramenta (não do T265, não pose 6D).

    Por que existe: a manipulação da chave controla só a posição da
    ponta (5 DOF não fecham pose 6D — ver fuzzy_wb_controller), e o
    braço tem múltiplos ramos de solução. Partir de uma postura fixa
    põe o DLS na bacia errada com frequência: em 2026-08-13 a missão
    estacionava a 4,5 cm do olhal com J3 cravado em −60° (o batente),
    enquanto a solução analítica do mesmo alvo pedia J3 = +31°.
    Resolver a IK antes e pré-posicionar o braço nela resolve isso —
    o controlador Cartesiano só precisa fechar o resíduo.

    CONTINUIDADE DE RAMO (2026-08-13): com várias soluções válidas, a
    escolha "melhor por erro" pulava de ramo entre execuções — a
    estimativa da parede varia alguns milímetros e a IK respondia com
    posturas completamente diferentes. Medido em duas execuções da
    missão com o mesmo código: uma escolheu [2.0, -9.2, -6.6, 58.8] e
    convergiu em 4 fases; a outra escolheu [-8.7, -29.8, 41.8, 28.1] e
    divergiu. Passando q_current, soluções próximas da postura atual são
    preferidas — os waypoints consecutivos ficam no mesmo ramo e o
    comportamento vira repetível.

    p_target_arm : (3,) posição alvo da ponta, no frame da BASE DO BRAÇO.
    q_seeds      : lista de sementes; default cobre os ramos principais.
    q_current    : postura atual; se dada, entra como primeira semente e
                   penaliza soluções distantes dela.
    continuity_weight : peso da penalidade de distância (rad → "metros
                   equivalentes" no critério de escolha).

    Retorna (q, erro_final). Multi-start: fica com o melhor resultado.
    """
    if q_seeds is None:
        q_seeds = [
            np.zeros(5),
            np.array([0.0,  0.6, -0.4, -1.2, 0.0]),   # cotovelo "para baixo"
            np.array([0.0, -0.9,  0.5,  0.7, 0.0]),   # cotovelo "para cima"
            np.array([0.0,  0.3,  0.3,  0.0, 0.0]),
            np.array([0.0, -0.3, -0.3,  0.5, 0.0]),
        ]
        if q_current is not None:
            # Semente prioritária: a própria postura atual.
            q_seeds = [np.array(q_current, dtype=float)] + q_seeds

    def _tip(q):
        return (fk_arm(q) @ T_T265_TOOLTIP)[:3, 3]

    candidatos = []
    for seed in q_seeds:
        q = np.clip(np.array(seed, dtype=float), JOINT_LOWER, JOINT_UPPER)
        for _ in range(max_iter):
            err = np.asarray(p_target_arm) - _tip(q)
            if np.linalg.norm(err) < 1e-4:
                break
            J = np.zeros((3, 5))
            for i in range(5):
                d = np.zeros(5); d[i] = IK_DQ_STEP
                J[:, i] = (_tip(q + d) - _tip(q - d)) / (2 * IK_DQ_STEP)
            dq = J.T @ np.linalg.solve(J @ J.T + IK_LAMBDA**2 * np.eye(3), err)
            n = np.max(np.abs(dq))
            if n > max_step:
                dq *= max_step / n
            q = np.clip(q + dq, JOINT_LOWER, JOINT_UPPER)
        e = float(np.linalg.norm(np.asarray(p_target_arm) - _tip(q)))
        dist = (float(np.linalg.norm(q - np.asarray(q_current, dtype=float)))
                if q_current is not None else 0.0)
        candidatos.append((e, dist, q.copy()))

    # ESCOLHA EM DOIS ESTÁGIOS. A versão anterior somava erro e distância
    # num score único (e + w·dist) — o que deixa a continuidade NEGOCIAR
    # contra a precisão: quando a solução correta exigia reconfigurar
    # bastante o braço, ela perdia para uma solução ruim que estava
    # perto, o resíduo estourava a tolerância e a missão abortava. Numa
    # bateria de repetibilidade isso derrubou 8 de 8 execuções
    # (2026-08-21).
    #
    # Continuidade é critério de DESEMPATE, não de compromisso: primeiro
    # filtra quem realmente alcança o alvo, e só entre esses escolhe o
    # mais próximo da postura atual. Sem nenhum que alcance, devolve o
    # de menor erro (quem chamou decide se serve).
    bons = [c for c in candidatos if c[0] < ik_reach_tol]
    if bons:
        e, _, q = min(bons, key=lambda c: c[1] * continuity_weight)
    else:
        e, _, q = min(candidatos, key=lambda c: c[0])
    return q, e


# ---------------------------------------------------------------------------
# IK com direção do degrau
# ---------------------------------------------------------------------------

def degrau_dir(q):
    """Direção do DEGRAU (o dedo horizontal) no frame da base do braço.

    O degrau se projeta em −X do frame `tool_tip` (ver o bloco DEDO FIXO
    em movemaster.urdf.xacro). Conferido contra a TF publicada pelo
    robot_state_publisher em 2026-08-27.
    """
    T = fk_arm(q) @ T_T265_TOOLTIP
    return -T[:3, 0]


def ik_tooltip_nivelado(p_target_arm, eixo_furo_arm, up_arm, q_seeds=None,
                        max_iter=400, max_step=0.08, q_current=None,
                        continuity_weight=0.35, ik_reach_tol=0.005,
                        peso_ang=0.05, ang_tol=0.15, peso_x=0.15,
                        sentido_fixo=False):
    """
    IK para ATRAVESSAR o furo: 2 de posição + 3 de orientação.

    POR QUE TROCAR UMA RESTRIÇÃO DE POSIÇÃO POR UMA DE ORIENTAÇÃO.
    `ik_tooltip_com_degrau` alinha o EIXO do degrau (2 restrições) e fixa
    a posição da ponta (3) — os 5 DOF do braço, todos consumidos. Sobra
    zero para o ROLL do degrau em torno do próprio eixo, que sai como o
    ramo da IK entregar.

    Medido em 2026-08-27, na fase "atravessa": eixo do degrau impecável
    (−0,2° de elevação) mas ROLADO 13,6°, o que sobe a altura efetiva
    apresentada ao furo de 17,0 para 21,2 mm. A ferramenta encostava no
    arame de cima do anel (contato medido em Z=+23 mm do centro do
    olhal, com Y centrado em −1 mm) em vez de atravessar.

    A troca vem de uma observação sobre a TAREFA, não sobre o
    controlador: a posição AO LONGO do eixo do furo é folgada — o degrau
    tem 30 mm e o arame 6 mm, então alguns milímetros a mais ou a menos
    nessa direção não mudam nada. O que precisa ser exato é a posição
    TRANSVERSAL (profundidade e altura, onde o furo dá 5 e 11,5 mm de
    folga) e a ATITUDE completa. Fica 2 + 3 = 5, e o degrau passa a ser
    nivelado por construção, como o desenho do Marco pede — é nivelado
    que ele sustenta o arame na fase de captura.

    p_target_arm  : (3,) alvo da ponta. A componente ao longo do eixo do
                    furo é IGNORADA de propósito.
    eixo_furo_arm : (3,) eixo do furo, no frame da base do braço.
    up_arm        : (3,) vertical do mundo, no frame da base do braço.
    peso_x        : peso da componente de posição AO LONGO do eixo do
                    furo (1,0 = tão importante quanto as outras).
    peso_ang      : metros equivalentes por radiano de erro de atitude.
    ang_tol       : erro de atitude (rad) aceito no filtro de candidatos.

    Retorna (q, erro_pos_m, erro_ang_rad).

    O ERRO DE POSIÇÃO DEVOLVIDO É O COMPLETO, não o ponderado.

    A ponderação (peso_x) existe para a OTIMIZAÇÃO: ela diz ao solver
    qual direção ceder primeiro. Mas devolver o valor ponderado fazia o
    número que sai daqui ser uma grandeza diferente da que a missão
    verifica contra a tolerância, e as duas metades do sistema entravam
    em impasse silencioso: medido em 2026-08-27, a IK anunciava 10 mm na
    fase 'orienta' enquanto a missão media 68 mm e reprovava — nenhuma
    das duas errando na sua própria conta. A fase não podia fechar por
    construção.

    Critério de parada e função objetivo podem ser diferentes de
    propósito, mas então precisam ter NOMES diferentes. Aqui o que sai é
    a distância de verdade até o alvo.
    """
    a = np.asarray(eixo_furo_arm, dtype=float)
    a = a / max(np.linalg.norm(a), 1e-12)
    up = np.asarray(up_arm, dtype=float)
    up = up / max(np.linalg.norm(up), 1e-12)
    alvo_p = np.asarray(p_target_arm, dtype=float)

    # PESO BAIXO no eixo do furo, NÃO projeção fora.
    #
    # A primeira versão zerava essa componente ("a posição ao longo do
    # eixo é folgada"). É verdade DENTRO de uma fase — o degrau tem 30 mm
    # e o arame 6 — mas é justamente ela que DISTINGUE as fases:
    # aproxima_lateral fica 90 mm ao lado e atravessa fica no furo, e a
    # diferença entre as duas é só X. Zerando, as duas colapsaram na
    # mesma solução (10,7 mm / 3,1° / 4,9° idênticos nas duas) e orienta,
    # arco2 e desengata estouraram para 155-180 mm de erro.
    #
    # Com peso, o problema vira mínimos quadrados ponderados: 6 vínculos
    # (3 de posição, 2 de eixo, 1 de roll) sobre 5 DOF, e quem cede
    # primeiro é a componente barata. Que é exatamente a intenção.
    W = np.eye(3) - (1.0 - peso_x) * np.outer(a, a)

    def _alvo_R(R_atual):
        """Atitude desejada do tool_tip.

        O degrau aponta em −X do tool_tip; a LARGURA (20 mm) é Y e a
        ALTURA (17 mm) é −Z. A atitude pedida põe a LARGURA na VERTICAL,
        e não a altura.

        Por quê: o furo é 30 mm na horizontal e 40 mm na vertical. Com a
        largura em pé o degrau apresenta 17 mm no vão de 30 (6,5 mm de
        folga por lado) e 20 mm no vão de 40 (10 mm) — mais equilibrado
        que o contrário, que daria 5 e 11,5 mm. E, medido em 2026-08-27,
        é a configuração que o braço ALCANÇA: pedindo a altura em pé, a
        IK não convergia e devolvia 80° de roll com 30 mm de erro
        transversal; 80° é precisamente esta orientação.

        O sinal da VERTICAL sai por proximidade da atitude atual (a
        largura em pé serve nos dois sentidos, e forçar um criaria giros
        de 180° gratuitos).

        O sinal do EIXO não: com `sentido_fixo` ele é imposto, porque a
        ferramenta tem um dedo só e os dois sentidos engatam de forma
        diferente — ver a nota extensa em ik_tooltip_com_degrau. Sem
        isso, esta formulação aceitaria o ramo espelhado exatamente como
        a outra aceitava.
        """
        if sentido_fixo:
            s = 1.0
        else:
            s = 1.0 if float((-R_atual[:, 0]) @ a) >= 0.0 else -1.0
        x = -s * a                       # degrau aponta em -X do tool_tip
        v = up - float(up @ x) * x       # vertical, ortogonalizada ao eixo
        n = np.linalg.norm(v)
        if n < 1e-6:                     # furo vertical: qualquer roll serve
            base = np.array([0.0, 0.0, 1.0])
            v = base - float(base @ x) * x
            n = np.linalg.norm(v)
        v = v / n
        t = 1.0 if float(R_atual[:, 1] @ v) >= 0.0 else -1.0
        y = t * v                        # LARGURA na vertical
        z = np.cross(x, y)
        return np.column_stack([x, y, z])

    if q_seeds is None:
        q_seeds = [
            np.zeros(5),
            np.array([0.0,  0.6, -0.4, -1.2, 0.0]),
            np.array([0.0, -0.9,  0.5,  0.7, 0.0]),
            np.array([0.0,  0.3,  0.3,  0.0, 0.0]),
            np.array([0.0, -0.3, -0.3,  0.5, 0.0]),
        ]
        q_seeds += [np.array([0.0, 0.6, -0.4, -1.2, ang])
                    for ang in (-2.36, -1.57, -0.79, 0.79, 1.57, 2.36)]
        if q_current is not None:
            q_seeds = [np.array(q_current, dtype=float)] + q_seeds

    def _T(q):
        return fk_arm(q) @ T_T265_TOOLTIP

    def _erro(q):
        T = _T(q)
        e_p = W @ (alvo_p - T[:3, 3])
        e_o = so3_log(_alvo_R(T[:3, :3]) @ T[:3, :3].T)
        return np.concatenate([e_p, peso_ang * e_o])

    def _metricas(q):
        """Métricas de AVALIAÇÃO — distância completa, não a ponderada.

        A ponderação vive só em `_erro`, que é o que o solver minimiza.
        Quem julga o resultado (o filtro de candidatos aqui e a
        tolerância da missão lá fora) usa a distância real.
        """
        T = _T(q)
        e_p = float(np.linalg.norm(alvo_p - T[:3, 3]))
        e_o = float(np.linalg.norm(so3_log(_alvo_R(T[:3, :3]) @ T[:3, :3].T)))
        return e_p, e_o

    candidatos = []
    for seed in q_seeds:
        q = np.clip(np.array(seed, dtype=float), JOINT_LOWER, JOINT_UPPER)
        for _ in range(max_iter):
            err = _erro(q)
            if np.linalg.norm(err) < 1e-4:
                break
            Jm = np.zeros((6, 5))
            for i in range(5):
                d = np.zeros(5); d[i] = IK_DQ_STEP
                Jm[:, i] = (_erro(q - d) - _erro(q + d)) / (2 * IK_DQ_STEP)
            dq = np.linalg.solve(Jm.T @ Jm + IK_LAMBDA**2 * np.eye(5),
                                 Jm.T @ err)
            m = np.max(np.abs(dq))
            if m > max_step:
                dq *= max_step / m
            q = np.clip(q + dq, JOINT_LOWER, JOINT_UPPER)
        e_p, e_o = _metricas(q)
        dist = (float(np.linalg.norm(q - np.asarray(q_current, dtype=float)))
                if q_current is not None else 0.0)
        candidatos.append((e_p, e_o, dist, q.copy()))

    # Mesmos três estágios de ik_tooltip_com_degrau, pelo mesmo motivo:
    # continuidade só decide DEPOIS do filtro rígido, senão o desempate
    # some e a IK alterna entre soluções espelhadas.
    bons = [c for c in candidatos if c[0] < ik_reach_tol and c[1] < ang_tol]
    if bons:
        e_p, e_o, _, q = min(bons, key=lambda c: c[2] * continuity_weight)
    else:
        alcancam = [c for c in candidatos if c[0] < ik_reach_tol]
        if alcancam:
            e_p, e_o, _, q = min(alcancam,
                                 key=lambda c: (c[1], c[2] * continuity_weight))
        else:
            e_p, e_o, _, q = min(candidatos,
                                 key=lambda c: (c[0], c[2] * continuity_weight))
    return q, e_p, e_o


def ik_tooltip_com_degrau(p_target_arm, eixo_furo_arm, q_seeds=None,
                          max_iter=400, max_step=0.08, q_current=None,
                          continuity_weight=0.35, ik_reach_tol=0.005,
                          peso_dir=0.05, ang_tol=0.20, sentido_fixo=False):
    """
    IK de POSIÇÃO da ponta + DIREÇÃO do degrau.

    POR QUE EXISTE. `ik_tooltip_position` controla só a posição, e a
    atitude da ferramenta é o que o ramo da IK calhar de entregar.
    Medido pela TF em 2026-08-27, na fase "atravessa": o degrau apontava
    para [−0,02 −0,78 −0,62] — para fora da parede e 38° abaixo da
    horizontal — quando precisava estar paralelo ao eixo do furo. A
    ferramenta chegava ao anel girada ~90° do projeto e, mesmo com mira
    perfeita, bateria de lado em vez de atravessar. Foi isso, e não o
    viés de mira, que manteve a lâmina em 0,0° nas 8 execuções da
    bateria de 2026-08-27.

    CONTAGEM DE GRAUS DE LIBERDADE. O braço tem 5. Posição da ponta
    consome 3; a direção de um eixo consome 2 — total 5, exatamente o
    que há. Por isso NÃO dá para exigir também a haste vertical: pose 6D
    precisaria de 6 DOF e o RV-M2 não tem (é a mesma limitação
    registrada em fuzzy_wb_controller). A restrição que a tarefa
    realmente precisa é a do degrau: é ele que atravessa o furo.

    EIXO OU VETOR — `sentido_fixo` DECIDE, E PARA ESTA TAREFA É VETOR.
    Com `sentido_fixo=False` (default histórico), `eixo_furo_arm` é
    tratado como EIXO: a cada iteração escolhe-se o sentido (±) mais
    próximo da atitude atual. O raciocínio era que atravessar o furo
    funciona nos dois sentidos, e que fixar um sinal criaria giros de
    180° gratuitos — o atrator de 180° que já custou caro em
    `pose_error`.

    O RACIOCÍNIO ESTAVA ERRADO PARA ESTA FERRAMENTA. Ele vale para uma
    haste simétrica. A do b166er tem UM DEDO SÓ (a garra perdeu o
    segundo dedo no modelo em 2026-08-25, conferindo com as fotos), e
    então os dois sentidos NÃO são equivalentes: num deles o degrau
    engata o olhal, no outro aponta para fora e só consegue empurrar.

    MEDIDO em 2026-08-31, com a mesma geometria de entrada em duas
    execuções seguidas:

      ramo A (J5 = -1°)    degrau -> [-0.216  +0.976  +0.016]
      ramo B (J5 = -179°)  degrau -> [-0.108  -0.994  +0.015]
      produto escalar -0,947 — 161° entre eles, LADOS OPOSTOS

    Nos dois a IK reportava resíduo 0,0000 m e "degrau 0,9°", porque o
    ângulo era medido contra o EIXO, sem sinal. Para a IK, o ramo
    espelhado era uma solução perfeita. Isso é candidato a explicar a
    observação do Marco de que "a ferramenta não engatou no olhal, ela
    puxou por trás do olhal": não era erro de milímetros, era o dedo do
    lado errado.

    QUAL É O SINAL CERTO. O de inserção: as fases vão de "orienta" em
    x = -0,090 (fora, do lado do robô) para "atravessa" em x = 0, ou
    seja no sentido +X do frame da parede. É o vetor que o chamador já
    passa como `eixo_furo_arm`. Verificado contra o ramo A, que produziu
    as melhores execuções: produto escalar 0,997 com +X da parede.

    Fixar o sinal também elimina metade da instabilidade de ramo de
    graça — o ramo espelhado deixa de ser solução admissível, então não
    há mais o que sortear entre execuções.

    p_target_arm  : (3,) posição alvo da ponta, no frame da base do braço.
    eixo_furo_arm : (3,) eixo do furo do olhal, no mesmo frame. Não
                    precisa ser unitário nem ter sentido definido.
    peso_dir      : metros equivalentes por unidade de erro de direção.
                    0,05 faz 90° de desalinho pesar como ~7 cm.
    ang_tol       : erro angular (rad) abaixo do qual a solução conta
                    como alinhada, para o filtro de candidatos.

    Retorna (q, erro_pos_m, erro_ang_rad).
    """
    eixo = np.asarray(eixo_furo_arm, dtype=float)
    n = np.linalg.norm(eixo)
    if n < 1e-9:
        raise ValueError('eixo_furo_arm nulo')
    eixo = eixo / n
    alvo_p = np.asarray(p_target_arm, dtype=float)

    if q_seeds is None:
        q_seeds = [
            np.zeros(5),
            np.array([0.0,  0.6, -0.4, -1.2, 0.0]),
            np.array([0.0, -0.9,  0.5,  0.7, 0.0]),
            np.array([0.0,  0.3,  0.3,  0.0, 0.0]),
            np.array([0.0, -0.3, -0.3,  0.5, 0.0]),
        ]
        # Sementes com J5 girado: o degrau gira principalmente com J5, e
        # sem estas o multi-start nasce todo na mesma atitude errada.
        q_seeds += [np.array([0.0, 0.6, -0.4, -1.2, a])
                    for a in (-2.36, -1.57, -0.79, 0.79, 1.57, 2.36)]
        if q_current is not None:
            q_seeds = [np.array(q_current, dtype=float)] + q_seeds

    def _feat(q):
        T = fk_arm(q) @ T_T265_TOOLTIP
        return T[:3, 3], -T[:3, 0]

    def _erro(q):
        p, d = _feat(q)
        if sentido_fixo:
            alvo_d = eixo
        else:
            alvo_d = eixo if float(d @ eixo) >= 0.0 else -eixo
        return np.concatenate([alvo_p - p, peso_dir * (alvo_d - d)])

    def _metricas(q):
        p, d = _feat(q)
        # COM SINAL quando sentido_fixo: sem isso, o ramo espelhado
        # (d ≈ -eixo) devolveria cos ≈ 1 e passaria como "alinhado" pelo
        # filtro de candidatos, que é exatamente o defeito corrigido.
        cos = float(d @ eixo) if sentido_fixo else abs(float(d @ eixo))
        return (float(np.linalg.norm(alvo_p - p)),
                float(np.arccos(np.clip(cos, -1.0, 1.0))))

    candidatos = []
    for seed in q_seeds:
        q = np.clip(np.array(seed, dtype=float), JOINT_LOWER, JOINT_UPPER)
        for _ in range(max_iter):
            err = _erro(q)
            if np.linalg.norm(err) < 1e-4:
                break
            # Jacobiano numérico do VETOR DE FEIÇÕES (posição + direção),
            # 6x5. Com J "alta" (mais linhas que colunas) a forma certa
            # do DLS é (JᵀJ + λ²I)⁻¹Jᵀe, não a usada na IK de posição
            # pura — lá J é 3x5 e a outra forma é a barata.
            Jm = np.zeros((6, 5))
            for i in range(5):
                dq_i = np.zeros(5); dq_i[i] = IK_DQ_STEP
                p1, d1 = _feat(q + dq_i)
                p0, d0 = _feat(q - dq_i)
                Jm[:3, i] = (p1 - p0) / (2 * IK_DQ_STEP)
                Jm[3:, i] = peso_dir * (d1 - d0) / (2 * IK_DQ_STEP)
            dq = np.linalg.solve(Jm.T @ Jm + IK_LAMBDA**2 * np.eye(5),
                                 Jm.T @ err)
            m = np.max(np.abs(dq))
            if m > max_step:
                dq *= max_step / m
            q = np.clip(q + dq, JOINT_LOWER, JOINT_UPPER)
        e_p, e_a = _metricas(q)
        dist = (float(np.linalg.norm(q - np.asarray(q_current, dtype=float)))
                if q_current is not None else 0.0)
        candidatos.append((e_p, e_a, dist, q.copy()))

    # ALCANCE PRIMEIRO, SEMPRE. O alinhamento é preferência ENTRE as
    # soluções que alcançam o ponto, nunca alternativa a alcançá-lo.
    #
    # A primeira versão disto invertia a ordem: sem candidato que
    # fechasse as duas coisas, escolhia pelo menor ângulo. Parecia
    # razoável ("chegar perto com o degrau torto não engata") e derrubou
    # o robô na primeira execução, 2026-08-27: aceitou uma solução com
    # 330 mm de erro de posição por estar 7° melhor alinhada, num ramo
    # com J4=+82° que enfia o braço na parede. A IMU acusou inclinação
    # crítica. Um erro de posição sem limite superior é sempre pior que
    # um erro de orientação, porque orientação errada não move o robô
    # para dentro de um obstáculo.
    # TRÊS ESTÁGIOS, e a ordem importa.
    #
    # A versão anterior ordenava por (ângulo, continuidade) como TUPLA.
    # Isso apaga o desempate: dois candidatos com 0,005 e 0,006 rad de
    # desalinho são "diferentes", o ângulo decide sozinho e a
    # continuidade nunca é consultada. Como a restrição é de EIXO, existe
    # sempre um par de soluções espelhadas (J5 defasado de 180°, J2 e J4
    # invertidos) que alinham igualmente bem — e a IK passou a alternar
    # entre elas a cada iteração.
    #
    # Medido em 2026-08-27, fase "captura": J5 saltando entre +1,5° e
    # -179,4° em iterações consecutivas, com o braço tentando girar 180°
    # POR DENTRO do mecanismo — defasagens de 105° e 164° em J4, dezenas
    # de milhares de contatos ferramenta x bracket, e o erro crescendo de
    # 34 mm para 293 mm.
    #
    # É a mesma lição já registrada em `ik_tooltip_position`:
    # continuidade é DESEMPATE, não compromisso. Filtro rígido primeiro,
    # continuidade pura depois.
    bons = [c for c in candidatos
            if c[0] < ik_reach_tol and c[1] < ang_tol]
    if bons:
        # Alcança E está alinhada: entre essas, a mais próxima da postura
        # atual. É isto que impede o salto entre as duas soluções
        # espelhadas.
        e_p, e_a, _, q = min(bons, key=lambda c: c[2] * continuity_weight)
    else:
        alcancam = [c for c in candidatos if c[0] < ik_reach_tol]
        if alcancam:
            # Alcança mas nenhuma alinhada o bastante: a mais alinhada.
            e_p, e_a, _, q = min(alcancam,
                                 key=lambda c: (c[1], c[2] * continuity_weight))
        else:
            # Nenhuma alcança. ALCANCE PRIMEIRO, SEMPRE — priorizar o
            # alinhamento aqui já derrubou o robô uma vez (330 mm de erro
            # de posição por 7° de alinhamento, braço dentro da parede).
            e_p, e_a, _, q = min(candidatos,
                                 key=lambda c: (c[0], c[2] * continuity_weight))
    return q, e_p, e_a
