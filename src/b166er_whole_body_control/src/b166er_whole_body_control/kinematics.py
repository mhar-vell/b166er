"""
kinematics.py — cinemática do b166er (Pioneer + RV-M2) em numpy puro.

Cadeia completa:
  world → base_link → top_plate → Base(braço) → J1 → L1 → J2 → L2
        → J3 → L3 → J4 → L4 → J5 → L5 → CameraSupport → t265_link

8-DOF whole-body: q_wb = [x_b, y_b, θ_b,  q1, q2, q3, q4, q5]
                          ← 3 base →       ←— 5 braço —→
"""

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
_T_L5_T265     = _trans(0, 0, -0.08) @ _tf([0, 0.075, -0.07], [0, 2.1, 1.57])

# Ferramenta de operação (vara + gancho, ver movemaster.urdf.xacro,
# JTool/JToolTip) — mesma cadeia rígida do L5 até a ponta do gancho,
# via GripCube (JCam + JGripCube + JTool + JToolTip, todas fixed).
_T_L5_TOOLTIP = (_trans(0, 0, -0.08)    # JCam:      L5 → CameraSupport
                @ _trans(0, 0, -0.005)  # JGripCube: CameraSupport → GripCube
                @ _trans(0, 0, -0.08)   # JTool:     GripCube → tool_rod
                @ _trans(0, 0, -0.20))  # JToolTip:  tool_rod → tool_tip (20cm, reduzida de 35cm em 2026-08-13)

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


def pose_error(T_current, T_target):
    """
    Erro 6-vetor [Δp, Δω] de SE(3).
    Δp  = p_target − p_current
    Δω  = parte vetorial de R_err = R_target · R_current^T
    """
    dp    = T_target[:3, 3] - T_current[:3, 3]
    R_err = T_target[:3, :3] @ T_current[:3, :3].T
    dw    = np.array([R_err[2,1]-R_err[1,2],
                      R_err[0,2]-R_err[2,0],
                      R_err[1,0]-R_err[0,1]]) * 0.5
    return np.concatenate([dp, dw])


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
                        q_current=None, continuity_weight=0.35):
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

    best_q, best_err, best_score = None, np.inf, np.inf
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
        # Critério de escolha: erro de posição + penalidade de
        # continuidade. Soluções que exigem reconfigurar o braço inteiro
        # perdem para soluções equivalentes perto de onde ele já está.
        score = e
        if q_current is not None:
            score += continuity_weight * float(
                np.linalg.norm(q - np.asarray(q_current, dtype=float)))
        if score < best_score:
            best_q, best_err, best_score = q.copy(), e, score
    return best_q, best_err
