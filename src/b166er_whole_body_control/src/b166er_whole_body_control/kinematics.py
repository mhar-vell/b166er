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
    DLS com Jacobiano central de diferenças finitas.

    Retorna (q, converged, res_pos, res_orient).
    """
    q = np.zeros(5) if q_init is None else np.array(q_init, dtype=float)

    for _ in range(IK_MAX_ITER):
        T_cur  = fk_arm(q)
        err    = pose_error(T_cur, T_target)
        rp, ro = np.linalg.norm(err[:3]), np.linalg.norm(err[3:])

        if rp < IK_TOL_POS and ro < IK_TOL_ORIENT:
            return q, True, rp, ro

        # Jacobiano de FK (diferenças centrais)
        J = np.zeros((6, 5))
        for i in range(5):
            dq_i    = np.zeros(5); dq_i[i] = IK_DQ_STEP
            J[:, i] = (_T_to_6vec(fk_arm(q + dq_i))
                       - _T_to_6vec(fk_arm(q - dq_i))) / (2 * IK_DQ_STEP)

        dq = J.T @ np.linalg.solve(J @ J.T + IK_LAMBDA**2 * np.eye(6), err)
        q  = np.clip(q + dq, JOINT_LOWER, JOINT_UPPER)

    T_cur = fk_arm(q); err = pose_error(T_cur, T_target)
    return q, False, np.linalg.norm(err[:3]), np.linalg.norm(err[3:])
