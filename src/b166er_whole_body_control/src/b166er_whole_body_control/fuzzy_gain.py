"""
fuzzy_gain.py — Mamdani fuzzy gain scheduler para o controlador whole-body.

Substitui o ganho fixo K do whole_body_planner por um ganho adaptativo
que ajusta velocidade e amortecimento DLS em função do erro e sua variação.

Entradas
--------
err_pos    : norma do erro de posição  (m)       ∈ [0, ∞)
err_orient : norma do erro de orientação (rad)   ∈ [0, π]
delta_err  : d(err_total)/dt  (m/s, ≈ rad/s)    ∈ (-∞, ∞)
             negativo → erro diminuindo (melhorando)
             positivo → erro aumentando (piorando)

Saídas
------
k_pos      : ganho de velocidade linear  ∈ [0.1, 2.0]
k_orient   : ganho de velocidade angular ∈ [0.1, 2.0]
lambda_dls : amortecimento DLS           ∈ [0.01, 0.25]

Regras (Mamdani, AND = mín, agregação = máx, defuzz = centróide)
----------------------------------------------------------------
err_pos / delta_err   IMPROVING   STABLE    WORSENING
─────────────────────────────────────────────────────
FAR    k_pos           FAST        FAST      MED
       lambda           LOW         MED      HIGH

MED    k_pos           MED         MED       SLOW
       lambda           LOW         MED      HIGH

NEAR   k_pos           SLOW        SLOW      SLOW
       lambda           LOW         LOW       MED

err_orient → k_orient:   FAR → FAST  |  MED → MED  |  NEAR → SLOW
"""

import numpy as np


# ---------------------------------------------------------------------------
# Funções de pertinência (membership functions)
# ---------------------------------------------------------------------------

def trimf(x, params):
    """Triangular: params = [a, b, c]."""
    a, b, c = params
    return float(np.clip(
        np.minimum((x - a) / (b - a + 1e-12),
                   (c - x) / (c - b + 1e-12)), 0, 1))


def trapmf(x, params):
    """Trapezoidal: params = [a, b, c, d]."""
    a, b, c, d = params
    return float(np.clip(
        np.minimum(
            np.minimum((x - a) / (b - a + 1e-12), 1.0),
            (d - x) / (d - c + 1e-12)), 0, 1))


# ---------------------------------------------------------------------------
# Universos de discurso e funções de pertinência por variável
# ---------------------------------------------------------------------------

# --- err_pos (m) ---
def _ep_near(x):   return trapmf(x, [0.00, 0.00, 0.03, 0.08])
def _ep_med(x):    return trimf(x,  [0.04, 0.12, 0.28])
def _ep_far(x):    return trapmf(x, [0.18, 0.35, 1.00, 1.00])

# --- err_orient (rad) ---
def _eo_near(x):   return trapmf(x, [0.00, 0.00, 0.05, 0.15])
def _eo_med(x):    return trimf(x,  [0.08, 0.20, 0.45])
def _eo_far(x):    return trapmf(x, [0.25, 0.55, 3.14, 3.14])

# --- delta_err (m/s), negativo = melhorando ---
def _de_impr(x):   return trapmf(x, [-0.50, -0.50, -0.005, 0.00])
def _de_stab(x):   return trimf(x,  [-0.005, 0.00,  0.010])
def _de_wors(x):   return trapmf(x, [ 0.005, 0.02,  0.50, 0.50])

# --- k_pos / k_orient (universo [0, 2]) ---
_K_RANGE  = np.linspace(0.1, 2.0, 200)
def _k_slow(x):    return trapmf(x, [0.10, 0.10, 0.35, 0.60])
def _k_med(x):     return trimf(x,  [0.40, 0.80, 1.20])
def _k_fast(x):    return trapmf(x, [0.90, 1.40, 2.00, 2.00])

# --- lambda_dls (universo [0.01, 0.25]) ---
_L_RANGE  = np.linspace(0.01, 0.25, 200)
def _l_low(x):     return trapmf(x, [0.010, 0.010, 0.03, 0.06])
def _l_med(x):     return trimf(x,  [0.040, 0.080, 0.14])
def _l_high(x):    return trapmf(x, [0.100, 0.180, 0.25, 0.25])


# ---------------------------------------------------------------------------
# Motor de inferência Mamdani
# ---------------------------------------------------------------------------

def _centroid(universe, mf_values):
    """Defuzzificação por centróide."""
    denom = np.sum(mf_values)
    if denom < 1e-10:
        return float(universe[len(universe) // 2])
    return float(np.sum(universe * mf_values) / denom)


def _mamdani_kpos(ep_near, ep_med, ep_far, de_impr, de_stab, de_wors):
    """9 regras para k_pos."""
    rules = [
        # (ativação,                      consequente)
        (min(ep_far,  de_impr), _k_fast),
        (min(ep_far,  de_stab), _k_fast),
        (min(ep_far,  de_wors), _k_med),
        (min(ep_med,  de_impr), _k_med),
        (min(ep_med,  de_stab), _k_med),
        (min(ep_med,  de_wors), _k_slow),
        (min(ep_near, de_impr), _k_slow),
        (min(ep_near, de_stab), _k_slow),
        (min(ep_near, de_wors), _k_slow),
    ]
    agg = np.zeros_like(_K_RANGE)
    for strength, mf in rules:
        agg = np.maximum(agg, np.minimum(strength,
                                          np.vectorize(mf)(_K_RANGE)))
    return _centroid(_K_RANGE, agg)


def _mamdani_korient(eo_near, eo_med, eo_far):
    """3 regras para k_orient."""
    rules = [
        (eo_far,  _k_fast),
        (eo_med,  _k_med),
        (eo_near, _k_slow),
    ]
    agg = np.zeros_like(_K_RANGE)
    for strength, mf in rules:
        agg = np.maximum(agg, np.minimum(strength,
                                          np.vectorize(mf)(_K_RANGE)))
    return _centroid(_K_RANGE, agg)


def _mamdani_lambda(ep_near, ep_med, ep_far, de_impr, de_stab, de_wors):
    """9 regras para lambda_dls."""
    rules = [
        (min(ep_far,  de_impr), _l_low),
        (min(ep_far,  de_stab), _l_med),
        (min(ep_far,  de_wors), _l_high),
        (min(ep_med,  de_impr), _l_low),
        (min(ep_med,  de_stab), _l_med),
        (min(ep_med,  de_wors), _l_high),
        (min(ep_near, de_impr), _l_low),
        (min(ep_near, de_stab), _l_low),
        (min(ep_near, de_wors), _l_med),
    ]
    agg = np.zeros_like(_L_RANGE)
    for strength, mf in rules:
        agg = np.maximum(agg, np.minimum(strength,
                                          np.vectorize(mf)(_L_RANGE)))
    return _centroid(_L_RANGE, agg)


# ---------------------------------------------------------------------------
# Interface pública
# ---------------------------------------------------------------------------

class FuzzyGain:
    """
    Ganho adaptativo Mamdani para o controlador whole-body.

    Uso
    ---
    fg = FuzzyGain()
    k_pos, k_orient, lam = fg.compute(err_pos, err_orient, delta_err)
    """

    def compute(self, err_pos: float, err_orient: float,
                delta_err: float = 0.0):
        """
        Parâmetros
        ----------
        err_pos    : norma do erro de posição (m)
        err_orient : norma do erro de orientação (rad)
        delta_err  : d(||err||)/dt  (m/s) — negativo = melhorando

        Retorna
        -------
        k_pos    : float  — ganho de posição
        k_orient : float  — ganho de orientação
        lam      : float  — amortecimento DLS (lambda)
        """
        # Fuzzificação
        ep_near = _ep_near(err_pos)
        ep_med  = _ep_med(err_pos)
        ep_far  = _ep_far(err_pos)

        eo_near = _eo_near(err_orient)
        eo_med  = _eo_med(err_orient)
        eo_far  = _eo_far(err_orient)

        de_impr = _de_impr(delta_err)
        de_stab = _de_stab(delta_err)
        de_wors = _de_wors(delta_err)

        # Inferência + defuzzificação
        k_pos    = _mamdani_kpos(ep_near, ep_med, ep_far,
                                  de_impr, de_stab, de_wors)
        k_orient = _mamdani_korient(eo_near, eo_med, eo_far)
        lam      = _mamdani_lambda(ep_near, ep_med, ep_far,
                                    de_impr, de_stab, de_wors)

        return k_pos, k_orient, lam
