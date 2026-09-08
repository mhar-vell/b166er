#!/usr/bin/env python3
"""A função dos arcos: olhal na ponta da lâmina (L = 0,2 m) girando em
torno do pivô, no frame da parede [eixo, prof, alt]; waypoints do YAML e
ângulos medidos por contato (run15)."""
import math, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Arc

L = 0.200                      # BLADE_LENGTH (chave_task.py, estimado)
CAPTURA = -0.010               # alt do tool_tip na captura (YAML, 2026-09-02)
WP = {  # fase: (theta_deg, prof, alt) — offset_xyz_m do YAML [eixo, prof, alt]
    "captura": (0.0, 0.0, -0.0100),
    "arco1":   (15.0, 0.0518, -0.0098),
    "arco2":   (30.0, 0.1000, -0.0298),
}
MEDIDO = {"arco1": 10.8, "arco2": 26.7}   # lâmina medida por contato, run15
th = np.radians(np.linspace(0, 40, 401))
prof = L * np.sin(th); alt = L * (np.cos(th) - 1)

fig = plt.figure(figsize=(15, 6.4))
gs = fig.add_gridspec(1, 2, width_ratios=[1.15, 1])
# ── A) geometria: pivô, lâmina, olhal e a trajetória da ferramenta ──
ax = fig.add_subplot(gs[0])
piv = np.array([0.0, -L])
for ang, cor, lab in ((0, "#333", "fechada 0°"), (15, "#1f77b4", "15°"), (30, "#d62728", "30°")):
    a = math.radians(ang); tip = np.array([L * math.sin(a), L * (math.cos(a) - 1)])
    ax.plot([piv[0], tip[0]], [piv[1], tip[1]], "-", color=cor, lw=3, alpha=.8, solid_capstyle="round")
    ax.plot(tip[0], tip[1], "o", color=cor, ms=8, mfc="white", mew=2)
    ax.annotate(lab, tip, xytext=(8, 4), textcoords="offset points", fontsize=9, color=cor)
ax.plot(prof, alt, "--", color="#555", lw=1.2, label="olhal: prof = L·sen θ,  alt = L·(cos θ − 1)")
# waypoints da ferramenta (tool_tip)
wpx = [v[1] for v in WP.values()]; wpz = [v[2] for v in WP.values()]
ax.plot(wpx, wpz, "s-", color="#2ca02c", ms=7, lw=1.5, label="waypoints do tool_tip (YAML)")
for k, (t, p, z) in WP.items():
    ax.annotate("%s\n(%.4f, %.4f)" % (k, p, z), (p, z), xytext=(8, -26), textcoords="offset points", fontsize=8, color="#2ca02c")
# a mesma curva do olhal deslocada da captura (onde a ferramenta DEVERIA correr)
ax.plot(prof, alt + CAPTURA, ":", color="#2ca02c", lw=1.2, label="olhal − 10 mm (captura de hoje)")
# ângulos medidos
for k, m in MEDIDO.items():
    a = math.radians(m); tip = np.array([L * math.sin(a), L * (math.cos(a) - 1)])
    ax.plot(tip[0], tip[1], "x", color="#d62728", ms=10, mew=2)
    ax.annotate("medido %.1f°" % m, tip, xytext=(6, 8), textcoords="offset points", fontsize=8, color="#d62728")
ax.plot(piv[0], piv[1], "k.", ms=10); ax.annotate("pivô da lâmina", piv, xytext=(8, -4), textcoords="offset points", fontsize=8)
ax.axvline(0, color="#999", lw=1); ax.text(-0.004, 0.02, "placa da chave", rotation=90, fontsize=8, color="#666", ha="right")
ax.set_xlabel("prof (m) — para fora da parede, em direção ao robô →"); ax.set_ylabel("alt (m) — em relação ao olhal fechado")
ax.set_title("A) o arco: lâmina de 0,2 m girando no pivô (frame da parede)", fontsize=10, loc="left")
ax.set_aspect("equal"); ax.grid(alpha=.3); ax.set_xlim(-0.03, 0.16); ax.set_ylim(-0.215, 0.04); ax.legend(fontsize=8, loc="lower right")

# ── B) a função: prof(θ) e alt(θ) ──
ax2 = fig.add_subplot(gs[1])
deg = np.degrees(th)
ax2.plot(deg, prof * 1000, "-", color="#1f77b4", lw=2, label="prof(θ) = L·sen θ")
ax2.plot(deg, alt * 1000, "-", color="#d62728", lw=2, label="alt(θ) = L·(cos θ − 1)")
ax2.plot(deg, (alt + CAPTURA) * 1000, ":", color="#d62728", lw=1.2, label="alt(θ) − 10 mm (tool_tip)")
for k, (t, p, z) in WP.items():
    if t > 0:
        ax2.plot(t, p * 1000, "s", color="#1f77b4", ms=7, mfc="white", mew=2); ax2.plot(t, z * 1000, "s", color="#d62728", ms=7, mfc="white", mew=2)
        ax2.annotate("%s: %+.1f / %+.1f mm" % (k, p * 1000, z * 1000), (t, p * 1000), xytext=(6, 6), textcoords="offset points", fontsize=8)
for k, m in MEDIDO.items():
    ax2.axvline(m, color="#999", lw=1, ls="--"); ax2.text(m + 0.4, 95, "medido\n%.1f°" % m, fontsize=8, color="#666")
ax2.axvline(15, color="#1f77b4", lw=.6, alpha=.4); ax2.axvline(30, color="#d62728", lw=.6, alpha=.4)
ax2.set_xlabel("ângulo da lâmina θ (graus)"); ax2.set_ylabel("deslocamento do olhal (mm)")
ax2.set_title("B) a função: prof e alt do olhal contra o ângulo da lâmina", fontsize=10, loc="left")
ax2.grid(alpha=.3); ax2.legend(fontsize=8, loc="center left"); ax2.set_xlim(0, 40)
# nota sobre a inconsistência dos offsets
ax2.text(0.5, -52, "waypoints do YAML em alt: arco1 −9,8 = L(cos15°−1) − 3 mm; arco2 −29,8 = L(cos30°−1) − 3 mm\n"
         "→ os −3 mm são a captura ANTIGA (degrau de 17 mm). Com a captura de hoje (−10 mm)\n"
         "seriam −16,8 e −36,8 mm. A tolerância de 15 mm em alt absorveu a diferença nas 7 missões.",
         fontsize=8, color="#7a3e00", bbox=dict(boxstyle="round", fc="#fff8ee", ec="#c98a3a", lw=.8))
fig.suptitle("Arcos da abertura da chave: o olhal percorre um círculo de raio L = 0,2 m em torno do pivô; a ferramenta segue 10 mm abaixo", fontsize=10.5)
fig.tight_layout(rect=[0, 0, 1, .95])
out = "/home/marco/.claude/jobs/89ade7b6/tmp/revalida/arcos_funcao.png"; fig.savefig(out, dpi=125); print("figura:", out)
for k, (t, p, z) in WP.items():
    if t > 0:
        a = math.radians(t); print("%s @%2.0f°: YAML prof %.4f alt %.4f | L·sen %.4f  L(cos−1) %.4f  → alt −10 mm = %.4f" % (k, t, p, z, L*math.sin(a), L*(math.cos(a)-1), L*(math.cos(a)-1)+CAPTURA))
