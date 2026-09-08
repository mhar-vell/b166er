#!/usr/bin/env python3
"""v2 (2026-09-03), depois dos desenhos, fotos e vídeo IMG_0525 do Marco.
A) mecanismo como visto no vídeo; B) sequência; C) fases propostas do tool_tip."""
import math, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Ellipse, FancyArrowPatch, Polygon, Circle

L = 0.200; D = 0.015; CAPT = -0.010
fig = plt.figure(figsize=(16, 12)); gs = fig.add_gridspec(2, 4, height_ratios=[1.15, 1.0])

def chave(ax, x0, th_lamina, pawl_down, cor="#333", alpha=1.0, rotulos=False):
    """Vista lateral, parede à esquerda (prof cresce para a direita). Pivô da lâmina em (x0, -L).
    A lingueta (pawl) gira num pino na lâmina, ~45 mm abaixo do olhal; carrega a ABA (para a parede) e o OLHAL (para fora)."""
    piv = np.array([x0, -L]); u = np.array([math.sin(th_lamina), math.cos(th_lamina)]); n = np.array([math.cos(th_lamina), -math.sin(th_lamina)])
    top = piv + u * (L + 0.07)
    ax.plot([piv[0], top[0]], [piv[1], top[1]], "-", color=cor, lw=7, alpha=alpha, solid_capstyle="butt")
    ax.plot(piv[0], piv[1], "o", color="k", ms=6)
    # contato fixo + barra com laço (fixos na parede)
    ax.add_patch(Rectangle((x0 - 0.045, 0.02), 0.03, 0.075, fc="#c9c9c9", ec="#666", alpha=.9))
    laco = np.array([x0 - 0.015, 0.012]); ax.add_patch(Ellipse(laco, 0.030, 0.014, fc="none", ec="#777", lw=4, alpha=.9))  # laço em U visto de lado
    # lingueta: pino a 45 mm abaixo do olhal, gira 'pawl_down' rad
    pino = piv + u * (L - 0.045); a = -pawl_down
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    def P(dl, dn):  # ponto no frame da lâmina (ao longo, normal p/ fora), rodado pela lingueta em torno do pino
        v = R @ (u * dl + n * dn); return pino + v
    aba = [P(0.045, -0.012), P(0.052, -0.040), P(0.040, -0.040), P(0.036, -0.012)]
    ax.add_patch(Polygon(aba, closed=True, fc="#444", ec="k", alpha=alpha))
    olhal_c = P(0.045, 0.045); ax.add_patch(Ellipse(olhal_c, 0.032, 0.046, fc="none", ec="#b8860b", lw=3, alpha=alpha))
    ax.plot([pino[0], olhal_c[0]], [pino[1], olhal_c[1]], "-", color="#b8860b", lw=3, alpha=alpha)
    ax.plot(pino[0], pino[1], "o", color="#b8860b", ms=5)
    ax.plot([pino[0] - 0.01, pino[0] - 0.01 - 0.012 * math.cos(a)], [pino[1] - 0.02, pino[1] - 0.02 - 0.012 * math.sin(a)], "-", color="#888", lw=1.5)
    if rotulos:
        ax.annotate("contato fixo", (x0 - 0.03, 0.10), fontsize=8, ha="center")
        ax.annotate("barra com LAÇO (fixa)\n— prende a aba", laco, xytext=(-0.12, 0.13), textcoords="data", fontsize=8, arrowprops=dict(arrowstyle="->"))
        ax.annotate("ABA da lingueta\n(dentro do laço = travada)", aba[1], xytext=(-0.13, -0.10), fontsize=8, arrowprops=dict(arrowstyle="->"))
        ax.annotate("OLHAL (latão), na lingueta,\nnão na lâmina", olhal_c, xytext=(0.06, 0.06), fontsize=8, color="#8a6508", arrowprops=dict(arrowstyle="->", color="#8a6508"))
        ax.annotate("pino da lingueta\n+ molas de arame", pino, xytext=(0.06, -0.09), fontsize=8, arrowprops=dict(arrowstyle="->"))
        ax.annotate("lâmina (garfo)", piv + u * 0.10, xytext=(0.05, -0.16), fontsize=8, arrowprops=dict(arrowstyle="->"))
        ax.annotate("pivô", piv, xytext=(0.03, -0.215), fontsize=8, arrowprops=dict(arrowstyle="->"))
    return olhal_c

ax = fig.add_subplot(gs[0, :2]); ax.set_aspect("equal")
chave(ax, 0.0, 0.0, 0.0, rotulos=True)
ax.set_xlim(-0.18, 0.2); ax.set_ylim(-0.24, 0.16); ax.axvline(-0.06, color="#bbb", lw=8, alpha=.4); ax.text(-0.062, -0.23, "parede", rotation=90, fontsize=8, color="#777", ha="right")
ax.set_title("A) o mecanismo, como o vídeo IMG_0525 mostra (vista lateral)", fontsize=10, loc="left"); ax.set_xticks([]); ax.set_yticks([])
ax.text(-0.17, 0.145, "Fechada: a aba da lingueta está DENTRO do laço da barra fixa.\nA lâmina não gira porque a aba não passa pelo laço.", fontsize=8, va="top", bbox=dict(fc="#fff8e6", ec="#c98a3a", lw=.6))

# ── B) sequência ──
etapas = [(0.0, 0.0, "1) travada:\naba dentro do laço"), (0.0, 0.34, "2) PUXA o olhal em −Z (~%d mm):\na lingueta gira, a aba sai do laço" % (D * 1000)),
          (0.14, 0.34, "3) segurando embaixo, gira:\na aba passa POR BAIXO do laço"), (0.40, 0.0, "4) livre: a mola volta a lingueta;\no arco segue (vídeo: até ~45°)")]
for k, (th, pd, lab) in enumerate(etapas):
    ax = fig.add_subplot(gs[1, k]); ax.set_aspect("equal")
    oc = chave(ax, 0.0, th, pd)
    if pd > 0: ax.add_patch(FancyArrowPatch((oc[0] + 0.035, oc[1] + 0.035), (oc[0] + 0.035, oc[1] - 0.01), arrowstyle="-|>", mutation_scale=18, color="#1f77b4", lw=2.5))
    if th > 0: ax.add_patch(FancyArrowPatch((oc[0] + 0.02, oc[1] - 0.02), (oc[0] + 0.08, oc[1] - 0.035), arrowstyle="-|>", mutation_scale=18, color="#d62728", lw=2.5))
    ax.axvline(-0.06, color="#bbb", lw=8, alpha=.4)
    ax.set_xlim(-0.1, 0.24); ax.set_ylim(-0.24, 0.14); ax.set_xticks([]); ax.set_yticks([])
    ax.set_title(lab, fontsize=9, loc="left")
    if k == 0: ax.set_ylabel("B) a sequência (desenho do Marco 1→4, conferida no vídeo)", fontsize=9)

# ── C) fases do tool_tip ──
ax = fig.add_subplot(gs[0, 2:]); ax.set_aspect("equal")
th = np.radians(np.linspace(0, 30, 61))
ax.plot(L * np.sin(th) * 1000, (L * (np.cos(th) - 1) - 0.003) * 1000, "-", color="#aaa", lw=2, label="YAML de hoje (arco puro, captura antiga)")
fases = [("captura", 0, CAPT), ("destrava", 0, CAPT - D), ("libera", 0.020, CAPT - D)]
for a in (15, 30):
    r = math.radians(a); fases.append(("arco%d" % (1 if a == 15 else 2), L * math.sin(r), CAPT - D + L * (math.cos(r) - 1)))
xs = [f[1] * 1000 for f in fases]; zs = [f[2] * 1000 for f in fases]
ax.plot(xs[:2], zs[:2], "-", color="#1f77b4", lw=3, label="destrava: −15 mm reto (mola da lingueta)")
ax.plot(xs[1:3], zs[1:3], "-", color="#9467bd", lw=3, label="libera: +20 mm em prof, segurando embaixo")
arc = np.radians(np.linspace(0, 30, 61)); ax.plot(L * np.sin(arc) * 1000, (CAPT - D + L * (np.cos(arc) - 1)) * 1000, "-", color="#d62728", lw=3, label="arco (raio 200, olhal mantido 15 mm abaixo)")
for (nome, p, z) in fases:
    ax.plot(p * 1000, z * 1000, "s", color="k", ms=6, mfc="white", mew=1.5); ax.annotate("%s (%.0f, %.1f)" % (nome, p * 1000, z * 1000), (p * 1000, z * 1000), xytext=(6, 5), textcoords="offset points", fontsize=8)
ax.set_xlim(-12, 125); ax.set_ylim(-75, 8); ax.grid(alpha=.3); ax.legend(fontsize=7.5, loc="lower left")
ax.set_xlabel("prof (mm)"); ax.set_ylabel("alt do tool_tip (mm), zero = centro do olhal em repouso")
ax.set_title("C) fases propostas (offsets no frame da parede)", fontsize=10, loc="left")
fig.suptitle("Gatilho da chave, v2: o olhal está na LINGUETA (mola), não na lâmina — puxar −Z solta a aba do laço; o arco só começa depois", fontsize=10.5)
fig.tight_layout(rect=[0, 0, 1, .96])
out = "/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/destrava_v2.png"; fig.savefig(out, dpi=115); print(out)
for nome, p, z in fases: print("%-9s offset_xyz_m: [0.0, %.4f, %.4f]" % (nome, p, z))
