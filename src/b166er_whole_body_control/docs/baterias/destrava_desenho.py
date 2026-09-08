#!/usr/bin/env python3
"""Minha interpretação do gatilho da chave (2026-09-03), para o Marco corrigir.
A) foto ampliada com o que eu acho que é cada peça; B) as etapas do movimento
no frame da parede [prof, alt]; C) trajetória do YAML de hoje contra a proposta."""
import math, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.patches import FancyArrowPatch, Rectangle, Circle, Ellipse

L = 0.200; D = 0.015          # lâmina pivô→olhal; curso do destravamento (doc 12 Ago: ~15 mm, 10–20 aceitável)
CAPT = -0.010                 # tool_tip na captura (YAML 2026-09-02)
fig = plt.figure(figsize=(17, 7.2))
gs = fig.add_gridspec(1, 3, width_ratios=[0.8, 1.35, 1.1])

# ── A) foto ──
ax = fig.add_subplot(gs[0]); im = mpimg.imread("/home/marco/.claude/jobs/89ade7b6/tmp/chave_zoom_trava.png")
ax.imshow(im); ax.set_xticks([]); ax.set_yticks([])
ax.set_title("A) o que eu leio na foto da bancada (?? = chute meu)", fontsize=10, loc="left")
notas = [((640, 560), (900, 300), "olhal (anel que a\nferramenta atravessa)"),
         ((440, 660), (150, 800), "?? aba/gancho do gatilho:\nfica atrás do pino do\ncontato fixo quando fechada"),
         ((570, 870), (900, 1000), "?? peça de latão\n= mola do gatilho\n(empurra a lâmina\npara CIMA)"),
         ((600, 250), (900, 120), "contato fixo\n(terminal superior)"),
         ((560, 1150), (150, 1180), "hastes da lâmina\n(garfo), descem\naté o pivô")]
for (x, y), (tx, ty), s in notas:
    ax.annotate(s, (x, y), xytext=(tx, ty), fontsize=8, color="#c00", ha="center",
                arrowprops=dict(arrowstyle="->", color="#c00", lw=1.2), bbox=dict(fc="white", ec="#c00", alpha=.9, lw=.6))

# ── B) etapas ──
ax = fig.add_subplot(gs[1]); ax.set_aspect("equal")
def lamina(ax, x0, dz, th, cor, lab, alpha=1.0):
    """pivô em (0,-L); a lâmina desliza dz ao longo do próprio eixo e gira th (rad) para fora (+prof)."""
    piv = np.array([0.0, -L]); u = np.array([math.sin(th), math.cos(th)])
    base = piv - u * 0.0; tip = piv + u * (L + dz)          # ?? a lâmina desliza NO pivô (rasgo embaixo)
    ax.plot([base[0]+x0, tip[0]+x0], [base[1], tip[1]], "-", color=cor, lw=6, alpha=alpha, solid_capstyle="butt")
    ax.add_patch(Ellipse((tip[0]+x0, tip[1]), 0.036, 0.046, fc="none", ec=cor, lw=2, alpha=alpha))
    ax.plot(x0, -L, "ko", ms=6); return tip + np.array([x0, 0])
ax.axvline(0, color="#999", lw=1); ax.axvline(0.30, color="#999", lw=1); ax.axvline(0.60, color="#999", lw=1)
# terminal fixo + pino
for x0 in (0, 0.30, 0.60):
    ax.add_patch(Rectangle((x0-0.012, 0.03), 0.024, 0.05, fc="#bbb", ec="#666")); ax.plot(x0+0.017, 0.032, "s", color="#333", ms=5)
    ax.text(x0-0.012, 0.085, "contato fixo", fontsize=7, color="#555")
# 0) fechada, ferramenta capturada
t0 = lamina(ax, 0.0, 0.0, 0.0, "#333", "fechada")
ax.plot(0.0, CAPT, "s", color="#2ca02c", ms=8); ax.annotate("tool_tip na captura\n(−10 mm sob o centro)", (0.0, CAPT), xytext=(-0.13, -0.07), fontsize=8, color="#2ca02c", arrowprops=dict(arrowstyle="->", color="#2ca02c"))
ax.plot([0.017, 0.0], [0.032, 0.02], "-", color="#c00", lw=2); ax.text(-0.005, 0.045, "gancho atrás\ndo pino (TRAVADA)", fontsize=7, color="#c00", ha="right")
ax.text(0.0, -0.45, "0) capturada\nlâmina vertical, travada", ha="center", fontsize=9)
# 1) destrava: lâmina desce D no próprio eixo
t1 = lamina(ax, 0.30, -D, 0.0, "#1f77b4", "destrava")
ax.add_patch(FancyArrowPatch((0.30+0.06, 0.0), (0.30+0.06, -D), arrowstyle="-|>", mutation_scale=18, color="#1f77b4", lw=2))
ax.text(0.30+0.07, -D/2, "−Z acentuado\n≈ %.0f mm (doc 12 Ago:\n10–20 aceitável)\ncomprime a mola,\ngancho sai do pino" % (D*1000), fontsize=8, color="#1f77b4", va="center")
ax.plot(0.30, CAPT - D, "s", color="#2ca02c", ms=8)
ax.text(0.30, -0.45, "1) DESTRAVA\ndescida reta, sem giro", ha="center", fontsize=9)
# 2) arco a partir da lâmina abaixada
for th, a in ((0.26, .35), (0.52, 1.0)):
    t2 = lamina(ax, 0.60, -D, th, "#d62728", "arco", alpha=a)
th = np.radians(np.linspace(0, 30, 31)); ax.plot(0.60 + (L - D) * np.sin(th), -L + (L - D) * np.cos(th), "--", color="#d62728", lw=1)
ax.text(0.60, -0.45, "2) ARCO (15° → 30°)\ngira no pivô, ainda\nabaixada (?? mola\nvolta a empurrar?)", ha="center", fontsize=9)
ax.set_xlim(-0.2, 0.85); ax.set_ylim(-0.5, 0.11)
ax.set_xlabel("prof → (para fora da parede)"); ax.set_ylabel("alt (m), zero = centro do olhal fechado")
ax.set_title("B) etapas como eu entendi (frame da parede; ?? = o que preciso que você confirme)", fontsize=10, loc="left")
ax.grid(alpha=.25)

# ── C) trajetórias ──
ax = fig.add_subplot(gs[2]); ax.set_aspect("equal")
th = np.radians(np.linspace(0, 30, 61))
# YAML de hoje: arco puro a partir da captura (com os −3 mm antigos)
ax.plot(L*np.sin(th)*1000, (L*(np.cos(th)-1) - 0.003)*1000, "-", color="#888", lw=2, label="YAML de hoje: arco puro (captura antiga −3)")
for k, (p, z) in {"arco1": (0.0518, -0.0098), "arco2": (0.1000, -0.0298)}.items():
    ax.plot(p*1000, z*1000, "s", color="#888", ms=7, mfc="white", mew=2); ax.annotate(k, (p*1000, z*1000), xytext=(5, 5), textcoords="offset points", fontsize=8, color="#666")
# proposta: captura −10, destrava −15 reto, arco com raio L−D a partir daí
zc = CAPT*1000; zd = (CAPT - D)*1000
ax.plot([0, 0], [zc, zd], "-", color="#1f77b4", lw=3, label="proposta: DESTRAVA (−15 mm reto em Z)")
R = L - D
ax.plot(R*np.sin(th)*1000, (CAPT - D + R*(np.cos(th)-1))*1000, "-", color="#d62728", lw=3, label="proposta: arco (raio L−15 = 185 mm)")
for ang in (15, 30):
    a = math.radians(ang); p, z = R*math.sin(a)*1000, (CAPT - D + R*(math.cos(a)-1))*1000
    ax.plot(p, z, "o", color="#d62728", ms=7, mfc="white", mew=2); ax.annotate("%d°: (%.1f, %.1f) mm" % (ang, p, z), (p, z), xytext=(6, -12), textcoords="offset points", fontsize=8, color="#d62728")
ax.plot(0, zc, "s", color="#2ca02c", ms=8); ax.annotate("captura (0, −10)", (0, zc), xytext=(6, 4), textcoords="offset points", fontsize=8, color="#2ca02c")
ax.plot(0, zd, "s", color="#1f77b4", ms=8); ax.annotate("destrava (0, −25)", (0, zd), xytext=(6, -12), textcoords="offset points", fontsize=8, color="#1f77b4")
ax.set_xlabel("prof (mm)"); ax.set_ylabel("alt do tool_tip (mm), zero = centro do olhal")
ax.set_title("C) tool_tip: o que a missão faz hoje × o que a chave pede", fontsize=10, loc="left")
ax.grid(alpha=.3); ax.legend(fontsize=7.5, loc="lower left"); ax.set_xlim(-15, 120); ax.set_ylim(-70, 8)
fig.suptitle("Gatilho da chave: o primeiro movimento é uma DESCIDA RETA que solta a trava; só depois vem o arco (a fase 'release' de 12 Ago, perdida na reescrita de 26 Ago)", fontsize=10.5)
fig.tight_layout(rect=[0, 0, 1, .95])
out = "/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/destrava_desenho.png"; fig.savefig(out, dpi=115); print(out)
