#!/usr/bin/env python3
"""Desenho das juntas do RV-M2 sem encoder (faixas do manual, fins de curso,
homing, limite de software do J2, observabilidade pela T265). Lê
config/arm_switches.yaml e a FK; rodar da raiz do workspace com o ros_env:
    python3 src/b166er_whole_body_control/scripts/desenho_juntas_switches.py
Saída: docs/baterias/malha_aberta/juntas_switches_homing.png (pedido do
Marco, 07 Out 2026: "monte um desenho com as informações das juntas")."""
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np, yaml, math
from matplotlib.patches import Wedge, FancyArrowPatch
from b166er_whole_body_control.kinematics import fk_arm_joint_frames

cfg = yaml.safe_load(open('src/b166er_whole_body_control/config/arm_switches.yaml'))
LO = np.array(cfg['lower_deg']); UP = np.array(cfg['upper_deg']); MG = cfg['margem_deg']
SIDE = cfg['home_side']; ORDEM = cfg['home_ordem']
import os
J2MIN = float(os.environ.get('B166ER_J2_MIN_DEG', '-65.0'))   # limite de software do J2 (padrão = extremo do modelo)
STOW = np.array([0.0, 1.10, -1.04, -1.8, 0.0]); STOWd = np.degrees(STOW)
DEPLOY = np.array([-0.19, -0.81, 1.047, 1.368, -0.19])
NOMES = ['J1 cintura', 'J2 ombro', 'J3 cotovelo', 'J4 punho (arfagem)', 'J5 punho (rolagem)']

def pontos(q):
    fr, T_ee = fk_arm_joint_frames(list(q))
    P = [f[:3, 3] for f in fr] + [T_ee[:3, 3]]
    return np.array(P)     # 0..4 = J1..J5, 5 = T265

fig = plt.figure(figsize=(22, 11))
fig.suptitle('RV-M2 sem encoder — juntas, fins de curso (ângulos do manual), homing e limites  ·  07 Out 2026', fontsize=14)
ax = fig.add_axes([0.03, 0.08, 0.50, 0.84]); ax.set_aspect('equal')
ax.set_title('vista lateral (plano x–z da base do braço), braço RECOLHIDO; fantasma = pré-deploy', fontsize=11)

def desenha_braco(q, cor, lw, alpha, rot=True):
    P = pontos(q)
    xs, zs = P[:, 0], P[:, 2]
    ax.plot(xs[1:], zs[1:], '-', color=cor, lw=lw, alpha=alpha, solid_capstyle='round')
    ax.plot([xs[0], xs[1]], [zs[0], zs[1]], '-', color=cor, lw=lw + 4, alpha=alpha)   # coluna/cintura
    for i in range(1, 5):
        ax.plot(xs[i], zs[i], 'o', color='white', mec=cor, mew=2, ms=9, alpha=alpha, zorder=5)
    ax.plot(xs[5], zs[5], 's', color='#2a2', ms=8, alpha=alpha, zorder=6)
    return P

Pd = desenha_braco(DEPLOY, '#999', 5, 0.35)
P = desenha_braco(STOW, '#333', 6, 1.0)
ax.text(P[5, 0] + 0.03, P[5, 2], 'T265 (único sensor\nde pose do braço)', fontsize=9, color='#2a2', va='center')
ax.text(P[0, 0] - 0.05, P[0, 2] - 0.06, 'J1 (cintura, eixo vertical): ±150°\nhoming para o switch +150° (por último, braço recolhido)', fontsize=9, ha='left')
ax.text(P[5, 0] - 0.02, P[5, 2] - 0.07, 'J5 (rolagem): ±180°\nhoming para o switch +180°', fontsize=9, ha='left', color='#555')

# arcos de J2, J3, J4: amostra a posição da junta seguinte variando só essa junta
cores = {1: '#c33', 2: '#36c', 3: '#c80'}
for k, prox in ((1, 2), (2, 3), (3, 5)):
    centro = P[k, [0, 2]]
    angs = np.linspace(LO[k], UP[k], 121)
    pts = []
    for a in angs:
        q = STOW.copy(); q[k] = math.radians(a); pts.append(pontos(q)[prox, [0, 2]])
    pts = np.array(pts); r = np.linalg.norm(pts - centro, axis=1).mean()
    ax.plot(pts[:, 0], pts[:, 1], '-', color=cores[k], lw=2, alpha=0.9)
    def ponto(a):
        q = STOW.copy(); q[k] = math.radians(a); return pontos(q)[prox, [0, 2]]
    # switches (fecham em nominal ∓ margem)
    for a, lado in ((LO[k] + MG, -1), (UP[k] - MG, +1)):
        p = ponto(a)
        ax.plot(p[0], p[1], 'D', color=cores[k], ms=8, zorder=7)
        ax.annotate('switch %+d\n%.1f°' % (lado, a), p, textcoords='offset points', xytext=(8, 8 if lado > 0 else -22),
                    fontsize=8, color=cores[k])
    # setor proibido do J2 (abaixo do limite de software)
    if k == 1 and J2MIN > LO[k] + 0.5:
        ang_p = np.linspace(LO[k], J2MIN, 40)
        pp = np.array([ponto(a) for a in ang_p])
        ax.fill(np.r_[centro[0], pp[:, 0]], np.r_[centro[1], pp[:, 1]], color='#c33', alpha=0.15, lw=0)
        p = ponto(J2MIN); ax.plot(p[0], p[1], 'x', color='#c33', ms=12, mew=3, zorder=8)
        ax.annotate('limite de SOFTWARE %.0f°\n(B166ER_J2_MIN_DEG)' % J2MIN, p,
                    textcoords='offset points', xytext=(-175, -34), fontsize=8.5, color='#c33', weight='bold')
    # seta do homing: do stow ao switch escolhido
    if SIDE[k] != 0:
        alvo = UP[k] - MG if SIDE[k] > 0 else LO[k] + MG
        a0 = STOWd[k]; aa = np.linspace(a0, alvo, 20); seg = np.array([ponto(a) for a in aa])
        ax.plot(seg[:, 0], seg[:, 1], '-', color=cores[k], lw=6, alpha=0.35)
        ax.annotate('', xy=seg[-1], xytext=seg[-3], arrowprops=dict(arrowstyle='-|>', lw=3, color=cores[k], mutation_scale=22))
    # rótulo da junta
    ax.annotate('%s\n%.0f° … %.0f° (manual)' % (NOMES[k], LO[k], UP[k]), centro, textcoords='offset points',
                xytext=((-10, 14), (-60, -46), (30, 34))[k - 1], fontsize=9.5, color=cores[k], weight='bold', ha='right' if k == 1 else 'left')

ax.set_xlabel('x (m) — frente do robô →'); ax.set_ylabel('z (m)')
ax.grid(alpha=0.3); ax.set_xlim(-0.55, 0.75); ax.set_ylim(-0.1, 0.9)
ax.text(-0.53, 0.86, 'faixa cheia = alcance do manual (switch em cada ponta)\nseta grossa = HOMING no início da missão (ordem J4 → J3 → J2 → J5 → J1; J1 e J5 fora do plano)\nsetor vermelho = proibido por software (J2 nunca para baixo)\nconvenção: J2 positivo = braço para TRÁS/cima; J3 e J4 positivos = dobram para a frente',
        fontsize=9, va='top', bbox=dict(fc='white', ec='#ccc'))

# ---------------- tabela
ax2 = fig.add_axes([0.55, 0.08, 0.44, 0.84]); ax2.axis('off')
ax2.set_title('o que cada junta tem — e o que o controle usa', fontsize=11)
obs = ['NÃO no robô real (yaw da T265\né relativo a onde ela acordou)', 'não (ramo ambíguo)', 'não (ramo ambíguo)', 'não (ramo ambíguo)', 'parcial (rolagem pela\ngravidade, com ambiguidade)']
linhas = []
for k in range(5):
    sw = '%.1f° / %.1f°' % (LO[k] + MG, UP[k] - MG)
    if SIDE[k] == 0: hom = '—'
    else: hom = '%s (%+d) · %dº' % ({0: 'esquerda/trás', 1: 'para TRÁS', 4: 'meia-volta'}.get(k, 'para baixo' if SIDE[k] < 0 else 'para cima'), SIDE[k], ORDEM.index(k + 1) + 1)
    lim = '%.0f° … %.0f° (software)' % (J2MIN, UP[1]) if k == 1 else '%.0f° … %.0f°' % (LO[k], UP[k])
    linhas.append([NOMES[k], '%.0f° … %.0f°' % (LO[k], UP[k]), sw, hom, lim, obs[k], '%.1f°' % STOWd[k]])
cols = ['junta', 'faixa (manual)', 'switches fecham em\n(nominal ∓ %.1f°)' % MG, 'homing ao ligar', 'curso permitido', 'observável só\npela T265?', 'stow']
tb = ax2.table(cellText=linhas, colLabels=cols, loc='upper center', cellLoc='center', colWidths=[0.17, 0.12, 0.16, 0.17, 0.19, 0.15, 0.07])
tb.auto_set_font_size(False); tb.set_fontsize(9); tb.scale(1, 2.6)
for (r, c), cell in tb.get_celld().items():
    if r == 0: cell.set_text_props(weight='bold'); cell.set_facecolor('#eee')
    if r == 2 and c in (3, 4): cell.set_facecolor('#fde8e8')
    if r in (2, 3, 4) and c == 3: cell.set_text_props(color=cores[r - 1], weight='bold')
notas = ('NOTAS\n'
         '• Sem encoder: a única medida ABSOLUTA de junta é o fim de curso. O homing leva J4, J3 e J2 ao switch\n'
         '  do lado do recolhido, depois J5 e J1 (09 Out: a guinada da T265 é relativa a onde ela acordou — sem\n'
         '  switch o zero do J1 é inventado); o estimador ancora cada junta no ângulo em que o switch fecha.\n'
         '• J2 só vai para TRÁS no homing (nunca para baixo: risco). Abaixo do limite de software o servo corta\n'
         '  posturas e zera velocidade negativa — vale para posturas, homing e Fuzzy. 115° cobre a missão (pré-deploy −46°).\n'
         '• Ramo do cotovelo (J2/J3/J4): a T265 não distingue; decidido pelo sinal de J3 na passagem pelo cotovelo\n'
         '  reto e pelos switches. J1 e J5: pelos switches (+150°, +180°) no início de toda missão.\n'
         '• Ângulos: os declarados no manual BFP-A5296, repartidos simetricamente (= URDF). Decisão: não medir na bancada.\n'
         '• Placas: devem publicar os pinos LS em /b166er/arm_limit_switch (−1/0/+1 por junta), como o firmware emulado.\n'
         '• Config: b166er_whole_body_control/config/arm_switches.yaml')
ax2.text(0.0, 0.42, notas, transform=ax2.transAxes, fontsize=9.5, va='top', family='monospace',
         bbox=dict(fc='#fafafa', ec='#ccc'))
out = 'src/b166er_whole_body_control/docs/baterias/malha_aberta/juntas_switches_homing.png'
plt.savefig(out, dpi=110); print(out)
