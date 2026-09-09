#!/usr/bin/env python3
"""Gera o STL (mm, para impressão) e o desenho cotado do DEDO FIXO do
RV-M2 — a ferramenta de manobra que atravessa o olhal da chave.

Pedido do Marco em 2026-09-09: "preciso do desenho técnico da
ferramenta, preciso do STL também pois vou imprimir".

A geometria é a do movemaster.urdf.xacro (bloco "dedo_*", revisado com o
Marco em 2026-09-02), em mm, ao longo de -Z a partir da montagem:

  garfo   25 de altura, 20 em X; em Y é um U: duas ABAS de 10 mm com um
          VÃO de 10,4 mm entre elas, que abraça a aba original de 10 mm
          da castanha da HM-01 (URDF: "vão 10,4 mm, 4 furos M3 por aba
          em grade 2x2"); largura total em Y = 2 x 10 + 10,4 = 30,4.
          O URDF desenha o garfo como caixa de 25 em Y — a caixa é
          aproximação visual; a peça real é o U de 30,4.
  afunil. 10: tronco de pirâmide de 20 x 25 para 10 x 10 (rampa contínua)
  haste   80, seção 10 x 10
  degrau  10 de altura, 10 de largura (Y), 20 de projeção em -X
          (o L: 30 de comprimento total em X, alinhado com a haste em +X)
  total   125

O QUE NÃO ESTÁ MEDIDO (2026-09-09): o ESPAÇAMENTO da grade 2x2 dos M3
(diâmetro 3,5 passante é o do URDF). Passe o medido por argumento:

  gera_dedo.py --furo-dx 10 --furo-dz 10 --furo-d 3.5 --rasgo 10.4 --aba 10 --saida DIR

Saída: dedo_fixo.stl (binário, mm, origem no topo do garfo, Z para
baixo como no URDF) e dedo_fixo_desenho.pdf/.png (3 vistas + isométrica,
cotas em mm). Só numpy e matplotlib — sem FreeCAD no shiroi.
"""
import argparse
import os
import struct

import numpy as np

# ---------------------------------------------------------------- geometria (mm)
GARFO_H, GARFO_X = 25.0, 20.0
GARFO_Y = 30.4   # recalculado em main(): 2*aba + rasgo
AFUN_H = 10.0
HASTE_L, HASTE_X, HASTE_Y = 80.0, 10.0, 10.0
DEGRAU_L, DEGRAU_Z = 20.0, 10.0
TOTAL = GARFO_H + AFUN_H + HASTE_L          # 115 até o topo do degrau
ALTURA_TOTAL = TOTAL + DEGRAU_Z             # 125


def orienta_convexo(tris):
    """Vira cada triângulo para a normal apontar para fora (válido para
    sólidos CONVEXOS: caixa e tronco). O STL de impressão não exige, mas
    o volume assinado só confere com normais consistentes."""
    pts = np.vstack(tris); c = pts.mean(axis=0)
    out = []
    for t in tris:
        n = np.cross(t[1] - t[0], t[2] - t[0])
        out.append(t if np.dot(n, t.mean(axis=0) - c) >= 0 else t[[0, 2, 1]])
    return out


def caixa(x0, x1, y0, y1, z0, z1):
    """12 triângulos de um paralelepípedo alinhado aos eixos."""
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], float)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
         (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return orienta_convexo([v[list(t)] for t in f])


def tronco(x_top, y_top, z_top, x_bot, y_bot, z_bot):
    """Tronco de pirâmide entre dois retângulos centrados em (0,0)."""
    a = np.array([[-x_top / 2, -y_top / 2, z_top], [x_top / 2, -y_top / 2, z_top],
                  [x_top / 2, y_top / 2, z_top], [-x_top / 2, y_top / 2, z_top]])
    b = np.array([[-x_bot / 2, -y_bot / 2, z_bot], [x_bot / 2, -y_bot / 2, z_bot],
                  [x_bot / 2, y_bot / 2, z_bot], [-x_bot / 2, y_bot / 2, z_bot]])
    tris = []
    for i in range(4):
        j = (i + 1) % 4
        tris.append(np.array([a[i], b[i], b[j]]))
        tris.append(np.array([a[i], b[j], a[j]]))
    tris.append(np.array([a[0], a[2], a[1]])); tris.append(np.array([a[0], a[3], a[2]]))
    tris.append(np.array([b[0], b[1], b[2]])); tris.append(np.array([b[0], b[2], b[3]]))
    return orienta_convexo(tris)


def furo_cilindro(cx, cz, y0, y1, d, n=24):
    """Furo passante em Y: cilindro de faces internas (normais para dentro)."""
    r = d / 2.0
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    tris = []
    for k in range(n):
        a0, a1 = ang[k], ang[(k + 1) % n]
        p00 = [cx + r * np.cos(a0), y0, cz + r * np.sin(a0)]
        p01 = [cx + r * np.cos(a1), y0, cz + r * np.sin(a1)]
        p10 = [cx + r * np.cos(a0), y1, cz + r * np.sin(a0)]
        p11 = [cx + r * np.cos(a1), y1, cz + r * np.sin(a1)]
        tris.append(np.array([p00, p11, p01])); tris.append(np.array([p00, p10, p11]))
    return tris


def escreve_stl(caminho, tris):
    with open(caminho, 'wb') as f:
        f.write(b'dedo fixo RV-M2 (mm)'.ljust(80, b'\0'))
        f.write(struct.pack('<I', len(tris)))
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0]); nn = np.linalg.norm(n)
            n = n / nn if nn > 0 else n
            f.write(struct.pack('<3f', *n))
            for p in t:
                f.write(struct.pack('<3f', *p))
            f.write(struct.pack('<H', 0))


def solido(furo_dx, furo_dz, furo_d, rasgo, rasgo_prof, aba):
    """Casca do dedo (várias caixas encostadas; o slicer une)."""
    tris = []
    e = 0.2   # sobreposição entre sólidos vizinhos: o fatiador une cascas
              # que se interpenetram; faces exatamente coincidentes confundem
    # Garfo: duas abas (Y) e o fundo do U
    z0, z1 = -GARFO_H, 0.0
    zf = -rasgo_prof                       # fundo do rasgo
    tris += caixa(-GARFO_X / 2, GARFO_X / 2, -GARFO_Y / 2, -GARFO_Y / 2 + aba + e, z0, z1)   # aba -Y
    tris += caixa(-GARFO_X / 2, GARFO_X / 2, GARFO_Y / 2 - aba - e, GARFO_Y / 2, z0, z1)     # aba +Y
    tris += caixa(-GARFO_X / 2, GARFO_X / 2, -GARFO_Y / 2 + aba, GARFO_Y / 2 - aba, z0, zf)  # fundo
    # Furos 2x2 atravessando as abas (só visual no STL; o slicer não
    # subtrai — o desenho cota; fure ou modele os furos no fatiador)
    # Afunilamento
    tris += tronco(GARFO_X, GARFO_Y, -GARFO_H + e, HASTE_X, HASTE_Y, -GARFO_H - AFUN_H)
    # Haste
    zh0 = -GARFO_H - AFUN_H - HASTE_L
    tris += caixa(-HASTE_X / 2, HASTE_X / 2, -HASTE_Y / 2, HASTE_Y / 2, zh0 - e, -GARFO_H - AFUN_H + e)
    # Degrau (L): de x=+5 (face +X da haste) a x=-25, 10 de altura abaixo da haste
    tris += caixa(-HASTE_X / 2 - DEGRAU_L, HASTE_X / 2, -HASTE_Y / 2, HASTE_Y / 2, zh0 - DEGRAU_Z, zh0)
    return tris


def desenho(caminho_base, furo_dx, furo_dz, furo_d, rasgo, rasgo_prof, aba):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Polygon, Circle

    fig = plt.figure(figsize=(16.5, 11.7))     # A3 paisagem
    fig.suptitle('DEDO FIXO — ferramenta de manobra do RV-M2 (b166er)   ·   cotas em mm   ·   '
                 'geometria do movemaster.urdf.xacro (2026-09-02)', fontsize=13, y=0.98)

    def cota(ax, p0, p1, txt, off=(0, 0), rot=0):
        (x0, y0), (x1, y1) = p0, p1
        ax.annotate('', xy=(x1, y1), xytext=(x0, y0),
                    arrowprops=dict(arrowstyle='<->', lw=0.8, color='k'))
        ax.text((x0 + x1) / 2 + off[0], (y0 + y1) / 2 + off[1], txt, ha='center',
                va='center', fontsize=9, rotation=rot,
                bbox=dict(fc='white', ec='none', pad=1))

    zh0 = -GARFO_H - AFUN_H - HASTE_L
    # ---------------- Vista de FRENTE (plano X–Z): o L aparece
    ax = fig.add_subplot(2, 2, 1); ax.set_title('Vista de frente (X–Z) — o L do degrau')
    ax.add_patch(Rectangle((-GARFO_X / 2, -GARFO_H), GARFO_X, GARFO_H, fill=False, lw=1.2))
    ax.add_patch(Polygon([(-GARFO_X / 2, -GARFO_H), (GARFO_X / 2, -GARFO_H),
                          (HASTE_X / 2, -GARFO_H - AFUN_H), (-HASTE_X / 2, -GARFO_H - AFUN_H)],
                         closed=True, fill=False, lw=1.2))
    ax.add_patch(Rectangle((-HASTE_X / 2, zh0), HASTE_X, HASTE_L, fill=False, lw=1.2))
    ax.add_patch(Rectangle((-HASTE_X / 2 - DEGRAU_L, zh0 - DEGRAU_Z), DEGRAU_L + HASTE_X, DEGRAU_Z,
                           fill=False, lw=1.2))
    for cx in (-furo_dx / 2, furo_dx / 2):
        for cz in (-GARFO_H / 2 - furo_dz / 2, -GARFO_H / 2 + furo_dz / 2):
            ax.add_patch(Circle((cx, cz), furo_d / 2, fill=False, lw=0.8, ls='--'))
    cota(ax, (28, 0), (28, -GARFO_H), '25', off=(4, 0), rot=90)
    cota(ax, (28, -GARFO_H), (28, -GARFO_H - AFUN_H), '10', off=(4, 0), rot=90)
    cota(ax, (28, -GARFO_H - AFUN_H), (28, zh0), '80', off=(4, 0), rot=90)
    cota(ax, (28, zh0), (28, zh0 - DEGRAU_Z), '10', off=(4, 0), rot=90)
    cota(ax, (40, 0), (40, zh0 - DEGRAU_Z), '125', off=(5, 0), rot=90)
    cota(ax, (-GARFO_X / 2, 6), (GARFO_X / 2, 6), '20', off=(0, 3))
    cota(ax, (-HASTE_X / 2, -60), (HASTE_X / 2, -60), '10', off=(0, 3))
    cota(ax, (-HASTE_X / 2 - DEGRAU_L, zh0 - DEGRAU_Z - 6), (HASTE_X / 2, zh0 - DEGRAU_Z - 6),
         '30 (20 de projeção além da haste)', off=(0, -3))
    cota(ax, (-furo_dx / 2, -GARFO_H - 4), (furo_dx / 2, -GARFO_H - 4), 'furos %g' % furo_dx, off=(0, -3))
    ax.text(-38, -GARFO_H / 2, '4 furos M3 (Ø%g)\nem grade 2×2, por aba\n(espaçamento\nA CONFERIR)' % furo_d,
            fontsize=8, ha='center', va='center')
    ax.set_xlim(-50, 50); ax.set_ylim(-135, 12); ax.set_aspect('equal'); ax.axis('off')

    # ---------------- Vista LATERAL (plano Y–Z): o rasgo do garfo
    ax = fig.add_subplot(2, 2, 2); ax.set_title('Vista lateral (Y–Z) — o U do garfo e a seção 10 × 10')
    ax.add_patch(Rectangle((-GARFO_Y / 2, -GARFO_H), GARFO_Y, GARFO_H, fill=False, lw=1.2))
    ax.add_patch(Rectangle((-rasgo / 2, -rasgo_prof), rasgo, rasgo_prof, fill=False, lw=1.2, hatch='//'))
    ax.add_patch(Polygon([(-GARFO_Y / 2, -GARFO_H), (GARFO_Y / 2, -GARFO_H),
                          (HASTE_Y / 2, -GARFO_H - AFUN_H), (-HASTE_Y / 2, -GARFO_H - AFUN_H)],
                         closed=True, fill=False, lw=1.2))
    ax.add_patch(Rectangle((-HASTE_Y / 2, zh0 - DEGRAU_Z), HASTE_Y, HASTE_L + DEGRAU_Z, fill=False, lw=1.2))
    for cz in (-GARFO_H / 2 - furo_dz / 2, -GARFO_H / 2 + furo_dz / 2):
        ax.plot([-GARFO_Y / 2, GARFO_Y / 2], [cz, cz], 'k--', lw=0.6)
    cota(ax, (-GARFO_Y / 2, 6), (GARFO_Y / 2, 6), '%g' % GARFO_Y, off=(0, 3))
    cota(ax, (-rasgo / 2, 12), (rasgo / 2, 12), 'vão %g (aba da castanha: 10)' % rasgo, off=(0, 3))
    cota(ax, (-GARFO_Y / 2, -GARFO_H - 4), (-GARFO_Y / 2 + aba, -GARFO_H - 4), 'aba %g' % aba, off=(0, -3))
    cota(ax, (GARFO_Y / 2 + 6, 0), (GARFO_Y / 2 + 6, -rasgo_prof), 'vão prof. %g' % rasgo_prof, off=(6, 0), rot=90)
    cota(ax, (GARFO_Y / 2 + 16, -GARFO_H / 2 - furo_dz / 2), (GARFO_Y / 2 + 16, -GARFO_H / 2 + furo_dz / 2), 'furos %g' % furo_dz, off=(6, 0), rot=90)
    cota(ax, (-HASTE_Y / 2, -60), (HASTE_Y / 2, -60), '10', off=(0, 3))
    ax.set_xlim(-50, 60); ax.set_ylim(-135, 18); ax.set_aspect('equal'); ax.axis('off')

    # ---------------- Vista SUPERIOR (plano X–Y): o degrau visto de cima
    ax = fig.add_subplot(2, 2, 3); ax.set_title('Vista de baixo (X–Y) — o degrau visto pela ponta')
    ax.add_patch(Rectangle((-GARFO_X / 2, -GARFO_Y / 2), GARFO_X, GARFO_Y, fill=False, lw=0.8, ls=':'))
    ax.add_patch(Rectangle((-HASTE_X / 2 - DEGRAU_L, -HASTE_Y / 2), DEGRAU_L + HASTE_X, HASTE_Y, fill=False, lw=1.2))
    ax.add_patch(Rectangle((-HASTE_X / 2, -HASTE_Y / 2), HASTE_X, HASTE_Y, fill=False, lw=1.2))
    cota(ax, (-HASTE_X / 2 - DEGRAU_L, -14), (HASTE_X / 2, -14), '30', off=(0, -3))
    cota(ax, (-HASTE_X / 2 - DEGRAU_L, 12), (-HASTE_X / 2, 12), '20', off=(0, 3))
    cota(ax, (12, -HASTE_Y / 2), (12, HASTE_Y / 2), '10', off=(4, 0), rot=90)
    ax.text(0, -22, 'garfo 20 × %g (pontilhado) — o degrau sai em −X, no eixo da haste' % GARFO_Y, fontsize=8, ha='center')
    ax.set_xlim(-50, 50); ax.set_ylim(-30, 30); ax.set_aspect('equal'); ax.axis('off')

    # ---------------- Isométrica simples
    ax = fig.add_subplot(2, 2, 4); ax.set_title('Isométrica (só orientação) e notas de impressão')
    def iso(p):
        x, y, z = p
        return (x - y) * np.cos(np.radians(30)), z + (x + y) * np.sin(np.radians(30))
    def poly(pts, **kw):
        ax.add_patch(Polygon([iso(p) for p in pts], closed=True, **kw))
    def caixa_iso(x0, x1, y0, y1, z0, z1):
        poly([(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)], fill=True, fc='#f4dd7a', ec='k', lw=0.8)
        poly([(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)], fill=True, fc='#d9c25f', ec='k', lw=0.8)
        poly([(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)], fill=True, fc='#b89f3a', ec='k', lw=0.8)
    caixa_iso(-HASTE_X / 2 - DEGRAU_L, HASTE_X / 2, -HASTE_Y / 2, HASTE_Y / 2, zh0 - DEGRAU_Z, zh0)
    caixa_iso(-HASTE_X / 2, HASTE_X / 2, -HASTE_Y / 2, HASTE_Y / 2, zh0, -GARFO_H - AFUN_H)
    caixa_iso(-GARFO_X / 2, GARFO_X / 2, -GARFO_Y / 2, GARFO_Y / 2, -GARFO_H - AFUN_H, -GARFO_H)
    # garfo em U: fundo e duas abas
    caixa_iso(-GARFO_X / 2, GARFO_X / 2, -GARFO_Y / 2, GARFO_Y / 2, -GARFO_H, -rasgo_prof)
    caixa_iso(-GARFO_X / 2, GARFO_X / 2, -GARFO_Y / 2, -GARFO_Y / 2 + aba, -rasgo_prof, 0)
    caixa_iso(-GARFO_X / 2, GARFO_X / 2, GARFO_Y / 2 - aba, GARFO_Y / 2, -rasgo_prof, 0)
    ax.text(-120, -60, 'Notas:\n'
            '• Z para baixo a partir do topo do garfo, como no URDF.\n'
            '• Imprimir DEITADO (haste e degrau no plano da mesa): as\n'
            '  camadas ficam ao longo da haste e o degrau em tração\n'
            '  não delamina. Perímetros ≥ 4, preenchimento ≥ 60 %.\n'
            '• Rampa 20×25 → 10×10 é uma transição contínua (tronco).\n'
            '• O U do garfo abraça a aba de 10 mm da castanha da HM-01\n'
            '  (vão 10,4); os 4 furos M3 (Ø3,5) por aba em grade 2×2 —\n'
            '  MEDIR o espaçamento na castanha antes de fatiar.\n'
            '• O STL não traz os furos (sólido): fure na impressão ou\n'
            '  subtraia no fatiador, nas posições cotadas.',
            fontsize=8.5, va='center', ha='left')
    ax.set_xlim(-125, 60); ax.set_ylim(-150, 30); ax.set_aspect('equal'); ax.axis('off')

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(caminho_base + '.pdf'); fig.savefig(caminho_base + '.png', dpi=150)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--furo-dx', type=float, default=10.0, help='espaçamento dos furos em X (mm)')
    ap.add_argument('--furo-dz', type=float, default=10.0, help='espaçamento dos furos em Z (mm)')
    ap.add_argument('--furo-d', type=float, default=3.5, help='diâmetro dos furos (mm; M3 passante, como no URDF)')
    ap.add_argument('--rasgo', type=float, default=10.4, help='vão do garfo em Y (aba da castanha de 10 mm + folga)')
    ap.add_argument('--rasgo-prof', type=float, default=20.0, help='profundidade do vão (mm)')
    ap.add_argument('--aba', type=float, default=10.0, help='espessura de cada aba do garfo em Y (mm)')
    ap.add_argument('--saida', default='.', help='pasta de saída')
    a = ap.parse_args()
    global GARFO_Y
    GARFO_Y = 2 * a.aba + a.rasgo
    os.makedirs(a.saida, exist_ok=True)
    tris = solido(a.furo_dx, a.furo_dz, a.furo_d, a.rasgo, a.rasgo_prof, a.aba)
    stl = os.path.join(a.saida, 'dedo_fixo.stl')
    escreve_stl(stl, tris)
    desenho(os.path.join(a.saida, 'dedo_fixo_desenho'), a.furo_dx, a.furo_dz, a.furo_d, a.rasgo, a.rasgo_prof, a.aba)
    pts = np.vstack(tris)
    print('%s: %d triângulos, bbox X %.1f..%.1f  Y %.1f..%.1f  Z %.1f..%.1f mm' % (
        stl, len(tris), pts[:, 0].min(), pts[:, 0].max(), pts[:, 1].min(), pts[:, 1].max(),
        pts[:, 2].min(), pts[:, 2].max()))
    print('desenho: %s.pdf / .png' % os.path.join(a.saida, 'dedo_fixo_desenho'))


if __name__ == '__main__':
    main()
