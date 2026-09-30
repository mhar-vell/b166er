#!/usr/bin/env python3
"""Gera o STL (mm, para impressão) e a prancha cotada do DEDO FIXO do
RV-M2 — a ferramenta de manobra que atravessa o olhal da chave.

Pedido do Marco em 2026-09-09: "preciso do desenho técnico da
ferramenta, preciso do STL também pois vou imprimir".

TRÊS VERSÕES, numeração do Marco (--versao N; saídas dedo_fixo_vN.stl e
dedo_fixo_vN_desenho.pdf/.png):

  v1  REV. A, 09 Set 2026 — geometria do bloco dedo_* do URDF revisada
      em 02 Set: garfo em U 20 x 30,4 x 25 (abas de 10, vão de 10,4, fundo
      de 5), rampa 10, HASTE 80, degrau 20 em -X; total 125. Sem furos no
      STL. Prancha antiga (3 vistas + isométrica).
  v2  REV. B, 10 Set 2026 — a peça IMPRESSA. Gerada fora deste repositório
      (o original está em meshes/movemaster/dedo_fixo_v2.stl e
      docs/dedo_fixo_v2_desenho.*, trazidos dos Downloads do Marco em
      29 Set); aqui é REPRODUZIDA a partir das cotas dessa prancha:
      garfo 20 x 30 x 25 com RASGO PASSANTE de 10 (duas abas de 10 sem
      fundo, apoiadas na rampa), 4 furos M3 Ø3,5 por aba em grade 10 (X)
      x 15 (Z) a 5 mm das bordas, ABERTOS no STL; rampa 10; HASTE 30;
      degrau 20 em -X; total 75.
  v3  REV. C, 29 Set 2026 — pedido do Marco: "o degrau tem que rotacionar
      90 graus" (opção B do desenho de alternativas) e "altere para 80
      também na versão 3". Geometria da v2 (garfo com rasgo passante e
      furos abertos) com HASTE 80 (total 125, como no modelo da simulação)
      e o degrau em +Y, no sentido das ABAS. kinematics.DEGRAU_DIR_TIP e o
      tool_tip do URDF acompanham; o comprimento já era o do modelo.

A prancha das v2/v3 segue o formato padrão da REV. B: A3 paisagem com
moldura, VISTA DE BAIXO / DE FRENTE / LATERAL cotadas em 1:1, DETALHE DA
FURAÇÃO em 2:1, perspectiva isométrica, notas numeradas e legenda
(material, volume, tolerância, desenho, data, escala, unidade, código
B166ER-FER-001 e revisão).

Só numpy e matplotlib — sem FreeCAD nem scipy no shiroi: as abas com
furos são trianguladas à mão (células retangulares em anel em torno de
cada furo, com bordas compartilhadas idênticas para a casca fechar).
"""
import argparse
import os
import struct

import numpy as np

# ---------------------------------------------------------------- geometria (mm)
GEO = {
    1: dict(garfo_h=25.0, garfo_x=20.0, aba=10.0, rasgo=10.4, rasgo_prof=20.0,
            afun_h=10.0, haste_l=80.0, haste=10.0, degrau_l=20.0, degrau_z=10.0,
            furo_dx=10.0, furo_dz=10.0, furo_d=3.5, furos_no_stl=False,
            degrau_eixo='-x', rev='A', data='2026-09-09',
            substitui='desenho à mão do Marco de 2026-08-25'),
    2: dict(garfo_h=25.0, garfo_x=20.0, aba=10.0, rasgo=10.0, rasgo_prof=25.0,
            afun_h=10.0, haste_l=30.0, haste=10.0, degrau_l=20.0, degrau_z=10.0,
            furo_dx=10.0, furo_dz=15.0, furo_d=3.5, furos_no_stl=True,
            degrau_eixo='-x', rev='B', data='2026-09-09',
            substitui='REV. A de 2026-09-02 (haste 80, total 125)'),
    3: dict(garfo_h=25.0, garfo_x=20.0, aba=10.0, rasgo=10.0, rasgo_prof=25.0,
            afun_h=10.0, haste_l=80.0, haste=10.0, degrau_l=20.0, degrau_z=10.0,
            furo_dx=10.0, furo_dz=15.0, furo_d=3.5, furos_no_stl=True,
            degrau_eixo='+y', rev='C', data='2026-09-29',
            substitui='REV. B de 2026-09-09 (haste 30, degrau em −X)'),
}
DENS_PLA = 1.24   # g/cm³


class Dedo(object):
    """Cotas derivadas de uma versão (tudo em mm, Z para baixo a partir
    do topo do garfo, como no URDF)."""

    def __init__(self, versao, **override):
        g = dict(GEO[versao]); g.update({k: v for k, v in override.items() if v is not None})
        self.v = versao
        for k, val in g.items():
            setattr(self, k, val)
        self.garfo_y = 2 * self.aba + self.rasgo
        self.z_garfo = -self.garfo_h
        self.z_afun = -self.garfo_h - self.afun_h
        self.zh0 = self.z_afun - self.haste_l          # topo do degrau
        self.z_fim = self.zh0 - self.degrau_z          # ponta da peça
        self.total = -self.z_fim
        # Furos: grade 2x2 por aba, centrada em X e em Z (v2/v3: 5 e 20 do
        # topo com furo_dz 15; v1: 7,5 e 17,5 com furo_dz 10).
        self.furos_x = (-self.furo_dx / 2, self.furo_dx / 2)
        zc = -self.garfo_h / 2
        self.furos_z = (zc + self.furo_dz / 2, zc - self.furo_dz / 2)

    def degrau_box(self):
        """(x0, x1, y0, y1, z0, z1) do L inteiro, incluindo a parte sob a haste."""
        h = self.haste / 2
        if self.degrau_eixo == '-x':
            return (-h - self.degrau_l, h, -h, h, self.z_fim, self.zh0)
        return (-h, h, -h, h + self.degrau_l, self.z_fim, self.zh0)

    def volume_mm3(self):
        """Analítico (o STL tem sobreposições de 0,2 mm que inflariam a soma)."""
        abas = 2 * self.garfo_x * self.aba * self.garfo_h
        fundo = self.garfo_x * self.rasgo * (self.garfo_h - self.rasgo_prof)
        furos = 2 * 4 * np.pi * (self.furo_d / 2) ** 2 * self.aba if self.furos_no_stl else 0.0
        a1, a2 = self.garfo_x * self.garfo_y, self.haste * self.haste
        rampa = self.afun_h / 3.0 * (a1 + a2 + np.sqrt(a1 * a2))
        haste = self.haste * self.haste * self.haste_l
        degrau = (self.degrau_l + self.haste) * self.haste * self.degrau_z
        return abas - furos + fundo + rampa + haste + degrau


# ---------------------------------------------------------------- malha
def _orienta(tris, esperado):
    """Vira cada triângulo para a normal ficar no semi-espaço de `esperado`
    (vetor, ou função ponto -> vetor)."""
    out = []
    for t in tris:
        n = np.cross(t[1] - t[0], t[2] - t[0])
        e = esperado(t.mean(axis=0)) if callable(esperado) else esperado
        out.append(t if np.dot(n, e) >= 0 else t[[0, 2, 1]])
    return out


def orienta_convexo(tris):
    """Normais para fora de um sólido CONVEXO (caixa, tronco)."""
    c = np.vstack(tris).mean(axis=0)
    return _orienta(tris, lambda p: p - c)


def caixa(x0, x1, y0, y1, z0, z1):
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], float)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
         (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return orienta_convexo([v[list(t)] for t in f])


def tronco(x_top, y_top, z_top, x_bot, y_bot, z_bot):
    a = np.array([[-x_top / 2, -y_top / 2, z_top], [x_top / 2, -y_top / 2, z_top],
                  [x_top / 2, y_top / 2, z_top], [-x_top / 2, y_top / 2, z_top]])
    b = np.array([[-x_bot / 2, -y_bot / 2, z_bot], [x_bot / 2, -y_bot / 2, z_bot],
                  [x_bot / 2, y_bot / 2, z_bot], [-x_bot / 2, y_bot / 2, z_bot]])
    tris = []
    for i in range(4):
        j = (i + 1) % 4
        tris.append(np.array([a[i], b[i], b[j]])); tris.append(np.array([a[i], b[j], a[j]]))
    tris.append(np.array([a[0], a[2], a[1]])); tris.append(np.array([a[0], a[3], a[2]]))
    tris.append(np.array([b[0], b[1], b[2]])); tris.append(np.array([b[0], b[2], b[3]]))
    return orienta_convexo(tris)


def _anel_2d(c, r, n, borda):
    """Triangula a região entre a circunferência (centro c, raio r, n
    pontos) e o polígono convexo `borda` que a contém, casando as duas
    sequências por ângulo em torno de c. Devolve (triângulos 2D, pontos
    da circunferência em ordem angular)."""
    ang = list(np.linspace(0, 2 * np.pi, n, endpoint=False))
    circ = [np.array([c[0] + r * np.cos(a), c[1] + r * np.sin(a)]) for a in ang]
    bl = sorted(borda, key=lambda p: np.arctan2(p[1] - c[1], p[0] - c[0]) % (2 * np.pi))
    bang = [np.arctan2(p[1] - c[1], p[0] - c[0]) % (2 * np.pi) for p in bl]
    ni, nj = len(circ), len(bl)
    tris, i, j = [], 0, 0
    while i < ni or j < nj:
        prox_c = ang[i + 1] if i + 1 < ni else 2 * np.pi
        prox_b = bang[j + 1] if j + 1 < nj else 2 * np.pi
        if i < ni and (prox_c <= prox_b or j >= nj):
            tris.append(np.array([circ[i], circ[(i + 1) % ni], bl[j % nj]])); i += 1
        else:
            tris.append(np.array([circ[i % ni], bl[(j + 1) % nj], bl[j]])); j += 1
    return tris, circ


def placa_com_furos(x0, x1, z0, z1, y0, y1, furos, d, n=24, passo=2.5):
    """Placa X[x0,x1] x Z[z0,z1] extrudada em Y[y0,y1] com furos passantes
    em Y nos centros `furos` [(cx, cz), ...]. Casca fechada: a placa é
    dividida em células retangulares (uma por furo, grade regular), cada
    célula é um anel entre o furo e a borda da célula, e as bordas
    compartilhadas usam a MESMA subdivisão dos dois lados."""
    xs = sorted(set(cx for cx, _ in furos)); zs = sorted(set(cz for _, cz in furos))
    xe = [x0] + [(xs[i] + xs[i + 1]) / 2 for i in range(len(xs) - 1)] + [x1]
    ze = [z0] + [(zs[i] + zs[i + 1]) / 2 for i in range(len(zs) - 1)] + [z1]

    def sub(a, b):
        k = max(1, int(round(abs(b - a) / passo)))
        return list(np.linspace(a, b, k + 1))

    tris2 = []; circs = []
    for cx, cz in furos:
        ix = next(i for i in range(len(xe) - 1) if xe[i] - 1e-9 <= cx <= xe[i + 1] + 1e-9)
        iz = next(i for i in range(len(ze) - 1) if ze[i] - 1e-9 <= cz <= ze[i + 1] + 1e-9)
        cx0, cx1, cz0, cz1 = xe[ix], xe[ix + 1], ze[iz], ze[iz + 1]
        borda = ([np.array([x, cz0]) for x in sub(cx0, cx1)] + [np.array([cx1, z]) for z in sub(cz0, cz1)[1:]]
                 + [np.array([x, cz1]) for x in sub(cx1, cx0)[1:]] + [np.array([cx0, z]) for z in sub(cz1, cz0)[1:-1]])
        t, circ = _anel_2d((cx, cz), d / 2, n, borda)
        tris2 += t; circs.append(circ)

    def p3(p2, y):
        return np.array([p2[0], y, p2[1]])
    tris = []
    tris += _orienta([np.array([p3(t[0], y0), p3(t[1], y0), p3(t[2], y0)]) for t in tris2], np.array([0, -1, 0]))
    tris += _orienta([np.array([p3(t[0], y1), p3(t[1], y1), p3(t[2], y1)]) for t in tris2], np.array([0, 1, 0]))
    for (cx, cz), circ in zip(furos, circs):
        for k in range(len(circ)):
            a, b = circ[k], circ[(k + 1) % len(circ)]
            q = [np.array([p3(a, y0), p3(b, y0), p3(b, y1)]), np.array([p3(a, y0), p3(b, y1), p3(a, y1)])]
            tris += _orienta(q, lambda p, cx=cx, cz=cz: np.array([cx - p[0], 0, cz - p[2]]))
    # paredes externas: a mesma subdivisão que as células usam nas arestas
    cx_m, cz_m = (x0 + x1) / 2, (z0 + z1) / 2
    for (ax_, az, bx, bz) in ((x0, z0, x1, z0), (x1, z0, x1, z1), (x1, z1, x0, z1), (x0, z1, x0, z0)):
        pts = set()
        divs = [ax_ + s * (bx - ax_) for s in [0.0, 1.0]] if abs(az - bz) < 1e-9 else []
        if abs(az - bz) < 1e-9:      # aresta horizontal: subdivisão por célula
            cortes = [x0] + [xd for xd in xe[1:-1]] + [x1]
            for a, b in zip(cortes[:-1], cortes[1:]):
                for x in sub(a, b): pts.add((round(x, 6), round(az, 6)))
        else:                        # aresta vertical
            cortes = [z0] + [zd for zd in ze[1:-1]] + [z1]
            for a, b in zip(cortes[:-1], cortes[1:]):
                for z in sub(a, b): pts.add((round(ax_, 6), round(z, 6)))
        pts = sorted(pts, key=lambda p: (p[0] - ax_) * (bx - ax_) + (p[1] - az) * (bz - az))
        for k in range(len(pts) - 1):
            a, b = np.array(pts[k]), np.array(pts[k + 1])
            q = [np.array([p3(a, y0), p3(b, y0), p3(b, y1)]), np.array([p3(a, y0), p3(b, y1), p3(a, y1)])]
            tris += _orienta(q, lambda p: np.array([p[0] - cx_m, 0, p[2] - cz_m]))
    return tris


def solido(d):
    """Casca do dedo (sólidos encostados com 0,2 mm de sobreposição; o
    fatiador une)."""
    tris = []
    e = 0.2
    gy = d.garfo_y
    if d.furos_no_stl:
        furos = [(x, z) for x in d.furos_x for z in d.furos_z]
        tris += placa_com_furos(-d.garfo_x / 2, d.garfo_x / 2, d.z_garfo - e, 0.0, -gy / 2, -gy / 2 + d.aba, furos, d.furo_d)
        tris += placa_com_furos(-d.garfo_x / 2, d.garfo_x / 2, d.z_garfo - e, 0.0, gy / 2 - d.aba, gy / 2, furos, d.furo_d)
    else:
        tris += caixa(-d.garfo_x / 2, d.garfo_x / 2, -gy / 2, -gy / 2 + d.aba + e, d.z_garfo, 0.0)
        tris += caixa(-d.garfo_x / 2, d.garfo_x / 2, gy / 2 - d.aba - e, gy / 2, d.z_garfo, 0.0)
    if d.rasgo_prof < d.garfo_h:   # fundo do U (só a v1)
        tris += caixa(-d.garfo_x / 2, d.garfo_x / 2, -gy / 2 + d.aba, gy / 2 - d.aba, d.z_garfo, -d.rasgo_prof)
    tris += tronco(d.garfo_x, gy, d.z_garfo + e, d.haste, d.haste, d.z_afun)
    h = d.haste / 2
    tris += caixa(-h, h, -h, h, d.zh0 - e, d.z_afun + e)
    tris += caixa(*d.degrau_box())
    return tris


def escreve_stl(caminho, tris, rotulo):
    with open(caminho, 'wb') as f:
        f.write(rotulo.encode('ascii', 'replace').ljust(80, b'\0')[:80])
        f.write(struct.pack('<I', len(tris)))
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0]); nn = np.linalg.norm(n)
            n = n / nn if nn > 0 else n
            f.write(struct.pack('<3f', *n))
            for p in t:
                f.write(struct.pack('<3f', *p))
            f.write(struct.pack('<H', 0))


# ---------------------------------------------------------------- prancha (formato REV. B)
def prancha(caminho_base, d):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Polygon, Circle

    W, H = 420.0, 297.0                     # A3 paisagem, coordenadas em mm de papel
    fig = plt.figure(figsize=(W / 25.4, H / 25.4))
    ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, W); ax.set_ylim(0, H); ax.set_aspect('equal'); ax.axis('off')
    CINZA = '#555555'
    ax.add_patch(Rectangle((10, 10), W - 20, H - 20, fill=False, lw=1.0, ec='k'))
    ax.text(14, H - 14, 'DEDO FIXO — REV. %s  ·  substitui a %s' % (d.rev, d.substitui),
            fontsize=7, color=CINZA, va='top')

    def rect(x, y, w, h, **kw):
        kw.setdefault('fill', False); kw.setdefault('lw', 1.0); kw.setdefault('ec', 'k')
        ax.add_patch(Rectangle((x, y), w, h, **kw))

    def seg(x0, y0, x1, y1, **kw):
        kw.setdefault('lw', 0.7); kw.setdefault('color', 'k')
        ax.plot([x0, x1], [y0, y1], **kw)

    def eixo(x0, y0, x1, y1):
        ax.plot([x0, x1], [y0, y1], color=CINZA, lw=0.5, dashes=(6, 2, 1, 2))

    def dim_h(x0, x1, y, txt, y_ref=None, acima=True):
        if y_ref is not None:
            seg(x0, y_ref, x0, y, color=CINZA, lw=0.5); seg(x1, y_ref, x1, y, color=CINZA, lw=0.5)
        ax.annotate('', xy=(x1, y), xytext=(x0, y), arrowprops=dict(arrowstyle='<|-|>', lw=0.7, color='k',
                                                                    mutation_scale=7, shrinkA=0, shrinkB=0))
        ax.text((x0 + x1) / 2, y + (1.6 if acima else -1.6), txt, ha='center', va='bottom' if acima else 'top', fontsize=7)

    def dim_v(x, y0, y1, txt, x_ref=None, direita=True):
        if x_ref is not None:
            seg(x_ref, y0, x, y0, color=CINZA, lw=0.5); seg(x_ref, y1, x, y1, color=CINZA, lw=0.5)
        ax.annotate('', xy=(x, y1), xytext=(x, y0), arrowprops=dict(arrowstyle='<|-|>', lw=0.7, color='k',
                                                                    mutation_scale=7, shrinkA=0, shrinkB=0))
        ax.text(x + (1.6 if direita else -1.6), (y0 + y1) / 2, txt, ha='left' if direita else 'right',
                va='center', fontsize=7, rotation=90)

    def titulo(x, y, t, sub):
        ax.text(x, y, t, ha='center', va='top', fontsize=9, fontweight='bold')
        ax.text(x, y - 4.5, sub, ha='center', va='top', fontsize=6.5, color=CINZA)

    h = d.haste / 2
    gx, gy = d.garfo_x / 2, d.garfo_y / 2
    dx0, dx1, dy0, dy1, dz0, dz1 = d.degrau_box()
    mais_y = d.degrau_eixo == '+y'

    # ================= VISTA DE BAIXO (plano XY) — canto superior esquerdo
    ox, oy = 70.0, 232.0
    rect(ox - gx, oy - gy, 2 * gx, 2 * gy, ls=(0, (2, 2)), lw=0.6, ec=CINZA)
    rect(ox + dx0, oy + dy0, dx1 - dx0, dy1 - dy0)
    rect(ox - h, oy - h, 2 * h, 2 * h)
    eixo(ox - 40, oy, ox + 40, oy); eixo(ox, oy - 25, ox, oy + 35)
    if not mais_y:
        dim_h(ox + dx0, ox + dx1, oy - 22, '%g' % (dx1 - dx0), y_ref=oy + dy0, acima=False)
        dim_v(ox + gx + 8, oy - h, oy + h, '%g' % (2 * h), x_ref=ox + dx1)
        dim_v(ox + gx + 16, oy - gy, oy + gy, '%g' % (2 * gy), x_ref=ox + gx)
    else:
        dim_h(ox - h, ox + h, oy - 22, '%g' % (2 * h), y_ref=oy - h, acima=False)
        dim_v(ox + gx + 8, oy + dy0, oy + dy1, '%g' % (dy1 - dy0), x_ref=ox + dx1)
        dim_v(ox + gx + 16, oy + h, oy + dy1, '%g (projeção)' % (dy1 - h), x_ref=ox + dx1)
        dim_h(ox - gx, ox + gx, oy + dy1 + 6, '%g' % (2 * gx), y_ref=oy + gy)
    titulo(ox, oy - 34, 'VISTA DE BAIXO', 'plano XY — o degrau visto pela ponta' +
           (' (sai em +Y, sentido das abas)' if mais_y else ''))

    # ================= VISTA DE FRENTE (plano XZ) — esquerda, embaixo
    ox, oy = 70.0, 170.0
    z = lambda zz: oy + zz
    rect(ox - gx, z(d.z_garfo), 2 * gx, d.garfo_h)
    ax.add_patch(Polygon([(ox - gx, z(d.z_garfo)), (ox + gx, z(d.z_garfo)), (ox + h, z(d.z_afun)), (ox - h, z(d.z_afun))],
                         closed=True, fill=False, lw=1.0))
    rect(ox - h, z(d.zh0), 2 * h, d.haste_l)
    if not mais_y:
        rect(ox + dx0, z(dz0), dx1 - dx0, dz1 - dz0)
    else:
        rect(ox - h, z(dz0), 2 * h, dz1 - dz0)
    for fx in d.furos_x:
        for fz in d.furos_z:
            ax.add_patch(Circle((ox + fx, z(fz)), d.furo_d / 2, fill=False, lw=0.8))
            eixo(ox + fx - 3, z(fz), ox + fx + 3, z(fz)); eixo(ox + fx, z(fz) - 3, ox + fx, z(fz) + 3)
    eixo(ox, z(d.z_fim) - 6, ox, z(0) + 6)
    dim_h(ox - gx, ox + gx, z(0) + 6, '%g' % d.garfo_x, y_ref=z(0))
    xr = ox + gx + 12
    dim_v(xr, z(d.z_garfo), z(0), '%g' % d.garfo_h, x_ref=ox + gx)
    dim_v(xr, z(d.z_afun), z(d.z_garfo), '%g' % d.afun_h, x_ref=ox + h)
    dim_v(xr, z(d.zh0), z(d.z_afun), '%g' % d.haste_l, x_ref=ox + h)
    dim_v(xr, z(d.z_fim), z(d.zh0), '%g' % d.degrau_z, x_ref=ox + (h if mais_y else dx1))
    dim_v(xr + 11, z(d.z_fim), z(0), '%g' % d.total)
    if not mais_y:
        dim_h(ox + dx0, ox + dx1, z(d.z_fim) - 8, '%g' % (dx1 - dx0), y_ref=z(d.z_fim), acima=False)
        dim_h(ox + dx0, ox - h, z(d.z_fim) - 16, '%g' % (-h - dx0), acima=False)
    else:
        dim_h(ox - h, ox + h, z(d.z_fim) - 8, '%g' % d.haste, y_ref=z(d.z_fim), acima=False)
    ax.annotate('4 × Ø%s passante' % ('%g' % d.furo_d).replace('.', ','), xy=(ox + d.furos_x[0] - d.furo_d / 2, z(d.furos_z[0])),
                xytext=(ox - gx - 26, z(0) + 3), fontsize=7, ha='left',
                arrowprops=dict(arrowstyle='-', lw=0.5, color='k', connectionstyle='arc3,rad=0.2'))
    titulo(ox, z(d.z_fim) - 24, 'VISTA DE FRENTE',
           'plano XZ — ' + ('a furação; o degrau aparece de ponta' if mais_y else 'o L do degrau e a furação'))

    # ================= VISTA LATERAL (plano YZ) — centro
    ox, oy = 165.0, 170.0
    z = lambda zz: oy + zz
    rect(ox - gy, z(d.z_garfo), d.aba, d.garfo_h); rect(ox + gy - d.aba, z(d.z_garfo), d.aba, d.garfo_h)
    if d.rasgo_prof < d.garfo_h:
        rect(ox - gy + d.aba, z(d.z_garfo), d.rasgo, d.garfo_h - d.rasgo_prof)
    for fz in d.furos_z:
        for (a, b) in ((ox - gy, ox - gy + d.aba), (ox + gy - d.aba, ox + gy)):
            seg(a, z(fz) + d.furo_d / 2, b, z(fz) + d.furo_d / 2, lw=0.5, dashes=(3, 2))
            seg(a, z(fz) - d.furo_d / 2, b, z(fz) - d.furo_d / 2, lw=0.5, dashes=(3, 2))
    ax.add_patch(Polygon([(ox - gy, z(d.z_garfo)), (ox + gy, z(d.z_garfo)), (ox + h, z(d.z_afun)), (ox - h, z(d.z_afun))],
                         closed=True, fill=False, lw=1.0))
    rect(ox - h, z(d.zh0), 2 * h, d.haste_l)
    if mais_y:
        rect(ox + dy0, z(dz0), dy1 - dy0, dz1 - dz0)
    else:
        rect(ox - h, z(dz0), 2 * h, dz1 - dz0)
    eixo(ox, z(d.z_fim) - 6, ox, z(0) + 6)
    dim_h(ox - gy, ox + gy, z(0) + 12, '%g' % d.garfo_y, y_ref=z(0))
    dim_h(ox - gy, ox - gy + d.aba, z(0) + 5, '%g' % d.aba)
    dim_h(ox - gy + d.aba, ox + gy - d.aba, z(0) + 5, '%g' % d.rasgo)
    dim_h(ox + gy - d.aba, ox + gy, z(0) + 5, '%g' % d.aba)
    xr = ox + (dy1 if mais_y else gy) + 8
    dim_v(xr, z(d.z_garfo), z(0), '%g' % d.garfo_h, x_ref=ox + gy)
    dim_v(xr, z(d.z_fim), z(d.zh0), '%g' % d.degrau_z, x_ref=ox + (dy1 if mais_y else h))
    if mais_y:
        dim_h(ox + dy0, ox + dy1, z(d.z_fim) - 8, '%g' % (dy1 - dy0), y_ref=z(d.z_fim), acima=False)
        dim_h(ox + h, ox + dy1, z(d.z_fim) - 16, '%g' % (dy1 - h), acima=False)
    else:
        dim_h(ox - h, ox + h, z(d.z_fim) - 8, '%g' % d.haste, y_ref=z(d.z_fim), acima=False)
    titulo(ox, z(d.z_fim) - 24, 'VISTA LATERAL',
           'plano YZ — as duas abas e ' + ('o L do degrau (em +Y)' if mais_y else 'a seção %g × %g' % (d.haste, d.haste)))

    # ================= DETALHE DA FURAÇÃO (2:1) — direita, em cima
    ox, oy, s = 290.0, 270.0, 2.0
    rect(ox - gx * s, oy - d.garfo_h * s, 2 * gx * s, d.garfo_h * s)
    for fx in d.furos_x:
        for fz in d.furos_z:
            ax.add_patch(Circle((ox + fx * s, oy + fz * s), d.furo_d / 2 * s, fill=False, lw=0.9))
            eixo(ox + fx * s - 5, oy + fz * s, ox + fx * s + 5, oy + fz * s)
            eixo(ox + fx * s, oy + fz * s - 5, ox + fx * s, oy + fz * s + 5)
    dim_h(ox - gx * s, ox + gx * s, oy + 12, '%g' % d.garfo_x, y_ref=oy)
    bx = [-gx, d.furos_x[0], d.furos_x[1], gx]
    for a, b in zip(bx[:-1], bx[1:]):
        dim_h(ox + a * s, ox + b * s, oy + 5, '%g' % (b - a))
    bz = [0.0, d.furos_z[0], d.furos_z[1], -d.garfo_h]
    for a, b in zip(bz[:-1], bz[1:]):
        dim_v(ox + gx * s + 6, oy + b * s, oy + a * s, '%g' % (a - b), x_ref=ox + gx * s)
    dim_v(ox + gx * s + 15, oy - d.garfo_h * s, oy, '%g' % d.garfo_h)
    ax.text(ox, oy - d.garfo_h * s - 8, '4 × Ø%s passante — parafuso M3' % ('%g' % d.furo_d).replace('.', ','), ha='center', fontsize=7)
    titulo(ox, oy - d.garfo_h * s - 15, 'DETALHE DA FURAÇÃO', 'escala 2:1 — aba vista por fora (face Y = −%g)' % gy)

    # ================= PERSPECTIVA ISOMÉTRICA — direita, meio
    # Projeção: +X vai para a direita e para cima, +Y para a esquerda e
    # para cima; o observador está no lado −X, −Y, +Z. Faces visíveis de
    # uma caixa: −X, −Y e topo. Pintura de trás para a frente: peças de
    # baixo antes das de cima e, no mesmo nível, maior (x + y) primeiro.
    ox, oy, si = 352.0, 204.0, 0.42     # escala reduzida: é só orientação (cabe até 125 mm de peça)
    def iso(p):
        x, y, zz = p
        return ox + si * (x - y) * np.cos(np.radians(30)), oy + si * (zz + (x + y) * np.sin(np.radians(30)))
    def poly(pts, fc):
        ax.add_patch(Polygon([iso(p) for p in pts], closed=True, fill=True, fc=fc, ec='k', lw=0.6))
    TOPO, FY, FX = '#f4dd7a', '#d9c25f', '#b89f3a'
    def caixa_iso(x0, x1, y0, y1, z0, z1):
        poly([(x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1)], FX)     # face −X
        poly([(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)], FY)     # face −Y
        poly([(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)], TOPO)   # topo
    caixa_iso(dx0, dx1, dy0, dy1, dz0, dz1)                                    # degrau
    caixa_iso(-h, h, -h, h, d.zh0, d.z_afun)                                   # haste
    poly([(-gx, -gy, d.z_garfo), (-gx, gy, d.z_garfo), (-h, h, d.z_afun), (-h, -h, d.z_afun)], FX)   # rampa −X
    poly([(-gx, -gy, d.z_garfo), (gx, -gy, d.z_garfo), (h, -h, d.z_afun), (-h, -h, d.z_afun)], FY)   # rampa −Y
    poly([(-gx, -gy, d.z_garfo), (gx, -gy, d.z_garfo), (gx, gy, d.z_garfo), (-gx, gy, d.z_garfo)], TOPO)  # topo da rampa
    if d.rasgo_prof < d.garfo_h:
        caixa_iso(-gx, gx, -gy + d.aba, gy - d.aba, d.z_garfo, -d.rasgo_prof)  # fundo do U (v1)
    def furos_iso(yface):
        if not d.furos_no_stl:
            return
        for fx in d.furos_x:
            for fz in d.furos_z:
                pts = [iso((fx + d.furo_d / 2 * np.cos(t), yface, fz + d.furo_d / 2 * np.sin(t)))
                       for t in np.linspace(0, 2 * np.pi, 24)]
                ax.add_patch(Polygon(pts, closed=True, fill=True, fc='white', lw=0.5, ec='k'))
    caixa_iso(-gx, gx, gy - d.aba, gy, d.z_garfo, 0); furos_iso(gy - d.aba)   # aba de trás (+Y)
    caixa_iso(-gx, gx, -gy, -gy + d.aba, d.z_garfo, 0); furos_iso(-gy)        # aba da frente (−Y)
    titulo(ox + 2, oy - si * d.total - 16, 'PERSPECTIVA ISOMÉTRICA', 'somente orientação — sem cotas')

    # ================= NOTAS
    nx, ny = 238.0, 121.0
    ax.text(nx, ny, 'NOTAS', fontsize=8, fontweight='bold', va='top')
    dec = lambda x: ('%g' % x).replace('.', ',')
    notas = [
        'Cotas em milímetros. Tolerância geral ±0,3 mm (impressão FDM).',
        'Origem no topo do garfo, Z para baixo — mesma convenção do movemaster.urdf.xacro.',
        ('Os 4 furos M3 já estão abertos no STL — imprimir direto, sem furar depois. Furo de Ø%s em FDM\n'
         '    costuma fechar um pouco: se o M3 não passar, calibrar com broca de 3,5 sem forçar, apoiando a aba.' % dec(d.furo_d))
        if d.furos_no_stl else 'O STL não traz os furos: fure na impressão ou subtraia no fatiador.',
        ('Imprimir DEITADO (haste e degrau no plano da mesa): as camadas correm ao longo da haste e o\n'
         '    degrau trabalha em tração, não em delaminação. Perímetros ≥ 4, preenchimento ≥ 60 %.'),
        ('O rasgo de %s mm é passante em toda a altura do garfo: as duas abas abraçam a aba da\n'
         '    castanha HM-01 e se apoiam na rampa.' % dec(d.rasgo))
        if d.rasgo_prof >= d.garfo_h else 'O vão de %g mm abraça a aba da castanha HM-01.' % d.rasgo,
        ('Haste e degrau formam um L reto — sem chanfro. REV. C: o degrau sai em +Y, no sentido das ABAS\n'
         '    (girado 90° em relação à REV. B). O olhal assenta no degrau na descida do dedo (Notebook 18, p. 7 e 10).')
        if mais_y else
        ('Haste e degrau formam um L reto — sem chanfro. O olhal da chave seccionadora assenta no degrau\n'
         '    durante a descida do dedo (ver Notebook 18, p. 7 e 10).'),
    ]
    yy = ny - 6
    for k, n in enumerate(notas, 1):
        ax.text(nx, yy, '%d.  %s' % (k, n), fontsize=6.0, va='top', linespacing=1.45)
        yy -= 3.9 * (n.count('\n') + 1) + 1.6

    # ================= LEGENDA
    x0, y0, x1, y1 = 235.0, 12.0, 410.0, 60.0
    rect(x0, y0, x1 - x0, y1 - y0, lw=1.0)
    seg(x0, y1 - 14, x1, y1 - 14); seg(x0, y0 + 9, x1, y0 + 9); seg(x0 + 108, y0 + 9, x0 + 108, y1 - 14)
    ax.text(x0 + 3, y1 - 4, 'DEDO FIXO', fontsize=12, fontweight='bold', va='top')
    ax.text(x0 + 3, y1 - 10.5, 'ferramenta de manobra do braço RV-M2 — projeto b166er', fontsize=6.5, color=CINZA, va='top')
    ax.text(x1 - 3, y1 - 4, 'B166ER-FER-001   REV. %s' % d.rev, fontsize=8, ha='right', va='top')
    vol = d.volume_mm3() / 1000.0
    esq = [('MATERIAL', 'PLA / PETG — impressão 3D'), ('ACABAMENTO', 'conforme impressão — rebarbar furos'),
           ('VOLUME', ('%.1f cm³ (sólido) — ~%.0f g em PLA' % (vol, vol * DENS_PLA)).replace('.', ',')),
           ('TOLERÂNCIA GERAL', '±0,3 mm   |   ângulos ±1°')]
    dir_ = [('DESENHO', 'Marco Reis'), ('DATA', d.data), ('ESCALA', '1:1'), ('UNIDADE', 'mm')]
    for k, (a, b) in enumerate(esq):
        yy = y1 - 18 - k * 5.2
        ax.text(x0 + 3, yy, a, fontsize=5.5, color=CINZA, va='center'); ax.text(x0 + 35, yy, b, fontsize=6.5, va='center')
    for k, (a, b) in enumerate(dir_):
        yy = y1 - 18 - k * 5.2
        ax.text(x0 + 111, yy, a, fontsize=5.5, color=CINZA, va='center'); ax.text(x0 + 135, yy, b, fontsize=6.5, va='center')
    ax.text(x0 + 3, y0 + 4.5, 'PROJEÇÃO NO 1º DIEDRO', fontsize=5.5, color=CINZA, va='center')
    ax.text(x1 - 3, y0 + 4.5, 'gerado por gera_dedo.py --versao %d — não editar à mão' % d.v,
            fontsize=5.5, color=CINZA, va='center', ha='right')

    fig.savefig(caminho_base + '.pdf'); fig.savefig(caminho_base + '.png', dpi=200)


# ---------------------------------------------------------------- prancha antiga (só a v1)
def desenho_v1(caminho_base, d):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Polygon, Circle
    GARFO_H, GARFO_X, GARFO_Y, AFUN_H = d.garfo_h, d.garfo_x, d.garfo_y, d.afun_h
    HASTE_L, HASTE_X, HASTE_Y, DEGRAU_L, DEGRAU_Z = d.haste_l, d.haste, d.haste, d.degrau_l, d.degrau_z
    furo_dx, furo_dz, furo_d, rasgo, rasgo_prof, aba = d.furo_dx, d.furo_dz, d.furo_d, d.rasgo, d.rasgo_prof, d.aba
    fig = plt.figure(figsize=(16.5, 11.7))
    fig.suptitle('DEDO FIXO v1 — ferramenta de manobra do RV-M2 (b166er)   ·   cotas em mm   ·   '
                 'degrau em −X, eixo do lado de 20 (2026-09-02)', fontsize=13, y=0.98)

    def cota(ax, p0, p1, txt, off=(0, 0), rot=0):
        (x0, y0), (x1, y1) = p0, p1
        ax.annotate('', xy=(x1, y1), xytext=(x0, y0), arrowprops=dict(arrowstyle='<->', lw=0.8, color='k'))
        ax.text((x0 + x1) / 2 + off[0], (y0 + y1) / 2 + off[1], txt, ha='center', va='center', fontsize=9,
                rotation=rot, bbox=dict(fc='white', ec='none', pad=1))

    zh0 = -GARFO_H - AFUN_H - HASTE_L
    ax = fig.add_subplot(2, 2, 1); ax.set_title('Vista de frente (X–Z) — o L do degrau')
    ax.add_patch(Rectangle((-GARFO_X / 2, -GARFO_H), GARFO_X, GARFO_H, fill=False, lw=1.2))
    ax.add_patch(Polygon([(-GARFO_X / 2, -GARFO_H), (GARFO_X / 2, -GARFO_H), (HASTE_X / 2, -GARFO_H - AFUN_H),
                          (-HASTE_X / 2, -GARFO_H - AFUN_H)], closed=True, fill=False, lw=1.2))
    ax.add_patch(Rectangle((-HASTE_X / 2, zh0), HASTE_X, HASTE_L, fill=False, lw=1.2))
    ax.add_patch(Rectangle((-HASTE_X / 2 - DEGRAU_L, zh0 - DEGRAU_Z), DEGRAU_L + HASTE_X, DEGRAU_Z, fill=False, lw=1.2))
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
    ax = fig.add_subplot(2, 2, 2); ax.set_title('Vista lateral (Y–Z) — o U do garfo e a seção 10 × 10')
    ax.add_patch(Rectangle((-GARFO_Y / 2, -GARFO_H), GARFO_Y, GARFO_H, fill=False, lw=1.2))
    ax.add_patch(Rectangle((-rasgo / 2, -rasgo_prof), rasgo, rasgo_prof, fill=False, lw=1.2, hatch='//'))
    ax.add_patch(Polygon([(-GARFO_Y / 2, -GARFO_H), (GARFO_Y / 2, -GARFO_H), (HASTE_Y / 2, -GARFO_H - AFUN_H),
                          (-HASTE_Y / 2, -GARFO_H - AFUN_H)], closed=True, fill=False, lw=1.2))
    ax.add_patch(Rectangle((-HASTE_Y / 2, zh0 - DEGRAU_Z), HASTE_Y, HASTE_L + DEGRAU_Z, fill=False, lw=1.2))
    for cz in (-GARFO_H / 2 - furo_dz / 2, -GARFO_H / 2 + furo_dz / 2):
        ax.plot([-GARFO_Y / 2, GARFO_Y / 2], [cz, cz], 'k--', lw=0.6)
    cota(ax, (-GARFO_Y / 2, 6), (GARFO_Y / 2, 6), '%g' % GARFO_Y, off=(0, 3))
    cota(ax, (-rasgo / 2, 12), (rasgo / 2, 12), 'vão %g (aba da castanha: 10)' % rasgo, off=(0, 3))
    cota(ax, (-GARFO_Y / 2, -GARFO_H - 4), (-GARFO_Y / 2 + aba, -GARFO_H - 4), 'aba %g' % aba, off=(0, -3))
    cota(ax, (GARFO_Y / 2 + 6, 0), (GARFO_Y / 2 + 6, -rasgo_prof), 'vão prof. %g' % rasgo_prof, off=(6, 0), rot=90)
    cota(ax, (-HASTE_Y / 2, -60), (HASTE_Y / 2, -60), '10', off=(0, 3))
    ax.set_xlim(-50, 60); ax.set_ylim(-135, 18); ax.set_aspect('equal'); ax.axis('off')
    ax = fig.add_subplot(2, 2, 3); ax.set_title('Vista de baixo (X–Y) — o degrau visto pela ponta')
    ax.add_patch(Rectangle((-GARFO_X / 2, -GARFO_Y / 2), GARFO_X, GARFO_Y, fill=False, lw=0.8, ls=':'))
    ax.add_patch(Rectangle((-HASTE_X / 2 - DEGRAU_L, -HASTE_Y / 2), DEGRAU_L + HASTE_X, HASTE_Y, fill=False, lw=1.2))
    ax.add_patch(Rectangle((-HASTE_X / 2, -HASTE_Y / 2), HASTE_X, HASTE_Y, fill=False, lw=1.2))
    cota(ax, (-HASTE_X / 2 - DEGRAU_L, -14), (HASTE_X / 2, -14), '30', off=(0, -3))
    cota(ax, (-HASTE_X / 2 - DEGRAU_L, 12), (-HASTE_X / 2, 12), '20', off=(0, 3))
    cota(ax, (12, -HASTE_Y / 2), (12, HASTE_Y / 2), '10', off=(4, 0), rot=90)
    ax.text(0, -22, 'garfo 20 × %g (pontilhado) — o degrau sai em −X, no eixo da haste' % GARFO_Y, fontsize=8, ha='center')
    ax.set_xlim(-50, 50); ax.set_ylim(-30, 30); ax.set_aspect('equal'); ax.axis('off')
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
    caixa_iso(-GARFO_X / 2, GARFO_X / 2, -GARFO_Y / 2, GARFO_Y / 2, -GARFO_H, -rasgo_prof)
    caixa_iso(-GARFO_X / 2, GARFO_X / 2, -GARFO_Y / 2, -GARFO_Y / 2 + aba, -rasgo_prof, 0)
    caixa_iso(-GARFO_X / 2, GARFO_X / 2, GARFO_Y / 2 - aba, GARFO_Y / 2, -rasgo_prof, 0)
    ax.text(-120, -60, 'Notas:\n• Z para baixo a partir do topo do garfo, como no URDF.\n'
            '• Imprimir DEITADO (haste e degrau no plano da mesa).\n• Rampa 20×25 → 10×10 é uma transição contínua (tronco).\n'
            '• O U do garfo abraça a aba de 10 mm da castanha da HM-01\n  (vão 10,4); 4 furos M3 (Ø3,5) por aba em grade 2×2.\n'
            '• O STL não traz os furos: fure na impressão ou no fatiador.', fontsize=8.5, va='center', ha='left')
    ax.set_xlim(-125, 60); ax.set_ylim(-150, 30); ax.set_aspect('equal'); ax.axis('off')
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(caminho_base + '.pdf'); fig.savefig(caminho_base + '.png', dpi=150)


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--versao', type=int, choices=(1, 2, 3), default=3,
                    help='1: REV. A (09 Set, haste 80); 2: REV. B reproduzida (haste 30, furos, degrau em -X); '
                         '3: REV. C (padrão) — v2 com o degrau girado 90°, em +Y')
    ap.add_argument('--furo-dx', type=float, default=None, help='espaçamento dos furos em X (mm)')
    ap.add_argument('--furo-dz', type=float, default=None, help='espaçamento dos furos em Z (mm)')
    ap.add_argument('--furo-d', type=float, default=None, help='diâmetro dos furos (mm)')
    ap.add_argument('--rasgo', type=float, default=None, help='vão do garfo em Y (mm)')
    ap.add_argument('--aba', type=float, default=None, help='espessura de cada aba do garfo em Y (mm)')
    ap.add_argument('--saida', default='.', help='pasta de saída')
    a = ap.parse_args()
    d = Dedo(a.versao, furo_dx=a.furo_dx, furo_dz=a.furo_dz, furo_d=a.furo_d, rasgo=a.rasgo, aba=a.aba)
    os.makedirs(a.saida, exist_ok=True)
    tris = solido(d)
    stl = os.path.join(a.saida, 'dedo_fixo_v%d.stl' % d.v)
    escreve_stl(stl, tris, 'DEDO FIXO rev %s (mm) - b166er - gera_dedo.py --versao %d' % (d.rev, d.v))
    base = os.path.join(a.saida, 'dedo_fixo_v%d_desenho' % d.v)
    (desenho_v1 if d.v == 1 else prancha)(base, d)
    pts = np.vstack(tris)
    print('%s: %d triângulos, bbox X %.1f..%.1f  Y %.1f..%.1f  Z %.1f..%.1f mm, volume %.1f cm³' % (
        stl, len(tris), pts[:, 0].min(), pts[:, 0].max(), pts[:, 1].min(), pts[:, 1].max(),
        pts[:, 2].min(), pts[:, 2].max(), d.volume_mm3() / 1000.0))
    print('desenho: %s.pdf / .png' % base)


if __name__ == '__main__':
    main()
