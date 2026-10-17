#!/usr/bin/env python3
"""Lê sonda_engate.csv (+ contatos.txt) de uma run e imprime, da captura ao fim da manipulação, a posição do degrau
em relação ao centro do anel (eixo/prof/alt, mm), o ângulo da lâmina e os contatos da ferramenta, a 0,5 s; marca a perda
(PERDEU/NÃO SOLTA no run.log) e detalha o segundo anterior a 20 Hz."""
import sys, csv, re, os
d = sys.argv[1]
rows = list(csv.DictReader(open(os.path.join(d, 'sonda_engate.csv'))))
log = open(os.path.join(d, 'run.log'), errors='replace').read()
res = re.findall(r'MISSION_(OK|ABORTED)', log); res = res[-1] if res else '?'
perda = re.search(r'\[([0-9.]+)\]: \[mission\] fase "([a-z0-9_]+)": ferramenta PERDEU O OLHAL — ponta derivou ([-+0-9.]+) mm', log)
nsolta = re.search(r'\[([0-9.]+)\]: \[mission\] arco1: SOLTURA observável — recuo ([0-9.]+) mm desde a captura \(mín 40\) com deriva de eixo ([-+0-9.]+) mm → (SOLTA|NÃO SOLTA)', log)
print("== %s resultado %s" % (d, res))
if perda: print("   PERDEU O OLHAL em %s na fase %s, deriva %s mm" % (perda.group(1), perda.group(2), perda.group(3)))
if nsolta: print("   arco1: recuo %s mm, deriva %s mm → %s (t %s)" % (nsolta.group(2), nsolta.group(3), nsolta.group(4), nsolta.group(1)))
fases = [r['fase'] for r in rows]
try:
    i0 = next(i for i, r in enumerate(rows) if r['fase'] == 'captura')
except StopIteration:
    print("   sem fase captura na sonda"); sys.exit(0)
t_fim = float(perda.group(1)) if perda else None
cont = {}
cp = os.path.join(d, 'contatos.txt')
if os.path.exists(cp):
    for ln in open(cp, errors='replace'):
        m = re.match(r'([0-9.]+) "([^"]+)" "([^"]+)" F=([0-9.]+)', ln)
        if not m: continue
        t = float(m.group(1)); k = round(t * 2) / 2
        def nome(c):
            # colisões "lumped" pelo gazebo: ...::L5::L5_fixed_joint_lump__tool_tip_collision_3 → tool_tip
            mm = re.search(r'lump__(.+?)_collision', c)
            if mm: return mm.group(1).replace('chave_', '').replace('_link', '')
            return c.split('::')[1].replace('chave_', '')
        a = nome(m.group(2)); b = nome(m.group(3))
        cont.setdefault(k, {}); kk = a + '×' + b
        cont[k][kk] = max(cont[k].get(kk, 0.0), float(m.group(4)))
print("   t(sim)  fase            lâmina  degrau−anel: eixo  prof   alt  (mm)  lingueta  contatos (link×link F máx)")
ultimo = None
for r in rows[i0:]:
    t = float(r['sim'])
    if t_fim and t > t_fim + 1.0: break
    if ultimo is not None and t - ultimo < 0.5 and not (t_fim and t_fim - 1.2 < t <= t_fim + 0.3): continue
    ultimo = t
    k = round(t * 2) / 2; c = cont.get(k, {})
    cs = ' '.join('%s %.0f' % (n.replace('chave_', '').replace('tool_', 't_'), f) for n, f in sorted(c.items(), key=lambda x: -x[1])[:3])
    print("   %8.2f %-15s %5s°   %6s %6s %6s      %5s   %s" % (t, r['fase'][:15], r['blade_deg'], r['d_eixo_mm'], r['d_prof_mm'], r['d_alt_mm'], r['lingueta_mm'], cs))
