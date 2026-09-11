#!/usr/bin/env python3
"""Quem fala em /cmd_vel durante o RETURN? Lê o bag da missão no NUC."""
import sys, math
import rosbag

bag = rosbag.Bag(sys.argv[1])
# janelas pelo rosout da missão
marcos = {}
rosout = []
for _, m, t in bag.read_messages(topics=['/rosout']):
    s = m.msg
    st = m.header.stamp.to_sec()
    rosout.append((st, m.name, s))
    for k in ('RETURN: alinhado', 'RETURN: chegou', 'RETURN: concluído',
              'RETURN — voltando', 'RETRACT\':\'ok', 'ABORT/retorno: alinhado',
              'ABORT/retorno: chegou'):
        if k in s and k not in marcos:
            marcos[k] = st
for k, v in sorted(marcos.items(), key=lambda kv: kv[1]):
    print('%10.3f  %s' % (v, k))

t0 = marcos.get('RETURN: alinhado'); t1 = marcos.get('RETURN: chegou')
if t0 is None:
    sys.exit('sem RETURN no bag')
t1 = t1 or (t0 + 120.0)
print('\n=== janela RETURN/DRIVE: %.1f s sim ===' % (t1 - t0))

# cmd_vel na janela: contagem, zeros, taxa, sequências
cmd = [(t.to_sec(), m.linear.x, m.angular.z)
       for _, m, t in bag.read_messages(topics=['/cmd_vel'])]
jan = [c for c in cmd if t0 <= c[0] <= t1]
zeros = [c for c in jan if abs(c[1]) < 1e-6 and abs(c[2]) < 1e-6]
naozero = [c for c in jan if not (abs(c[1]) < 1e-6 and abs(c[2]) < 1e-6)]
print('cmd_vel: %d msgs (%.1f Hz sim) — %d não-nulas, %d ZEROS'
      % (len(jan), len(jan) / max(t1 - t0, 1e-3), len(naozero), len(zeros)))
if naozero:
    vs = [c[1] for c in naozero]
    print('  v não-nulo: min %.3f  med %.3f  max %.3f' % (min(vs), sum(vs)/len(vs), max(vs)))
# padrão de intercalação: quantas trocas zero<->nãozero
trocas = sum(1 for a, b in zip(jan, jan[1:])
             if (abs(a[1]) < 1e-6) != (abs(b[1]) < 1e-6))
print('  trocas zero<->não-zero na janela: %d' % trocas)

# odom: velocidade média real na janela
od = [(t.to_sec(), m.pose.pose.position.x, m.pose.pose.position.y)
      for _, m, t in bag.read_messages(topics=['/pioneer3at/odom'])]
oj = [o for o in od if t0 <= o[0] <= t1]
if len(oj) > 1:
    d = math.hypot(oj[-1][1] - oj[0][1], oj[-1][2] - oj[0][2])
    print('odom: %.2f m em %.1f s → %.3f m/s' % (d, oj[-1][0] - oj[0][0], d / max(oj[-1][0] - oj[0][0], 1e-3)))

# quem logou na janela (além da missão)
print('\n=== rosout de outros nós na janela ===')
for st, n, s in rosout:
    if t0 - 2 <= st <= t1 + 2 and 'chave_mission' not in n:
        print('%10.3f %-22s %s' % (st, n, s[:100]))

# fora da janela, para comparar: cmd_vel zeros por fase inteira
print('\n=== cmd_vel no bag inteiro: zeros por trecho de 30 s sim ===')
if cmd:
    tb = cmd[0][0]
    bins = {}
    for c in cmd:
        k = int((c[0] - tb) // 30)
        z = abs(c[1]) < 1e-6 and abs(c[2]) < 1e-6
        bins.setdefault(k, [0, 0])[1 if z else 0] += 1
    for k in sorted(bins):
        nz, z = bins[k]
        print('  %4d–%4d s: %4d não-nulas  %4d zeros' % (30*k, 30*k+30, nz, z))
bag.close()
