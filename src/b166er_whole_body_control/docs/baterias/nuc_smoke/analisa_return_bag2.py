#!/usr/bin/env python3
"""Velocidade real da base em CADA trecho de navegação da missão, com a
postura do braço e o teto vigentes — para ver se o RETURN é o único que
rasteja e o que o distingue."""
import sys, math
import rosbag

bag = rosbag.Bag(sys.argv[1])
ev = []
for _, m, t in bag.read_messages(topics=['/rosout']):
    if 'chave_mission' in m.name:
        ev.append((m.header.stamp.to_sec(), m.msg))
# janelas: "<tag>: alinhado, avançando (d m)" -> "<tag>: chegou"
jan = []
for i, (st, s) in enumerate(ev):
    if 'alinhado, avançando' in s:
        tag = s.split('[mission] ')[1].split(':')[0]
        d = float(s.split('avançando (')[1].split(' m')[0])
        for st2, s2 in ev[i+1:]:
            if tag + ': chegou' in s2 or 'timeout' in s2:
                jan.append((tag, d, st, st2)); break
odom = [(t.to_sec(), m.pose.pose.position.x, m.pose.pose.position.y,
         m.twist.twist.linear.x) for _, m, t in bag.read_messages(topics=['/pioneer3at/odom'])]
cmd = [(t.to_sec(), m.linear.x) for _, m, t in bag.read_messages(topics=['/cmd_vel'])]
rs = []
for _, m, t in bag.read_messages(topics=['/b166er/robot_state']):
    q = list(m.q_arm) if hasattr(m, 'q_arm') else []
    rs.append((t.to_sec(), q))
cap = [(t.to_sec(), list(m.data)) for _, m, t in bag.read_messages(topics=['/b166er/base_cap'])]

print('%-16s %6s %7s %8s %8s %8s  %s' % ('trecho', 'dist', 'dt_sim', 'v_odom', 'v_twist', 'v_cmd', 'q_arm médio (graus)'))
for tag, d, a, b in jan:
    oj = [o for o in odom if a <= o[0] <= b]
    cj = [c[1] for c in cmd if a <= c[0] <= b]
    qj = [q for st, q in rs if a <= st <= b and q]
    if len(oj) < 2:
        continue
    dist = math.hypot(oj[-1][1]-oj[0][1], oj[-1][2]-oj[0][2])
    vtw = sum(o[3] for o in oj)/len(oj)
    vcmd = sum(cj)/len(cj) if cj else float('nan')
    qm = [sum(q[k] for q in qj)/len(qj) for k in range(len(qj[0]))] if qj else []
    print('%-16s %6.2f %7.1f %8.3f %8.3f %8.3f  %s' % (tag, d, b-a, dist/max(b-a,1e-3), vtw, vcmd,
          [round(math.degrees(x), 1) for x in qm]))
    cj2 = [c for c in cap if a <= c[0] <= b]
    if cj2:
        print('%-16s teto: v_cap %.2f w_cap %.2f a_lin %.2f' % ('', min(c[1][0] for c in cj2), min(c[1][1] for c in cj2), min(c[1][2] for c in cj2)))

# perfil temporal do RETURN: v_twist da odom por 5 s
print('\n=== RETURN: v_twist da odom a cada 5 s sim ===')
for tag, d, a, b in jan:
    if tag != 'RETURN':
        continue
    k = a
    while k < b:
        oj = [o[3] for o in odom if k <= o[0] < k+5]
        cj = [c[1] for c in cmd if k <= c[0] < k+5]
        if oj:
            print('  %6.1f–%6.1f  v_odom %.3f  v_cmd %.3f  n_cmd %d' % (k-a, k-a+5, sum(oj)/len(oj), (sum(cj)/len(cj)) if cj else -1, len(cj)))
        k += 5
bag.close()
