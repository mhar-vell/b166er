#!/usr/bin/env python3
"""Pose do estimador (robot_state.base_odom) × pose do Gazebo (odom) e o
comando angular durante cada trecho de navegação: o rumo que a missão vê
é o rumo real?"""
import sys, math, bisect
import rosbag
from tf.transformations import euler_from_quaternion

bag = rosbag.Bag(sys.argv[1])
ev = [(m.header.stamp.to_sec(), m.msg) for _, m, t in bag.read_messages(topics=['/rosout']) if 'chave_mission' in m.name]
jan = []
for i, (st, s) in enumerate(ev):
    if 'alinhado, avançando' in s:
        tag = s.split('[mission] ')[1].split(':')[0]
        for st2, s2 in ev[i+1:]:
            if tag + ': chegou' in s2 or 'timeout' in s2:
                jan.append((tag, st, st2)); break

def yaw_of(o):
    return euler_from_quaternion([o.x, o.y, o.z, o.w])[2]

gz = [(t.to_sec(), m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
      for _, m, t in bag.read_messages(topics=['/pioneer3at/odom'])]
es = []
for _, m, t in bag.read_messages(topics=['/b166er/robot_state']):
    p = m.base_odom.pose.pose
    es.append((t.to_sec(), p.position.x, p.position.y, yaw_of(p.orientation), m.base_odom.header.stamp.to_sec()))
cmd = [(t.to_sec(), m.linear.x, m.angular.z) for _, m, t in bag.read_messages(topics=['/cmd_vel'])]
gt = [g[0] for g in gz]

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

print('taxa robot_state: %.1f Hz sim; odom gazebo: %.1f Hz sim' % (len(es)/(es[-1][0]-es[0][0]), len(gz)/(gz[-1][0]-gz[0][0])))
for tag, a, b in jan:
    print('\n== %s (%.1f s) ==' % (tag, b - a))
    ej = [e for e in es if a <= e[0] <= b]
    cj = [c for c in cmd if a <= c[0] <= b]
    if cj:
        wz = [c[2] for c in cj]
        print('  cmd w: med %.3f  |w| med %.3f  max %.3f' % (sum(wz)/len(wz), sum(abs(x) for x in wz)/len(wz), max(abs(x) for x in wz)))
    dy, dp, lag = [], [], []
    for e in ej:
        i = bisect.bisect_left(gt, e[0]) - 1
        if i < 0: continue
        g = gz[i]
        dy.append(math.degrees(wrap(e[3] - g[3])))
        dp.append(math.hypot(e[1]-g[1], e[2]-g[2]))
        lag.append(e[0] - e[4])
    if dy:
        print('  yaw_est - yaw_gz (graus): med %.2f  |.| med %.2f  max %.2f' % (sum(dy)/len(dy), sum(abs(x) for x in dy)/len(dy), max(abs(x) for x in dy)))
        print('  |pos_est - pos_gz| (m): med %.3f  max %.3f' % (sum(dp)/len(dp), max(dp)))
        print('  atraso do stamp do base_odom vs recepção (s sim): med %.3f  max %.3f' % (sum(lag)/len(lag), max(lag)))
    # perfil a cada 10 s no RETURN
    if tag == 'RETURN':
        k = a
        while k < b:
            ejk = [e for e in ej if k <= e[0] < k+10]
            cjk = [c[2] for c in cj if k <= c[0] < k+10]
            if ejk:
                e = ejk[-1]; i = bisect.bisect_left(gt, e[0]) - 1; g = gz[i]
                print('  t+%3.0f: est yaw %6.1f° gz yaw %6.1f° | est (%.2f,%.2f) gz (%.2f,%.2f) | cmd w med %.3f'
                      % (k-a, math.degrees(e[3]), math.degrees(g[3]), e[1], e[2], g[1], g[2], (sum(cjk)/len(cjk)) if cjk else 0))
            k += 10
bag.close()
