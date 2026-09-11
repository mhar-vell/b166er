#!/usr/bin/env python3
"""Trajetória fina do RETURN: distância ao alvo, erro de rumo e comando
angular a cada 0,5 s — a base passou perto do alvo e não entrou na
tolerância?"""
import sys, math, bisect
import rosbag
from tf.transformations import euler_from_quaternion

GOAL = (0.0, 1.0)
TOL = 0.06
bag = rosbag.Bag(sys.argv[1])
ev = [(m.header.stamp.to_sec(), m.msg) for _, m, t in bag.read_messages(topics=['/rosout']) if 'chave_mission' in m.name]
a = b = None
for st, s in ev:
    if 'RETURN — voltando' in s: a0 = st
    if 'RETURN: alinhado' in s and a is None: a = st
    if a and ('RETURN: chegou' in s or 'timeout' in s): b = st; break
gz = [(t.to_sec(), m.pose.pose.position.x, m.pose.pose.position.y,
       euler_from_quaternion([m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w])[2])
      for _, m, t in bag.read_messages(topics=['/pioneer3at/odom'])]
cmd = [(t.to_sec(), m.linear.x, m.angular.z) for _, m, t in bag.read_messages(topics=['/cmd_vel'])]
ct = [c[0] for c in cmd]

def wrap(x): return math.atan2(math.sin(x), math.cos(x))

print('RETURN voltando %.1f, alinhado %.1f (fase TURN %.1f s), fim %.1f' % (a0, a, a - a0, b))
print('%6s %7s %7s %7s %8s %8s %7s' % ('t', 'x', 'y', 'yaw°', 'dist', 'err°', 'cmd_w'))
dmin = (9, None)
k = a - 1.0
while k < min(b, a + 40):
    oj = [g for g in gz if k <= g[0] < k + 0.5]
    if oj:
        g = oj[0]
        dist = math.hypot(GOAL[0] - g[1], GOAL[1] - g[2])
        bearing = math.atan2(GOAL[1] - g[2], GOAL[0] - g[1])
        err = math.degrees(wrap(bearing - g[3]))
        i = bisect.bisect_left(ct, g[0]) - 1
        w = cmd[i][2] if i >= 0 else float('nan')
        print('%6.1f %7.3f %7.3f %7.1f %8.3f %8.1f %7.3f' % (g[0] - a, g[1], g[2], math.degrees(g[3]), dist, err, w))
    k += 0.5
# menor distância ao alvo em toda a janela (amostragem 100 Hz)
dm = min((math.hypot(GOAL[0]-g[1], GOAL[1]-g[2]), g[0]-a) for g in gz if a <= g[0] <= b)
print('\nmenor distância ao alvo na janela: %.3f m em t+%.1f s (tol %.2f)' % (dm[0], dm[1], TOL))
# quantas passagens a < 0,30 m
passes = 0; dentro = False
for g in gz:
    if not (a <= g[0] <= b): continue
    d = math.hypot(GOAL[0]-g[1], GOAL[1]-g[2])
    if d < 0.30 and not dentro: passes += 1; dentro = True
    elif d >= 0.30: dentro = False
print('passagens a menos de 0,30 m do alvo: %d' % passes)
bag.close()
