#!/usr/bin/env python3
"""Atitude da base (z, roll, pitch), giro, folga do laser e campos do
robot_state em cada trecho de navegação — a base está inclinada/erguida
no RETURN?"""
import sys, math
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
odom = []
for _, m, t in bag.read_messages(topics=['/pioneer3at/odom']):
    o = m.pose.pose.orientation
    r, p, y = euler_from_quaternion([o.x, o.y, o.z, o.w])
    odom.append((t.to_sec(), m.pose.pose.position.z, r, p, y, m.twist.twist.angular.z))
clr = [(t.to_sec(), m.data) for _, m, t in bag.read_messages(topics=['/b166er/front_clearance'])]
rs0 = next(bag.read_messages(topics=['/b166er/robot_state']))[1]
print('robot_state campos:', [s for s in rs0.__slots__])
rs = [(t.to_sec(), m) for _, m, t in bag.read_messages(topics=['/b166er/robot_state'])]

def stats(v):
    return (min(v), sum(v)/len(v), max(v)) if v else (float('nan'),)*3

for tag, a, b in jan:
    oj = [o for o in odom if a <= o[0] <= b]
    cj = [c[1] for c in clr if a <= c[0] <= b]
    print('\n== %s (%.1f s) ==' % (tag, b - a))
    print('  z      min/med/max: %.4f %.4f %.4f' % stats([o[1] for o in oj]))
    print('  roll°  min/med/max: %.2f %.2f %.2f' % tuple(math.degrees(x) for x in stats([o[2] for o in oj])))
    print('  pitch° min/med/max: %.2f %.2f %.2f' % tuple(math.degrees(x) for x in stats([o[3] for o in oj])))
    print('  wz     min/med/max: %.3f %.3f %.3f' % stats([o[5] for o in oj]))
    print('  front_clearance min/med/max: %.2f %.2f %.2f' % stats(cj))
    rj = [m for st, m in rs if a <= st <= b]
    if rj:
        m = rj[len(rj)//2]
        for f in m.__slots__:
            v = getattr(m, f)
            if isinstance(v, (int, float, bool)):
                print('  robot_state.%s = %s' % (f, round(v, 4) if isinstance(v, float) else v))
bag.close()
