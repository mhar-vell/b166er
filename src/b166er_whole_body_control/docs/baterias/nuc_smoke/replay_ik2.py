#!/usr/bin/env python3
"""Reproduz o LAÇO do estimador como na missão (seed = estimativa anterior),
a 20 Hz sobre o bag de entradas, para (base latest × nearest) × (max_iter
300 × 40): não-convergências, custo por ciclo, e erro da estimativa."""
import sys, time, bisect, numpy as np, rosbag
from tf.transformations import quaternion_matrix
sys.path.insert(0, "/home/marco/b166er/src/b166er_whole_body_control/src")
from b166er_whole_body_control.kinematics import JOINT_NAMES, T_BASELINK_ARM, fk_arm, ik_arm
def odom_T(p, o):
    T = quaternion_matrix([o.x, o.y, o.z, o.w]); T[:3, 3] = [p.x, p.y, p.z]; return T
bag = rosbag.Bag(sys.argv[1])
base = [(t.to_sec(), m.header.stamp.to_sec(), odom_T(m.pose.pose.position, m.pose.pose.orientation)) for _, m, t in bag.read_messages(topics=['/pioneer/pose'])]
t265 = [(t.to_sec(), m.header.stamp.to_sec(), odom_T(m.pose.pose.position, m.pose.pose.orientation)) for _, m, t in bag.read_messages(topics=['/t265/odom/sample'])]
js = []
for _, m, t in bag.read_messages(topics=['/joint_states']):
    d = dict(zip(m.name, m.position))
    if all(n in d for n in JOINT_NAMES): js.append((m.header.stamp.to_sec(), np.array([d[n] for n in JOINT_NAMES])))
bag.close()
brx = [b[0] for b in base]; bst = [b[1] for b in base]; t265_rx = [x[0] for x in t265]; jt = [j[0] for j in js]
t_ini, t_fim = t265[0][0], t265[-1][0]
HOME = np.array([0.0, 1.13, -1.04, -1.8, 0.0])
for modo in ('latest', 'nearest'):
    for mi in (300, 40):
        q_prev = HOME.copy(); nc = 0; n = 0; custo = []; erro = []
        t = t_ini
        while t < t_fim:
            i = bisect.bisect_right(t265_rx, t) - 1
            if i < 0: t += 0.05; continue
            t_rx, st, T_wt = t265[i]
            if modo == 'latest':
                k = bisect.bisect_right(brx, t) - 1; Tb = base[k][2]
            else:
                k = min(range(max(0, bisect.bisect_left(bst, st) - 3), min(len(base), bisect.bisect_left(bst, st) + 3)), key=lambda z: abs(bst[z] - st)); Tb = base[k][2]
            T_target = np.linalg.inv(Tb @ T_BASELINK_ARM) @ T_wt
            t0 = time.time(); q, c, rp, ro = ik_arm(T_target, q_init=q_prev.copy(), max_iter=mi); custo.append(time.time() - t0)
            j = min(max(bisect.bisect_left(jt, st), 0), len(js) - 1); erro.append(np.degrees(np.abs(q - js[j][1])).max())
            n += 1; nc += (not c); q_prev = q; t += 0.05
        custo = np.array(custo); erro = np.array(erro)
        print('%-8s max_iter=%3d  ciclos=%d  não-conv=%d (%.1f%%)  custo med %.4f s  p99 %.4f  max %.4f  | erro q vs verdade: med %.2f° max %.2f°'
              % (modo, mi, n, nc, 100*nc/n, custo.mean(), np.percentile(custo, 99), custo.max(), erro.mean(), erro.max()))
