#!/usr/bin/env python3
"""Reproduz a IK do estimador a partir do bag de entradas: para cada T265,
monta o alvo com (A) o base_odom mais recente recebido, (B) o de stamp
mais próximo, (C) interpolado entre os dois vizinhos; seed = joint_states
de stamp mais próximo (como o estimador). Mede convergência, custo e a
inconsistência do alvo (pos/rot) contra o FK das juntas verdadeiras."""
import sys, time, bisect, math
import numpy as np, rosbag
from tf.transformations import quaternion_matrix, quaternion_slerp, quaternion_from_matrix
sys.path.insert(0, "/home/marco/b166er/src/b166er_whole_body_control/src")
from b166er_whole_body_control import kinematics as K
from b166er_whole_body_control.kinematics import JOINT_NAMES, T_BASELINK_ARM, fk_arm, ik_arm, pose_error

def odom_T(p, o):
    T = quaternion_matrix([o.x, o.y, o.z, o.w]); T[:3, 3] = [p.x, p.y, p.z]; return T

bag = rosbag.Bag(sys.argv[1])
base = []   # (t_rx, stamp, T)
for _, m, t in bag.read_messages(topics=['/pioneer/pose']):
    base.append((t.to_sec(), m.header.stamp.to_sec(), odom_T(m.pose.pose.position, m.pose.pose.orientation), m))
t265 = [(t.to_sec(), m.header.stamp.to_sec(), odom_T(m.pose.pose.position, m.pose.pose.orientation)) for _, m, t in bag.read_messages(topics=['/t265/odom/sample'])]
js = []
for _, m, t in bag.read_messages(topics=['/joint_states']):
    d = dict(zip(m.name, m.position))
    if all(n in d for n in JOINT_NAMES):
        js.append((m.header.stamp.to_sec(), np.array([d[n] for n in JOINT_NAMES])))
odom = [(t.to_sec(), m.twist.twist.linear.x, m.twist.twist.angular.z) for _, m, t in bag.read_messages(topics=['/pioneer3at/odom'])]
bag.close()
print('base %d msgs (%.0f Hz), t265 %d (%.0f Hz), joint_states %d' % (len(base), len(base)/(base[-1][0]-base[0][0]), len(t265), len(t265)/(t265[-1][0]-t265[0][0]), len(js)))
base_rx = [b[0] for b in base]; base_st = [b[1] for b in base]; js_t = [j[0] for j in js]; od_t = [o[0] for o in odom]

def base_latest(t_rx):
    i = bisect.bisect_right(base_rx, t_rx) - 1
    return base[i][2] if i >= 0 else None
def base_nearest(st):
    i = min(range(len(base)), key=lambda k: abs(base_st[k] - st)); return base[i][2]
def base_interp(st):
    i = bisect.bisect_left(base_st, st)
    if i <= 0: return base[0][2]
    if i >= len(base): return base[-1][2]
    (t0, T0), (t1, T1) = (base_st[i-1], base[i-1][2]), (base_st[i], base[i][2])
    a = (st - t0) / max(t1 - t0, 1e-9)
    q0, q1 = quaternion_from_matrix(T0), quaternion_from_matrix(T1)
    T = quaternion_matrix(quaternion_slerp(q0, q1, a)); T[:3, 3] = (1-a)*T0[:3, 3] + a*T1[:3, 3]; return T

res = {'A latest': [], 'B nearest': [], 'C interp': []}
for k, (t_rx, st, T_wt) in enumerate(t265):
    if k % 4: continue          # 1 em 4 (o estimador roda a 20 Hz, o t265 a ~50)
    j = min(range(len(js)), key=lambda i: abs(js_t[i] - st)); q_seed = js[j][1]
    io = bisect.bisect_right(od_t, t_rx) - 1; v, w = (odom[io][1], odom[io][2]) if io >= 0 else (0, 0)
    for nome, Tb in (('A latest', base_latest(t_rx)), ('B nearest', base_nearest(st)), ('C interp', base_interp(st))):
        if Tb is None: continue
        T_target = np.linalg.inv(Tb @ T_BASELINK_ARM) @ T_wt
        e = pose_error(fk_arm(q_seed), T_target)           # inconsistência do alvo vs juntas verdadeiras
        t0 = time.time(); q, c, rp, ro = ik_arm(T_target, q_init=q_seed.copy()); dt = time.time() - t0
        res[nome].append((c, rp, ro, dt, np.linalg.norm(e[:3]), np.linalg.norm(e[3:]), abs(v) > 0.02 or abs(w) > 0.02))
for nome, r in res.items():
    r = np.array(r, dtype=float); mov = r[r[:, 6] == 1]; par = r[r[:, 6] == 0]
    for rot, sub in (('base andando', mov), ('base parada', par)):
        if len(sub) == 0: continue
        print('%-10s %-13s n=%3d  não-conv %3d (%4.0f%%)  custo med %.3f s max %.3f | inconsist. alvo: pos med %.4f max %.4f m, rot med %.4f max %.4f rad | resíduo não-conv: pos med %.4f rot med %.4f'
              % (nome, rot, len(sub), (sub[:, 0] == 0).sum(), 100*(sub[:, 0] == 0).mean(), sub[:, 3].mean(), sub[:, 3].max(),
                 sub[:, 4].mean(), sub[:, 4].max(), sub[:, 5].mean(), sub[:, 5].max(),
                 sub[sub[:, 0] == 0][:, 1].mean() if (sub[:, 0] == 0).any() else 0, sub[sub[:, 0] == 0][:, 2].mean() if (sub[:, 0] == 0).any() else 0))
