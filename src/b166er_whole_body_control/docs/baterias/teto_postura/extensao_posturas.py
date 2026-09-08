import sys, numpy as np
sys.path.insert(0, "/home/marco/b166er/devel/lib/python3/dist-packages"); sys.path.insert(0, "/home/marco/b166er/src/b166er_whole_body_control/src")
from b166er_whole_body_control import kinematics as K
posts = {"stow_home":[0.0,1.13,-1.04,-1.8,0.0], "travel":[-0.5289,1.13,-1.04,-1.8,0.0], "search":[0.0,1.0,-1.0,0.0,0.0], "deploy":[0.0,0.6,-0.4,-1.2,0.0], "captura(~)":[-0.07,-0.28,0.0,1.42,-0.03]}
for n,q in posts.items():
    T = K.T_BASELINK_ARM @ K.fk_arm(np.array(q))
    frames, _ = K.fk_arm_joint_frames(np.array(q))
    # CG do braço: massas por elo na origem de cada frame de junta (aproximação)
    com = np.zeros(3); m=0.0
    for Tj, mj in zip(frames, K._LINK_MASSES):
        Tj = np.asarray(Tj)
        if Tj.shape == (3,4): Tj = np.vstack([Tj, [0,0,0,1]])
        p = (K.T_BASELINK_ARM @ Tj)[:3,3]; com += mj*p; m += mj
    com/=m
    print("%-12s ponta x=%+.3f y=%+.3f z=%+.3f m | CG braço x=%+.3f z=%+.3f" % (n, T[0,3], T[1,3], T[2,3], com[0], com[2]))
