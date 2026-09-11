#!/usr/bin/env python3
"""robot_state.ik_converged / resíduo por fase da missão, e erro da estimativa
q_arm contra /joint_states (verdade) quando disponível."""
import sys, math, bisect, numpy as np, rosbag
JN=['J1','J2','J3','J4','J5']
bag=rosbag.Bag(sys.argv[1])
fases=[]
for _,m,t in bag.read_messages(topics=['/rosout']):
    if 'chave_mission' in m.name and 'State machine transitioning' in m.msg:
        fases.append((m.header.stamp.to_sec(), m.msg.split("-->'")[1].split("'")[0]))
rs=[(t.to_sec(), m.ik_converged, m.ik_residual_pos, m.ik_residual_orient, np.array(m.q_arm)) for _,m,t in bag.read_messages(topics=['/b166er/robot_state'])]
js=[]
for _,m,t in bag.read_messages(topics=['/joint_states']):
    d=dict(zip(m.name,m.position))
    if all(n in d for n in JN): js.append((m.header.stamp.to_sec(), np.array([d[n] for n in JN])))
jt=[j[0] for j in js]
def fase_em(t):
    f='INICIO'
    for st,n in fases:
        if st<=t: f=n
    return f
from collections import defaultdict
agg=defaultdict(list)
for t,c,rp,ro,q in rs:
    dq=float('nan')
    if js:
        i=bisect.bisect_left(jt,t); i=min(max(i,0),len(js)-1)
        dq=np.degrees(np.abs(q-js[i][1])).max()
    agg[fase_em(t)].append((c,rp,ro,dq))
print('%-12s %5s %8s %10s %10s %12s'%('fase','n','não-conv','rp(nc) mm','ro(nc) rad','|dq| max °'))
for f in ['INICIO','SEARCH','APPROACH','REFINE','DEPLOY','MANIPULATE','RETRACT','RETURN','ABORT_SAFE']:
    a=agg.get(f)
    if not a: continue
    a=np.array(a,dtype=float); nc=a[a[:,0]==0]
    print('%-12s %5d %7.0f%% %10.1f %10.4f %12.1f'%(f,len(a),100*(a[:,0]==0).mean(), 1000*nc[:,1].mean() if len(nc) else 0, nc[:,2].mean() if len(nc) else 0, np.nanmax(a[:,3]) if not np.all(np.isnan(a[:,3])) else float('nan')))
bag.close()
