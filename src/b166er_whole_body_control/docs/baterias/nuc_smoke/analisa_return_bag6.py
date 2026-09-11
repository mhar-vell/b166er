#!/usr/bin/env python3
"""Buracos na publicação de /b166er/robot_state × passagens pelo alvo × avisos de IK do estimador."""
import sys, math
import rosbag
GOAL=(0.0,1.0)
bag=rosbag.Bag(sys.argv[1])
ev=[(m.header.stamp.to_sec(),m.name,m.msg) for _,m,t in bag.read_messages(topics=['/rosout'])]
a=next(st for st,n,s in ev if 'chave_mission' in n and 'RETURN: alinhado' in s)
b=next(st for st,n,s in ev if 'chave_mission' in n and st>a and ('RETURN: chegou' in s or 'timeout' in s))
rs=[t.to_sec() for _,m,t in bag.read_messages(topics=['/b166er/robot_state'])]
gaps=[(rs[i+1]-rs[i], rs[i]) for i in range(len(rs)-1)]
print('robot_state: %d msgs; intervalo mediano %.3f s; maiores buracos (s, em t+... do RETURN):'%(len(rs), sorted(g[0] for g in gaps)[len(gaps)//2]))
for g,t in sorted(gaps, reverse=True)[:12]:
    print('  %.2f s  em t%+.1f  (%s)'%(g, t-a, 'RETURN' if a<=t<=b else 'fora'))
print('\nburacos > 0,5 s dentro do RETURN, com o que o estimador logou perto:')
for g,t in gaps:
    if g>0.5 and a<=t<=b:
        prox=[(st-t, s[:70]) for st,n,s in ev if 'state_estimator' in n and abs(st-(t+g))<3.0]
        print('  t%+.1f→%+.1f (%.2f s): %s'%(t-a, t+g-a, g, prox[:2]))
gz=[(t.to_sec(),m.pose.pose.position.x,m.pose.pose.position.y) for _,m,t in bag.read_messages(topics=['/pioneer3at/odom'])]
print('\npassagens pelo alvo (dist<0,06) e se havia buraco:')
dentro=False
for t,x,y in gz:
    if not(a<=t<=b): continue
    d=math.hypot(GOAL[0]-x,GOAL[1]-y)
    if d<0.06 and not dentro:
        dentro=True; t_in=t
    elif d>=0.06 and dentro:
        dentro=False
        cob=[(g,tt) for g,tt in gaps if tt<=t and tt+g>=t_in]
        print('  t%+.1f→%+.1f (%.2f s dentro)  buraco cobrindo: %s'%(t_in-a,t-a,t-t_in, [(round(g,2), round(tt-a,1)) for g,tt in cob if g>0.2]))
print('\nrosout do estimador no RETURN (contagem por mensagem):')
from collections import Counter
c=Counter(s[:60] for st,n,s in ev if 'state_estimator' in n and a<=st<=b)
for k,v in c.most_common(5): print('  %3d × %s'%(v,k))
bag.close()
