#!/usr/bin/env python3
"""Amostra o esforço do J4 em /joint_states por N s: máx |esforço|, tempo saturado (>= 95% do limite), e a fase da missão pelo /b166er/mission_status se existir."""
import sys, time, rospy
from sensor_msgs.msg import JointState
LIM=float(sys.argv[2]) if len(sys.argv)>2 else 4.2; DUR=float(sys.argv[1]) if len(sys.argv)>1 else 120
rospy.init_node("sonda_j4", anonymous=True)
d={"n":0,"max":0.0,"sat":0,"pos_max":0.0,"pos_min":0.0}
def cb(m):
    if "J4" not in m.name: return
    i=m.name.index("J4"); e=abs(m.effort[i]) if m.effort else 0.0; p=m.position[i]
    d["n"]+=1; d["max"]=max(d["max"],e); d["sat"]+= 1 if e>=0.95*LIM else 0
    d["pos_max"]=max(d["pos_max"],p); d["pos_min"]=min(d["pos_min"],p)
rospy.Subscriber("/joint_states", JointState, cb)
t0=time.time()
while time.time()-t0<DUR and not rospy.is_shutdown(): time.sleep(0.5)
import math
print("amostras=%d  |J4| max=%.2f N·m  saturado(>=%.2f)=%.1f%%  J4 pos %.0f..%.0f°" % (d["n"], d["max"], 0.95*LIM, 100.0*d["sat"]/max(1,d["n"]), math.degrees(d["pos_min"]), math.degrees(d["pos_max"])))
