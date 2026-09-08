#!/usr/bin/env python3
"""Setpoint dos controladores de posição (o que a ponte manda) × posição real das
juntas × esforço, a 10 Hz, com a fase. Para separar 'controlador pediu' de
'contato empurrou'. CSV em argv[1]."""
import sys, math, rospy
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
N = ["J1", "J2", "J3", "J4", "J5"]
out = open(sys.argv[1], "w"); out.write("sim,fase,modo," + ",".join("cmd_%s" % j for j in N) + "," + ",".join("pos_%s" % j for j in N) + "," + ",".join("eff_%s" % j for j in N) + "\n")
rospy.init_node("sonda_pid", anonymous=True)
st = {"fase": "", "modo": ""}; cmd = {j: float("nan") for j in N}; pos = {j: float("nan") for j in N}; eff = {j: float("nan") for j in N}
def cb_st(m):
    for parte in m.data.split("|"):
        if "=" in parte:
            k, v = parte.split("=", 1)
            if k in st: st[k] = v
def mk(j):
    def cb(m): cmd[j] = math.degrees(m.data)
    return cb
for j in N: rospy.Subscriber("/%s_position_controller/command" % j, Float64, mk(j))
def cb_js(m):
    d = dict(zip(m.name, m.position)); e = dict(zip(m.name, m.effort)) if m.effort else {}
    for j in N:
        if j in d: pos[j] = math.degrees(d[j]); eff[j] = e.get(j, float("nan"))
rospy.Subscriber("/joint_states", JointState, cb_js); rospy.Subscriber("/b166er/mission_status", String, cb_st)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    out.write("%.2f,%s,%s,%s,%s,%s\n" % (rospy.get_time(), st["fase"], st["modo"], ",".join("%.1f" % cmd[j] for j in N), ",".join("%.1f" % pos[j] for j in N), ",".join("%.2f" % eff[j] for j in N))); out.flush(); r.sleep()
