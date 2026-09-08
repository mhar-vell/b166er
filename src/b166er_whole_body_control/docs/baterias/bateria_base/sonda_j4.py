#!/usr/bin/env python3
"""J4 estimado (robot_state.q_arm, o que o controlador vê) × J4 verdadeiro
(/joint_states, só na sim) × fase, a 10 Hz. CSV em argv[1]."""
import sys, math, time, rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from b166er_whole_body_control.msg import RobotState
out = open(sys.argv[1], "w"); out.write("sim,j4_est,j4_true,fase,modo\n")
rospy.init_node("sonda_j4", anonymous=True)
st = {"fase": "", "modo": ""}; est = [None]; tru = [None]
def cb_st(m):
    for parte in m.data.split("|"):
        if "=" in parte:
            k, v = parte.split("=", 1)
            if k in st: st[k] = v
def cb_rs(m): est[0] = math.degrees(m.q_arm[3]) if len(m.q_arm) > 3 else None
def cb_js(m):
    d = dict(zip(m.name, m.position))
    if "J4" in d: tru[0] = math.degrees(d["J4"])
rospy.Subscriber("/b166er/mission_status", String, cb_st); rospy.Subscriber("/b166er/robot_state", RobotState, cb_rs); rospy.Subscriber("/joint_states", JointState, cb_js)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    out.write("%.2f,%s,%s,%s,%s\n" % (rospy.get_time(), "%.1f" % est[0] if est[0] is not None else "", "%.1f" % tru[0] if tru[0] is not None else "", st["fase"], st["modo"])); out.flush(); r.sleep()
