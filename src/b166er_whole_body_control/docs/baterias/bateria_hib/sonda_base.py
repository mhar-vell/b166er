#!/usr/bin/env python3
"""Sonda da base durante a manipulação (10 Hz): pose da base (odom), distância
à parede (y=3,0 no mundo), /cmd_vel, juntas do braço, ganhos Fuzzy, estado/fase.
Para investigar por que a profundidade estaciona no modo whole-body:
a base anda? o plano de exclusão (0,55 m) está ativo? o braço está no batente?"""
import sys, math, time, rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
out = open(sys.argv[1], "w"); out.write("wall,sim,bx,by,byaw,dist_parede,cmd_v,cmd_w,j1,j2,j3,j4,j5,k_pos,k_ori,lam,estado,fase,modo\n")
rospy.init_node("sonda_base", anonymous=True)
st = {"estado": "", "fase": "", "modo": ""}; od = [None]; cv = [0.0, 0.0]; q = [None]; g = [None]
def cb_st(m):
    for parte in m.data.split("|"):
        if "=" in parte:
            k, v = parte.split("=", 1)
            if k in st: st[k] = v
def cb_od(m):
    o = m.pose.pose.orientation; p = m.pose.pose.position
    yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z)); od[0] = (p.x, p.y, yaw)
def cb_cv(m): cv[0], cv[1] = m.linear.x, m.angular.z
def cb_js(m):
    d = dict(zip(m.name, m.position))
    if all(k in d for k in ("J1", "J2", "J3", "J4", "J5")): q[0] = [math.degrees(d[k]) for k in ("J1", "J2", "J3", "J4", "J5")]
def cb_g(m): g[0] = list(m.data)
rospy.Subscriber("/b166er/mission_status", String, cb_st); rospy.Subscriber("/odom", Odometry, cb_od); rospy.Subscriber("/pioneer3at/odom", Odometry, cb_od)
rospy.Subscriber("/cmd_vel", Twist, cb_cv); rospy.Subscriber("/joint_states", JointState, cb_js)
rospy.Subscriber("/b166er/fuzzy_gains", Float64MultiArray, cb_g)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    o = od[0] or (float("nan"),) * 3; qq = q[0] or [float("nan")] * 5; gg = g[0] or [float("nan")] * 3
    out.write("%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%s,%s,%s,%s,%s\n" % (time.time(), rospy.get_time(), o[0], o[1], o[2], 3.0 - o[1], cv[0], cv[1],
              ",".join("%.1f" % v for v in qq), ",".join("%.3f" % v for v in gg), st["estado"], st["fase"], st["modo"]))
    out.flush(); r.sleep()
