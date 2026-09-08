#!/usr/bin/env python3
"""Sonda da bateria (10 Hz): lingueta (mm), lâmina (°), estado/fase da missão
e se o Fuzzy está movendo o braço (amostras de /b166er/arm_vel_cmd com alguma
velocidade != 0 desde a última linha). CSV em argv[1]."""
import sys, math, time, json, rospy
from gazebo_msgs.srv import GetJointProperties
from std_msgs.msg import String
from sensor_msgs.msg import JointState
out = open(sys.argv[1], "w"); out.write("wall,sim,lingueta_mm,lamina_deg,estado,fase,armvel_nz,armvel_n\n")
rospy.init_node("sonda_wb", anonymous=True)
rospy.wait_for_service("/gazebo/get_joint_properties", timeout=30)
gj = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
st = {"estado": "", "fase": ""}; cnt = {"nz": 0, "n": 0}
def cb_status(m):
    for parte in m.data.split("|"):
        if "=" in parte:
            k, v = parte.split("=", 1)
            if k in st: st[k] = v
def cb_vel(m):
    cnt["n"] += 1
    if any(abs(v) > 1e-4 for v in m.velocity): cnt["nz"] += 1
rospy.Subscriber("/b166er/mission_status", String, cb_status)
rospy.Subscriber("/b166er/arm_vel_cmd", JointState, cb_vel)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    try:
        l = 1000 * gj("chave_lingueta_joint").position[0]; b = math.degrees(gj("chave_blade_joint").position[0])
    except Exception:
        break
    out.write("%.2f,%.2f,%.2f,%.2f,%s,%s,%d,%d\n" % (time.time(), rospy.get_time(), l, b, st["estado"], st["fase"], cnt["nz"], cnt["n"]))
    out.flush(); cnt["nz"] = cnt["n"] = 0
    r.sleep()
