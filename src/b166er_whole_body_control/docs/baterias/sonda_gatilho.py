#!/usr/bin/env python3
"""Sonda a 10 Hz durante a missão: lingueta (mm), lâmina (°), z da ponta do
robô (T265 do estado) e fase corrente. CSV em argv[1]."""
import sys, math, time, rospy
from gazebo_msgs.srv import GetJointProperties
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
out = open(sys.argv[1], "w"); out.write("wall,sim,lingueta_mm,lamina_deg,fase\n")
rospy.init_node("sonda_gatilho", anonymous=True)
rospy.wait_for_service("/gazebo/get_joint_properties", timeout=20)
gj = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
fase = [""]
def cb(m):
    import json
    try: fase[0] = json.loads(m.data).get("fase", "") or json.loads(m.data).get("estado", "")
    except Exception: fase[0] = m.data[:30]
topics = [t for t, ty in rospy.get_published_topics() if "mission" in t and ty == "std_msgs/String"]
if topics: rospy.Subscriber(topics[0], String, cb)
r = rospy.Rate(10); lmax = 0.0
while not rospy.is_shutdown():
    try:
        l = 1000 * gj("chave_lingueta_joint").position[0]; b = math.degrees(gj("chave_blade_joint").position[0])
    except Exception:
        break
    lmax = max(lmax, l)
    out.write("%.2f,%.2f,%.2f,%.2f,%s\n" % (time.time(), rospy.get_time(), l, b, fase[0])); out.flush()
    r.sleep()
print("lingueta máx %.1f mm" % lmax)
