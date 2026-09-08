#!/usr/bin/env python3
"""Sai com 0 se a base está a menos de 5° e 5 cm da pose pedida (x y yaw_deg)."""
import sys, math, rospy
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion
x, y, yaw = map(float, sys.argv[1:4])
rospy.init_node("checa_pose", anonymous=True)
s = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)("b166er", "world"); o = s.pose.orientation
yz = math.degrees(euler_from_quaternion([o.x, o.y, o.z, o.w])[2])
e_yaw = (yz - yaw + 180) % 360 - 180; e_xy = math.hypot(s.pose.position.x - x, s.pose.position.y - y)
print("pose medida: x=%.3f y=%.3f yaw=%.1f (erro %.1f°, %.3f m)" % (s.pose.position.x, s.pose.position.y, yz, e_yaw, e_xy))
sys.exit(0 if abs(e_yaw) < 5 and e_xy < 0.05 else 1)
