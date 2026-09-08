#!/usr/bin/env python3
"""Diagnóstico: com a lingueta empurrada (−Z na lingueta), aplica o puxão NA LÂMINA
(ponta, +Y do frame dela) — se ainda não girar, o bloqueio não é a aba."""
import math, time, rospy
from geometry_msgs.msg import Wrench, Point
from gazebo_msgs.srv import ApplyBodyWrench, GetJointProperties
M = "chave_seccionadora_fixture"
def juntas():
    gj = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
    return math.degrees(gj("chave_blade_joint").position[0]), 1000 * gj("chave_lingueta_joint").position[0]
def w(body, f, p, dur):
    aw = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
    wr = Wrench(); wr.force.x, wr.force.y, wr.force.z = f
    aw(body_name=M + "::" + body, reference_frame=M + "::" + body, reference_point=Point(*p), wrench=wr, start_time=rospy.Time(0), duration=rospy.Duration(dur))
rospy.init_node("diag_gatilho", anonymous=True)
rospy.wait_for_service("/gazebo/apply_body_wrench", timeout=10)
print("repouso", juntas())
print("1) só a lâmina, 20 N na ponta, lingueta em cima (deve segurar):")
w("chave_blade", (0, 20, 0), (0, 0, 0.19), 2.0)
for k in range(4): time.sleep(0.5); print("   ", juntas())
time.sleep(1.5)
print("2) lingueta empurrada (−8 N) + 20 N na ponta da LÂMINA:")
w("chave_lingueta", (0, 0, -8), (0, 0, 0.2), 4.0)
time.sleep(0.5); w("chave_blade", (0, 20, 0), (0, 0, 0.19), 3.0)
for k in range(7): time.sleep(0.5); print("   ", juntas())
time.sleep(1.5); print("solto", juntas())
