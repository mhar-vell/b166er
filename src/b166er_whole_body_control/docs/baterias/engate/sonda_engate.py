#!/usr/bin/env python3
"""SONDA DO ENGATE (16 Out 2026, diagnóstico só de simulação): a 20 Hz, ponta da ferramenta (T265 ⊕ tooltip, o que o
controle vê) × centro do anel pela verdade do Gazebo (ângulo da lâmina), no mundo e no frame da parede (eixo = x,
prof = −y, alt = z), mais lingueta, fase e juntas estimadas. CSV em argv[1]. Para ver ONDE o degrau estava em relação ao
arame no instante em que o dedo perde o anel.
Rodar com B166ER_GARRA_DX igual ao garra_dx do stack (a cinemática lê a variável ao importar; sem ela a tooltip fica
30 mm deslocada pelo eixo — foi o viés da bateria de 16 Out)."""
import sys, math, rospy, numpy as np
from std_msgs.msg import String
from gazebo_msgs.srv import GetJointProperties
from tf.transformations import quaternion_matrix
from b166er_whole_body_control.msg import RobotState
from b166er_whole_body_control.kinematics import T_T265_TOOLTIP
PIVO = np.array([0.201, 2.868, 0.605]); R = 0.200       # pivô da lâmina (fixture: olhal a 0,805 em θ=0, L=200 mm)
out = open(sys.argv[1], "w")
out.write("sim,fase,e_eixo,e_prof,e_alt,tip_x,tip_y,tip_z,blade_deg,ring_y,ring_z,d_eixo_mm,d_prof_mm,d_alt_mm,lingueta_mm,q1,q2,q3,q4,q5\n")
rospy.init_node("sonda_engate", anonymous=True)
st = {"fase": "", "e_eixo_mm": "", "e_prof_mm": "", "e_alt_mm": ""}; rs = [None]
def cb_st(m):
    for parte in m.data.split("|"):
        if "=" in parte:
            k, v = parte.split("=", 1)
            if k in st: st[k] = v
def cb_rs(m): rs[0] = m
rospy.Subscriber("/b166er/mission_status", String, cb_st); rospy.Subscriber("/b166er/robot_state", RobotState, cb_rs)
rospy.wait_for_service("/gazebo/get_joint_properties", timeout=20)
gjp = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    m = rs[0]
    if m is None: rate.sleep(); continue
    p = m.ee_pose.pose.position; o = m.ee_pose.pose.orientation
    T = quaternion_matrix([o.x, o.y, o.z, o.w]); T[:3, 3] = [p.x, p.y, p.z]
    tip = (T @ T_T265_TOOLTIP)[:3, 3]
    try:
        th = gjp("chave_blade_joint").position[0]; li = gjp("chave_lingueta_joint").position[0] * 1000
    except Exception:
        th, li = float("nan"), float("nan")
    ring = PIVO + np.array([0.0, -R * math.sin(th), R * math.cos(th)])
    d = (tip - ring) * 1000
    q = [math.degrees(v) for v in m.q_arm] if len(m.q_arm) == 5 else [float("nan")] * 5
    out.write("%.2f,%s,%s,%s,%s,%.4f,%.4f,%.4f,%.1f,%.4f,%.4f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n" % (
        rospy.get_time(), st["fase"], st["e_eixo_mm"], st["e_prof_mm"], st["e_alt_mm"], tip[0], tip[1], tip[2],
        math.degrees(th), ring[1], ring[2], d[0], -d[1], d[2], li, *q))
    out.flush(); rate.sleep()
