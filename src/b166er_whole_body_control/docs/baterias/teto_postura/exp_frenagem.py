#!/usr/bin/env python3
"""Ensaio de FRENAGEM SECA por postura. O controlador whole-body leva a
base a 0,3 m/s rumo a um alvo 2,5 m à frente; quando a odometria passa
de v_gatilho, o alvo salta para 1,5 m ATRÁS (erro de rumo de 180°): a
manobra entra em ALIGN, que zerava v de uma vez e girava a 0,5 rad/s —
o mesmo evento que a recuperação da tag ou um novo alvo do SEARCH
produzem na missão. Mede a inclinação máxima nos 5 s seguintes, se a
trava crítica disparou e o maior degrau de |Δv| entre comandos.
args: rótulo, csv de saída, postura, [v_gatilho=0.27]"""
import sys, math, json, rospy, numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix, quaternion_from_matrix
sys.path.insert(0, "/home/marco/b166er/devel/lib/python3/dist-packages")
from b166er_whole_body_control.msg import RobotState

rotulo, saida, POSTURA = sys.argv[1], sys.argv[2], sys.argv[3]
V_GATILHO = float(sys.argv[4]) if len(sys.argv) > 4 else 0.27
rospy.init_node("exp_frenagem", anonymous=True)
rospy.set_param("/fuzzy_wb_controller/fixed_gains", [])
st = {"rs": None, "od": None, "tilt": False, "tiltmax": 0.0, "cmd": [], "tiltmax_antes": 0.0}
rospy.Subscriber("/b166er/robot_state", RobotState, lambda m: st.__setitem__("rs", m))
rospy.Subscriber("/pioneer3at/odom", Odometry, lambda m: st.__setitem__("od", m))
rospy.Subscriber("/b166er/tilt_critical", Bool, lambda m: st.__setitem__("tilt", st["tilt"] or m.data))
rospy.Subscriber("/b166er/tilt", Float64, lambda m: st.__setitem__("tiltmax", max(st["tiltmax"], m.data)))
rospy.Subscriber("/cmd_vel", Twist, lambda m: st["cmd"].append((rospy.get_time(), m.linear.x, m.angular.z)))
pub_target = rospy.Publisher("/b166er/ee_target", PoseStamped, queue_size=1, latch=True)
pub_wb = rospy.Publisher("/b166er/wb_enable", Bool, queue_size=1, latch=True)
pub_lock = rospy.Publisher("/b166er/base_lock", Bool, queue_size=1, latch=True)
pub_tip = rospy.Publisher("/b166er/servo_tooltip", Bool, queue_size=1, latch=True)
pub_post = rospy.Publisher("/b166er/arm_posture_cmd", JointState, queue_size=1, latch=True)
rospy.sleep(1.0)
post = rospy.get_param("/arm_postures/" + POSTURA)
js = JointState(); js.name = ["J1", "J2", "J3", "J4", "J5"]; js.position = list(post); pub_post.publish(js)
rospy.sleep(6.0)
od = rospy.wait_for_message("/pioneer3at/odom", Odometry, timeout=10)
rs = rospy.wait_for_message("/b166er/robot_state", RobotState, timeout=10)
pb = np.array([od.pose.pose.position.x, od.pose.pose.position.y, 0.0])
ob = od.pose.pose.orientation; yaw0 = math.atan2(2 * (ob.w * ob.z + ob.x * ob.y), 1 - 2 * (ob.y ** 2 + ob.z ** 2))
pe = rs.ee_pose.pose.position; oe = rs.ee_pose.pose.orientation
T_now = quaternion_matrix([oe.x, oe.y, oe.z, oe.w]); T_now[:3, 3] = [pe.x, pe.y, pe.z]
fwd = np.array([math.cos(yaw0), math.sin(yaw0), 0.0])
def alvo(dist, vira):
    Rz = np.eye(3)
    if vira:
        Rz = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]], dtype=float)
    T = np.eye(4); T[:3, :3] = Rz @ T_now[:3, :3]; T[:3, 3] = pb + dist * fwd + Rz @ (T_now[:3, 3] - pb)
    tg = PoseStamped(); tg.header.frame_id = "odom"; tg.header.stamp = rospy.Time.now()
    tg.pose.position.x, tg.pose.position.y, tg.pose.position.z = T[:3, 3]
    q = quaternion_from_matrix(T); tg.pose.orientation.x, tg.pose.orientation.y, tg.pose.orientation.z, tg.pose.orientation.w = q
    return tg
pub_target.publish(alvo(2.5, False)); rospy.sleep(0.3)
pub_lock.publish(Bool(False)); pub_tip.publish(Bool(False)); pub_wb.publish(Bool(True))
t0 = rospy.get_time(); v_max = 0.0; t_salto = None
r = rospy.Rate(20)
while not rospy.is_shutdown():
    now = rospy.get_time()
    if st["od"] is not None:
        v = abs(st["od"].twist.twist.linear.x); v_max = max(v_max, v)
    if t_salto is None:
        st["tiltmax_antes"] = st["tiltmax"]
        if v_max >= V_GATILHO or now - t0 > 8.0:
            t_salto = now; st["tiltmax"] = 0.0; n_cmd0 = len(st["cmd"])
            pub_target.publish(alvo(-1.5, True))
    elif now - t_salto > 5.0 or st["tilt"]:
        break
    r.sleep()
pub_wb.publish(Bool(False)); rospy.sleep(0.5)
cmds = st["cmd"][max(n_cmd0 - 1, 0):]   # inclui o último comando ANTES do salto: o degrau é dele para o primeiro depois
dv = max([abs(cmds[i][1] - cmds[i - 1][1]) for i in range(1, len(cmds))] + [0.0])
dw = max([abs(cmds[i][2] - cmds[i - 1][2]) for i in range(1, len(cmds))] + [0.0])
# tempo que v levou para cruzar 0,05 m/s depois do salto
t_zero = next((c[0] - t_salto for c in cmds if abs(c[1]) < 0.05), None)
res = {"rotulo": rotulo, "postura": POSTURA, "v_odom_no_salto": v_max, "tilt_max_antes": st["tiltmax_antes"],
       "tilt_max_depois": st["tiltmax"], "tombou": st["tilt"], "max_dv_cmd": dv, "max_dw_cmd": dw, "t_v_zero_s": t_zero}
open(saida, "a").write(json.dumps(res) + "\n")
print("[frenagem] %s %s: v=%.2f | tilt antes %.3f depois %.3f | tombou %s | max dv %.3f dw %.3f | v->0 em %s" % (
    rotulo, POSTURA, v_max, st["tiltmax_antes"], st["tiltmax"], st["tilt"], dv, dw, ("%.2f s" % t_zero) if t_zero is not None else "?"))
