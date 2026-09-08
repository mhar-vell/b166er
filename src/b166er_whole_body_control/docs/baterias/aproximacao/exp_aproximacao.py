#!/usr/bin/env python3
"""Rastreamento de LONGE com o whole-body: da pose de partida (braço em
postura de busca), leva a PONTA da ferramenta até o alvo da fase 'orienta'
da chave (15 cm da parede, 9 cm ao lado do olhal), base livre com plano de
exclusão — a manobra girar-avançar-girar é exercitada porque a partida
aponta para +x e a chave está em +y. Uma execução; args: rótulo, ganhos
("[]" = Fuzzy), csv de saída. Mede tempo até 20 mm sustentado por 1 s,
menor erro, eventos ALIGN/ADVANCE, |cmd_vel| máx, tombamento."""
import sys, math, time, json, rospy, numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Log
from tf.transformations import quaternion_matrix, quaternion_from_matrix
sys.path.insert(0, "/home/marco/b166er/devel/lib/python3/dist-packages")
from b166er_whole_body_control.msg import RobotState
from b166er_whole_body_control import chave_task
from b166er_whole_body_control.kinematics import T_T265_TOOLTIP

rotulo, ganhos, saida = sys.argv[1], sys.argv[2], sys.argv[3]
TOL, SUSTENTA, TIMEOUT = 0.030, 1.0, 120.0   # 30 mm / 0,1 rad: o teste com travel ficou em 23 mm (o tol interno do controlador é 2 mm e ele segue caçando)
WALL_POS = np.array([0.0, 3.0, 0.0]); WALL_R = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]], dtype=float)   # yaw = pi
ORIENTA = [-0.090, 0.150, 0.0085]

rospy.init_node("exp_aproximacao", anonymous=True)
rospy.set_param("/fuzzy_wb_controller/fixed_gains", json.loads(ganhos))
st = {"rs": None, "tilt": False, "align": 0, "advance": 0, "vmax": 0.0, "wmax": 0.0}
rospy.Subscriber("/b166er/robot_state", RobotState, lambda m: st.__setitem__("rs", m))
rospy.Subscriber("/b166er/tilt_critical", Bool, lambda m: st.__setitem__("tilt", st["tilt"] or m.data))
def cb_log(m):
    if "manobra: ADVANCE -> ALIGN" in m.msg: st["align"] += 1
    if "manobra: ALIGN -> ADVANCE" in m.msg or "ALIGN excedeu" in m.msg: st["advance"] += 1
rospy.Subscriber("/rosout", Log, cb_log)
def cb_cv(m): st["vmax"] = max(st["vmax"], abs(m.linear.x)); st["wmax"] = max(st["wmax"], abs(m.angular.z))
rospy.Subscriber("/cmd_vel", Twist, cb_cv)
pub_target = rospy.Publisher("/b166er/ee_target", PoseStamped, queue_size=1, latch=True)
pub_wb = rospy.Publisher("/b166er/wb_enable", Bool, queue_size=1, latch=True)
pub_lock = rospy.Publisher("/b166er/base_lock", Bool, queue_size=1, latch=True)
pub_tip = rospy.Publisher("/b166er/servo_tooltip", Bool, queue_size=1, latch=True)
pub_keep = rospy.Publisher("/b166er/base_keepout", PoseStamped, queue_size=1, latch=True)
pub_post = rospy.Publisher("/b166er/arm_posture_cmd", JointState, queue_size=1, latch=True)
rospy.sleep(1.0)
# postura de busca (a mesma que a missão usa antes de manipular)
# Postura de viagem (braço recolhido), como a missão navega. Com a postura
# de busca (braço à frente) o teste de 23:41 tombou o robô a 0,3 m/s e
# 0,5 rad/s — o modo de falha de 13 Ago — e ficou oscilando ALIGN/ADVANCE
# com o erro de rumo em cima do limiar de 0,35 rad (5 trocas em 25 s).
POSTURA = sys.argv[4] if len(sys.argv) > 4 else "travel"
post = rospy.get_param("/arm_postures/" + POSTURA, [-0.5289, 1.13, -1.04, -1.8, 0.0])
js = JointState(); js.name = ["J1", "J2", "J3", "J4", "J5"]; js.position = list(post); pub_post.publish(js)
rospy.sleep(6.0)
# plano de exclusão: parede em y=3, normal segura para -y (lado do robô)
kp = PoseStamped(); kp.header.frame_id = "odom"; kp.header.stamp = rospy.Time.now()
kp.pose.position.x, kp.pose.position.y, kp.pose.position.z = WALL_POS
x = np.array([0.0, -1.0, 0.0]); z = np.array([0.0, 0.0, 1.0]); y = np.cross(z, x)
T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z
qx, qy, qz, qw = quaternion_from_matrix(T); kp.pose.orientation.x, kp.pose.orientation.y, kp.pose.orientation.z, kp.pose.orientation.w = qx, qy, qz, qw
pub_keep.publish(kp)
# ALVO 6D DA T265 (lei whole-body geral, não a variante de ponta): a pose
# que a T265 tem quando o robô está a 0,8 m da parede virado para ela,
# com a mesma postura de braço — a pose atual girada 90° em torno do
# centro da base e transladada. A variante de ponta (servo_tooltip) não
# serve para longe: base pesada 12x + teto de 0,06 m/s deixam a base a
# 1-2 cm/s (teste de 23:36: 1,8 m -> 0,84 m em 90 s, sem ALIGN).
from nav_msgs.msg import Odometry
od = rospy.wait_for_message("/pioneer3at/odom", Odometry, timeout=10)
rs = rospy.wait_for_message("/b166er/robot_state", RobotState, timeout=10)
pb = np.array([od.pose.pose.position.x, od.pose.pose.position.y, 0.0])
ob = od.pose.pose.orientation; yaw0 = math.atan2(2 * (ob.w * ob.z + ob.x * ob.y), 1 - 2 * (ob.y * ob.y + ob.z * ob.z))
pe = rs.ee_pose.pose.position; oe = rs.ee_pose.pose.orientation
T_now = quaternion_matrix([oe.x, oe.y, oe.z, oe.w]); T_now[:3, 3] = [pe.x, pe.y, pe.z]
dyaw = (math.pi / 2) - yaw0                     # virar para +y (a parede)
Rz = np.array([[math.cos(dyaw), -math.sin(dyaw), 0], [math.sin(dyaw), math.cos(dyaw), 0], [0, 0, 1]])
BASE_ALVO = np.array([0.0, 2.2, 0.0])
T_alvo = np.eye(4); T_alvo[:3, :3] = Rz @ T_now[:3, :3]; T_alvo[:3, 3] = BASE_ALVO + Rz @ (T_now[:3, 3] - pb)
alvo = T_alvo[:3, 3].copy()
qx, qy, qz, qw = quaternion_from_matrix(T_alvo)
tg = PoseStamped(); tg.header.frame_id = "odom"; tg.header.stamp = rospy.Time.now()
tg.pose.position.x, tg.pose.position.y, tg.pose.position.z = alvo
tg.pose.orientation.x, tg.pose.orientation.y, tg.pose.orientation.z, tg.pose.orientation.w = qx, qy, qz, qw
R_alvo = T_alvo[:3, :3]
pub_target.publish(tg); rospy.sleep(0.3)
pub_lock.publish(Bool(False)); pub_tip.publish(Bool(False)); pub_wb.publish(Bool(True))
def tip():
    m = st["rs"]
    if m is None: return None
    p = m.ee_pose.pose.position
    return np.array([p.x, p.y, p.z])
def err_ori():
    m = st["rs"]
    if m is None: return None
    o = m.ee_pose.pose.orientation; R = quaternion_matrix([o.x, o.y, o.z, o.w])[:3, :3]
    Re = R_alvo @ R.T; return float(math.acos(max(-1.0, min(1.0, (np.trace(Re) - 1) / 2))))
t0 = rospy.get_time(); erro0 = None; melhor = 1e9; t_dentro = None; t_ok = None; t_50 = None; traj = []
r = rospy.Rate(10)
while not rospy.is_shutdown():
    now = rospy.get_time(); p = tip()
    if p is not None:
        e = float(np.linalg.norm(alvo - p)); melhor = min(melhor, e)
        if erro0 is None: erro0 = e
        traj.append((now - t0, e))
        eo = err_ori() or 9.9
        if e < 0.050 and t_50 is None: t_50 = now - t0
        if e < TOL and eo < 0.10:
            t_dentro = t_dentro or now
            if now - t_dentro >= SUSTENTA: t_ok = now - t0 - SUSTENTA; break
        else: t_dentro = None
    if st["tilt"] or now - t0 > TIMEOUT: break
    r.sleep()
pub_wb.publish(Bool(False)); rospy.sleep(0.5)
res = {"rotulo": rotulo, "ganhos": ganhos, "postura": POSTURA, "erro_inicial_m": erro0, "tempo_s": t_ok, "t_50mm_s": t_50, "melhor_m": melhor,
       "align": st["align"], "advance": st["advance"], "vmax": st["vmax"], "wmax": st["wmax"], "tombou": st["tilt"], "timeout": t_ok is None and not st["tilt"]}
open(saida, "a").write(json.dumps(res) + "\n")
print("[exp] %s: erro inicial %.3f m -> %s | melhor %.3f | ALIGN %d | vmax %.2f wmax %.2f | tombou %s" % (
    rotulo, erro0 or -1, ("%.1f s" % t_ok) if t_ok is not None else "TIMEOUT", melhor, st["align"], st["vmax"], st["wmax"], st["tilt"]))
