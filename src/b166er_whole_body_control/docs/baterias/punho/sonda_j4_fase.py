#!/usr/bin/env python3
"""CSV a 10 Hz: t, estado, fase, J4 pos (°), J4 esforço (N·m), J2/J3 esforço, saturado.
Uso: sonda_j4_fase.py SAIDA.csv DURACAO_S [LIMITE]"""
import sys, time, csv, math, json, re, rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
out, dur = sys.argv[1], float(sys.argv[2]); LIM = float(sys.argv[3]) if len(sys.argv) > 3 else 4.2
rospy.init_node("sonda_j4_fase", anonymous=True)
st = {"estado": "", "fase": ""}; js = {}
def cb_st(m):
    s = m.data.strip()
    try:
        d = json.loads(s)
    except Exception:
        d = dict(kv.split("=", 1) for kv in re.split(r"[|;\s]+", s) if "=" in kv)
    st["estado"] = str(d.get("estado", st["estado"])); st["fase"] = str(d.get("fase", st["fase"]))
def cb_js(m):
    if "J4" not in m.name or not m.effort: return
    for j in ("J2", "J3", "J4"):
        i = m.name.index(j); js[j] = (m.position[i], m.effort[i])
rospy.Subscriber("/b166er/mission_status", String, cb_st)
rospy.Subscriber("/joint_states", JointState, cb_js)
with open(out, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["t", "estado", "fase", "j4_deg", "j4_Nm", "j2_Nm", "j3_Nm", "sat"])
    t0 = time.time()
    while time.time() - t0 < dur and not rospy.is_shutdown():
        if "J4" in js:
            p, e = js["J4"]
            w.writerow(["%.1f" % (time.time() - t0), st["estado"], st["fase"], "%.1f" % math.degrees(p), "%.2f" % e,
                        "%.2f" % js.get("J2", (0, 0))[1], "%.2f" % js.get("J3", (0, 0))[1], int(abs(e) >= 0.95 * LIM)])
        time.sleep(0.1)
print("sonda_j4_fase: gravado", out)
