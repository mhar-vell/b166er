#!/usr/bin/env python3
"""Observa /b166er/tilt por N s SEM reset, para separar a parada seca do
stand-down do teleporte do reset. args: rótulo, segundos"""
import sys, rospy
from std_msgs.msg import Float64, Bool
rospy.init_node("observa_tilt", anonymous=True)
st = {"max": 0.0, "crit": False, "t_crit": None}
t0 = [None]
def cb(m):
    if t0[0] is None: t0[0] = rospy.get_time()
    if m.data > st["max"]: st["max"] = m.data
    if m.data >= 0.45 and st["t_crit"] is None: st["t_crit"] = rospy.get_time() - t0[0]
rospy.Subscriber("/b166er/tilt", Float64, cb)
rospy.Subscriber("/b166er/tilt_critical", Bool, lambda m: st.__setitem__("crit", st["crit"] or m.data))
rospy.sleep(float(sys.argv[2]))
print("[observa] %s: tilt max %.3f rad em %s s de observação | crítico %s | t_crítico %s" % (
    sys.argv[1], st["max"], sys.argv[2], st["crit"], ("%.2f s" % st["t_crit"]) if st["t_crit"] is not None else "-"))
