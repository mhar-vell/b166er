#!/usr/bin/env python3
"""Tabela do ensaio de frenagem: por postura, sem × com regra (média das reps)."""
import sys, json, collections
rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
g = collections.defaultdict(list)
for r in rows:
    modo = "com" if r["rotulo"].startswith("teto") else "sem"
    g[(r["postura"], modo)].append(r)
print("| postura | regra | v no salto (m/s) | tilt máx depois (rad) | degrau máx de v (m/s) | v→0 (s) | tombou |")
print("|---|---|---|---|---|---|---|")
for p in ["travel", "stow_home", "search", "deploy"]:
    for modo in ["sem", "com"]:
        rs = g.get((p, modo), [])
        if not rs: continue
        f = lambda k: "/".join("%.3f" % r[k] if r[k] is not None else "?" for r in rs)
        print("| %s | %s | %s | %s | %s | %s | %s |" % (p, modo, "/".join("%.2f" % r["v_odom_no_salto"] for r in rs),
              f("tilt_max_depois"), f("max_dv_cmd"), "/".join(("%.2f" % r["t_v_zero_s"]) if r["t_v_zero_s"] is not None else "?" for r in rs),
              "/".join("SIM" if r["tombou"] else "não" for r in rs)))
