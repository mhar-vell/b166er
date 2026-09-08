#!/usr/bin/env python3
"""Bateria de poses de partida: uma linha por execução a partir dos run*.log.
Colunas: pose, resultado, yaw estimado pelo SEARCH (°, verdadeiro 180),
amostras/outliers do SEARCH, amostras da remedida, fase que falhou, lâmina, duração."""
import re, csv, glob, os, sys, math
D = sys.argv[1] if len(sys.argv) > 1 else "/home/marco/.claude/jobs/89ade7b6/tmp/bateria_poses"
idx = list(csv.DictReader(open(os.path.join(D, "indice.csv"))))
def t(l):
    m = re.search(r"\[INFO\] \[[\d.]+, ([\d.]+)\]", l); return float(m.group(1)) if m else None
rows = []
for r in idx:
    f = os.path.join(D, r["rotulo"], "run%s.log" % r["run"])
    if not os.path.exists(f): continue
    ls = [l for l in open(f, errors="ignore") if "DEBUG" not in l]
    d = dict(run=int(r["run"]), pose=r["rotulo"], x=r["x"], y=r["y"], yaw0=r["yaw"], res=r["resultado"].replace("MISSION_", ""),
             yaw_est=None, n_srch=None, out_srch=None, n_rem=None, yaw_rem=None, falha="", lamina=None, dur=None, lingueta=None)
    t0 = t1 = None
    for l in ls:
        if "State machine starting" in l: t0 = t(l)
        if "resultado: MISSION" in l: t1 = t(l)
        m = re.search(r"SEARCH: (\d+) amostras — parede \([^)]*\) yaw=([-\d.]+)", l)
        if m: d["n_srch"] = int(m.group(1)); d["yaw_est"] = math.degrees(float(m.group(2)))
        m = re.search(r"SEARCH: (\d+) de (\d+) amostras descartadas", l)
        if m: d["out_srch"] = "%s/%s" % (m.group(1), m.group(2))
        m = re.search(r"remedida: só (\d+) amostras", l)
        if m: d["n_rem"] = int(m.group(1))
        m = re.search(r"remedida: (\d+) amostras — parede \([^)]*\) yaw=([-\d.]+)", l)
        if m: d["n_rem"] = int(m.group(1)); d["yaw_rem"] = math.degrees(float(m.group(2)))
        m = re.search(r"REFINE: (\d+) amostras — parede \([^)]*\) yaw=([-\d.]+)", l)
        if m: d["yaw_ref"] = math.degrees(float(m.group(2)))
        m = re.search(r"transitioning '(\w+)':'(failed|timeout|aborted)'", l)
        if m and not d["falha"]: d["falha"] = "%s:%s" % (m.group(1), m.group(2))
        m = re.search(r"lâmina medida em ([\d.]+)°", l)
        if m: d["lamina"] = float(m.group(1))
        m = re.search(r"lâmina em ([\d.]+)° \(mínimo", l)
        if m: d["lamina"] = float(m.group(1))
        m = re.search(r"perdi a tag", l)
        if m: d["falha"] += " perdi-tag"
        m = re.search(r"lingueta[^\d]*([\d.]+) ?mm", l)
        if m: d["lingueta"] = float(m.group(1))
    d["dur"] = (t1 - t0) if (t0 and t1) else None
    rows.append(d)
def f(v, fmt="%.1f"): return "-" if v is None else (fmt % v)
print("%-4s %-7s %-8s %-8s %8s %7s %6s %9s %7s %6s %5s  %s" % ("run", "pose", "partida", "result", "yawSRCH", "outl", "nRem", "yawRem", "yawREF", "lâm", "dur", "falha"))
for d in rows:
    print("%-4d %-7s %-8s %-8s %8s %7s %6s %9s %7s %6s %5s  %s" % (d["run"], d["pose"], "%s,%s,%s" % (d["x"], d["y"], d["yaw0"]), d["res"],
          f(d["yaw_est"]), d["out_srch"] or "-", f(d["n_rem"], "%d"), f(d["yaw_rem"]), f(d.get("yaw_ref")), f(d["lamina"]), f(d["dur"], "%.0f"), d["falha"]))
