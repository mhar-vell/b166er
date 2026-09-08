#!/usr/bin/env python3
"""Resumo por run: resultado, e por fase de contato (destrava/libera/arco1/arco2) o |J4| máx, fração saturada e excursão do J4 (°)."""
import csv, sys, glob, os, collections
D=sys.argv[1]; LIM=float(sys.argv[2]) if len(sys.argv)>2 else 2.9
idx={r["run"]:r["resultado"] for r in csv.DictReader(open(os.path.join(D,"indice.csv")))}
fases=["destrava","libera","arco1","arco2"]
print("%-4s %-10s | %s" % ("run","resultado"," | ".join("%-22s"%f for f in fases)))
for f in sorted(glob.glob(os.path.join(D,"run*_sonda_j4.csv"))):
    n=os.path.basename(f).split("_")[0][3:]
    rows=[r for r in csv.DictReader(open(f)) if r["estado"]=="MANIPULATE"]
    g=collections.defaultdict(list)
    for r in rows: g[r["fase"]].append(r)
    cel=[]
    for fase in fases:
        rs=g.get(fase)
        if not rs: cel.append("%-22s"%"-"); continue
        j4=[abs(float(r["j4_Nm"])) for r in rs]; pos=[float(r["j4_deg"]) for r in rs]
        sat=100.0*sum(int(abs(float(r["j4_Nm"]))>=0.95*LIM) for r in rs)/len(rs)
        cel.append("%-22s" % ("%.2f Nm %3.0f%% %+.0f°" % (max(j4), sat, max(pos)-min(pos))))
    print("%-4s %-10s | %s" % (n, idx.get(n,"?").replace("MISSION_",""), " | ".join(cel)))
