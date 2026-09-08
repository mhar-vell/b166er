#!/usr/bin/env python3
"""Por (estado, fase): amostras, |J4| máx e média, fração saturada, faixa de J4 (°), J2/J3 máx."""
import csv, sys, collections
rows=list(csv.DictReader(open(sys.argv[1])))
g=collections.OrderedDict()
for r in rows:
    k=(r["estado"], r["fase"]); g.setdefault(k, []).append(r)
print("%-11s %-17s %5s %7s %7s %6s %10s %7s %7s" % ("estado","fase","n","J4max","J4med","sat","J4 pos °","J2max","J3max"))
for (e,f),rs in g.items():
    j4=[abs(float(r["j4_Nm"])) for r in rs]; pos=[float(r["j4_deg"]) for r in rs]
    sat=sum(int(r["sat"]) for r in rs)/len(rs)
    print("%-11s %-17s %5d %7.2f %7.2f %5.0f%% %4.0f..%4.0f %7.2f %7.2f" % (e or "-", f or "-", len(rs), max(j4), sum(j4)/len(j4), 100*sat, min(pos), max(pos),
          max(abs(float(r["j2_Nm"])) for r in rs), max(abs(float(r["j3_Nm"])) for r in rs)))
