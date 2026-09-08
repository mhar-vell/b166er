#!/usr/bin/env python3
"""Compara modo whole-body (rosout acumulado das runs 21-25 + sondas) com o
modo postura (rosout das runs 19/20). Por run: resultado, lâmina, e por fase
da manipulação: tempo (s) e resíduo (mm)."""
import re, csv, glob, sys, datetime as dt
FASES = ["orienta", "aproxima_lateral", "atravessa", "captura", "destrava", "libera", "arco1", "arco2", "desengata"]
def ts(line):
    m = re.search(r"(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}),(\d{3})", line)
    return dt.datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S").timestamp() + int(m.group(2)) / 1000 if m else None
def runs_de(path):
    """Divide o rosout em execuções (State machine starting ... resultado)."""
    out, cur = [], None
    for l in open(path, errors="ignore"):
        if "State machine starting" in l:
            cur = {"t0": ts(l), "linhas": []}
        if cur is not None:
            cur["linhas"].append(l)
            if "[mission] resultado:" in l:
                cur["t1"] = ts(l); out.append(cur); cur = None
    return out
def analisa(run):
    r = {"resultado": "?", "lamina": None, "fases": {}}
    t_ini = {}
    for l in run["linhas"]:
        m = re.search(r"resultado: (MISSION_\w+)", l)
        if m: r["resultado"] = m.group(1)
        m = re.search(r"MANIPULATE concluída — chave ABERTA, lâmina medida em ([\d.]+)°", l)
        if m: r["lamina"] = float(m.group(1))
        m = re.search(r'fase "(\w+)": whole-body assumiu', l)
        if m: t_ini[m.group(1)] = ts(l)
        m = re.search(r'âncora engatada \(fase "(\w+)"\)', l)
        if m and m.group(1) not in t_ini: t_ini[m.group(1)] = ts(l)
        m = re.search(r'fase "(\w+)" alcançada por whole-body em ([\d.]+)s \(([\d.]+) m\)', l)
        if m: r["fases"][m.group(1)] = (float(m.group(2)), 1000 * float(m.group(3)), "wb")
        m = re.search(r'fase "(\w+)" alcançada em (\d+) iteração\(ões\) \(([\d.]+) m\)', l)
        if m:
            f = m.group(1); t = ts(l) - t_ini.get(f, ts(l))
            r["fases"][f] = (t, 1000 * float(m.group(3)), "ik%s" % m.group(2))
        m = re.search(r'fase "(\w+)": (5 iterações sem fechar|whole-body não fechou)', l)
        if m: r["fases"][m.group(1)] = (None, None, "FALHOU")
    r["dur"] = run["t1"] - run["t0"]
    return r
def sonda(path):
    try: rows = list(csv.DictReader(open(path)))
    except Exception: return None
    if not rows: return None
    lmax = max(float(x["lingueta_mm"]) for x in rows)
    manip = [x for x in rows if x.get("estado") == "MANIPULATE"]
    nz = sum(int(x["armvel_nz"]) for x in manip); n = sum(int(x["armvel_n"]) for x in manip)
    porfase = {}
    for f in FASES:
        xs = [x for x in manip if x.get("fase") == f]
        porfase[f] = (sum(int(x["armvel_nz"]) for x in xs), sum(int(x["armvel_n"]) for x in xs))
    return lmax, nz, n, porfase
def tabela(rotulo, runs, sondas):
    print("\n=== %s ===" % rotulo)
    print("%-6s %-14s %6s %5s | %s" % ("run", "resultado", "lâmina", "dur", " ".join("%13s" % f[:12] for f in FASES)))
    for nome, run in runs:
        a = analisa(run)
        cel = []
        for f in FASES:
            v = a["fases"].get(f)
            cel.append("%13s" % ("-" if v is None else ("FALHOU" if v[2] == "FALHOU" else "%4.1fs %4.1fmm" % (v[0], v[1]))))
        print("%-6s %-14s %6s %5.0f | %s" % (nome, a["resultado"], "%.1f°" % a["lamina"] if a["lamina"] else "-", a["dur"], " ".join(cel)))
        s = sondas.get(nome)
        if s:
            lmax, nz, n, pf = s
            print("       sonda: lingueta máx %.1f mm | arm_vel_cmd≠0 na manipulação: %d de %d amostras | por fase: %s" % (lmax, nz, n, " ".join("%s %d/%d" % (f[:5], pf[f][0], pf[f][1]) for f in FASES if pf[f][1])))
wb = runs_de("run25_rosout_acumulado.log")
sondas = {"run%d" % (21 + k): sonda("run%d_sonda.csv" % (21 + k)) for k in range(5)}
tabela("WHOLE-BODY (use_wholebody:=true), runs 21-25", [("run%d" % (21 + k), r) for k, r in enumerate(wb)], sondas)
ik = runs_de("/home/marco/.claude/projects/-home-marco-b166er/sessoes/2026-09-03_ferramentas/run19_run20_rosout.log")
tabela("POSTURA (IK iterativa), runs 19-20 (mesmo fixture com gatilho)", [("run%d" % (19 + k), r) for k, r in enumerate(ik)], {})
