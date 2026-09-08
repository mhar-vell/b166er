#!/usr/bin/env bash
# Três baterias de 5 com ganhos fixos (linha de base do escalonador Fuzzy),
# híbrido por fase, mesma régua. Sai ao terminar.
set +u
B=/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/bateria_wb.sh
OUT=/home/marco/.claude/jobs/89ade7b6/tmp/bateria_fixos
mkdir -p "$OUT"
"$B" 73 5 "$OUT/conservador" 'fixed_gains:=[0.3,0.3,0.05]' 2>&1 | tail -3
"$B" 78 5 "$OUT/medio"       'fixed_gains:=[0.8,0.8,0.08]' 2>&1 | tail -3
"$B" 83 5 "$OUT/agressivo"   'fixed_gains:=[1.4,1.4,0.03]' 2>&1 | tail -3
echo "[fixos] fim $(date +%H:%M:%S)"
