#!/usr/bin/env bash
# run6 da bateria do punho com a sonda do J4 por fase gravando em paralelo.
set +u
G=/home/marco/.claude/jobs/89ade7b6/tmp/gatilho
OUT=/home/marco/.claude/jobs/89ade7b6/tmp/bateria_punho
bash "$G/roda_sonda_j4_fase.sh" "$OUT/run6_sonda_j4.csv" 330 > "$OUT/run6_sonda_j4.log" 2>&1 &
sp=$!
sleep 3
"$G/run_once_pose.sh" 6 "$OUT/ref" 0.0 1.0 0 "" 2>&1 | tail -2
res=$(grep -oE "MISSION_[A-Z_]+" "$OUT/ref/run6.log" 2>/dev/null | tail -1)
echo "6,ref,0.0,1.0,0,${res:-SEM_RESULTADO}" >> "$OUT/indice.csv"
wait $sp
echo "[run6] fim $(date +%H:%M:%S)"
