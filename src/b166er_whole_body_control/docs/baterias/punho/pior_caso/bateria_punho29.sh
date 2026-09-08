#!/usr/bin/env bash
# Pior caso do punho (J4 2,9 N·m): pose ref x5, híbrido, sonda do J4 por fase em cada run.
set +u
G=/home/marco/.claude/jobs/89ade7b6/tmp/gatilho
OUT=/home/marco/.claude/jobs/89ade7b6/tmp/bateria_punho29
mkdir -p "$OUT/ref"; echo "run,rotulo,x,y,yaw,resultado" > "$OUT/indice.csv"
for n in 1 2 3 4 5; do
  echo "[punho29] run$n ref $(date +%H:%M:%S)"
  bash "$G/roda_sonda_j4_fase.sh" "$OUT/run${n}_sonda_j4.csv" 330 2.9 > "$OUT/run${n}_sonda_j4.log" 2>&1 &
  sp=$!
  sleep 3
  "$G/run_once_pose.sh" "$n" "$OUT/ref" 0.0 1.0 0 "" 2>&1 | tail -2
  res=$(grep -oE "MISSION_[A-Z_]+" "$OUT/ref/run$n.log" 2>/dev/null | tail -1)
  echo "$n,ref,0.0,1.0,0,${res:-SEM_RESULTADO}" >> "$OUT/indice.csv"
  kill "$sp" 2>/dev/null; wait "$sp" 2>/dev/null
  sleep 5
done
echo "[punho29] fim $(date +%H:%M:%S)"
