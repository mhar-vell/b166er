#!/usr/bin/env bash
# J4 limitado a 4,2 N·m (exp/punho-limitado): pose 'ref' x5, híbrido padrão.
set +u
R=/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/run_once_pose.sh
OUT=/home/marco/.claude/jobs/89ade7b6/tmp/bateria_punho
mkdir -p "$OUT"; echo "run,rotulo,x,y,yaw,resultado" > "$OUT/indice.csv"
for n in 1 2 3 4 5; do
  echo "[punho] run$n ref $(date +%H:%M:%S)"
  "$R" "$n" "$OUT/ref" 0.0 1.0 0 "" 2>&1 | tail -2
  res=$(grep -oE "MISSION_[A-Z_]+" "$OUT/ref/run$n.log" 2>/dev/null | tail -1)
  echo "$n,ref,0.0,1.0,0,${res:-SEM_RESULTADO}" >> "$OUT/indice.csv"
  sleep 5
done
echo "[punho] fim $(date +%H:%M:%S)"
