#!/usr/bin/env bash
# Valida a guarda de deriva de eixo + indicador de soltura: pose 'longe'
# (onde a run8 falhou) x5, híbrido padrão.
set +u
R=/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/run_once_pose.sh
OUT=/home/marco/.claude/jobs/89ade7b6/tmp/bateria_soltura2
mkdir -p "$OUT"; echo "run,rotulo,x,y,yaw,resultado" > "$OUT/indice.csv"
for n in 1 2 3 4 5; do
  echo "[soltura2] run$n longe $(date +%H:%M:%S)"
  "$R" "$n" "$OUT/longe" 0.0 0.5 0 "" 2>&1 | tail -2
  res=$(grep -oE "MISSION_[A-Z_]+" "$OUT/longe/run$n.log" 2>/dev/null | tail -1)
  echo "$n,longe,0.0,0.5,0,${res:-SEM_RESULTADO}" >> "$OUT/indice.csv"
  sleep 5
done
echo "[soltura2] fim $(date +%H:%M:%S)"
