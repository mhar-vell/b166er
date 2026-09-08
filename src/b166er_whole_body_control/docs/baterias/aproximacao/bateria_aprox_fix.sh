#!/usr/bin/env bash
# Repetição com align_min_planar_dist = 0,90: Fuzzy x5 e conservador x5.
set +u
OUT=/home/marco/.claude/jobs/89ade7b6/tmp/bateria_aprox_fix
S=/home/marco/b166er/src/b166er_whole_body_control/scripts
E=/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/exp_aproximacao.py
mkdir -p "$OUT"
source /home/marco/miniforge3/etc/profile.d/conda.sh; conda activate ros_env; source /home/marco/b166er/devel/setup.bash
"$S/sim_stack.sh" preflight > /dev/null || { echo "[aprox-fix] PREFLIGHT REPROVADO"; exit 1; }
for reg in "fuzzy:[]" "conservador:[0.3,0.3,0.05]"; do
    rot=${reg%%:*}; g=${reg#*:}
    for k in 1 2 3 4 5; do
        python3 "$S/reset_sim.py" > /dev/null 2>&1 || { echo "[aprox-fix] RESET FALHOU ($rot $k)"; continue; }
        sleep 3
        timeout 180 python3 "$E" "$rot" "$g" "$OUT/resultados.jsonl" travel 2>&1 | grep "^\[exp\]"
    done
done
echo "[aprox-fix] fim $(date +%H:%M:%S)"
