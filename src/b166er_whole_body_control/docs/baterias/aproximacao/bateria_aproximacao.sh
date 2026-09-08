#!/usr/bin/env bash
# Bateria de aproximação de longe: 4 regimes x N execuções, reset entre elas.
set +u
N="${1:-5}"; OUT="${2:-/home/marco/.claude/jobs/89ade7b6/tmp/bateria_aprox}"
S=/home/marco/b166er/src/b166er_whole_body_control/scripts
E=/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/exp_aproximacao.py
mkdir -p "$OUT"
source /home/marco/miniforge3/etc/profile.d/conda.sh; conda activate ros_env; source /home/marco/b166er/devel/setup.bash
"$S/sim_stack.sh" preflight || { echo "[aprox] PREFLIGHT REPROVADO"; exit 1; }
for reg in "fuzzy:[]" "conservador:[0.3,0.3,0.05]" "medio:[0.8,0.8,0.08]" "agressivo:[1.4,1.4,0.03]"; do
    rot=${reg%%:*}; g=${reg#*:}
    for k in $(seq 1 $N); do
        python3 "$S/reset_sim.py" > /dev/null 2>&1 || { echo "[aprox] RESET FALHOU ($rot $k)"; continue; }
        sleep 3
        timeout 180 python3 "$E" "$rot" "$g" "$OUT/resultados.jsonl" travel 2>&1 | grep "^\[exp\]"
    done
done
echo "[aprox] fim $(date +%H:%M:%S)"
