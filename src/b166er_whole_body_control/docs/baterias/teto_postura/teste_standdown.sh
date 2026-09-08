#!/bin/bash
# Separa a parada seca do stand-down do teleporte do reset: roda a
# aproximação em search por 40 s (sem a regra e depois com) e observa a
# inclinação por 12 s SEM resetar. uso: teste_standdown.sh SAIDA.jsonl modo...
source /home/marco/miniforge3/etc/profile.d/conda.sh && conda activate ros_env && source /home/marco/b166er/devel/setup.bash
OUT=$1; shift
for modo in "$@"; do
  rosparam set /fuzzy_wb_controller/teto_postura/enable $modo
  for tent in 1 2 3; do python3 /home/marco/b166er/src/b166er_whole_body_control/scripts/reset_sim.py > /dev/null && { echo "[reset] ok ($tent)"; break; }; echo "[reset] falhou ($tent)"; sleep 3; done
  timeout 120 python3 /home/marco/.claude/jobs/89ade7b6/tmp/gatilho/exp_aproximacao.py "standdown_regra_$modo" "[]" "$OUT" search 40 2>&1 | grep --line-buffered "^\[exp\]\|Traceback"
  python3 /home/marco/.claude/jobs/89ade7b6/tmp/gatilho/observa_tilt.py "regra_$modo" 12 2>&1 | grep --line-buffered "^\[observa\]\|Traceback"
done
echo "[bat] fim"
