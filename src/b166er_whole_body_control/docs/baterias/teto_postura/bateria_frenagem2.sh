#!/bin/bash
# Revalidação da frenagem com o teto angular pela margem lateral. uso: bateria_frenagem2.sh SAIDA.jsonl
source /home/marco/miniforge3/etc/profile.d/conda.sh && conda activate ros_env && source /home/marco/b166er/devel/setup.bash
OUT=$1
run() { rosparam set /fuzzy_wb_controller/teto_postura/enable $1
  for tent in 1 2 3; do python3 /home/marco/b166er/src/b166er_whole_body_control/scripts/reset_sim.py > /dev/null && break; echo "[bat] reset falhou ($tent)"; sleep 3; done
  timeout 90 python3 /home/marco/.claude/jobs/89ade7b6/tmp/gatilho/exp_frenagem.py "$3" "$OUT" "$2" 2>&1 | grep --line-buffered "^\[frenagem\]\|Traceback\|Error"; }
for i in 1 2; do run true search "teto2_search_$i"; run true deploy "teto2_deploy_$i"; done
run true travel teto2_travel_1
echo "[bat] fim"
