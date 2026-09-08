#!/bin/bash
# A/B do teto por postura no MESMO stack (o controlador relê o parâmetro
# a cada wb_enable). uso: bateria_teto.sh SAIDA.jsonl
source /home/marco/miniforge3/etc/profile.d/conda.sh && conda activate ros_env && source /home/marco/b166er/devel/setup.bash
OUT=$1
run() { # modo postura rotulo
  rosparam set /fuzzy_wb_controller/teto_postura/enable $1
  for tent in 1 2 3; do python3 /home/marco/b166er/src/b166er_whole_body_control/scripts/reset_sim.py && break; echo "[bat] reset falhou ($tent)"; sleep 3; done
  timeout 200 python3 /home/marco/.claude/jobs/89ade7b6/tmp/gatilho/exp_aproximacao.py "$3" "[]" "$OUT" "$2" 2>&1 | grep --line-buffered "^\[exp\]\|Traceback\|Error"
}
for i in 1 2; do run false search "sem_search_$i"; run true search "teto_search_$i"; done
run true travel teto_travel_1; run true deploy teto_deploy_1
echo "[bat] fim"
