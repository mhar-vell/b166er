#!/bin/bash
# A/B do teto por postura na aproximação, alvo em y=1,9 (ponta fora da
# fixture) e escala conjunta base+braço. uso: bateria_teto2.sh SAIDA.jsonl
source /home/marco/miniforge3/etc/profile.d/conda.sh && conda activate ros_env && source /home/marco/b166er/devel/setup.bash
OUT=$1
run() { # enable postura rotulo
  rosparam set /fuzzy_wb_controller/teto_postura/enable $1
  for tent in 1 2 3; do python3 /home/marco/b166er/src/b166er_whole_body_control/scripts/reset_sim.py > /dev/null && { echo "[reset] ok ($tent)"; break; }; echo "[reset] falhou ($tent)"; sleep 3; done
  timeout 200 python3 /home/marco/.claude/jobs/89ade7b6/tmp/gatilho/exp_aproximacao.py "$3" "[]" "$OUT" "$2" 120 1.9 2>&1 | grep --line-buffered "^\[exp\]\|Traceback\|Error"
}
for i in 1 2; do run false search "sem_search_$i"; run true search "teto_search_$i"; done
run true deploy teto_deploy_1; run false deploy sem_deploy_1
echo "[bat] fim"
