#!/bin/bash
# A/B do teto por postura no ensaio de frenagem. uso: bateria_frenagem.sh SAIDA.jsonl REPS
source /home/marco/miniforge3/etc/profile.d/conda.sh && conda activate ros_env && source /home/marco/b166er/devel/setup.bash
OUT=$1; REPS=${2:-2}
run() { # enable postura rotulo
  rosparam set /fuzzy_wb_controller/teto_postura/enable $1
  for tent in 1 2 3; do python3 /home/marco/b166er/src/b166er_whole_body_control/scripts/reset_sim.py > /dev/null && break; echo "[bat] reset falhou ($tent)"; sleep 3; done
  timeout 90 python3 /home/marco/.claude/jobs/89ade7b6/tmp/gatilho/exp_frenagem.py "$3" "$OUT" "$2" 2>&1 | grep --line-buffered "^\[frenagem\]\|Traceback\|Error"
}
for i in $(seq 1 $REPS); do
  for P in travel stow_home search deploy; do
    run false $P "sem_${P}_$i"
    run true  $P "teto_${P}_$i"
  done
done
echo "[bat] fim"
