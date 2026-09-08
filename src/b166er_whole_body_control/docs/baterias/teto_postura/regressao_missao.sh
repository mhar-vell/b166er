#!/bin/bash
# Regressão da missão completa com o teto por postura LIGADO (pose ref, híbrido).
source /home/marco/miniforge3/etc/profile.d/conda.sh && conda activate ros_env && source /home/marco/b166er/devel/setup.bash
OUT=$1; N=${2:-2}
rosparam set /fuzzy_wb_controller/teto_postura/enable true
mkdir -p "$OUT"
for i in $(seq 1 $N); do
  /home/marco/.claude/jobs/89ade7b6/tmp/gatilho/run_once_pose.sh $i "$OUT" 0.0 1.0 0 "" 2>&1 | grep --line-buffered "resultado\|reset\|Traceback\|fase\|FALH" | cut -c1-200
done
echo "[bat] fim"
