#!/bin/bash
# Varredura de posturas na aproximação de longe (teto atual 0,3 m/s / 0,5 rad/s).
# uso: varre_posturas.sh ROTULO SAIDA.jsonl [posturas...]
source /home/marco/miniforge3/etc/profile.d/conda.sh && conda activate ros_env && source /home/marco/b166er/devel/setup.bash
ROT=$1; OUT=$2; shift 2
for P in "$@"; do
  for tent in 1 2 3; do
    python3 /home/marco/b166er/src/b166er_whole_body_control/scripts/reset_sim.py && break
    echo "[varre] reset falhou ($tent)"; sleep 3
  done
  timeout 200 python3 /home/marco/.claude/jobs/89ade7b6/tmp/gatilho/exp_aproximacao.py "${ROT}_${P}" "[]" "$OUT" "$P" 2>&1 | grep "^\[exp\]\|Traceback\|Error"
done
echo "[varre] fim"
