#!/bin/bash
# Reset após execução com o braço estendido e em movimento. uso: teste_reset.sh N [--sem-recolher]
source /home/marco/miniforge3/etc/profile.d/conda.sh && conda activate ros_env && source /home/marco/b166er/devel/setup.bash
N=$1; FLAG=$2
for i in $(seq 1 $N); do
  python3 /home/marco/b166er/src/b166er_whole_body_control/scripts/reset_sim.py > /dev/null 2>&1 || python3 /home/marco/b166er/src/b166er_whole_body_control/scripts/reset_sim.py > /dev/null 2>&1
  timeout 120 python3 /home/marco/.claude/jobs/89ade7b6/tmp/gatilho/exp_aproximacao.py "reset_$i" "[]" /dev/null search 40 1.9 2>&1 | grep --line-buffered "^\[exp\]" | cut -c1-90
  python3 /home/marco/b166er/src/b166er_whole_body_control/scripts/reset_sim.py $FLAG 2>&1 | grep --line-buffered "^reset:" | sed "s/^/[ciclo $i$FLAG] /" | cut -c1-200
done
echo "[bat] fim"
