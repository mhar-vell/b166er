#!/bin/bash
# Reproduz a run8: destrava fecha cedo (gatilho travado) e a libera puxa. uso: bateria_run8.sh ROTULO YAML N
source /home/marco/miniforge3/etc/profile.d/conda.sh && conda activate ros_env && source /home/marco/b166er/devel/setup.bash
S=/home/marco/b166er/src/b166er_whole_body_control/scripts; OUT=/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/run8
ROT=$1; YAML=$2; N=${3:-3}
for i in $(seq 1 $N); do
  python3 $S/reset_sim.py --x 0.19 --y 1.99 --yaw 90 > /dev/null 2>&1 || python3 $S/reset_sim.py --x 0.19 --y 1.99 --yaw 90 > /dev/null 2>&1
  sleep 2
  LOG=$OUT/${ROT}_run$i.log
  roslaunch b166er_whole_body_control chave_mission.launch phase_timeout:=60 estado_inicial:=REFINE task_yaml:=$YAML > $LOG 2>&1 &
  pid=$!
  for _ in $(seq 1 120); do grep -qE "resultado: MISSION" $LOG 2>/dev/null && break; sleep 4; done
  kill -INT $pid 2>/dev/null; sleep 4
  echo "[$ROT run$i] $(grep -aE 'destrava: ponta desceu|descida estagnou|libera.*alcançada|PERDEU|SOLTURA|lâmina medida em|resultado: MISSION' $LOG | sed 's/.*\[mission\] //' | cut -c1-110 | tr '\n' ';')"
done
echo "[bat] fim"
