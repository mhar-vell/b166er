#!/bin/bash
# Exercita o reassentamento forçado: 1ª estagnação curta forçada no destrava. uso: bateria_reassenta.sh N
source /home/marco/miniforge3/etc/profile.d/conda.sh && conda activate ros_env && source /home/marco/b166er/devel/setup.bash
S=/home/marco/b166er/src/b166er_whole_body_control/scripts; OUT=/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/descentrada; N=${1:-2}
for i in $(seq 1 $N); do
  python3 $S/reset_sim.py --x 0.19 --y 1.99 --yaw 90 > /dev/null 2>&1 || python3 $S/reset_sim.py --x 0.19 --y 1.99 --yaw 90 > /dev/null 2>&1
  sleep 2
  rosparam set /chave_mission/ensaio_forca_reassenta true
  LOG=$OUT/reassenta_run$i.log
  roslaunch b166er_whole_body_control chave_mission.launch phase_timeout:=60 estado_inicial:=REFINE > $LOG 2>&1 &
  pid=$!
  for _ in $(seq 1 120); do grep -qE "resultado: MISSION" $LOG 2>/dev/null && break; sleep 4; done
  kill -INT $pid 2>/dev/null; sleep 4
  echo "[reassenta run$i] $(grep -aE 'FORÇADA|REASSENTANDO|reassenta: captura|destrava: ponta desceu|descida estagnou|não conseguiu|recaptura não|resultado: MISSION' $LOG | sed 's/.*\[mission\] //' | cut -c1-80 | tr '\n' ';')"
done
rosparam set /chave_mission/ensaio_forca_reassenta false
echo "[bat] fim"
