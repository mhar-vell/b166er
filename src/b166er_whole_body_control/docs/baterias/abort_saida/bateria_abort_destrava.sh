#!/bin/bash
# Timeout do destrava forçado (wb_phase_timeout curto) a partir de REFINE → observa o ABORT. uso: bateria_abort_destrava.sh ROTULO N
source /home/marco/miniforge3/etc/profile.d/conda.sh && conda activate ros_env && source /home/marco/b166er/devel/setup.bash
S=/home/marco/b166er/src/b166er_whole_body_control/scripts; OUT=/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/abort_destrava; mkdir -p $OUT
ROT=$1; N=${2:-2}; TO=${3:-6.0}
for i in $(seq 1 $N); do
  python3 $S/reset_sim.py --x 0.19 --y 1.99 --yaw 90 > /dev/null 2>&1 || python3 $S/reset_sim.py --x 0.19 --y 1.99 --yaw 90 > /dev/null 2>&1
  sleep 2
  rosparam set /chave_mission/wb_phase_timeout $TO
  rosparam set /chave_mission/reassenta_max 0
  LOG=$OUT/${ROT}_run$i.log
  roslaunch b166er_whole_body_control chave_mission.launch phase_timeout:=60 estado_inicial:=REFINE > $LOG 2>&1 &
  pid=$!
  for _ in $(seq 1 120); do grep -qE "resultado: MISSION" $LOG 2>/dev/null && break; sleep 4; done
  kill -INT $pid 2>/dev/null; sleep 4
  LAM=$(rosservice call /gazebo/get_joint_properties "joint_name: 'chave_blade_joint'" 2>/dev/null | grep "^position" | grep -o "\[[-0-9.e]*\]" | tr -d "[]")
  LIN=$(rosservice call /gazebo/get_joint_properties "joint_name: 'chave_lingueta_joint'" 2>/dev/null | grep "^position" | grep -o "\[[-0-9.e]*\]" | tr -d "[]")
  echo "[$ROT run$i] lâmina_rad=$LAM lingueta_m=$LIN | $(grep -aE 'não fechou em|ABORT: ponta|saida.*it[0-9]: ponta|saida.*(alcançada|sem fechar)|afasta: concluído|lâmina|resultado: MISSION' $LOG | sed 's/.*\[mission\] //' | cut -c1-95 | tr '\n' ';')"
done
rosparam set /chave_mission/wb_phase_timeout 40.0; rosparam set /chave_mission/reassenta_max 1
echo "[bat] fim"
