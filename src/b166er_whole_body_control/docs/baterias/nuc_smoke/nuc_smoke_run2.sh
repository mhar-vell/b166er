#!/usr/bin/env bash
# Missão da chave no NUC gravando cmd_vel/odom/base_cap/robot_state/rosout
# para achar quem freia a base no RETURN. Saída em $OUT.
set -uo pipefail
OUT="${1:-$HOME/b166er_nuc_return}"
mkdir -p "$OUT"
set +u
source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate ros_env
source "$HOME/b166er/devel/setup.bash"
set -u
S="$HOME/b166er/src/b166er_whole_body_control/scripts"

echo "inicio $(date +%T)" > "$OUT/estado.txt"
python3 "$S/reset_sim.py" > "$OUT/reset.log" 2>&1
echo "reset rc=$? $(date +%T)" >> "$OUT/estado.txt"
grep -q -- "-> OK" "$OUT/reset.log" || { echo "RESET_FALHOU" >> "$OUT/estado.txt"; exit 1; }

rostopic info /cmd_vel > "$OUT/cmd_vel_info_antes.txt" 2>&1
rosbag record -O "$OUT/return.bag" --lz4 \
    /cmd_vel /pioneer3at/odom /b166er/base_cap /b166er/robot_state \
    /b166er/front_clearance /b166er/obstacle_close /rosout /clock \
    > "$OUT/rosbag.log" 2>&1 &
BAG=$!

(
  while true; do
    idle=$(vmstat 1 2 | tail -1 | awk '{print $15}')
    ram=$(free -m | awk '/Mem/{print $3}')
    rtf=$(timeout 3 gz stats -p 2>/dev/null | tail -1 | cut -d, -f1 | tr -d ' ')
    echo "$(date +%T) cpu=$((100 - idle)) ram=$ram rtf=${rtf:-?}"
    sleep 8
  done
) > "$OUT/amostras.txt" 2>/dev/null &
SAMPLER=$!

sleep 4
roslaunch b166er_whole_body_control chave_mission.launch \
    phase_timeout:=60 tol_pos:=0.020 > "$OUT/run.log" 2>&1 &
MISSAO=$!
echo "missao pid=$MISSAO $(date +%T)" >> "$OUT/estado.txt"
for _ in $(seq 1 900); do
    grep -qE "resultado: MISSION" "$OUT/run.log" 2>/dev/null && break
    kill -0 "$MISSAO" 2>/dev/null || break
    sleep 2
done
sleep 3
rostopic info /cmd_vel > "$OUT/cmd_vel_info_depois.txt" 2>&1
r=$(grep -oE "MISSION_(OK|ABORTED)" "$OUT/run.log" | tail -1)
echo "resultado ${r:-SEM_RESULTADO} $(date +%T)" >> "$OUT/estado.txt"
kill "$MISSAO" 2>/dev/null; sleep 5; kill -9 "$MISSAO" 2>/dev/null
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}' >/dev/null 2>&1
kill -INT "$BAG" 2>/dev/null; sleep 4
kill "$SAMPLER" 2>/dev/null
echo "fim $(date +%T)" >> "$OUT/estado.txt"
