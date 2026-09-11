#!/usr/bin/env bash
# Captura as ENTRADAS do estimador com a base andando em stow.
set -uo pipefail
OUT="$1"; mkdir -p "$OUT"
set +u; source "$HOME/miniforge3/etc/profile.d/conda.sh"; conda activate ros_env; source "$HOME/b166er/devel/setup.bash"; set -u
S="$HOME/b166er/src/b166er_whole_body_control/scripts"
python3 "$S/reset_sim.py" > "$OUT/reset.log" 2>&1
rosbag record -O "$OUT/ik_in.bag" --lz4 /pioneer/pose /t265/odom/sample /joint_states /b166er/robot_state /pioneer3at/odom /rosout /clock > "$OUT/rosbag.log" 2>&1 &
BAG=$!
sleep 3
# anda 12 s reto a 0,15 m/s, depois gira 6 s, depois para
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.15}}' > /dev/null 2>&1 &
P1=$!; sleep 12; kill $P1
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.10}, angular: {z: 0.3}}' > /dev/null 2>&1 &
P2=$!; sleep 6; kill $P2
rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}' > /dev/null 2>&1
sleep 2
kill -INT $BAG; sleep 3
echo "fim"
