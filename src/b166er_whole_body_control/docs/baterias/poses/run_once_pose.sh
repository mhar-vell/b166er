#!/usr/bin/env bash
# Uma execução da missão a partir de uma pose de partida dada.
# Uso: run_once_pose.sh N OUT X Y YAW_DEG "<extra args>"
set +u
N="$1"; OUT="$2"; X="$3"; Y="$4"; YAW="$5"; EXTRA="$6"
S=/home/marco/b166er/src/b166er_whole_body_control/scripts
mkdir -p "$OUT"
"$S/sim_stack.sh" preflight || { echo "[run$N] PREFLIGHT REPROVADO"; exit 1; }
source /home/marco/miniforge3/etc/profile.d/conda.sh
conda activate ros_env
source /home/marco/b166er/devel/setup.bash
ok=0
for tent in 1 2 3; do
    python3 "$S/reset_sim.py" --x "$X" --y "$Y" --yaw "$YAW" | tee -a "$OUT/run${N}_reset.log" | grep -q -- "-> OK" || continue
    sleep 3
    python3 /home/marco/.claude/jobs/89ade7b6/tmp/gatilho/checa_pose.py "$X" "$Y" "$YAW" | tee -a "$OUT/run${N}_reset.log" && { ok=1; break; }
    echo "[run$N] pose fora do pedido na tentativa $tent, repetindo reset"
done
[ "$ok" = 1 ] || { echo "[run$N] RESET FALHOU (pose)"; exit 1; }
sleep 2
LOG="$OUT/run$N.log"
echo "[run$N] pose ($X, $Y, ${YAW}°) lançando $(date +%H:%M:%S) -> $LOG ($EXTRA)"
roslaunch b166er_whole_body_control chave_mission.launch phase_timeout:=60 $EXTRA > "$LOG" 2>&1 &
pid=$!
for _ in $(seq 1 180); do
    grep -qE "resultado: MISSION" "$LOG" 2>/dev/null && break
    sleep 4
done
echo "[run$N] fim $(date +%H:%M:%S)"
grep -E "resultado: MISSION" "$LOG" || echo "[run$N] SEM RESULTADO em 12 min"
kill "$pid" 2>/dev/null; sleep 3
pkill -f "chave_mission.launch" 2>/dev/null
f=$(ls -t ~/.ros/log/*/chave_mission*.log 2>/dev/null | head -1)
[ -n "$f" ] && cp "$f" "$OUT/run${N}_rosout_acumulado.log"
exit 0
