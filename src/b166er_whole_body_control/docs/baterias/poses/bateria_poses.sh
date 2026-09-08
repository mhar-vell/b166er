#!/usr/bin/env bash
# Bateria de poses de partida: 8 poses x 3 execuções, híbrido padrão.
# Chave em (0, 3.0) na parede y=3; pose padrão da missão = (0, 1, 0°).
set +u
R=/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/run_once_pose.sh
OUT=/home/marco/.claude/jobs/89ade7b6/tmp/bateria_poses
mkdir -p "$OUT"; : > "$OUT/indice.csv"
echo "run,rotulo,x,y,yaw,resultado" >> "$OUT/indice.csv"
n=1
while read -r rot x y yaw; do
    [ -z "$rot" ] && continue
    for k in 1 2 3; do
        echo "[poses] run$n $rot ($x,$y,$yaw) $(date +%H:%M:%S)"
        "$R" "$n" "$OUT/$rot" "$x" "$y" "$yaw" "" 2>&1 | tail -2
        res=$(grep -oE "MISSION_[A-Z_]+" "$OUT/$rot/run$n.log" 2>/dev/null | tail -1)
        echo "$n,$rot,$x,$y,$yaw,${res:-SEM_RESULTADO}" >> "$OUT/indice.csv"
        n=$((n+1)); sleep 5
    done
done <<'POSES'
ref        0.0  1.0    0
perto      0.0  1.5    0
longe      0.0  0.5    0
esq        -0.5 1.0    0
dir        0.5  1.0    0
frente     0.0  1.0   90
costas     0.0  1.0  180
diag       0.5  1.5  -45
POSES
echo "[poses] fim $(date +%H:%M:%S)"
