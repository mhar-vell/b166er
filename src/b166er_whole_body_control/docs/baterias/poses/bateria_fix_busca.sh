#!/usr/bin/env bash
# Revalida as poses que falharam (frente 0/3, esq 2/3) + ref, com a busca
# oblíqua e a recuperação da tag. Mesmo run_once_pose.sh.
set +u
R=/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/run_once_pose.sh
OUT=/home/marco/.claude/jobs/89ade7b6/tmp/bateria_fix_busca
mkdir -p "$OUT"; echo "run,rotulo,x,y,yaw,resultado" > "$OUT/indice.csv"
n=1
while read -r rot x y yaw; do
    [ -z "$rot" ] && continue
    for k in 1 2 3; do
        [ "$rot" = ref ] && [ "$k" = 3 ] && continue
        echo "[fixbusca] run$n $rot ($x,$y,$yaw) $(date +%H:%M:%S)"
        "$R" "$n" "$OUT/$rot" "$x" "$y" "$yaw" "" 2>&1 | tail -2
        res=$(grep -oE "MISSION_[A-Z_]+" "$OUT/$rot/run$n.log" 2>/dev/null | tail -1)
        echo "$n,$rot,$x,$y,$yaw,${res:-SEM_RESULTADO}" >> "$OUT/indice.csv"
        n=$((n+1)); sleep 5
    done
done <<'POSES'
frente     0.0  1.0   90
esq        -0.5 1.0    0
ref        0.0  1.0    0
POSES
echo "[fixbusca] fim $(date +%H:%M:%S)"
