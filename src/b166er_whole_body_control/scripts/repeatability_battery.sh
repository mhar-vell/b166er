#!/usr/bin/env bash
#
# repeatability_battery.sh — roda a missão da chave N vezes e resume.
#
# Uma bateria custa ~8 min por execução. Em 2026-08-24 três baterias
# seguidas (≈3 h) foram perdidas por condições que ninguém checou antes
# de começar: processos órfãos de sessões anteriores, o state_estimator
# morto com o registro vivo no master, e a fixture ausente do mundo. Por
# isso aqui nada roda antes do preflight passar, e o reset entre
# execuções é verificado — se ele falhar, a bateria PARA em vez de
# continuar gerando execuções sem valor.
#
# Uso: repeatability_battery.sh [n_execuções] [dir_de_saída]

set -uo pipefail

N="${1:-8}"
OUT="${2:-/tmp/b166er_battery_$(date +%Y%m%d_%H%M%S)}"
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

mkdir -p "$OUT"
echo "[bateria] $N execuções -> $OUT"

"$AQUI/sim_stack.sh" preflight || {
    echo "[bateria] preflight reprovado — corrija antes de gastar $((N * 8)) min." >&2
    exit 1
}

source /home/marco/miniforge3/etc/profile.d/conda.sh
conda activate ros_env
# shellcheck disable=SC1091
source /home/marco/b166er/devel/setup.bash

for i in $(seq 1 "$N"); do
    if ! python3 "$AQUI/reset_sim.py"; then
        echo "[bateria] run $i: RESET FALHOU — interrompida" | tee -a "$OUT/resumo.txt"
        break
    fi
    sleep 4

    log="$OUT/run_$i.log"
    roslaunch b166er_whole_body_control chave_mission.launch \
        phase_timeout:=60 tol_pos:=0.020 > "$log" 2>&1 &
    pid=$!
    for _ in $(seq 1 150); do
        grep -qE "resultado: MISSION" "$log" 2>/dev/null && break
        sleep 2
    done
    sleep 3
    pkill -9 -f chave_mission.py 2>/dev/null
    # NÃO matar o apriltag_localizer nem nada do stack: eles pertencem ao
    # stack principal e derrubá-los aqui cegava todas as execuções
    # seguintes (erro cometido em 2026-08-21).
    wait $pid 2>/dev/null

    r=$(grep -oE "MISSION_(OK|ABORTED)" "$log" | tail -1)
    echo "run $i: ${r:-SEM_RESULTADO}" | tee -a "$OUT/resumo.txt"
done

echo "──────── RESUMO ────────" | tee -a "$OUT/resumo.txt"
ok=$(grep -c "MISSION_OK" "$OUT/resumo.txt" 2>/dev/null || echo 0)
echo "sucesso: $ok/$N" | tee -a "$OUT/resumo.txt"
for f in "$OUT"/run_*.log; do
    [ -e "$f" ] || continue
    causa=$(grep -oE "ERROR.*\[mission\].*" "$f" | tail -1 | sed 's/.*\[mission\] //' | cut -c1-70)
    [ -n "$causa" ] && echo "  $(basename "$f"): $causa" | tee -a "$OUT/resumo.txt"
done
echo "TODAS_CONCLUIDAS"
