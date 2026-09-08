#!/usr/bin/env bash
# Bateria: N execuções de run_once.sh com args extras, cada uma com a sonda.
# Uso: bateria_wb.sh <primeiro_N> <quantas> <dir> "<extra args do launch>"
set +u
N0="$1"; Q="$2"; OUT="$3"; EXTRA="$4"
RUN=/home/marco/.claude/projects/-home-marco-b166er/sessoes/2026-09-02_ferramentas/run_once.sh
SONDA=/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/sonda_wb.py
mkdir -p "$OUT"
for k in $(seq 0 $((Q - 1))); do
    n=$((N0 + k))
    echo "[bateria] run$n  $(date +%H:%M:%S)  extra: $EXTRA"
    ( sleep 25; bash -c "source /home/marco/miniforge3/etc/profile.d/conda.sh && conda activate ros_env && source /home/marco/b166er/devel/setup.bash && timeout 700 python3 $SONDA $OUT/run${n}_sonda.csv" ) > "$OUT/run${n}_sonda.log" 2>&1 &
    sp=$!
    "$RUN" "$n" "$OUT" "$EXTRA" 2>&1 | tail -2
    kill "$sp" 2>/dev/null
    # o rosout da missão fica no diretório do master (mesmo arquivo entre runs): copia o estado atual
    f=$(ls -t ~/.ros/log/*/chave_mission*.log 2>/dev/null | head -1)
    [ -n "$f" ] && cp "$f" "$OUT/run${n}_rosout_acumulado.log"
    sleep 5
done
echo "[bateria] fim $(date +%H:%M:%S)"
