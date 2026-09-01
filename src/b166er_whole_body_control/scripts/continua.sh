#!/usr/bin/env bash
# Libera a próxima etapa quando a missão está em modo ~pausa_por_fase.
#
# Existe para o Marco poder olhar a simulação parada no fim de cada fase
# e só então mandar seguir — pedido de 2026-08-31, depois de um dia
# inteiro em que ele via o problema na tela antes de eu ver no log.
set -uo pipefail
CONDA_SH="${CONDA_SH:-/home/marco/miniforge3/etc/profile.d/conda.sh}"
WS="${B166ER_WS:-/home/marco/b166er}"
set +u
source "$CONDA_SH" && conda activate "${CONDA_ENV:-ros_env}" \
    && source "$WS/devel/setup.bash"
set -u
rostopic pub -1 /b166er/mission_continue std_msgs/Empty "{}" >/dev/null 2>&1
echo "[continua] próxima etapa liberada"
