#!/usr/bin/env bash
# Provoca a recuperação da tag: força o SEARCH a amostrar de FRENTE
# (janela 0-0,15) na pose "frente", onde o yaw sai ~32° torto e a tag some
# na etapa intermediária. Parâmetros privados do nó, postos no master
# antes do launch e apagados depois.
set +u
R=/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/run_once_pose.sh
OUT=/home/marco/.claude/jobs/89ade7b6/tmp/teste_recuperacao
mkdir -p "$OUT"
source /home/marco/miniforge3/etc/profile.d/conda.sh; conda activate ros_env; source /home/marco/b166er/devel/setup.bash
rosparam set /chave_mission/search_raio_min 0.0
rosparam set /chave_mission/search_raio_max 0.15
for n in 1 2; do
  "$R" "$n" "$OUT" 0.0 1.0 90 "" 2>&1 | tail -2
  sleep 5
done
rosparam delete /chave_mission/search_raio_min; rosparam delete /chave_mission/search_raio_max
echo "[recup] fim $(date +%H:%M:%S)"
