#!/usr/bin/env bash
# Provoca a recuperação: standoff_lateral 1.2; rosparam set /chave_mission/coarse_distance 0.95 m só na etapa intermediária
# (a final ignora o parâmetro e para de frente para a tag). A 1,3 m do
# olhal e 1,8 m de lado a tag fica ~54° fora do eixo (raio ~0,64), na zona
# em que o PnP a rejeita → remedida com 0 amostras → _recupera_tag.
set +u
R=/home/marco/.claude/jobs/89ade7b6/tmp/gatilho/run_once_pose.sh
OUT=/home/marco/.claude/jobs/89ade7b6/tmp/teste_recuperacao3
mkdir -p "$OUT"
source /home/marco/miniforge3/etc/profile.d/conda.sh; conda activate ros_env; source /home/marco/b166er/devel/setup.bash
rosparam set /chave_mission/standoff_lateral 1.2; rosparam set /chave_mission/coarse_distance 0.95
for n in 1 2; do
  "$R" "$n" "$OUT" 0.0 1.0 0 "" 2>&1 | tail -2
  sleep 5
done
rosparam delete /chave_mission/standoff_lateral; rosparam delete /chave_mission/coarse_distance
echo "[recup3] fim $(date +%H:%M:%S)"
