#!/usr/bin/env bash
# Instala Miniforge3 e recria o ros_env (RoboStack Noetic) na NUC.
# Roda como usuário 'robo' na NUC.
# Pré-requisito: ros_env.yml no mesmo diretório deste script.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MINIFORGE_INSTALLER="/tmp/miniforge_installer.sh"
MINIFORGE_DIR="$HOME/miniforge3"
ENV_FILE="${SCRIPT_DIR}/ros_env.yml"

# --- 1. Verifica o arquivo de ambiente ---
if [ ! -f "${ENV_FILE}" ]; then
  echo "[ERRO] ros_env.yml não encontrado em ${SCRIPT_DIR}"
  exit 1
fi

# --- 2. Instala Miniforge3 (se não existir) ---
if [ ! -d "${MINIFORGE_DIR}" ]; then
  echo "==> Baixando Miniforge3..."
  curl -fsSL https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh \
    -o "${MINIFORGE_INSTALLER}"
  bash "${MINIFORGE_INSTALLER}" -b -p "${MINIFORGE_DIR}"
  rm "${MINIFORGE_INSTALLER}"
  echo "==> Miniforge3 instalado em ${MINIFORGE_DIR}"
else
  echo "==> Miniforge3 já instalado, pulando."
fi

# --- 3. Inicializa conda no shell ---
source "${MINIFORGE_DIR}/etc/profile.d/conda.sh"
conda config --add channels conda-forge
conda config --add channels robostack-noetic
conda config --set channel_priority strict

# --- 4. Cria/recria o ros_env ---
if conda env list | grep -q "^ros_env "; then
  echo "==> Ambiente 'ros_env' já existe. Removendo para recriar..."
  conda env remove -n ros_env -y
fi

echo "==> Criando ros_env a partir de ${ENV_FILE}..."
echo "    (pode levar 10-20 minutos dependendo da conexão)"
conda env create -f "${ENV_FILE}"

# --- 5. Configura init automático no ~/.bashrc ---
if ! grep -q "conda initialize" "$HOME/.bashrc"; then
  "${MINIFORGE_DIR}/bin/conda" init bash
fi

# Adiciona ativação automática do ros_env
if ! grep -q "conda activate ros_env" "$HOME/.bashrc"; then
  cat >> "$HOME/.bashrc" << 'BASHRC'

# RoboStack / ROS Noetic
conda activate ros_env
BASHRC
fi

echo ""
echo "======================================"
echo " ROS Noetic (RoboStack) configurado!"
echo " Ambiente : ros_env"
echo " Para usar: conda activate ros_env"
echo "            source \$CONDA_PREFIX/setup.bash"
echo "======================================"
echo " Reinicie o shell ou execute:"
echo "   source ~/.bashrc"
