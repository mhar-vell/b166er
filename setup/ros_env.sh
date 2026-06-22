#!/usr/bin/env bash
# Configura o ambiente ROS no notebook para usar a NUC como master.
# Uso: source ~/b166er/setup/ros_env.sh
#
# A NUC deve estar acessível via mDNS (b166er-nuc.local) antes de executar.

NUC_HOST="b166er-nuc.local"
CONDA_ROS_PREFIX="$HOME/miniforge3/envs/ros_env"
WORKSPACE_DEVEL="$HOME/b166er/devel/setup.bash"

# Verifica se a NUC está acessível
if ! ping -c 1 -W 2 "${NUC_HOST}" &>/dev/null; then
  echo "[ros_env] AVISO: '${NUC_HOST}' não respondeu ao ping. Verifique a conexão."
  return 1
fi

export ROS_MASTER_URI="http://${NUC_HOST}:11311"
export ROS_HOSTNAME=$(hostname).local
export ROS_IP=$(ip route get "$(getent hosts "${NUC_HOST}" | awk '{print $1}')" \
              | awk '/src/{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' | head -1)

# Adiciona binários ROS ao PATH
export PATH="${CONDA_ROS_PREFIX}/bin:${PATH}"
export PYTHONPATH="${CONDA_ROS_PREFIX}/lib/python3.11/site-packages:${PYTHONPATH:-}"

# Sourcing do workspace local (detecta shell)
if [ -n "${ZSH_VERSION:-}" ] && [ -f "${WORKSPACE_DEVEL/setup.bash/setup.zsh}" ]; then
  source "${WORKSPACE_DEVEL/setup.bash/setup.zsh}"
elif [ -f "${WORKSPACE_DEVEL}" ]; then
  source "${WORKSPACE_DEVEL}"
fi

echo "[ros_env] ROS_MASTER_URI = ${ROS_MASTER_URI}"
echo "[ros_env] ROS_HOSTNAME   = ${ROS_HOSTNAME}"
echo "[ros_env] ROS_IP         = ${ROS_IP}"
