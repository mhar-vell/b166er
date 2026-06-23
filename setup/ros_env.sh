#!/usr/bin/env bash
# Configura o ambiente ROS no notebook para usar a NUC como master.
# Uso: source ~/b166er/setup/ros_env.sh
#
# Prioridade de conexão:
#   1. ZeroTier (10.32.1.176) — funciona em qualquer rede com internet
#   2. mDNS (b166er-nuc.local) — funciona na mesma LAN / AP direto

NUC_ZEROTIER="10.32.1.176"
NUC_MDNS="b166er-nuc.local"
SHIROI_ZEROTIER="10.32.1.149"
CONDA_ROS_PREFIX="$HOME/miniforge3/envs/ros_env"
WORKSPACE_DEVEL="$HOME/b166er/devel/setup.bash"

# Detecta qual rota está disponível
if ping -c 1 -W 2 "${NUC_ZEROTIER}" &>/dev/null; then
  NUC_HOST="${NUC_ZEROTIER}"
  SHIROI_HOST="${SHIROI_ZEROTIER}"
  echo "[ros_env] Usando ZeroTier (${NUC_ZEROTIER})"
elif ping -c 1 -W 2 "${NUC_MDNS}" &>/dev/null; then
  NUC_HOST="${NUC_MDNS}"
  SHIROI_HOST="$(hostname).local"
  echo "[ros_env] Usando mDNS (${NUC_MDNS})"
else
  echo "[ros_env] ERRO: NUC não encontrada via ZeroTier nem mDNS."
  return 1
fi

export ROS_MASTER_URI="http://${NUC_HOST}:11311"
export ROS_HOSTNAME="${SHIROI_HOST}"
export ROS_IP=$(ip route get "$(getent hosts "${NUC_HOST}" 2>/dev/null | awk '{print $1}' || echo "${NUC_HOST}")" \
              2>/dev/null | awk '/src/{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' | head -1)

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
