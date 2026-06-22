#!/usr/bin/env bash
# Configura auto-login e roscore como serviço systemd na NUC.
# Pré-requisito: nuc_setup.sh e nuc_ros_setup.sh já executados.
# Uso: bash setup/nuc_services_setup.sh

set -euo pipefail

ROS_ENV_PREFIX="/home/robo/miniforge3/envs/ros_env"
NUC_HOSTNAME="b166er-nuc"

# ---------------------------------------------------------------------------
# 1. Auto-login do usuário robo no console (tty1)
# ---------------------------------------------------------------------------
echo "==> Configurando auto-login para robo em tty1..."
sudo mkdir -p /etc/systemd/system/getty@tty1.service.d
printf '[Service]\nExecStart=\nExecStart=-/sbin/agetty --autologin robo --noclear %%I $TERM\n' \
  | sudo tee /etc/systemd/system/getty@tty1.service.d/override.conf > /dev/null
sudo systemctl daemon-reload
echo "    ok — robo fará login automático no próximo boot."

# ---------------------------------------------------------------------------
# 2. Script wrapper para roscore (ativa PATH do conda antes do source)
# ---------------------------------------------------------------------------
echo "==> Instalando /usr/local/bin/roscore_start.sh..."
sudo tee /usr/local/bin/roscore_start.sh > /dev/null << WRAPPER
#!/bin/bash
export PATH="${ROS_ENV_PREFIX}/bin:/home/robo/miniforge3/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source ${ROS_ENV_PREFIX}/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=${NUC_HOSTNAME}.local
exec ${ROS_ENV_PREFIX}/bin/roscore
WRAPPER
sudo chmod +x /usr/local/bin/roscore_start.sh

# ---------------------------------------------------------------------------
# 3. Serviço systemd roscore
# ---------------------------------------------------------------------------
echo "==> Instalando e habilitando roscore.service..."
sudo tee /etc/systemd/system/roscore.service > /dev/null << 'UNIT'
[Unit]
Description=ROS Core
After=network-online.target avahi-daemon.service
Wants=network-online.target

[Service]
Type=simple
User=robo
ExecStart=/usr/local/bin/roscore_start.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
UNIT

sudo systemctl daemon-reload
sudo systemctl enable roscore
sudo systemctl restart roscore

echo ""
echo "======================================"
echo " Serviços configurados com sucesso!"
echo " Auto-login : ativo no próximo boot"
echo " roscore    : $(sudo systemctl is-active roscore)"
echo "======================================"
