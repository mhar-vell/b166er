#!/usr/bin/env bash
# Configura auto-login e roscore como serviço systemd na NUC.
# Pré-requisito: nuc_setup.sh e nuc_ros_setup.sh já executados.
# Uso: bash setup/nuc_services_setup.sh

set -euo pipefail

ROS_ENV_PREFIX="/home/robo/miniforge3/envs/ros_env"
NUC_HOSTNAME="b166er-nuc"

# ---------------------------------------------------------------------------
# 1a. Auto-login GDM (interface gráfica)
# ---------------------------------------------------------------------------
echo "==> Configurando auto-login GDM para robo..."
sudo sed -i 's/^#\s*AutomaticLoginEnable\s*=.*/AutomaticLoginEnable = true/' /etc/gdm3/custom.conf
sudo sed -i 's/^#\s*AutomaticLogin\s*=.*/AutomaticLogin = robo/' /etc/gdm3/custom.conf
# Garante que as linhas existam caso o sed não tenha encontrado
grep -q '^AutomaticLoginEnable' /etc/gdm3/custom.conf || \
  sudo sed -i '/^\[daemon\]/a AutomaticLoginEnable = true\nAutomaticLogin = robo' /etc/gdm3/custom.conf
echo "    ok — GDM fará login automático como robo no próximo boot."

# ---------------------------------------------------------------------------
# 1b. Auto-login getty tty1 (fallback sem interface gráfica)
# ---------------------------------------------------------------------------
echo "==> Configurando auto-login getty tty1 (fallback)..."
sudo mkdir -p /etc/systemd/system/getty@tty1.service.d
printf '[Service]\nExecStart=\nExecStart=-/sbin/agetty --autologin robo --noclear %%I $TERM\n' \
  | sudo tee /etc/systemd/system/getty@tty1.service.d/override.conf > /dev/null
sudo systemctl daemon-reload
echo "    ok — getty tty1 também configurado."

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
