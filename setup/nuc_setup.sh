#!/usr/bin/env bash
# Roda UMA VEZ na NUC (com monitor/teclado conectados).
# Instala SSH, avahi (mDNS), tmux e registra a chave do notebook.

set -euo pipefail

NUC_HOSTNAME="b166er-nuc"
NOTEBOOK_PUBKEY="ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIBnQJEC67mteyorgglZY5bcf3p7/2aAoNjC73iTveOyP marcoreis@me.com"

echo "==> Atualizando pacotes..."
sudo apt-get update -q

echo "==> Instalando openssh-server, avahi-daemon, tmux..."
sudo apt-get install -y openssh-server avahi-daemon avahi-utils tmux libnss-mdns

echo "==> Habilitando e iniciando serviços..."
sudo systemctl enable --now ssh
sudo systemctl enable --now avahi-daemon

echo "==> Definindo hostname para '${NUC_HOSTNAME}'..."
sudo hostnamectl set-hostname "${NUC_HOSTNAME}"
# Garante resolução local do próprio hostname
grep -qxF "127.0.1.1 ${NUC_HOSTNAME}" /etc/hosts || \
  echo "127.0.1.1 ${NUC_HOSTNAME}" | sudo tee -a /etc/hosts

echo "==> Registrando chave SSH do notebook..."
mkdir -p ~/.ssh
chmod 700 ~/.ssh
echo "${NOTEBOOK_PUBKEY}" >> ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys

echo "==> Configurando tmux (prefixo Ctrl+A)..."
cat > ~/.tmux.conf << 'TMUX'
set -g prefix C-a
unbind C-b
bind C-a send-prefix
set -g mouse on
set -g history-limit 10000
TMUX

echo ""
echo "======================================"
echo " NUC configurada com sucesso!"
echo " Hostname : ${NUC_HOSTNAME}"
echo " mDNS     : ${NUC_HOSTNAME}.local"
echo " SSH      : ssh robo@${NUC_HOSTNAME}.local"
echo "======================================"
echo " Reinicie para aplicar o hostname:"
echo "   sudo reboot"
