#!/usr/bin/env bash
# Instala udev rules para a Intel RealSense T265.
# Roda UMA VEZ em cada máquina (Shiroi e NUC) com a T265 conectada.
# Uso: bash setup/udev_t265.sh
#
# VIDs:
#   03e7:2150 — Myriad VPU (bootloader antes do firmware)
#   8087:0b37 — T265 modo operacional (pós-firmware)

set -euo pipefail

RULES_FILE="/etc/udev/rules.d/99-realsense-t265.rules"

echo "==> Instalando udev rules para Intel RealSense T265..."

sudo tee "${RULES_FILE}" > /dev/null << 'EOF'
# Intel RealSense T265 Tracking Camera (modo operacional, VID=8087)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8087", ATTRS{idProduct}=="0b37", MODE:="0666", GROUP:="plugdev"
# T265 em modo DFU
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8087", ATTRS{idProduct}=="0b3a", MODE:="0666", GROUP:="plugdev"
# Dispositivos HID associados (IMU interno)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8087", ATTRS{idProduct}=="0b52", MODE:="0666", GROUP:="plugdev"
# Myriad VPU (T265 pre-firmware, bootloader mode, VID=03e7)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="03e7", ATTRS{idProduct}=="2150", MODE:="0666", GROUP:="plugdev"
EOF

echo "==> Adicionando usuário ao grupo plugdev..."
sudo usermod -aG plugdev "${USER}"

echo "==> Recarregando udev..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "======================================"
echo " udev rules instaladas: ${RULES_FILE}"
echo " Reconecte a T265 para aplicar."
echo " (Re-login necessário para o grupo plugdev)"
echo "======================================"
