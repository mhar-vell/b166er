#!/usr/bin/env bash
# Instala udev rule na NUC para fixar a porta serial do Pioneer 3-AT em /dev/ttyPioneer.
# Roda uma vez na NUC como usuário robo.
#
# O Pioneer 3-AT usa adaptador USB-Serial (tipicamente FTDI ou Prolific).
# Para descobrir os atributos do seu adaptador, conecte o cabo e rode:
#   lsusb
#   udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep -E "idVendor|idProduct|serial"

set -euo pipefail

RULE_FILE="/etc/udev/rules.d/99-pioneer.rules"

# --- Detecta o dispositivo serial conectado ---
echo "==> Dispositivos USB-Serial detectados:"
ls /dev/ttyUSB* 2>/dev/null || echo "  Nenhum /dev/ttyUSB* encontrado. Conecte o Pioneer e tente novamente."
echo ""

# --- Obtém atributos do primeiro ttyUSB (ajuste se houver mais de um) ---
DEVICE=${1:-/dev/ttyUSB0}

if [ ! -e "${DEVICE}" ]; then
  echo "[ERRO] Dispositivo ${DEVICE} não encontrado."
  echo "       Conecte o Pioneer à NUC e passe o device como argumento:"
  echo "       bash udev_pioneer.sh /dev/ttyUSBx"
  exit 1
fi

ID_VENDOR=$(udevadm info --name="${DEVICE}" --attribute-walk 2>/dev/null \
  | grep "idVendor" | head -1 | grep -oP '"\K[^"]+')
ID_PRODUCT=$(udevadm info --name="${DEVICE}" --attribute-walk 2>/dev/null \
  | grep "idProduct" | head -1 | grep -oP '"\K[^"]+')
ID_SERIAL=$(udevadm info --name="${DEVICE}" --attribute-walk 2>/dev/null \
  | grep "ATTRS{serial}" | head -1 | grep -oP '"\K[^"]+')

echo "==> Atributos detectados em ${DEVICE}:"
echo "    idVendor  = ${ID_VENDOR}"
echo "    idProduct = ${ID_PRODUCT}"
echo "    serial    = ${ID_SERIAL:-N/A}"

# --- Cria a regra udev ---
if [ -n "${ID_SERIAL:-}" ]; then
  # Usa número de série se disponível (mais preciso com múltiplos adaptadores)
  RULE="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"${ID_VENDOR}\", ATTRS{idProduct}==\"${ID_PRODUCT}\", ATTRS{serial}==\"${ID_SERIAL}\", SYMLINK+=\"ttyPioneer\", MODE=\"0666\""
else
  RULE="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"${ID_VENDOR}\", ATTRS{idProduct}==\"${ID_PRODUCT}\", SYMLINK+=\"ttyPioneer\", MODE=\"0666\""
fi

echo ""
echo "==> Escrevendo regra em ${RULE_FILE}..."
echo "${RULE}" | sudo tee "${RULE_FILE}"

# --- Adiciona usuário ao grupo dialout ---
sudo usermod -aG dialout robo

# --- Recarrega udev ---
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "==> Testando: aguardando /dev/ttyPioneer..."
sleep 2
if [ -L /dev/ttyPioneer ]; then
  echo "    /dev/ttyPioneer -> $(readlink /dev/ttyPioneer)  OK"
else
  echo "    /dev/ttyPioneer não criado. Reconecte o cabo USB e verifique."
fi
