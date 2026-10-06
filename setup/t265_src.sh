#!/usr/bin/env bash
# Pilha da Intel RealSense T265 para o ros_env (RoboStack, Python 3.12).
#
# POR QUE DO FONTE (2026-10-06). O suporte à T265 foi REMOVIDO do
# librealsense a partir da 2.54; o pacote ros-noetic-librealsense2 do
# RoboStack para py3.12 é 2.56.5 e o rs-enumerate-devices não vê a câmera
# (ela fica enumerada como "Movidius MA2X5X", 03e7:2150, esperando um
# firmware que ninguém carrega). O ambiente da NUC reprovisionado em 11 Set
# ficou assim; o shiroi, onde a T265 funciona, usa librealsense 2.53.1 do
# conda-forge e o realsense-ros 2.3.2 (ros1-legacy) + ddynamic_reconfigure
# 0.4.2 compilados dentro do workspace, com três ajustes de CMake para o
# toolchain do conda (CMake >= 3.5, C++17, testes do ddynamic sem gmock).
# Este script reproduz exatamente isso. Uso, com o ros_env já criado:
#
#   bash setup/t265_src.sh        # instala a lib, clona em src/, aplica os patches
#   catkin build realsense2_camera ddynamic_reconfigure
#   # replugar a T265 no USB depois do build (ela carrega o firmware ao enumerar)
#   rs-enumerate-devices          # deve listar "Intel RealSense T265"
set -euo pipefail
WS="${WS:-$HOME/b166er}"
MINIFORGE_DIR="${MINIFORGE_DIR:-$HOME/miniforge3}"
ENV_NAME="${ENV_NAME:-ros_env}"

echo "==> librealsense 2.53.1 (conda-forge) no $ENV_NAME; removendo os pacotes RoboStack sem T265"
"$MINIFORGE_DIR/bin/mamba" remove -n "$ENV_NAME" ros-noetic-realsense2-camera ros-noetic-librealsense2 -y 2>/dev/null || true
"$MINIFORGE_DIR/bin/mamba" install -n "$ENV_NAME" -c conda-forge "librealsense=2.53.1" -y

cd "$WS/src"
[ -d realsense-ros ] || git clone --branch 2.3.2 --depth 1 https://github.com/IntelRealSense/realsense-ros.git
[ -d ddynamic_reconfigure ] || git clone --branch 0.4.2 --depth 1 https://github.com/pal-robotics/ddynamic_reconfigure.git

echo "==> patches de CMake para o toolchain do conda"
sed -i 's/cmake_minimum_required(VERSION 2.8.3)/cmake_minimum_required(VERSION 3.5)/' \
  realsense-ros/realsense2_camera/CMakeLists.txt \
  realsense-ros/realsense2_description/CMakeLists.txt \
  ddynamic_reconfigure/CMakeLists.txt
sed -i 's/-std=c++11/-std=c++17/g' realsense-ros/realsense2_camera/CMakeLists.txt
sed -i 's/^if (CATKIN_ENABLE_TESTING)/if (FALSE) # CATKIN_ENABLE_TESTING — desabilitado: gmock indisponível no ros_env conda/' \
  ddynamic_reconfigure/CMakeLists.txt
echo "==> pronto: catkin build realsense2_camera ddynamic_reconfigure"
