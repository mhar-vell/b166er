# Roadmap b166er

Dois pilares de integração para o doutorado em robótica móvel (UTFPR):
o robô deve navegar autonomamente até um objetivo definido pelo usuário
e manipular objetos usando servovisão Fuzzy sem encoders.

---

## Pilar 1 — Navegação Autônoma (Base + T265)

**Meta:** usuário define um ponto objetivo → plataforma navega sozinha
usando a T265 como sensor de pose visual-inercial.

| Etapa | Status |
|---|---|
| T265: udev, launch nodelet, validado Shiroi + NUC | ✅ |
| Pioneer 3-AT: teste básico `/cmd_vel` e odometria | ⬜ |
| `robot_localization`: fusão odometria Pioneer + T265 + IMU | ⬜ |
| `move_base`: planejamento de trajetória até objetivo | ⬜ |
| Teste integrado: ponto A → ponto B autônomo | ⬜ |

---

## Pilar 2 — Manipulação com Servovisão Fuzzy (Braço + T265)

**Meta:** T265 detecta posição do end-effector → Fuzzy ajusta eixos
do RV-M2 para atingir alvo visual sem encoders.

| Etapa | Status |
|---|---|
| Arduino firmware: controle PWM dos 5 eixos do RV-M2 | ⬜ |
| rosserial: NUC ↔ Arduino (comando + feedback) | ⬜ |
| URDF/Xacro: cinemática direta do RV-M2 | ⬜ |
| Lógica Fuzzy: erro visual → delta PWM por eixo | ⬜ |
| Calibração câmera-braço (hand-eye T265 → flange) | ⬜ |
| Teste integrado: servovisão até alvo estático | ⬜ |

---

## Infraestrutura Compartilhada

| Etapa | Status |
|---|---|
| NUC: auto-login GDM + roscore systemd | ✅ |
| Sparton AHRS-8 IMU: driver + launch | ⬜ |
| URDF unificado (Pioneer + RV-M2 + T265 + IMU) | ⬜ |
| E-Stop físico (relé) + watchdog ROS | ⬜ |

---

## Setup por máquina

| Script | Finalidade |
|---|---|
| `setup/nuc_setup.sh` | SSH, avahi, hostname |
| `setup/nuc_ros_setup.sh` | Miniforge3 + ros_env + realsense2_camera |
| `setup/nuc_services_setup.sh` | GDM auto-login + roscore.service |
| `setup/udev_pioneer.sh` | `/dev/ttyPioneer` |
| `setup/udev_t265.sh` | T265 (`8087:0b37` + Myriad VPU `03e7:2150`) |
