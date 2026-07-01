# Roadmap b166er

Doutorado UTFPR — robótica móvel com manipulação: o b166er deve navegar
autonomamente e manipular objetos via servovisão Fuzzy whole-body sem encoders
de junta.

**Arquitetura central:**
T265 (EE) + Pioneer odom → state_estimator (IK) → Fuzzy WB Controller (8-DOF)
→ cmd_vel (base) + arm_vel_cmd (braço)

---

## Fase 1 — Infraestrutura e Modelagem ✅ concluída

| Etapa | Status |
|---|---|
| URDF unificado: Pioneer 3-AT + RV-M2 + T265 + Hokuyo + IMU | ✅ |
| Posicionamento dos sensores por foto do robô real | ✅ |
| Parâmetros DH e limites de junta do RV-M2 (manual oficial) | ✅ |
| Massas RV-M2 escaladas para 28 kgf (manual Mitsubishi) | ✅ |
| Driver Sparton AHRS-8: launch + tópico `/imu/data` | ✅ |
| NUC: auto-login GDM + roscore systemd + udev rules | ✅ |
| Bug Gazebo resolvido: migração para Python 3.12 + gazebo-ros 2.9.3 | ✅ |

---

## Fase 2 — Controle Whole-Body no Gazebo ✅ concluída (PR #18)

**Meta:** pipeline end-to-end validado em simulação:
`ee_target → fuzzy WB → cmd_vel + arm_vel_cmd`

| Etapa | Status |
|---|---|
| Cinemática direta FK + Jacobiana whole-body 8-DOF | ✅ |
| `state_estimator`: T265 + odom → IK (DLS) → q_arm estimado | ✅ |
| Seed do IK sincronizado por timestamp do T265 (buffer deque) | ✅ |
| `fuzzy_wb_controller`: Mamdani (k_pos, k_orient, λ adaptativos) | ✅ |
| `arm_vel_integrator`: integra q̇ → posição para hardware | ✅ |
| `gazebo_arm_bridge`: equivalente para simulação (ros_control) | ✅ |
| `gazebo_sensor_sim`: FK sintético → /t265/odom/sample | ✅ |
| Launch unificado `b166er_wb.launch` (sim / hardware / gazebo) | ✅ |
| 4 rodas do Pioneer visíveis no RViz (pioneer_wheel_state_pub) | ✅ |
| Posição inicial J2=0.5 rad no spawn (evita colisão com Pioneer) | ✅ |
| Workspace seguro documentado: z ≥ 0.60 m (frame odom) | ✅ |
| Documentação técnica: controle sem encoders + diagrama de blocos | ✅ |

---

## Fase 3 — Redução do Shaking (braço) 🔄 próxima etapa

**Problema:** braço oscila em torno da posição de equilíbrio após o warm-start
porque os controladores PID de posição (ros_control) não têm compensação de
gravidade — o torque gravitacional não é cancelado, gerando oscilação amortecida.

**Abordagem em etapas:**

### 3.1 Diagnóstico (medir a oscilação)
| Etapa | Status |
|---|---|
| Gravar `ros_bag` dos tópicos `/joint_states` e `/b166er/robot_state` | ⬜ |
| Plotar q(t) por junta e identificar frequência natural e amplitude | ⬜ |
| Identificar quais juntas oscilam mais (esperado: J2 e J3, mais pesadas) | ⬜ |

### 3.2 Feedforward de gravidade no bridge (simulação)
| Etapa | Status |
|---|---|
| Calcular torque gravitacional por junta: τ_grav(q) = ∂U/∂q | ⬜ |
| Adicionar termo feedforward em `gazebo_arm_bridge`: `q_cmd += K_ff · τ_grav · dt` | ⬜ |
| Validar redução da oscilação no Gazebo (comparar bag antes/depois) | ⬜ |

### 3.3 Ajuste PID (se feedforward não for suficiente)
| Etapa | Status |
|---|---|
| Reduzir P de J2/J3 (menos agressividade) e aumentar D (mais amortecimento) | ⬜ |
| Testar combinação P/D para criticamente amortecido sem lentidão excessiva | ⬜ |

### 3.4 Validação final
| Etapa | Status |
|---|---|
| Amplitude de oscilação < 0.05 rad em steady-state | ⬜ |
| IK converge sem warnings por ≥ 60 s contínuos | ⬜ |
| EE tracking de alvo estático com erro < 5 mm | ⬜ |

---

## Fase 4 — Validação em Hardware Real

**Pré-requisito:** Fases 2 e 3 concluídas.

| Etapa | Status |
|---|---|
| Conectar braço RV-M2 via rosserial (Arduino) | ⬜ |
| Validar `arm_vel_integrator` em hardware (velocidade → posição serial) | ⬜ |
| Calibração hand-eye: T265 → flange do RV-M2 | ⬜ |
| Teste de rastreamento: alvo estático com braço real | ⬜ |
| Teste de rastreamento: alvo em movimento lento (servovisão) | ⬜ |
| Ajuste PID / feedforward para dinâmica real (diferente da simulação) | ⬜ |

---

## Fase 5 — Navegação Autônoma (Pilar 1)

| Etapa | Status |
|---|---|
| Pioneer 3-AT: validar `/cmd_vel` em hardware | ⬜ |
| `robot_localization`: fusão odometria Pioneer + T265 + IMU | ⬜ |
| Hokuyo UST-05LX: IP fixo na rede Ethernet + launch | ⬜ |
| `move_base`: planejamento de trajetória com mapa local | ⬜ |
| Teste integrado: ponto A → ponto B autônomo | ⬜ |

---

## Fase 6 — Integração Whole-Body + Navegação

**Meta final:** robô navega até a peça e a manipula de forma autônoma.

| Etapa | Status |
|---|---|
| Integrar planner de navegação com whole-body controller | ⬜ |
| Coordenação base-braço: navegar enquanto posiciona EE | ⬜ |
| E-Stop físico (relé) + watchdog ROS | ⬜ |
| Demonstração completa: navegação + manipulação autônoma | ⬜ |

---

## Scripts de Setup

| Script | Finalidade |
|---|---|
| `setup/nuc_setup.sh` | SSH, avahi, hostname |
| `setup/nuc_ros_setup.sh` | Miniforge3 + ros_env + realsense2_camera |
| `setup/nuc_services_setup.sh` | GDM auto-login + roscore.service |
| `setup/udev_pioneer.sh` | `/dev/ttyPioneer` |
| `setup/udev_t265.sh` | T265 (`8087:0b37` + Myriad VPU `03e7:2150`) |
