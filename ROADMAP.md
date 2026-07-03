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

## Fase 3 — Estabilização do braço e da simulação ✅ concluída (PR #20)

**Problema original:** braço oscilava em torno do equilíbrio após o warm-start
(PID de posição sem compensação de gravidade). Durante a fase, o escopo cresceu:
o Pioneer deslocava sozinho no Gazebo e as rodas apareciam colapsadas na origem
do RViz — ambos exigiram diagnóstico ao vivo na simulação.

### 3.1 Shaking do braço (causa: gravidade não compensada)
| Etapa | Status |
|---|---|
| Feedforward de gravidade no `gazebo_arm_bridge`: q_cmd = q + K_ff·τ_grav/P | ✅ |
| `gravity_torque_arm(q)` em `kinematics.py` (massas do manual Mitsubishi) | ✅ |
| Ganhos PID por junta em `arm_controllers.yaml` (P/D dimensionados, I=0) | ✅ |
| Sequência load → unpause → switch no `arm_controller_loader` (sem deadlock) | ✅ |

### 3.2 Rodas colapsadas no RViz ("roda branca")
| Etapa | Status |
|---|---|
| Causa: `rospy.WallRate` inexistente matava o `pioneer_wheel_state_pub` após 1 publish | ✅ |
| TF das rodas publicado a 50 Hz em wall-time (imune a pausas do sim_time) | ✅ |

### 3.3 Deslocamento espontâneo do Pioneer no Gazebo
| Etapa | Status |
|---|---|
| Diagnóstico ao vivo: robô inteiro em queda perpétua (vz≈-0.08 m/s), rodas paradas | ✅ |
| Causa: contato subamortecido (kp=1e6, kd=100 → ζ≈1,6%) + braço horizontal retificando a vibração | ✅ |
| kd 100 → 1e4 nas 4 rodas; remoção de bloco `<gazebo>` duplicado; spawn a 1 mm do chão | ✅ |
| Spawn em q=0 (sem `-J J2 0.5`, que corria contra o unpause e chutava o chassi) | ✅ |
| Fricção de junta das rodas 0,5 N·m (frenagem passiva sem stiction no ajuste fino) | ✅ |

### 3.4 Validação final (medida no Gazebo)
| Critério | Resultado |
|---|---|
| Amplitude de oscilação < 0.05 rad em steady-state | ✅ dq ≈ 0.0002 rad/s |
| IK converge sem warnings por ≥ 60 s contínuos | ✅ `ik_converged` estável em sessão longa |
| EE tracking de alvo estático com erro < 5 mm | ✅ 4,7 mm (alvo longitudinal, base+braço) |
| Piso de velocidade da base (10 mm/s) + histerese no fuzzy | ✅ |

**Limitação conhecida (decisão de projeto em aberto):** alvos com erro
puramente lateral estacionam a ~99 mm — a projeção não-holonômica não gera
esterçamento (v_fwd ≈ 0 para erro ⊥ heading) e o braço não compensa sem
violar a orientação do EE. Estratégias candidatas: manobra girar-avançar-girar,
relaxamento transitório da orientação, ou alinhamento de heading via espaço nulo.

**Estratégia de simulação (decidido em 2026-07-03):** o **Gazebo permanece**
como digital twin de sistema — expõe as mesmas interfaces ROS do hardware
(`/cmd_vel`, `ros_control`, TF, sensores sintéticos), permitindo validar o
pipeline end-to-end sem alterar nenhum nó. **PyBullet não substitui** o Gazebo
(exigiria reescrever toda a ponte ROS já depurada, com um conjunto novo de
quirks de física), mas fica aprovado como **bancada complementar de tuning
sem ROS**: carregar o mesmo URDF e importar `kinematics.py` + lógica fuzzy
diretamente, rodando centenas de episódios por minuto para varredura de
ganhos (k_pos, k_orient, λ) e comparação quantitativa das estratégias de
manobra lateral acima, antes de implementar a escolhida no controlador.

---

## Fase 4 — Validação em Hardware Real

**Pré-requisito:** Fases 2 e 3 concluídas. ✅ (Fase 3 mergeada no PR #20)

| Etapa | Status |
|---|---|
| RealSense T265 no NUC: bringup `realsense2_camera` (udev `8087:0b37` + Myriad VPU) | ⬜ |
| Validar `/t265/odom/sample` real com a câmera montada no EE do braço | ⬜ |
| Conectar braço RV-M2 via rosserial (Arduino) | ⬜ |
| Validar `arm_vel_integrator` em hardware (velocidade → posição serial) | ⬜ |
| Calibração hand-eye: T265 → flange do RV-M2 | ⬜ |
| Validar ambiente py312 + gazebo-ros 2.9.3 com hardware real (pendência do PR #16) | ⬜ |
| Fonte do erro visual da servovisão: fisheye da T265 (ArUco/AprilTag) — única RealSense disponível (T265C, s/n 925122110468); avaliar aquisição de D435/D455 se profundidade for necessária | ⬜ |
| Teste de rastreamento: alvo estático com braço real | ⬜ |
| Teste de rastreamento: alvo em movimento lento (servovisão) | ⬜ |
| Ajuste PID / feedforward para dinâmica real (diferente da simulação) | ⬜ |

---

## Fase 5 — Navegação Autônoma (Pilar 1)

| Etapa | Status |
|---|---|
| Pioneer 3-AT: validar `/cmd_vel` em hardware | ⬜ |
| `robot_localization`: fusão odometria Pioneer + T265 + IMU | ⬜ |
| Hokuyo UST-05LX: IP fixo na rede Ethernet + driver `urg_node` + launch | ⬜ |
| Hokuyo: validar TF real (`laser_hokuyo_link` no top_plate) contra o URDF — scan alinhado no RViz | ⬜ |
| Hokuyo: obstacle layer nos costmaps do `move_base` (desvio de obstáculos nas missões) | ⬜ |
| Hokuyo: zona de parada de segurança por scan (obstáculo < d_min à frente → parada suave via watchdog) | ⬜ |
| `move_base`: planejamento de trajetória com mapa local | ⬜ |
| Teste integrado: ponto A → ponto B autônomo desviando de obstáculos | ⬜ |

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
