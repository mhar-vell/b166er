# Controle do Braço RV-M2 sem Encoders de Junta

**Pacote:** `b166er_whole_body_control`  
**Robô:** b166er — Pioneer 3-AT + Mitsubishi RV-M2  
**Versão:** Fase 2 (Gazebo validado)

---

## 1. O Problema

O braço Mitsubishi RV-M2 no b166er **não possui encoders de junta acessíveis** na
interface atual de controle. O driver `movemaster_control` comunica-se com a
controladora original via protocolo serial proprietário que fornece apenas
comandos de posição em graus — sem retorno de posição atual ou velocidade das
juntas.

Isso elimina a estratégia clássica de controle em **espaço de juntas**:

```
[encoder] → q_medido → controlador PID → torque → junta
```

A solução adotada é controlar o braço em **espaço de tarefa** (task-space),
usando a câmera T265 montada no end-effector como sensor de posição.

---

## 2. O Sensor de Tarefa: Câmera T265 no End-Effector

A câmera Intel RealSense T265 (odometria visual-inercial) está montada no
end-effector do RV-M2 e publica continuamente sua pose no frame do mundo:

```
/t265/odom/sample  →  T_world_EE  (posição + orientação 6-DOF)
```

Isso é equivalente a ter um **encoder cartesiano de 6 DOF** diretamente na
ponta do braço. O controle de posição do end-effector dispensa qualquer
medição de junta individual — **a posição das juntas é implícita na pose do EE**.

### Por que isso funciona

Um braço serial com N juntas tem sua pose de EE determinada exclusivamente
pelos ângulos de junta via Cinemática Direta:

```
T_EE = FK(q₁, q₂, q₃, q₄, q₅)
```

Se `T_EE` é medido diretamente pela T265, podemos fechar a malha de controle
**sem precisar conhecer q** — desde que a lei de controle opere em espaço
cartesiano.

---

## 3. Estimação de Estado via Cinemática Inversa

Embora o controle seja cartesiano, a lei de controle whole-body precisa da
**Jacobiana** `J(q)` para mapear velocidades cartesianas em velocidades de
junta. A Jacobiana depende da configuração atual do braço, que é desconhecida
sem encoders.

Solução: **estimar q a partir da T265** via Cinemática Inversa numérica (IK).

```
T_EE  (medido pela T265)
    ↓
IK numérica (DLS — Damped Least Squares)
    ↓
q_est = (q₁_est, q₂_est, q₃_est, q₄_est, q₅_est)
    ↓
J(q_est)  →  usado pelo controlador
```

### Implementação (`state_estimator.py`)

1. Recebe `T_world_EE` da T265 e `T_world_base` do Pioneer (odometria)
2. Computa `T_arm_EE = T_world_arm_base⁻¹ · T_world_EE`
3. Resolve IK numérica (DLS, 300 iterações máx.): `q_est = IK(T_arm_EE)`
4. Publica `/b166er/robot_state` com `q_arm`, `dq_arm`, pose do EE, etc.

### Seed sincronizado (Gazebo)

Em simulação, o `joint_state_controller` publica os ângulos reais das juntas.
O estimador mantém um buffer temporal `deque(maxlen=50)` de pares
`(timestamp, q_real)` e usa o `q` cujo stamp coincide com o stamp do T265 como
seed do IK — o que faz o IK convergir em **0 iterações** (seed = solução exata).

Em hardware real (sem encoders), o seed é a estimativa anterior `q_prev`,
e o IK converge iterativamente a cada ciclo de 20 Hz.

---

## 4. O Controlador Fuzzy Whole-Body: Peça Central

O `fuzzy_wb_controller.py` é o nó de controle principal. Ele não controla
braço e base separadamente — trata o sistema como um **manipulador de 8 DOF**:

```
q_wb = [x_b, y_b, θ_b,  q₁, q₂, q₃, q₄, q₅]
        ← base Pioneer →  ←— braço RV-M2 —→
```

### 4.1 Lei de Controle

```
q̇_wb = J_wb†(λ) · K_fuzzy · err_EE  +  (I − J_wb† J_wb) · q̇₀
```

| Símbolo | Significado |
|---------|-------------|
| `J_wb` | Jacobiana whole-body (6×8): mapeia ẋ_EE → q̇_wb |
| `J_wb†(λ)` | Pseudoinversa DLS: `Jᵀ(JJᵀ + λ²I)⁻¹` |
| `K_fuzzy` | Ganho diagonal adaptativo `diag(k_pos×I₃, k_orient×I₃)` |
| `err_EE` | Erro cartesiano 6D: `[Δp, Δω]` entre pose atual e alvo |
| `(I − J†J)·q̇₀` | Projeção no espaço nulo para afastar limites de junta |

### 4.2 O Motor Fuzzy Mamdani

O ponto central é o ajuste **automático** de `K_fuzzy` e `λ` em função do
estado do erro — sem ganhos fixos. O sistema Mamdani usa:

**Entradas (fuzzificadas):**
- `err_pos` — norma do erro de posição (m): conjuntos `NEAR / MED / FAR`
- `err_orient` — norma do erro de orientação (rad): conjuntos `NEAR / MED / FAR`
- `delta_err` — derivada temporal de `‖err‖`: conjuntos `IMPROVING / STABLE / WORSENING`

**Saídas (defuzzificadas por centróide):**
- `k_pos ∈ [0.1, 2.0]` — ganho de velocidade linear do EE
- `k_orient ∈ [0.1, 2.0]` — ganho de velocidade angular do EE
- `λ_dls ∈ [0.01, 0.25]` — amortecimento da pseudoinversa (robustez a singularidades)

**Tabela de regras resumida (k_pos):**

| err_pos \ delta_err | IMPROVING | STABLE | WORSENING |
|---------------------|-----------|--------|-----------|
| FAR                 | FAST      | FAST   | MED       |
| MED                 | MED       | MED    | SLOW      |
| NEAR                | SLOW      | SLOW   | SLOW      |

**Por que Fuzzy e não PID:**
- O espaço de trabalho do braço tem regiões de ganho muito diferente (longe
  do alvo vs. próximo, próximo de singularidade vs. longe)
- O Fuzzy adapta `λ` automaticamente perto de singularidades (aumenta
  amortecimento) sem parâmetros separados de detecção
- A derivada `delta_err` fornece comportamento "freante": ao aproximar do alvo
  (`IMPROVING`), o ganho cai antes de atingir a tolerância, evitando overshoot

---

## 5. Como o Braço se Move: Integração de Velocidade

O controlador fuzzy publica **velocidades de junta** em `/b166er/arm_vel_cmd`.
O braço não possui controle de posição com encoder — por isso o driver real
(e a simulação) integra velocidade em posição:

### Hardware (`arm_vel_integrator.py`)
```
q_cmd(t) = q_cmd(t-dt) + q̇_cmd · dt        (integração Euler)
q_cmd → graus → protocolo serial → controladora RV-M2
```

A controladora original do RV-M2 aceita comandos de **posição absoluta em
graus**. O integrador mantém o acumulador interno `q_cmd` e envia a cada ciclo
a posição integrada — efetivamente convertendo o controlador de posição da
controladora em um controlador de velocidade.

### Simulação (`gazebo_arm_bridge.py`)
```
q_sim(t) = q_sim(t-dt) + q̇_cmd · dt
q_sim → Jx_position_controller/command  (ros_control)
```

Mesmo princípio: a simulação usa `position_controllers` do `ros_control`, e o
bridge integra velocidade → posição a 20 Hz.

### Implicação: drift acumulado

Sem encoder de realimentação de junta, o acumulador `q_cmd` pode divergir do
estado real do braço ao longo do tempo (slip mecânico, erros de integração,
perturbações). A **T265 corrige isso implicitamente**: se o EE derivou, o erro
cartesiano `err_EE` aumenta e o controlador gera novas velocidades que trazem
o EE de volta à trajetória — **sem precisar conhecer o q real**.

---

## 6. Diagrama de Blocos Completo

```
        ┌─────────────────────────────────────────────────────────┐
        │                      b166er                             │
        │                                                         │
        │   ┌──────────┐     T_world_EE      ┌───────────────┐  │
        │   │  T265    │────────────────────►│               │  │
        │   │ (EE cam) │                     │ state_        │  │
        │   └──────────┘                     │ estimator     │  │
        │                                    │               │  │
        │   ┌──────────┐     T_world_base    │ IK numérica   │  │
        │   │ Pioneer  │────────────────────►│ DLS (seed     │  │
        │   │  odom    │                     │  = T265 stamp)│  │
        │   └──────────┘                     └──────┬────────┘  │
        │                                           │            │
        │                               /b166er/robot_state      │
        │                           (q_est, dq_est, T_EE, ...)  │
        │                                           │            │
        │                                    ┌──────▼────────┐  │
        │   /b166er/ee_target ──────────────►│               │  │
        │   (pose alvo, frame odom)          │  Fuzzy WB     │  │
        │                                    │  Controller   │  │
        │                                    │               │  │
        │                                    │ Mamdani:      │  │
        │                                    │ k_pos, k_ori  │  │
        │                                    │ λ_dls         │  │
        │                                    │               │  │
        │                                    │ J_wb(q_est)   │  │
        │                                    │ DLS pseudo-   │  │
        │                                    │ inversa       │  │
        │                                    └──┬─────────┬──┘  │
        │                                       │         │      │
        │                          /cmd_vel      │         │      │
        │                      (v, ω Pioneer)   │         │      │
        │                                       │   /b166er/     │
        │   ┌──────────┐                        │   arm_vel_cmd  │
        │   │ Pioneer  │◄───────────────────────┘   (q̇₁..q̇₅)  │
        │   │ base     │                             │            │
        │   └──────────┘                      ┌──────▼────────┐  │
        │                                     │ arm_vel_      │  │
        │                                     │ integrator    │  │
        │                                     │ (hardware) ou │  │
        │                                     │ gazebo_arm_   │  │
        │                                     │ bridge (sim)  │  │
        │                                     └──────┬────────┘  │
        │                                            │            │
        │                                    q_cmd (posição)      │
        │                                            │            │
        │   ┌──────────┐                      ┌──────▼────────┐  │
        │   │  RV-M2   │◄─────────────────────│ Controladora  │  │
        │   │  braço   │   serial / ros_ctrl  │ original RV-M2│  │
        │   └──────────┘                      └───────────────┘  │
        └─────────────────────────────────────────────────────────┘

Malha de realimentação:
  T265 mede T_EE → state_estimator estima q_est → fuzzy calcula q̇
  → integrador acumula q_cmd → braço move → T265 mede nova T_EE  ✓
```

---

## 7. Propriedades e Limitações

### 7.1 O que o sistema GARANTE

- **Convergência cartesiana**: o erro `‖err_EE‖` diminui enquanto o EE estiver
  dentro do workspace e longe de singularidades — a T265 fecha a malha externa.
- **Sem acúmulo de erro cinemático**: drift de `q_cmd` é corrigido automaticamente
  pela realimentação cartesiana.
- **Robustez a singularidades**: `λ_dls` adaptativo aumenta o amortecimento
  automaticamente quando a Jacobiana está próxima de perder posto.
- **Objetivo secundário**: o espaço nulo de `J_wb` é explorado para manter as
  juntas afastadas dos limites, melhorando a manobra do braço.

### 7.2 O que o sistema NÃO garante

- **Trajetória no espaço de juntas**: o caminho percorrido pelas juntas para
  atingir o EE alvo pode variar entre execuções — não há planejamento de junta.
- **Velocidade de junta exata**: o integrador aplica velocidade constante por
  intervalo — sem compensação de gravidade ou dinâmica.
- **Posição absoluta sem referência**: se o sistema for reiniciado com o braço
  em posição desconhecida, a T265 fornece o EE atual mas `q_cmd` começa do zero
  — o braço pode fazer um movimento inicial brusco até a malha fechar.

### 7.3 Singularidades

O RV-M2 tem singularidade cinemática em `q=[0,0,0,0,0]` (todos zerados):
os eixos J2, J3, J4 ficam paralelos e a Jacobiana perde posto. O `λ_dls`
dinâmico atenua os picos de velocidade, mas a convergência é mais lenta
perto dessa configuração. Evitar o zero como posição de repouso (usar
`J2=0.5 rad` no spawn Gazebo).

---

## 8. Workspace Seguro (b166er, frame odom)

Pioneer spawned em `z=0.05 m`; shoulder (J1) em `z≈0.74 m`.

| Parâmetro | Valor | Observação |
|-----------|-------|------------|
| z mínimo (EE) | 0.60 m | acima do chassi do Pioneer |
| z máximo (EE) | ~1.10 m | braço totalmente estendido para cima |
| raio horizontal (EE) | 0.25–0.80 m | medido a partir da base do braço |
| J2 inicial recomendado | 0.5 rad | evita queda no chassi antes dos controladores |

Target de teste validado:
```
position: {x: 0.55, y: 0.10, z: 0.70}  frame_id: odom
```

---

## 9. Referências Internas

| Arquivo | Função |
|---------|--------|
| `nodes/state_estimator.py` | IK numérica, buffer de sync por timestamp |
| `nodes/fuzzy_wb_controller.py` | lei de controle whole-body, publicação cmd_vel/arm_vel_cmd |
| `src/b166er_whole_body_control/fuzzy_gain.py` | motor Mamdani (fuzzificação, regras, defuzz) |
| `src/b166er_whole_body_control/kinematics.py` | FK, Jacobiana whole-body, DLS pseudoinversa |
| `nodes/arm_vel_integrator.py` | integração q̇ → q_cmd → serial RV-M2 (hardware) |
| `nodes/gazebo_arm_bridge.py` | integração q̇ → q_cmd → position_controllers (sim) |
| `nodes/gazebo_sensor_sim.py` | FK sintético → /t265/odom/sample (sim apenas) |
