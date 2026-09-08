# Plano de bancada — abertura da chave seccionadora com o b166er real

Escrito em 2026-09-08 a pedido do Marco ("segue com o plano de bancada"),
depois de a simulação fechar: híbrido 20/24 nas poses de partida e 8/8
depois das correções da busca (RELATORIO9), punho dimensionado pela
especificação do RV-M2 (RELATORIO11/12). Este plano ordena os ensaios pelo
**risco que a simulação apontou**, do maior para o menor, e diz para cada
um o que medir, com que tópico, e qual número da simulação ele confirma
ou derruba. Princípio (regra do Marco, 2026-09-02): a simulação é ensaio da
bancada — mesmos tópicos, mesmo sensor, mesma lente; nada que o robô real
não tenha entra no caminho crítico.

## 0. Pré-requisitos — o que bloqueia antes do primeiro ensaio

| item | estado (memória da bancada) | ação | quem |
|---|---|---|---|
| NUC | inacessível (sem mDNS/ZeroTier) desde ago. | rodar tudo do **shiroi** por USB, como em 2026-08-07 | — |
| ModemManager | corrompe o rosserial dos Arduinos | `sudo systemctl stop ModemManager` antes de ligar as placas | **Marco** (sudo interativo) |
| Portas dos Arduinos | `/dev/arduino_1` (J1,J2), `_2` (J3,J4), `_3` (J5,garra) pelo udev; `Arduino_*.py` têm porta fixa e trocada | passar a porta por argumento: `python3 Arduino_2.py /dev/arduino_2` — nunca `/dev/ttyACM*` | — |
| **Firmware dos Arduinos** | gravado o de **teste** (`*_test.ino`): ignora `set_1..set_5`, usa `GoHome` como seletor | **decisão obrigatória**: regravar com o firmware operacional do repo (`src/arduino/`) ou o braço não obedece setpoint nenhum. Em 26/08 o Marco decidiu não regravar — precisa ser revisto para a missão | **Marco** |
| Freios | J2: código 7 (30 s solto); J3: código 9 (5 ciclos de 2 s); o de 30 s do J3 é inalcançável | ensaio E0 verifica antes de qualquer movimento | — |
| T265 | aparece como Movidius no `lsusb` (normal); conferir com `rs-enumerate-devices`; as DUAS fisheyes habilitadas; tópicos em `/t265/fisheye1/*` | conferir namespace antes de medir; o localizador deve apontar para `/t265` | — |
| Hand-eye | extrínseca `t265_link→fisheye1` medida (32 mm em Y); hand-eye câmera→ferramenta ainda pendente (Fase 4) | ensaio E1 mede o resíduo e decide se basta | — |
| Hokuyo | x = 0,3189 m do `base_link` (plataforma alongada) — já no modelo | conferir que o `laser_safety` reporta a parede certo (trena) | — |
| Ground truth | `blade_angle_now()`/`lingueta_now()` são serviços do Gazebo: na bancada devolvem `None` | a missão precisa **não** depender deles para fechar (hoje só loga); sucesso = contato auxiliar da chave ou o operador | — |
| Tag | 132 mm; detecta bem até ~1,2 m, ruidosa até 1,8 m, colapsa a 2,5 m; viés fixo de +5..8 cm | partida a **2,0 m** (a padrão); corrigir o viés por offset no localizador antes de E5 | — |
| Chave real | olhal a 805 mm, 130 mm da parede, furo oval 40×30, tag a 170 mm na mesma altura; **comprimento da lâmina pivô→olhal (200 mm) e forma do terminal fixo ainda são chute** | medir os dois com trena antes de E3 (definem o arco) | **Marco** |

## 1. Ensaios, em ordem de risco

Cada ensaio tem: objetivo, montagem, o que medir e por onde, critério, e o
que a simulação prevê (para saber se estamos no regime dela).

### E0 — Energização, freios e parada

- **Objetivo:** ninguém se machuca e nada quebra antes de o primeiro
  setpoint sair.
- **Montagem:** braço na postura de stow apoiado; base com rodas livres
  do chão OU travada; parada de emergência ao alcance.
- **Medir:** `rosserial` dos 3 Arduinos estável por 5 min (sem `Lost
  sync`); freio J2 solta/trava por código 7; freio J3 cicla por código 9;
  `/b166er/tilt_critical` publica e a trava de inclinação congela o braço
  quando o IMU é inclinado à mão; `laser_safety` lê a parede a ±3 cm da
  trena.
- **Critério:** tudo acima; caso contrário não há E1.

### E1 — Percepção estática com a T265 real (sem mover o robô)

- **Objetivo:** saber com que erro a bancada mede a parede, e se a regra
  do anel oblíquo vale na fisheye real.
- **Montagem:** robô parado a 0,75 / 1,0 / 1,5 / 2,0 m da tag, três
  atitudes por distância: tag centrada (raio < 0,15), no anel
  (0,35–0,5) e na borda (> 0,6). 25 amostras por ponto, com dither
  ±0,05 rad/s como na missão.
- **Medir:** `/b166er/wall_pose` (posição e yaw) contra trena e esquadro;
  `tag_pixel` para o raio; taxa de aceitação do PnP.
- **Critério:** |erro de yaw| ≤ 20° a 2,0 m no anel (a missão sobrevive
  até 20°; acima de 27° abortou 4/24 na simulação); viés de distância
  reproduz os +5..8 cm e é corrigido por offset.
- **Simulação prevê:** centrado 148° (−32°) três vezes, no anel 1–4°.
  Se a fisheye real NÃO mostrar a diferença, a regra do anel fica sem
  explicação — mas continua inofensiva.

### E2 — Punho: o risco número um (libera)

- **Objetivo:** medir a cedência do J4 real no puxão do libera, que é
  onde o modelo trabalhou no limite (satura 48–65 %, cede 12–22° sem
  perder o anel entre 4,2 e 2,9 N·m).
- **Montagem:** base estacionada à mão a 0,62 m do olhal, de frente
  para a tag; braço levado por posturas IK até a **captura** (degrau
  apoiado no arame). Só depois disso, com o Marco ao lado da parada:
  1. **destrava**: comando de −25 mm em altura (whole-body) — o real
     exige ~8 N para baixo (teste_gatilho da simulação);
  2. **libera**: +30 mm para fora segurando embaixo;
  3. **puxa** com as rodas (22 mm) e **arco1**.
- **Medir:** trajetória da ponta pela T265 no frame da parede
  (`/b166er/mission_status`: `e_eixo_mm`, `e_prof_mm`, `e_alt_mm`,
  `descida_mm`, `deriva_eixo_mm`, `soltura`); **ângulo do J4** — sem
  encoder, estimar pela pose da ponta (a T265 está no punho) e, se der,
  com um transferidor/inclinômetro no punho filmado; corrente do
  motor do J4 se a ponte H expuser.
- **Critério de aceitação:** deriva de eixo ≤ 15 mm (guarda); cedência
  do J4 ≤ 22°; `soltura` = 1 no arco1 (recuo ≥ 40 mm). Se o punho ceder
  mais que 22° ou a deriva passar de 15 mm: **redesenho** — puxão só
  com a base e punho travado (o `puxa` já existe; a fase libera vira
  uma fase de rodas).
- **Simulação prevê:** destrava desce 12–14 mm e estagna; libera fecha
  em 3–10 s; arco1 recuo 47–60 mm, deriva ≤ 1 mm; lâmina abre 27–33°.

### E3 — Manipulação completa a partir do standoff (sem navegação)

- **Objetivo:** as nove fases da Tabela do artigo (IV-E) com o robô
  real, base estacionada à mão em (0,62 m; lateral 0), REFINE ligado.
- **Montagem:** como E2, mas a missão entra em `REFINE` → `DEPLOY` →
  `MANIPULATE` (pular SEARCH/APPROACH por parâmetro ou estado inicial).
  Antes: **medir a lâmina e o terminal fixo** (item 0) e atualizar
  `chave_seccionadora_task.yaml` se o raio do arco mudar.
- **Medir:** tudo de E2 mais o fechamento por fase (`fase … alcançada`,
  resíduos por eixo) e a abertura (operador/contato auxiliar).
- **Critério:** 3/3 aberturas; nenhuma fase por timeout; deriva ≤ 15 mm.
- **Simulação prevê:** captura eixo +5..+10 mm, prof/alt dentro de 6/10
  mm; destrava 3–4 s; libera 3–10 s; arcos 0,4 s.
- **Se falhar no atravessa/captura:** é a folga de 5 mm por lado do furo
  oval em Y contra a precisão de estacionamento — voltar a E1 e ao
  offset do viés antes de mexer na fase.

### E4 — Aproximação (sem manipulação)

- **Objetivo:** SEARCH + APPROACH + REFINE de 2,0 m com o braço em
  postura de viagem, medindo o erro de estacionamento.
- **Montagem:** partida padrão (0, 1, 0°): 2 m da parede, chave a 90° à
  esquerda. Depois as partidas de frente (90°) e deslocada (±0,5 m), que
  foram as que falharam antes das correções.
- **Medir:** yaw estimado no SEARCH (`SEARCH: N amostras — parede … yaw`),
  remedida intermediária (amostras), pose final da base por trena vs
  `standoff_base_pose`, tempo; `ALIGN`/`ADVANCE` (`fuzzy_wb_ctrl`
  manobra) — deve haver 1 ALIGN por execução.
- **Critério:** erro de estacionamento ≤ 30 mm em Y (a folga do furo) e
  ≤ 5° em yaw; REFINE fecha a parede em ≤ 2°; recuperação da tag, se
  disparar, reencontra em < 30 s.
- **Simulação prevê:** 8/8 nas poses que falhavam; erro do SEARCH
  0,1–3,4° de frente, até −24° deslocado; t até 50 mm ~20 s de 1,9 m.
- **Teto de velocidade:** começar com **0,15 m/s** na base (metade do
  simulado): a 0,3 m/s com o braço à frente o modelo tombou.

### E5 — Missão completa

- **Objetivo:** `chave_mission.launch` do começo ao fim, N ≥ 5 na pose
  padrão, depois 3 por pose nas partidas de E4.
- **Medir:** resultado, duração, todos os campos de E2–E4, rosbag.
- **Critério:** ≥ 4/5 na pose padrão com a falha explicada (a simulação
  deu 20/20 na manipulação que chegou ao REFINE).
- **Abort:** o `ABORT_SAFE` recua 0,25 m com as rodas e volta à partida;
  com o robô real, confirmar o recuo com a parede atrás livre.

## 2. O que gravar (rosbag) em todo ensaio

    /t265/fisheye1/image_raw  /t265/fisheye1/camera_info  /t265/odom/sample
    /b166er/wall_pose  /b166er/mission_status  /tag_pixel
    /joint_states (ponte)  /estimated_joint_states  /cmd_vel  /odom  /scan
    /imu/data  /b166er/tilt_critical  /rosout

E um vídeo do punho em E2/E3 (a cedência do J4 é o número que o modelo
não garante além de 22°).

## 3. Tabela de referência — o que a simulação diz que deve acontecer

| grandeza | simulação (híbrido, ref) | fonte |
|---|---|---|
| yaw do SEARCH a 2 m, no anel | 0,1–4° (de frente), −6..−14° (padrão) | RELATORIO9 |
| REFINE a 0,88 m | ≤ 2° | RELATORIO9 |
| captura no frame da parede | eixo +5..+10, prof ±3, alt ±5 mm | RELATORIO10 |
| destrava: descida da ponta | 12–14 mm, estagna | RELATORIO6/11 |
| destrava: esforço do J4 | 1,5–2,5 N·m | RELATORIO11/12 |
| libera: cedência do J4 | 12° (4,2 N·m) … 22° (2,9 N·m) | RELATORIO11/12 |
| arco1: recuo desde a captura | 47–60 mm, deriva de eixo ≤ 1 mm | RELATORIO10 |
| lâmina ao fim | 27–33° | todas |
| deriva de eixo que significa "perdeu o olhal" | > 15 mm (furo 40×30, dedo 10) | RELATORIO10 |

## 4. O que a simulação não cobre (e a bancada vai mostrar primeiro)

- atrito real dedo × arame e a força real da mola do gatilho (o modelo
  usou k = 200 N/m e abriu com 8 N);
- a cedência real do J4 (motor + redução + freio) — o modelo só tem o
  limite de esforço;
- deriva da odometria da T265 durante a manipulação (o modelo usa a
  âncora da base contra 2,6 mm/s de deriva do Gazebo, não da T265);
- o firmware: latência e quantização do setpoint pelos Arduinos.

## 5. Ordem e paradas

E0 → E1 → E2 → E3 → E4 → E5. **Parar e voltar para a simulação** se: o
punho ceder > 22° (E2), a deriva de eixo passar de 15 mm (E2/E3), o yaw
do SEARCH passar de 20° no anel (E1), ou o erro de estacionamento passar
de 30 mm em Y (E4). Cada parada tem o redesenho já apontado acima.
