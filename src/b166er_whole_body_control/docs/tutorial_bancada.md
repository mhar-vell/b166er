# Tutorial de bancada pelo terminal — abertura da chave com o b166er real

Companheiro do `plano_de_bancada.md` (o *porquê* e os critérios) e do
`scripts/bancada.sh` (o roteiro interativo). Este arquivo é o *como*,
para ler antes de ir ao laboratório e ter à mão durante o ensaio.
Escrito em 2026-09-09 a pedido do Marco ("vc pode criar um tutorial via
terminal dos testes de bancada?").

Convenções: **Terminal A** é o do stack; **B1, B2, B3** são um por
Arduino; **C** é o do roteiro (`bancada.sh`); **D** é o da missão quando
for lançada à mão. Tudo roda do shiroi por USB, como em 2026-08-07 (o
NUC segue inacessível). Antes de qualquer comando ROS:

    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros_env
    source ~/b166er/devel/setup.bash

O roteiro faz isso sozinho. Ele nunca roda `sudo`, nunca regrava firmware
e nunca comanda movimento sem o seu Enter; tudo o que você responde vai
para `~/bancada/AAAA-MM-DD/bancada.md`, com hora, junto dos rosbags.

Para **ensaiar o roteiro na simulação** antes do laboratório:

    scripts/sim_stack.sh start
    scripts/bancada.sh E4 --sim        # ou pre, E0, E1, E2, E3, E5

---

## 0. Pré-requisitos — `scripts/bancada.sh pre`

O que o roteiro confere e o que ele pede a você:

1. **ModemManager parado.** Ele sonda portas seriais novas e corrompe o
   protocolo do rosserial (`Input/output error`, "multiple access on
   port"). Só o seu terminal tem sudo:

       sudo systemctl stop ModemManager

   Reversível com `start`. Fazer ANTES de ligar as placas.
2. **Portas dos Arduinos pelos links do udev**, nunca `/dev/ttyACM*`
   (a numeração muda a cada replug):

   | link | placa | juntas |
   |---|---|---|
   | `/dev/arduino_1` | Joints12 | J1, J2 |
   | `/dev/arduino_2` | Joints34 | J3, J4 |
   | `/dev/arduino_3` | Joints56 | J5, garra |

   Os `Arduino_*.py` do `movemaster_control` têm porta fixa e trocada;
   não editar — passar a porta por argumento (ver E0).
3. **`/dev/ttyPioneer` e `/dev/ttyAHRS`** existem (RosAria e IMU Sparton).
4. **T265 enumerada:** `rs-enumerate-devices` mostra nome, série e
   firmware. No `lsusb` ela aparece como *Movidius* e isso é normal —
   não é bootloader. As duas fisheyes ficam habilitadas (a câmera recusa
   uma só). Tópicos reais em `/t265/...`; a simulação usa `/camera/...`.
5. **Firmware dos Arduinos.** Hoje as três placas têm o firmware de
   TESTE (`Joints*_test.ino`): ignora `set_1..set_5` e usa `GoHome` como
   seletor de teste. E0 e E1 rodam com ele; **E2 em diante exige o
   firmware operacional** (`src/arduino/Joints12/`, `Joints34/`,
   `Joints56/`). Decisão sua.
6. **Medidas** que o roteiro pergunta e que mudam parâmetros:
   - lâmina pivô→olhal (o modelo usa 200 mm) → raio do arco em
     `config/chave_seccionadora_task.yaml`;
   - forma do terminal fixo;
   - massa da base como ela roda, sem o braço (o modelo usa 13 kg a
     0,15 m) → `base_massa`/`base_cg_z` em `launch/b166er_wb.launch`
     (teto por postura, RELATORIO13). Com ~30 kg reais a margem de
     tombamento em busca vai de 0,9 para ~2,3 m/s².

---

## E0 — Energização, freios e parada — `scripts/bancada.sh E0`

**Montagem:** braço em stow apoiado; base com as rodas fora do chão ou
travada; parada de emergência ao alcance.

**Terminal A** (stack de hardware: RosAria, T265, IMU, estimador,
controlador, monitor de inclinação, laser de segurança, painel):

    roslaunch b166er_whole_body_control b166er_wb.launch mode:=hardware

**Terminais B1–B3** (uma ponte por placa, com a porta por argumento):

    cd ~/b166er/src/movemaster_control/src
    python3 Arduino_1.py /dev/arduino_1     # B1
    python3 Arduino_2.py /dev/arduino_2     # B2
    python3 Arduino_3.py /dev/arduino_3     # B3

Duas coisas a acertar na bancada, que o repo ainda não resolve:
`movemaster_hardware.launch` (incluído pelo stack) sobe UM rosserial em
`/dev/ttyArduino`; se a porta não existir o nó morre e não faz mal, mas
nunca deixe dois rosserial na mesma placa. E não há launch do Hokuyo no
repo: o driver do UST-05LX (ethernet, `urg_node`, que também não está
instalado no `ros_env`) sobe à parte e precisa publicar em
`/pioneer3at/laser_hokuyo/scan`, que é o tópico que o `laser_safety` lê
por padrão (ou passe `~scan_topic` a ele). O roteiro confere esse tópico.

**O que o roteiro mede** (Terminal C):

- taxas de `/joint_states`, `/imu/data`, `/pioneer3at/laser_hokuyo/scan`, `/t265/odom/sample`,
  `/t265/fisheye1/image_raw`, `/b166er/base_cap`;
- `/status_1`, `/status_3`, `/status_5` (uma junta por placa) três vezes
  em 60 s — e você observa os terminais B por 5 min: nenhum
  `Lost sync with device`;
- **freios**, só com o firmware de TESTE (com o operacional `GoHome` é
  flag de homing — o roteiro pergunta e pula): código **7** na placa 1
  solta o freio do **J2** por 30 s; código **9** na placa 2 cicla o freio
  do **J3** cinco vezes (2 s solto / 2 s travado). O `Lost sync` durante
  os ciclos do 9 é esperado. O 30 s do J3 (código 50) é inalcançável no
  firmware gravado;
- **trava de inclinação:** incline o IMU à mão além de 26°;
  `/b166er/tilt_critical` vai a True e o braço congela;
- **laser × trena:** `/b166er/front_clearance` contra a trena, ±3 cm.

**Critério:** tudo acima. Se não, não há E1.

---

## E1 — Percepção estática com a T265 — `scripts/bancada.sh E1`

**Montagem:** robô parado a 0,75 / 1,0 / 1,5 / 2,0 m da tag; para cada
distância, três atitudes: tag centrada (raio < 0,15 do quadro), no anel
(0,35–0,5) e na borda (> 0,6). O roteiro pede trena e esquadro em cada
ponto e coleta 25 amostras de `/b166er/wall_pose` e `/b166er/tag_pixel`,
imprimindo distância média ± desvio, viés contra a trena, yaw médio ±
desvio e erro contra o esquadro. Tudo vai para um rosbag `E1_*.bag`.

**Critério:** |erro de yaw| ≤ 20° a 2,0 m no anel; o viés de distância
reproduz os +5..8 cm e vira offset no localizador antes de E5.
**Simulação prevê:** centrado 148° (−32°) três vezes; no anel 1–4°. Se
a fisheye real não mostrar a diferença, a regra do anel fica sem
explicação — e continua inofensiva.

---

## E2 — Punho no libera (o risco número um) — `scripts/bancada.sh E2`

**Por quê:** o modelo saturou o punho em 48–65 % do tempo do libera e
cedeu 12–22° entre 4,2 e 2,9 N·m sem perder o anel. É o número que a
bancada precisa medir primeiro.

**Montagem:** firmware operacional; lâmina e terminal medidos; base
estacionada à mão a **0,88 m do olhal** (a `deploy_distance`, onde o
APPROACH pararia), de frente para a tag; braço em stow; você ao lado da
parada. Não a 0,62 m: essa é a pose de manipulação, aonde o DEPLOY leva
a base sozinho — e a 0,62 m com o braço em busca a ferramenta encosta
na parede (na simulação o J4 ficou preso 22° antes do alvo).

**Como:** a missão começa em `REFINE` e anda fase a fase:

    roslaunch b166er_whole_body_control chave_mission.launch \
        estado_inicial:=REFINE pausa_por_fase:=true      # o roteiro faz isso

    scripts/continua.sh        # em outro terminal, libera cada fase

Ao começar adiante, a missão faz sozinha o que STOW_INIT e SEARCH
fariam: registra a pose de partida (para o RETURN/ABORT), põe o braço na
postura de busca (a T265 está no punho e em stow não vê a tag), espera a
tag aparecer por até 15 s e amostra a parede parada. Se a tag não
aparecer, aborta antes de mover qualquer coisa: estacione de frente
para ela.

Sequência: REFINE → DEPLOY → orienta → aproxima_lateral → atravessa →
captura → **destrava** (−25 mm; o real pede ~8 N) → **libera** (+30 mm
para fora) → **puxa** com as rodas → **arco1**. Pare com Ctrl-C depois do
arco1 se só quiser o punho. Filme o punho.

**O que o roteiro extrai do log:** `captura: ponta registrada`, o
`SOLTURA observável` do arco1 (recuo e deriva de eixo), fases alcançadas
ou sem fechar, abortos. Ele pergunta a cedência do J4 (vídeo ou
inclinômetro) e se a chave abriu.

**Critérios:** deriva de eixo ≤ 15 mm; cedência ≤ 22°; `soltura` no
arco1 (recuo ≥ 40 mm). **Se falhar:** redesenho — o puxão passa a ser só
com a base e o punho travado (a fase `puxa` já existe).

---

## E3 — Manipulação completa a partir do standoff — `scripts/bancada.sh E3 --n 3`

Igual a E2, sem parar: as nove fases até a abertura, três vezes.
**Critério:** 3/3 aberturas, nenhuma fase por timeout, deriva ≤ 15 mm.
**Se falhar no atravessa/captura:** é a folga de 5 mm por lado do furo
oval contra a precisão de estacionamento — voltar a E1 e ao offset do
viés antes de mexer na fase.

---

## E4 — Aproximação sem manipulação — `scripts/bancada.sh E4`

**Montagem:** partida padrão (2,0 m da parede, chave a 90° à esquerda),
braço em viagem; depois as partidas de frente (yaw 90°) e deslocadas
(±0,5 m), que eram as que falhavam antes das correções da busca.

**Como:** a missão termina depois do REFINE:

    roslaunch b166er_whole_body_control chave_mission.launch estado_final:=REFINE

**Teto:** 0,15 m/s na base (`nav_linear_vel`) e o teto por postura em
`/b166er/base_cap`, que em viagem fica cheio (0,30 m/s). **O que o
roteiro extrai:** as linhas do SEARCH (`N amostras — parede … yaw`), a
remedida, o REFINE, as trocas ALIGN/ADVANCE (deve haver 1 por
execução). Ele pergunta o erro de estacionamento pela trena e o yaw pelo
esquadro.

**Critérios:** ≤ 30 mm em Y (a folga do furo) e ≤ 5° em yaw; REFINE ≤ 2°;
recuperação da tag, se disparar, em < 30 s.

---

## E5 — Missão completa — `scripts/bancada.sh E5 --n 5`

`chave_mission.launch` do começo ao fim, cinco vezes na pose padrão,
depois três por pose de E4. O `ABORT_SAFE` recua 0,25 m com as rodas e
volta à partida: **parede atrás livre**. **Critério:** ≥ 4/5 na pose
padrão com a falha explicada.

---

## O que fica gravado em todo ensaio

Rosbag (o roteiro grava e fecha sozinho):

    /t265/fisheye1/image_raw  /t265/fisheye1/camera_info  /t265/odom/sample
    /b166er/wall_pose  /b166er/mission_status  /b166er/tag_pixel  /b166er/base_cap
    /joint_states  /estimated_joint_states  /cmd_vel  /pioneer/pose  /pioneer3at/laser_hokuyo/scan
    /imu/data  /b166er/tilt  /b166er/tilt_critical  /b166er/front_clearance  /rosout

Mais o vídeo do punho em E2/E3 (é o número que o modelo não garante
além de 22°). O registro `bancada.md` é o diário do ensaio.

## Paradas

E0 → E1 → E2 → E3 → E4 → E5. Parar e voltar para a simulação se: o
punho ceder > 22° (E2), a deriva de eixo passar de 15 mm (E2/E3), o yaw
do SEARCH passar de 20° no anel (E1), ou o erro de estacionamento passar
de 30 mm em Y (E4). Cada parada tem o redesenho no plano.
