# Tutorial de bancada pelo terminal — abertura da chave com o b166er real

Companheiro do `plano_de_bancada.md` (o *porquê* e os critérios) e do
`scripts/bancada.sh` (o roteiro interativo). Este arquivo é o *como*,
para ler antes de ir ao laboratório e ter à mão durante o ensaio.
Escrito em 2026-09-09 a pedido do Marco ("vc pode criar um tutorial via
terminal dos testes de bancada?") e **reescrito em 2026-10-16** depois
das três semanas em que a simulação passou a ensaiar o braço que existe:
sem encoder, em malha aberta, com os fins de curso nas posições do
Cerbaro, homing no início de toda missão e standoff de 0,68 m.

## O que mudou desde setembro (leia antes de tudo)

1. **O braço não tem encoder funcional em nenhuma junta.** O firmware
   "operacional" de 2022 (`Joints12.ino` etc.) faz PD sobre uma contagem
   de pulsos que não chega — **não serve e não deve ser gravado**. A
   malha fecha no espaço da tarefa pela T265 no efetuador (estimador +
   servo + Fuzzy). As placas só executam velocidade em malha aberta:
   `src/arduino/Joints12_vel`, `Joints34_vel`, `Joints56_vel` (sentido +
   PWM, zona morta, freio, fins de curso, watchdog).
2. **A única referência absoluta de junta é o fim de curso.** Toda
   missão começa com HOMING (cada junta até o seu switch) e HOME
   POSITION (cintura e rolagem em zero, ombro na vertical). As posições
   dos switches são as que o Cerbaro documentou na dissertação (Tab. 2 e
   3): J1 ±100°, J2 −30…+100° com o switch de trás em **106°** (16° depois
   da vertical), J3 −120…0° com o switch em **−123°** (o cotovelo só dobra
   para um lado), J4 ±100°, J5 ±180°. Isso está em
   `config/arm_switches.yaml`, no URDF e na cinemática.
3. **A ordem do homing é ombro → punho (ao switch e de volta a zero) →
   cotovelo → rolagem → cintura.** Qualquer outra ordem dobra o
   antebraço para dentro da base (aconteceu na simulação). Nunca levar o
   J3 ao switch com o J2 baixo.
4. **Standoff de manipulação = 0,68 m do olhal** (era 0,62); pré-engate a
   10 cm da parede. Com os limites reais o punho trabalha a −55…−62° nas
   fases de manipulação, longe do switch.
5. **A base não gira em torno do próprio centro**: gira em torno do CG,
   que com o braço fica ~14 cm à frente, e cada giro parado translada o
   chassi (na simulação, 20 cm por 100°). A navegação já não dá meia-volta
   perto do alvo; na bancada esse deslocamento tem de ser MEDIDO.
6. Fases engatadas (destrava, libera, arcos) em whole-body com a atitude
   do T265 mantida; o arco segue a circunferência da lâmina (raio 200 mm)
   e termina em 33°; o desengate sai pelo eixo do furo sem re-alinhar.

Convenções: **Terminal A** é o do stack; **B1, B2, B3** são um por
Arduino; **C** é o do roteiro (`bancada.sh`); **D** é o da missão quando
for lançada à mão. Pode rodar do shiroi por USB (como em 2026-08-07) ou
do NUC (stack inteiro headless, RELATORIO17). Antes de qualquer comando
ROS:

    source ~/miniforge3/etc/profile.d/conda.sh && conda activate ros_env
    source ~/b166er/devel/setup.bash

O roteiro faz isso sozinho. Ele nunca roda `sudo`, nunca regrava firmware
e nunca comanda movimento sem o seu Enter; tudo o que você responde vai
para `~/bancada/AAAA-MM-DD/bancada.md`, com hora, junto dos rosbags.

Para **ensaiar o roteiro na simulação** antes do laboratório:

    scripts/sim_stack.sh start braco:=malha_aberta
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
3. **`/dev/ttyPioneer` e `/dev/sparton/ahrs8`** existem (RosAria e IMU
   Sparton; o da IMU vem da regra
   `sparton_ahrs8_driver/udev/51-sparton-ahrs8.rules`, o do Pioneer ainda
   não tem regra no repo).
4. **T265 enumerada:** `rs-enumerate-devices` mostra nome, série e
   firmware. No `lsusb` ela aparece como *Movidius* e isso é normal —
   não é bootloader. As duas fisheyes ficam habilitadas (a câmera recusa
   uma só). Tópicos reais em `/t265/...`; a simulação usa `/camera/...`.
5. **Firmware dos Arduinos — o item que decide o ensaio.** Hoje as três
   placas têm o firmware de TESTE (`Joints*_test.ino`): ignora
   `set_1..set_5` e usa `GoHome` como seletor de teste. E0 roda com ele;
   **E0b em diante exige o firmware de VELOCIDADE** (`Joints12_vel`,
   `Joints34_vel`, `Joints56_vel`), compilado na sua IDE e gravado por
   você. Antes de gravar, conferir em cada `.ino`:
   - `DIR_SIGN_x` por junta: +1 se o "CW" do `motorGo()` é o sentido
     positivo da junta no modelo. Confere-se na bancada: comandar +5 °/s
     e ver a estimativa (T265) crescer. Sinal errado = o servo diverge.
   - `PWM_MIN` por junta: o menor PWM que vence o atrito.
   - **Lacuna conhecida:** o firmware de velocidade CORTA o sentido no
     switch, mas ainda **não publica o estado dos fins de curso** em
     `/b166er/arm_limit_switch` (JointState, −1/0/+1 por junta, como o
     firmware emulado da simulação). Sem isso o HOMING não sabe que
     chegou e o estimador não ancora. É uma adição pequena nas três
     placas (ler os pinos `LS_xA/LS_xB` e publicar); decisão sua se vai
     no `.ino` ou numa ponte Python a partir dos pinos.
6. **Medidas** que o roteiro pergunta e que mudam parâmetros:
   - lâmina pivô→olhal (o modelo usa 200 mm) → `arco_raio_m` em
     `config/chave_seccionadora_task.yaml` (arco1 e arco2);
   - ângulo máximo de abertura da lâmina (o modelo para em 34,4°; o fim
     do arco2 está em 33° para não empurrar contra o batente);
   - forma do terminal fixo;
   - massa da base como ela roda, sem o braço (o modelo usa 13 kg a
     0,15 m no Gazebo; o modo hardware já usa 42,8 kg, CG 0,139 m atrás e
     0,139 m acima, medidos a olho em 01 Out) → `base_massa`/`base_cg_x`/
     `base_cg_z` em `launch/b166er_wb.launch`;
   - **quanto o robô translada num giro parado de 90°** (ver E0, item
     "pivô"): não há parâmetro ainda; é o número que decide se a
     navegação precisa compensar.

---

## E0 — Energização, freios, fins de curso e parada — `scripts/bancada.sh E0`

**Montagem:** braço na configuração em que foi deixado (o homing vai
achá-lo), apoiado; base com as rodas fora do chão ou travada; parada de
emergência ao alcance. Chave de segurança com o cabo FIRME (em 06 Out um
cabo solto nela era a causa dos motores mudos e dos reboots do NUC).

**Terminal A** (stack de hardware: RosAria, T265, IMU, Hokuyo, estimador,
servo do braço em malha aberta, controlador, monitor de inclinação,
laser de segurança, painel):

    roslaunch b166er_whole_body_control b166er_wb.launch mode:=hardware braco:=malha_aberta homing:=false

`homing:=false` é o padrão: quem faz o homing é o estado HOMING da
missão, não o servo ao ligar.

**Terminais B1–B3** (uma ponte por placa, com a porta por argumento):

    cd ~/b166er/src/movemaster_control/src
    python3 Arduino_1.py /dev/arduino_1     # B1
    python3 Arduino_2.py /dev/arduino_2     # B2
    python3 Arduino_3.py /dev/arduino_3     # B3

Uma coisa a acertar na bancada, que o repo ainda não resolve:
`movemaster_hardware.launch` (incluído pelo stack) sobe UM rosserial em
`/dev/ttyArduino`; se a porta não existir o nó morre e não faz mal, mas
nunca deixe dois rosserial na mesma placa.

O Hokuyo sobe junto com o stack em modo hardware
(`hardware/hokuyo_hardware.launch`, `urg_node`) e publica em
`/pioneer3at/laser_hokuyo/scan`. O UST-05LX é ethernet com IP fixo
192.168.0.10: a porta cabeada do NUC precisa continuar em 192.168.0.15/16.
Cabo solto = tópico não aparece; `ping 192.168.0.10` diz.

**O que o roteiro mede** (Terminal C):

- taxas de `/joint_states`, `/imu/data`, `/pioneer3at/laser_hokuyo/scan`,
  `/t265/odom/sample`, `/t265/fisheye1/image_raw`, `/b166er/base_cap`;
- `/status_1`, `/status_3`, `/status_5` (uma junta por placa) três vezes
  em 60 s — e você observa os terminais B por 5 min: nenhum
  `Lost sync with device`;
- **freios**, só com o firmware de TESTE (com o de velocidade `GoHome` é
  ignorado — o roteiro pergunta e pula): código **7** na placa 1 solta o
  freio do **J2** por 30 s; código **9** na placa 2 cicla o freio do
  **J3** cinco vezes (2 s solto / 2 s travado). O `Lost sync` durante os
  ciclos do 9 é esperado;
- **fins de curso, à mão, sem energia nos motores.** Para cada junta,
  leve-a aos dois batentes e anote o ângulo em que o switch fecha
  (inclinômetro ou transferidor). O que o modelo espera
  (`config/arm_switches.yaml`, "fecha" = nominal ∓ 1,5°):

  | junta | switch de trás / baixo | switch da frente / cima | observação |
  |---|---|---|---|
  | J1 cintura | −100° (B) | +100° (A) | chapa invertida pelo Cerbaro; **qual lado tem espaço no laboratório?** o homing vai para +100 (esquerda vista de trás); se for o outro, `home_side[0] = −1` |
  | J2 ombro | **+106°** = 16° depois da vertical (foto de 14 Out) | −30° | só faz homing para trás; nunca para baixo |
  | J3 cotovelo | **−123°** (antebraço dobrado para a frente) | 0° (reto) | só dobra para um lado; se dobrar para o outro, o modelo está errado |
  | J4 punho | −100° | +100° | o manual diz ±110; o Cerbaro moveu a chapa |
  | J5 rolagem | −180° | +180° | |

  Se um ângulo diferir de mais de 3°, corrigir `arm_switches.yaml`
  (lower/upper), o URDF e `kinematics.JOINT_LOWER/UPPER` ANTES de qualquer
  homing;
- **pivô do giro**: com a base no chão e o braço em HOME POSITION, marque
  a posição das quatro rodas, comande um giro de 90° parado
  (`rostopic pub /cmd_vel ... angular.z 0.4`) e meça com a trena quanto o
  centro do chassi se deslocou. A simulação deu ~20 cm por 100°; o número
  real vai para o RELATORIO29;
- **trava de inclinação:** incline o IMU à mão além de 26°;
  `/b166er/tilt_critical` vai a True e o braço congela;
- **laser × trena:** `/b166er/front_clearance` contra a trena, ±3 cm.

**Critério:** tudo acima, switches conferidos. Se não, não há E0b.

---

## E0b — Primeiro homing real, devagar — (manual; `bancada.sh E0b` quando existir)

**Pré-condição:** firmware de VELOCIDADE gravado nas três placas, `DIR_SIGN`
conferido em cada junta (comando +5 °/s → estimativa cresce), fins de
curso publicados em `/b166er/arm_limit_switch`.

**Montagem:** braço longe de qualquer coisa; a mão na parada de
emergência durante todo o procedimento. A primeira vez com
`home_v_deg` pela metade (`config/arm_switches.yaml`: [10, 5, 5, 5, 10] →
[5, 3, 3, 3, 5]).

**Como:** com o stack de pé (E0), peça o homing ao servo:

    rostopic pub -1 /b166er/arm_home_cmd std_msgs/Empty "{}"
    rostopic echo /b166er/arm_homed      # True no fim, False se alguma junta falhou

Ordem e o que esperar ver, junta a junta (o painel mostra HOMING em
amarelo até a HOME POSITION):

1. **J2 para trás** até o switch de 106°: braço superior passa da
   vertical e para 16° depois dela.
2. **J4 para baixo** até o switch de −100°, e **volta a zero** (ferramenta
   alinhada com o antebraço). Sem esse retorno a ferramenta aponta para o
   braço superior quando o cotovelo dobrar.
3. **J3 para baixo** até −123°: antebraço dobra para a frente, 17° abaixo
   da horizontal.
4. **J5** meia-volta até +180°.
5. **J1** para a esquerda (vista de trás) até +100°, com o braço já
   recolhido.
6. **HOME POSITION**: J1 e J5 voltam a zero, J2 vem à vertical (90°), J3
   fica no switch, J4 em zero. O servo publica `/b166er/arm_homed`.

**Junta presa:** se a ponta (T265) não se move por 2 s com a junta
comandada, o servo recua 1,5 s e tenta mais uma vez; na segunda falha
marca BLOQUEADA e segue. Isso nunca foi exercitado em contato: a
primeira vez que acontecer, pare e olhe.

**Critérios:** cinco switches em < 120 s; HOME POSITION a olho (ombro
vertical, antebraço à frente, ferramenta no plano do antebraço, cintura
de frente); `/b166er/robot_state.q_arm` ≈ [0, 90, −121,5, 0, 0]° ± 3°.
**Se o J2 fechar o switch longe de 106° ou o J3 dobrar para o outro
lado, pare:** é o modelo, não o robô.

---

## E1 — Percepção estática com a T265 — `scripts/bancada.sh E1`

**Montagem:** robô parado a 0,75 / 1,0 / 1,5 / 2,0 m da tag; para cada
distância, três atitudes: tag centrada (raio < 0,15 do quadro), no anel
(0,35–0,5) e na borda (> 0,6). O roteiro pede trena e esquadro em cada
ponto e coleta 25 amostras de `/b166er/wall_pose` e `/b166er/tag_pixel`,
imprimindo distância média ± desvio, viés contra a trena, yaw médio ±
desvio e erro contra o esquadro. Tudo vai para um rosbag `E1_*.bag`.

**Critério:** |erro de yaw| ≤ 20° a 2,0 m no anel; o viés de distância
reproduz os +5..8 cm e vira offset no localizador antes de E5. A busca
oblíqua (PR #53) já cobre o caso "tag centrada" na missão: na bateria de
poses de 16 Out a partida de frente fechou 2/2 (era 0/3 em setembro).

---

## E2 — Punho no libera (o risco número um) — `scripts/bancada.sh E2`

**Por quê:** o modelo saturou o punho em 48–65 % do tempo do libera e
cedeu 12–22° entre 4,2 e 2,9 N·m sem perder o anel. É o número que a
bancada precisa medir primeiro. Com os limites reais e o standoff de
0,68 m o punho trabalha a −55…−62° na captura — 40° longe do switch —,
então o que resta medir é a cedência sob carga, não o batente.

**Montagem:** firmware de velocidade; homing feito (E0b); lâmina e
terminal medidos; base estacionada à mão a **0,88 m do olhal** (a
`deploy_distance`, onde o APPROACH pararia), de frente para a tag. Não a
0,68 m: essa é a pose de manipulação, aonde o DEPLOY leva a base sozinho.

**Yaw da base importa mais que a distância.** Na missão normal o
APPROACH alinha a base à parede (REFINE ≤ 2°); começando em REFINE, o
alinhamento é o seu esquadro. Com a base 3° torta, na simulação, a
ferramenta escorregou 32 mm ao longo do eixo da chave no arco1 e a
guarda de 15 mm abortou. Estacione com o esquadro, dentro de 2°.

**Como:** a missão começa em `REFINE` e anda fase a fase:

    roslaunch b166er_whole_body_control chave_mission.launch \
        estado_inicial:=REFINE pausa_por_fase:=true      # o roteiro faz isso

    scripts/continua.sh        # em outro terminal, libera cada fase

Ao começar adiante, a missão faz sozinha o que HOMING, HOME_POSITION e
SEARCH fariam: registra a pose de partida (para o RETURN/ABORT), põe o
braço na postura de busca (a T265 está no punho e em home não vê a
tag), espera a tag aparecer por até 15 s e amostra a parede parada. Se a
tag não aparecer, aborta antes de mover qualquer coisa: estacione de
frente para ela. **Atenção:** começar em REFINE pula o homing — só faça
isso com o homing já feito nesta energização (E0b), senão as juntas
estimadas não têm referência.

Sequência: REFINE → DEPLOY → orienta → aproxima_lateral → atravessa →
captura → **destrava** (desce até estagnar, 13–18 mm; o real pede ~8 N)
→ **libera** (+30 mm para fora) → **arco1** (whole-body pela
circunferência, base incluída) → **arco2** (até 33°) → desengata. Pare
com Ctrl-C depois do arco1 se só quiser o punho. Filme o punho.

**O que o roteiro extrai do log:** `captura: ponta registrada`, o
`SOLTURA observável` do arco1 (recuo e deriva de eixo), fases alcançadas
ou sem fechar, abortos, `saturada(s)` (junta no switch durante a tarefa).
Ele pergunta a cedência do J4 (vídeo ou inclinômetro) e se a chave abriu.

**Critérios:** deriva de eixo ≤ 15 mm; cedência ≤ 22°; `soltura` no
arco1 (recuo ≥ 40 mm). **Se falhar:** redesenho — o puxão passa a ser só
com a base e o punho travado.

---

## E3 — Manipulação completa a partir do standoff — `scripts/bancada.sh E3 --n 3`

Igual a E2, sem parar: as nove fases até a abertura, três vezes.
**Critério:** 3/3 aberturas, nenhuma fase por timeout, deriva ≤ 15 mm.
**Se falhar no atravessa/captura:** é a folga de 5 mm por lado do furo
oval contra a precisão de estacionamento — voltar a E1 e ao offset do
viés antes de mexer na fase. **Se perder o anel na libera/arco1** (a ponta
sai do arame): é o evento raro que a simulação mostrou em 1 de 8 nas
poses laterais, sem causa fechada; filmar o degrau no instante da perda.

---

## E4 — Aproximação sem manipulação — `scripts/bancada.sh E4`

**Montagem:** partida padrão (2,0 m da parede, chave a 90° à esquerda),
braço em HOME POSITION (a missão faz HOMING → HOME_POSITION → SEARCH);
depois as partidas de frente (yaw 90°), deslocadas (±0,5 m), de costas e
diagonal — as 8 poses do RELATORIO9/31.

**Como:** a missão termina depois do REFINE:

    roslaunch b166er_whole_body_control chave_mission.launch estado_final:=REFINE

**Teto:** 0,15 m/s na base (`nav_linear_vel`) e o teto por postura em
`/b166er/base_cap`. **Chegada:** perto do alvo o giro é limitado e passar
ao lado dentro de 12 cm vale como chegada (sem meia-volta — cada giro
parado custa ~20 cm de translação). **O que o roteiro extrai:** as linhas
do SEARCH (`N amostras — parede … yaw`), a remedida, o REFINE, as trocas
ALIGN/ADVANCE, e qualquer "passou ao lado". Ele pergunta o erro de
estacionamento pela trena e o yaw pelo esquadro.

**Critérios:** ≤ 30 mm em Y (a folga do furo) e ≤ 5° em yaw; REFINE ≤ 2°;
recuperação da tag, se disparar, em < 30 s.

---

## E5 — Missão completa — `scripts/bancada.sh E5 --n 5`

`chave_mission.launch` do começo ao fim (HOMING → … → RETURN em HOME),
cinco vezes na pose padrão, depois duas por pose de E4. O `ABORT_SAFE`
recua 0,25 m com as rodas e volta à partida: **parede atrás livre**.
**Critério:** ≥ 4/5 na pose padrão com a falha explicada; a simulação
fechou 12/16 nas 8 poses (RELATORIO31) e 5/5 na padrão.

---

## O que fica gravado em todo ensaio

Rosbag (o roteiro grava e fecha sozinho, em lz4, com a imagem
comprimida — a crua deu 2 a 3 GB por missão no ensaio da simulação):

    /t265/fisheye1/image_raw/compressed  /t265/fisheye1/camera_info  /t265/odom/sample
    /b166er/wall_pose  /b166er/mission_status  /b166er/tag_pixel  /b166er/base_cap
    /joint_states  /b166er/robot_state  /setpoints  /status_1 /status_3 /status_5
    /b166er/arm_limit_switch  /b166er/arm_homed  /cmd_vel  /pioneer/pose
    /pioneer3at/laser_hokuyo/scan  /imu/data  /b166er/tilt  /b166er/tilt_critical
    /b166er/front_clearance  /rosout

Mais o vídeo do punho em E2/E3 (é o número que o modelo não garante
além de 22°) e do homing em E0b. O registro `bancada.md` é o diário do
ensaio. Se uma run for interrompida à mão, matar o `rosbag record` dela
(`pgrep -af "rosbag recor[d]"`) antes de commitar a pasta.

## Paradas

E0 → E0b → E1 → E2 → E3 → E4 → E5. Parar e voltar para a simulação se: um
switch fechar longe do ângulo do modelo (E0), o homing não fechar os
cinco switches ou a HOME POSITION sair de ±3° (E0b), o punho ceder > 22°
(E2), a deriva de eixo passar de 15 mm (E2/E3), o yaw do SEARCH passar de
20° no anel (E1), ou o erro de estacionamento passar de 30 mm em Y (E4).
Cada parada tem o redesenho no plano.
