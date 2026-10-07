# RELATÓRIO 25 — braço SEM encoders na simulação: firmware emulado, servo de postura e dedo remontado

Data: 2026-10-06 a 2026-10-07, shiroi. Branch `feat/braco-malha-aberta`
(da `main`). Garra fechada (`garra_dx:=0.0`), `braco:=malha_aberta`.

## Por quê

O RV-M2 da bancada não tem encoder funcional em nenhuma junta (orientador,
06/10: "NÓS NÃO TEMOS ENCODERS. TODOS ELES ESTÃO INUTILIZADOS"). A
arquitetura do projeto fecha a malha no espaço da tarefa — T265 no
efetuador, estimador por IK e Fuzzy — e as placas só fazem sentido + PWM.
Até aqui a simulação usava os controladores de POSIÇÃO do Gazebo, que
seguram a junta onde se pede por construção: um ensaio que o robô real não
consegue reproduzir. Pedido do orientador: "quero testar isso na
simulação, mas testar o que será o REAL".

## O que entrou na simulação (modo `braco:=malha_aberta`)

- **Firmware emulado** (`arm_openloop_sim.py`): recebe `/setpoints`
  (graus/s por junta, a mesma mensagem que os `Joints*_vel.ino` vão
  receber), aplica zona morta de 1,5 °/s, saturação em 60 °/s, ganho de
  execução por junta e watchdog de 0,5 s, e publica nos controladores de
  VELOCIDADE do `gazebo_ros_control` (`arm_controllers_vel.yaml`). Freio
  e autotravamento dos redutores emulados: junta com comando zero é
  segurada onde parou (lê `/joint_states` só para isso).
- **Servo de postura** (`arm_joint_servo.py`): substitui a
  `gazebo_arm_bridge` e o `arm_vel_integrator` com a mesma interface;
  executa posturas e repassa as velocidades do Fuzzy, tudo em malha
  aberta de junta, julgando "chegou" pela pose MEDIDA pelo T265.
- **Estimador**: dead reckoning dos comandos para escolher o ramo do
  cotovelo (a IK a partir do T265 tem dois ramos com o mesmo punho).
- **Firmware de velocidade** para as três placas (`Joints12_vel.ino`,
  `Joints34_vel.ino`, `Joints56_vel.ino`), não compilado no shiroi (sem
  toolchain); `DIR_SIGN_x` por junta a conferir na bancada.

## Dedo: a peça inteira monta girada +90° (07/10)

Orientador, olhando o Gazebo com o braço recolhido e de trás do robô: "vc
posicionou o dedo errado, ele tem q virar 90 graus para esquerda" — e,
diante do desenho de alternativas (`montagem_90/dedo_90_esquerda_leituras.png`):
"o dedo completo gira, toda a peça". A v3 não muda; o `JTool` do URDF
ganhou yaw de +1,5708 e a cinemática o mesmo giro (`JTOOL_YAW`), conferido
por `checa_garra_dx`. Efeitos: no stow o degrau aponta para a esquerda do
robô; o garfo passou a abraçar a lingueta da castanha no eixo em que ela
anda (`montagem_90/dedo_montagem_90_rviz_stow.png` — antes ficava de lado,
sinal de que a montagem antiga era inconsistente com a garra); na tarefa a
IK compensa no J5 (−11° em vez de +79°). Missão e YAML não mudam.

## Baterias (5 missões cada, garra fechada)

| bateria | servo | OK | lâmina (°) | o que derrubou |
|---|---|---|---|---|
| 1 `garra/malha_aberta_bateria` (06–07/10, montagem antiga) | lei de junta, chegou = ponta < 30 mm | 3/5 | 33,7 · 34,4 · 32,1 | 5 iterações sem fechar no aproxima_lateral (alt +12 mm) e no atravessa (alt +14 mm): as 22 posturas fecharam com a ponta a 7–21 mm do alvo |
| 2 `montagem_90_servo_tent1` (interrompida) | tol_ee 6 mm, piso vetorial | 0/1 | — | dead reckoning integrava comandos abaixo da zona morta (0,5–1,4 °/s em J1/J4): q_dr derivou ~20°/min, ramo oscilando a cada ciclo, posturas por timeout |
| 3 `montagem_90_servo_tent2` | zona morta/saturação no dr, lei por junta com piso, ok-flag | 2/5 | 32,5 · 31,7 | 21 posturas por TIMEOUT: (a) ponta em contato, J1 comandado a 3 °/s não anda, q_dr −19° → +70° em 1 min; (b) cotovelo quase reto (J3 ≈ 0): os dois ramos são vizinhos (J3 ±13°), a estimativa alterna e a lei de junta não reduz o erro da ponta |
| 4 `montagem_90_servo_tent3` | duas etapas: junta longe, resolved-rate na pose medida perto; reancoragem suave; bloqueio 2 s | 3/5 | 25,0 · 34,4 · 33,7 | os dois abortos são do DESTRAVA/LIBERA, não do servo: run1 descida estagnou em 16,7 mm com lingueta 9,6 (< 12) e a libera não fechou; run5 desceu 18,5 mm mas lingueta 4,1 e a libera perdeu o olhal (deriva −54 mm no eixo). 1 TIMEOUT e 35 BLOQUEADA (muitas falsas: ponta a 7–9 mm com todas as juntas em repouso pela zona morta fina) |
| 5 `montagem_90_servo_tent4` | idem + junta que mais ajuda anda no piso; bloqueio 3 s | 2/5 | 34,4 · 32,7 | 30 TIMEOUT, 512 avisos de ramo: com q_dr corrompido (junta na borda da zona morta alternava 0/±piso, o freio engolia os pulsos e q_dr integrava: J1 −25° → −75°) a re-IK caía em soluções NO BATENTE (J3 ±60°, J4 110°) e o estimador alternava entre elas e a verdadeira a cada ciclo — a lei de junta invertia o sinal a cada ciclo e o braço não saía do lugar (orienta a 0,27 m) |
| 6 `montagem_90_servo_tent5` | idem + histerese 3× nas zonas mortas; ramo só troca para solução fora do batente e sem piorar o resíduo | 3/5 (+1 reset falhado) | 30,7 · 34,4 · 34,4 | run4: captura a 46 mm — o braço foi levado ao batente de J3 (−60°) e a estimativa ficou presa no espelho (+60°, que a regra do batente agora se recusava a trocar); o reset da run5 não conseguiu recolher o braço. 16 TIMEOUT, 369 avisos de ramo |
| 7 `montagem_90_servo_tent6` | estimador SEM verdade do Gazebo: semente = estimativa anterior, dead reckoning desligado, ramo pelo sinal de J3 na passagem pelo cotovelo reto | 2/5 (+2 resets falhados) | 27,6 · 31,1 | run3: libera perdeu o olhal (deriva −15,2 mm, no limite da guarda). Depois do aborto a estimativa ficou presa no espelho (sinal de J3 esperado +1 enquanto o contato levou o J3 real a −60°); o servo empurrou J2/J3 ao batente e GIROU o J4 por 681° (batente do ODE cede); os dois resets não recolheram o braço |
| 8 `montagem_90_servo` | idem + FINS DE CURSO emulados na placa (cortam o sentido e publicam /b166er/arm_limit_switch) e usados pelo estimador como referência absoluta | **3/5** | 34,4 · 32,1 · 31,7 | **0 TIMEOUT** (primeira vez), 28 BLOQUEADA (ponta a 6–26 mm, mediana 13: contato real), resets 5/5. Abortos de TAREFA: run3 captura com prof +12,6 mm (tol 6) em 5 iterações; run5 atravessa a 9 mm sem fechar por eixo |
| 9 `homing_switches` | idem + HOMING por switch ao ligar (J4, J3, J2), switches por junta em `arm_switches.yaml`, firmware emulado com saída em POSIÇÃO integrada, integral em J2/J3 | 3/5 | 31,2 · 29,8 · 31,7 | homing 3/3 (22/12/13 s); 1 TIMEOUT, 22 BLOQUEADA. Abortos de tarefa: run2 libera perdeu o olhal (deriva −35 mm), run4 atravessa em 5 iterações (11 mm) |

Posturas do servo: 22/22 "alcançadas" mas grossas (b1); 55 alcançadas /
21 TIMEOUT (b3); 53 / 1 / 35 BLOQUEADA (b4); 38 / 30 / 32 (b5); 36 / 16 /
37 (b6); 36 / 15 / 5 (b7); **59 / 0 / 28 (b8)**. Avisos de ramo do
estimador por dead reckoning: 438 (b3) → 153 (b4) → 512 (b5) → 369 (b6);
trocas pelo sinal de J3: 12 (b7), 16 (b8).

## O que aprendemos sobre o braço sem encoder

1. **A placa engole comando miúdo e o dead reckoning não pode integrá-lo.**
   O estimador agora aplica a zona morta e a saturação da placa (por junta)
   antes de integrar, e o servo nunca emite velocidade abaixo do piso
   (3 °/s): ou a junta descansa (freio) ou anda acima da zona morta.
2. **Junta bloqueada por contato.** PWM aplicado e nada se move é o caso
   normal do atravessa/captura. O dead reckoning é puxado suavemente (τ 2 s)
   para a estimativa enquanto as duas concordam — um salto de ramo (> 20°)
   continua não sendo seguido —, e a postura fecha como BLOQUEADA quando a
   ponta não progride por 3 s, com `ok=False` para o estimador não
   reancorar no alvo.
3. **Cotovelo reto é a pior configuração para o T265.** As posturas da
   tarefa têm J3 ≈ 0 (alcance máximo); ali os dois ramos da IK diferem
   13° em J3 com o mesmo punho e a lei de junta persegue uma estimativa
   que alterna. A lei resolved-rate sobre a pose MEDIDA (J⁺ por DLS,
   linhas angulares pesadas por 0,1 m/rad) não depende do ramo: o braço
   assenta no ramo vizinho (J3 +12..15°) com a ponta a 2–6 mm — e é a ponta
   que a tarefa usa. Longe do alvo (> 4 cm / 20°) a lei de junta é a
   robusta: a lei de tarefa direta do stow ao deploy, com 1,6 rad de erro
   angular, fechou "BLOQUEADA" porque as linhas angulares dominavam a DLS.
4. **O ramo do cotovelo não é observável pela pose do T265 — e a simulação
   escondia isso.** O estimador semeava a IK com a verdade do Gazebo; sem
   ela, seguir só por continuidade prende a estimativa no espelho na
   primeira passagem pelo cotovelo reto (J3 estimado −60° com a verdade em
   +60°). O dead reckoning dos comandos (baterias 2–6) nunca ficou
   confiável: piso, histerese e contato fazem comandado ≠ executado, e a
   re-IK a partir dele trocava ramo certo por errado. O teste pela
   velocidade medida da ponta (J(q_a)·v contra J(q_b)·v) escolheu o
   espelho quando as duas soluções estavam no batente. O que ficou é a
   regra que o robô real sabe: o ramo só muda quando o cotovelo passa por
   zero, e nesse instante o sinal de J3 é o da velocidade comandada em
   J3; fora da zona, estimativa com sinal contrário é resolvida do espelho.
   Teste de 7 posturas: estimativa a ≤ 2,5° da verdade. A regra ainda
   falha quando o cotovelo cruza zero empurrado pelo CONTATO (bateria 7):
   o que resolve isso é o único sensor absoluto de junta que o braço tem,
   os FINS DE CURSO — a placa corta o sentido que encosta no switch e
   publica o estado; o estimador semeia a junta no limite e fixa o ramo.
5. **O que sobra é a tarefa, não o servo.** Com o servo em duas etapas,
   orienta/aproxima/atravessa/captura fecharam em 5/5 runs; os abortos
   foram no destrava/libera (gatilho não solto, deriva de eixo), a classe
   de problema dos RELATORIOS 14 e 16, agora com a captura menos precisa
   (eixo −6..−8 mm, alt −11..−16 mm) do que no braço de posição.

## Homing por fins de curso (pedido do orientador, 07/10 à tarde)

"Vc não acha interessante simular os switchs de cada junta?" — sim: os
switches são a única medida ABSOLUTA de junta do braço, e até aqui só
eram usados quando a junta esbarrava neles. Entrou `config/arm_switches.yaml`
(ângulo dos dois switches por junta, margem em que fecham, lado e ordem do
homing), lido por três nós: o firmware emulado (fecha, corta o sentido,
publica `/b166er/arm_limit_switch`), o estimador (junta no switch ancorada
no ângulo em que ele fecha) e o servo (ao ligar, J4, J3 e J2 vão aos
switches do lado do stow — a cadeia de arfagem, onde o ramo é ambíguo;
J1/J5 são observáveis pelo T265 — e só então o braço recolhe;
`/b166er/arm_home_cmd` refaz, `/b166er/arm_homed` informa). Os ângulos do
yaml são os declarados no manual do RV-M2 (repartidos simetricamente, como
no URDF) — decisão do orientador: não medir na bancada.

Dois defeitos da emulação apareceram no caminho:

1. A velocidade de junta que o Gazebo/ODE reporta sob carga não é
   Δposição/Δt: J2 subia a 1,5 °/s com o controlador de velocidade
   "vendo" 5 °/s (e descia a 8 "vendo" 4). O firmware emulado passou a
   integrar a velocidade executada num setpoint dos controladores de
   POSIÇÃO, com anti-windup por junta (folga [4, 10, 6, 3, 3]°: tem de
   cobrir a queda estática do P puro) — é como o motor real com redutor
   harmônico se comporta. `velocity_controllers` só com `arm_iface:=velocity`.
2. O P puro de J2 cedia 3,5° sob gravidade e o `JointPositionController`
   limita o comando ao URDF: o teto real do J2 era 61,5° e o switch
   superior (63,5°) nunca fechava. Termo integral com `i_clamp` pequeno em
   J2/J3; stow do J2 de 1,13 para 1,10 rad (1,13 ficava dentro da zona do
   switch).

Teste: homing 3/3; sete posturas com a estimativa a ≤ 1,3° da verdade.
Bateria 9: 3/5, mesma classe de abortos de tarefa.

## Onde ficou

O braço sem encoder, na simulação honesta (sem verdade do Gazebo em
nenhum nó de controle), executa a missão de ponta a ponta: 3/5 na última
bateria, zero posturas por timeout, resets 5/5. O que separa 3/5 de 5/5
é a precisão de posicionamento por iteração (~1 cm, contra ~3 mm do braço
de posição) diante das tolerâncias de 6 mm em profundidade do atravessa e
da captura — problema de TAREFA (relaxar/iterar mais, ou medir a
profundidade pelo laser como na REFINE), não de servo.

## Pendências

- Atravessa/captura: 5 iterações não bastam com ~1 cm por iteração; avaliar
  mais iterações ou tolerância de profundidade de 6 → 10 mm (o alvo já tem
  +8 mm de folga à aba, RELATORIO20).
- Placas: publicar os fins de curso (pinos LS) em `/b166er/arm_limit_switch`
  como o firmware emulado faz — é a única referência absoluta de junta
  (ângulos: os do manual, `arm_switches.yaml`, decisão do orientador).
- Avisos de ramo ainda ocorrem (153 por bateria): investigar com log por
  ciclo durante o atravessa/captura.
- Destrava/libera com captura menos precisa: rever `curso_min_m`/reassenta
  (RELATORIO14) para o braço em malha aberta.
- Bancada: compilar os `Joints*_vel.ino` (IDE do orientador), conferir
  `DIR_SIGN_x` comandando +5 °/s e vendo a estimativa crescer; medir
  PWM_MIN por junta. `tutorial_bancada.md` ainda fala em "firmware
  operacional" (PD sobre encoder) — está errado e precisa ser reescrito.
