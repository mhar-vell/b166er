# RELATÓRIO 17 — Teste de fumaça: o stack inteiro rodando no NUC (2026-09-11)

## Pergunta

O computador de bordo do robô (Intel NUC, i5 de 5ª geração, 4 núcleos,
8 GB, HD Graphics 6000, Ubuntu 24.04) carrega o nosso stack completo —
Gazebo, controladores, estimador, guardas e a missão da chave — e com
quanto de folga? Não é um ensaio de missão: o resultado da missão em si a
gente conhece do shiroi. É a prova de que o ambiente do NUC está íntegro
antes de ele comandar o robô real, e uma medida de quanto de CPU os nossos
nós pedem numa máquina fraca.

## O que precisou acontecer antes

1. **O `ros_env` do NUC estava na versão com o bug do Gazebo.** Python 3.11
   com `gazebo-ros` 2.9.2, a combinação em que o `gzserver` aborta (exit
   134) ao carregar o plugin do ROS — o mesmo bug isolado em junho e
   corrigido no shiroi reconstruindo o ambiente em Python 3.12 (PR #16). O
   NUC nunca tinha sido reprovisionado. Rodamos a receita do próprio
   repositório, `setup/nuc_ros_setup.sh` → `setup/ros_env.yml`: Python
   3.12.13, `gazebo-ros` 2.9.3, `realsense2-camera` 2.3.2. Doze minutos.
   Depois `catkin clean` + `catkin build`: 8/8.
2. **Disco.** O NUC tinha 5,8 GB livres e o ambiente novo pede uns 8. O
   Marco apagou uma instalação completa do MATLAB R2018b (23 GB) que morava
   em `~/catkin_ws`; ficaram 25 GB livres.
3. **Scripts portáteis.** `sim_stack.sh`, `bancada.sh`, `grava_missao.sh`,
   `continua.sh` e `repeatability_battery.sh` tinham `/home/marco` fixo.
   Agora deduzem o workspace da própria posição no repositório (PR #72).
4. **rosaria sem `make install`.** O NUC não tem a Aria em `/usr/local`; o
   CMake do rosaria aceita `ARIA=$HOME/b166er/src/AriaCoda` e a
   `libAria.so` já vinha compilada no submódulo. Fica no `.bashrc` do robo.

## Protocolo

Stack headless por SSH (`sim_stack.sh start mode:=gazebo gui:=false
rviz:=false`), preflight, e um roteiro (`nuc_smoke_run.sh`, cópia nesta
pasta) que faz `reset_sim.py`, lança `chave_mission.launch
phase_timeout:=60 tol_pos:=0.020` e amostra CPU (via `vmstat`), RAM e
fator de tempo real (`gz stats`) a cada ~10 s até `resultado: MISSION_*`.

## Execução 1 — abortou no SEARCH: câmera sem imagem

`MISSION_ABORTED` em 4 min de relógio: "SEARCH: tag não encontrada em
90 s". O tópico `/camera/fisheye1/image_raw` existia e não publicava nada.
**O `gzserver` subiu sem `DISPLAY`** (sessão SSH) e a câmera simulada
precisa de um contexto de renderização mesmo sem janela; no shiroi o
headless sempre funcionou porque a sessão gráfica exporta `:0`. O laser
(sensor de raios, sem renderização) funcionava normalmente.

Solução: o NUC tem uma sessão Wayland ativa no seat0 com Xwayland em `:0`;
`DISPLAY=:0 XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.*` exportados
antes do `sim_stack.sh start` bastam. A câmera passou a publicar a 17 Hz.

| | CPU média | CPU máx | RAM máx | RTF mín–máx |
|---|---|---|---|---|
| Exec. 1, sem renderização | 88 % | 93 % | 2,4 GB | 0,64–0,78 |

## Execução 2 — chave ABERTA, RETURN estourou o timeout

Com a câmera renderizando: SEARCH → APPROACH → REFINE → DEPLOY →
MANIPULATE → RETRACT em 9,5 min de relógio. **A chave abriu: lâmina em
31,0°**, destrava com lingueta em 8 mm (estagnação curta, como no shiroi),
soltura observável no arco1 (recuo 51,8 mm, deriva +0,8 mm), saída em
degraus limpa. Depois o **RETURN falhou por timeout de 120 s (de
simulação) em DRIVE**: alinhou, começou a avançar 1,06 m e nunca chegou —
a base terminou a 0,18 m do ponto de partida (odometria: x = −0,09,
y = 1,16; alvo 0,00, 1,00; tolerância 0,06). O ABORT_SAFE fechou a missão
como `MISSION_ABORTED`.

| | CPU média | CPU máx | RAM máx | RTF mín / mediana / máx |
|---|---|---|---|---|
| Exec. 2, com renderização | 94 % | 97 % | 2,7 GB | 0,33 / 0,40 / 0,44 |

Sobre a falha do RETURN, o que dá para afirmar e o que não dá:

- Não foi a guarda de folga frontal: quando ela dispara, a missão avisa
  ("laser reporta obstáculo … parando avanço") e muda de fase. Não há esse
  aviso. O `laser_safety` registrou UMA vez, às 11:21:21, um retorno abaixo
  do piso de auto-filtro (0,07 m) tratado como obstáculo a 0,38 m — um
  evento de menos de 1 s, sem efeito visível.
- A navegação usa só tempo de simulação (`rospy.Time`, `rospy.Rate`), então
  o RTF de 0,40 não muda os prazos. Mas muda a cadência real: o laço de
  drive a 10 Hz de simulação vira 4 Hz de relógio.
- A base andou ~0,9 dos 1,06 m e parou perto do alvo, sem "chegou". Os
  logs não dizem por quê: falta a odometria e o `/cmd_vel` ao longo do
  tempo. **Causa em aberto.** Para fechar: repetir no NUC gravando
  `/pioneer3at/odom`, `/cmd_vel` e `/b166er/base_cap`, e a mesma execução
  no shiroi para comparar. O RETURN não falha no shiroi (RELATÓRIOS 13–16).

## Adendo — a causa do RETURN, encontrada e corrigida

Execução 3 (`run3_bag/`), gravando `/cmd_vel`, `/pioneer3at/odom`,
`/b166er/robot_state`, `/b166er/base_cap` e `/rosout`: `MISSION_OK`, mas
o RETURN levou **89,5 s de simulação para 1,06 m**, contra 7 s no shiroi
(`analisa_return_bag*.py` são os cortes, na ordem em que foram feitos).

1. **Ninguém brigou pela base.** 1791 mensagens em `/cmd_vel` a 20 Hz,
   1790 não-nulas a 0,150 m/s, um único zero. O controlador ficou em
   silêncio (stand-down) e o watchdog não disparou.
2. **As rodas giravam a 0,147 m/s e o chassi avançava 0,012 m/s.** Não
   é patinação: a odometria mostra ω de 0,185 rad/s de mediana — **a base
   orbitava o alvo** a ~0,8 m de raio, com o comando angular saturado em
   0,40 rad/s e o rumo dando uma volta completa a cada 30 s.
3. **A pose que a missão lê é a real.** Estimador × Gazebo: 0,05° em
   rumo, 1 mm em posição, 10 ms de atraso. Mas ela chegava com
   **buracos de 1,5–1,8 s a cada 2–3 s**, e a base passou a **1 mm do
   alvo em t+7,4 s**, dentro da tolerância de 6 cm por 0,8 s, exatamente
   dentro de um buraco (`cmd_w` congelado em 0,046). A segunda passagem,
   a 25 mm em t+36 s, caiu em outro.
4. **Cada buraco é uma solve da IK do estimador que não converge.** Com
   a base andando, `/pioneer/pose` e `/t265/odom/sample` são amostrados
   em instantes diferentes, e o alvo da IK do braço sai inconsistente por
   ~4 mm (0,15 m/s × 30 ms); com tolerância de 3 mm ela nunca fecha.
   Cada solve que não converge gastava as 300 iterações × 11 FKs, e o
   `_solve_ik` ainda retentava com o seed de postura — tudo síncrono,
   antes de publicar o estado, e a pose da base saía junto no atraso.
   No shiroi a mesma coisa custa 0,09 s e passa despercebida.
5. **Depois de passar cega, o avanço não sabia dar meia-volta**: com o
   rumo invertido (176°) mantinha 0,15 m/s e ω no teto — órbita até o
   timeout.

Correção, em três partes pequenas (branch `fix/return-orbita`):

- `kinematics.ik_arm` devolve quando o resíduo não melhora por 20
  iterações (ponto fixo), em vez de gastar as 300. Caso espelhado de
  22 cm: 0,09 s → 0,02 s, mesmo resíduo (vai ao retry como antes).
- `state_estimator._solve_ik` só retenta com o seed de postura quando o
  resíduo é ≥ 0,02 m (`~ik_retry_min_residual`) — o mínimo local para o
  qual o retry foi criado, não os 4 mm que só não fecharam a tolerância.
- `chave_mission._navigate_to`: pose mais velha que 0,4 s
  (`~nav_pose_stale`) → para e espera; erro de rumo > 90° no avanço → para
  e realinha; v cai linearmente de 0,30 m (`~nav_slow_dist`) até 30 %
  (`~nav_v_min_frac`), para a janela de chegada valer 2,7 s e não 0,8.

| RETURN 1,06–1,16 m | dt sim | v líquida | buracos > 0,5 s | resultado |
|---|---|---|---|---|
| NUC antes (run 3) | 89,5 s | 0,012 m/s | ~25 | OK por sorte |
| NUC depois (run 4, `run4_corrigida/`) | 10,7 s | 0,094 m/s | 2 (guarda parou 2×) | `MISSION_OK`, 29,8°, 8,7 min de relógio |
| shiroi antes (linha de base) | 7,0 s | 0,157 m/s | — | OK |
| shiroi depois (`shiroi_corrigida/`) | 8,7 s | 0,126 m/s | 0 (máx 0,07 s) | `MISSION_OK`, 29,5° |

O custo da correção no shiroi é 1,7 s por RETURN, a desaceleração perto
do alvo. Os dois buracos que restam no NUC são solves que ainda
convergem devagar sem estagnar; a guarda de pose velha os cobre, e é ela
que interessa no robô real, onde o estimador roda no NUC.

## O que o teste responde

- **O NUC carrega o stack inteiro**, com o Gazebo e a renderização da
  câmera, a ~94 % de CPU e 2,7 GB de RAM, e a física a 0,40 do tempo real.
  Sem o Gazebo (no robô real), o que sobra — controladores, estimador,
  guardas, localizador de tag e a missão — é uma fração disso; a medida
  precisa vem do ensaio de bancada, mas o teto está longe.
- **A manipulação inteira passou** numa máquina 2–3× mais lenta que o
  shiroi: a lógica de fases não depende do relógio de parede. A única
  fase que falhou, o RETURN, expôs um defeito real do estimador e da
  navegação (adendo acima), que o shiroi mascarava por ser rápido.
- **Simulação no NUC não é o uso normal.** Serve como este teste de
  ambiente. Para desenvolvimento a simulação segue no shiroi; para a
  bancada, o NUC roda o hardware.

## Notas operacionais (para não redescobrir)

- Headless no NUC exige `DISPLAY=:0` e o `XAUTHORITY` do Xwayland da sessão
  gráfica; sem isso a câmera do Gazebo não publica e o SEARCH nunca acha a
  tag. Se um dia o NUC ficar sem sessão gráfica, a alternativa é `Xvfb`
  (não instalado).
- **Auto-kill pelo SSH**: a string do comando remoto vira o argv do shell
  de login no NUC. Um `sim_stack.sh stop` no mesmo comando que um
  `grep gzserver` mata o próprio shell (exit 255). Nomes de alvo nunca
  na mesma string que o `stop`; para conferir, `gz[s]erver`.
- `pgrep -f nome_do_script` dentro do próprio ssh casa com o shell que o
  chamou e diz "ainda rodando" para sempre.
- `/opt/ros/melodic` no NUC é resto de instalação antiga; o ROS de verdade
  é o RoboStack em `~/miniforge3/envs/ros_env`.

## Arquivos

- `nuc_smoke_run.sh` — roteiro das execuções 1 e 2; `nuc_smoke_run2.sh`
  — o mesmo gravando o rosbag (execuções 3 e 4 e a do shiroi).
- `run1/`, `run2/`, `run3_bag/`, `run4_corrigida/`, `shiroi_corrigida/` —
  `run.log` da missão, `amostras.txt` (hora, cpu %, ram MB, rtf) e
  `estado.txt`; os bags (12 MB cada) ficaram nas máquinas
  (`~/b166er_nuc_return*/return.bag` no NUC).
- `analisa_return_bag.py` … `bag6.py` — os cortes do bag, na ordem da
  investigação: quem publica em cmd_vel; velocidade por trecho; atitude
  da base; estimador × Gazebo; trajetória fina; buracos do robot_state.
