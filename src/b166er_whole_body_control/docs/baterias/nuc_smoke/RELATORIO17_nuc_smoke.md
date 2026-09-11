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

## O que o teste responde

- **O NUC carrega o stack inteiro**, com o Gazebo e a renderização da
  câmera, a ~94 % de CPU e 2,7 GB de RAM, e a física a 0,40 do tempo real.
  Sem o Gazebo (no robô real), o que sobra — controladores, estimador,
  guardas, localizador de tag e a missão — é uma fração disso; a medida
  precisa vem do ensaio de bancada, mas o teto está longe.
- **A manipulação inteira passou** numa máquina 2–3× mais lenta que o
  shiroi: a lógica de fases não depende do relógio de parede. A única
  fase que falhou é a que menos importa para a bancada e a única cuja
  causa ficou aberta.
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

- `nuc_smoke_run.sh` — roteiro usado.
- `run1/` e `run2/` — `run.log` da missão, `reset.log`, `amostras.txt`
  (hora, cpu %, ram MB, rtf) e `estado.txt` de cada execução.
