# RELATÓRIO 26 — os abortos do braço sem encoder, e o estado HOME

Data: 2026-10-08, 09:20–10:20, shiroi. Branch `fix/abortos-malha-aberta`
(da `main` com a PR #89). Garra fechada, `braco:=malha_aberta`, homing ao
ligar. Pedido do orientador: "investigue os abortos".

## O que havia

Nas nove baterias do RELATORIO25 o braço em malha aberta fechava 3/5 com
duas classes de aborto que pareciam "de tarefa": fases de aproximação que
não fechavam em 5 iterações e a libera que perdia o olhal. Os logs de 31
missões (`garra/malha_aberta_bateria`, `montagem_90_servo*`,
`homing_switches`) foram lidos iteração a iteração
(`scratchpad/abortos.py`):

| fase | fechou | 5 iterações sem fechar |
|---|---|---|
| orienta | 31 | 1 |
| aproxima_lateral | 30 | 1 |
| atravessa | 25 | 5 |
| captura | 24 | 2 |

Última iteração de cada fase, erro no frame da parede (mm, média ± desvio):

| fase | tol eixo/prof/alt | eixo | prof | alt |
|---|---|---|---|---|
| atravessa | 8 / 6 / 5 | −0,3 ± 1,7 | **+3,3 ± 3,0** | **+2,8 ± 2,0** |
| captura | 40 / 6 / 10 | +6,4 ± 7,5 | +3,8 ± 4,9 | +1,8 ± 4,6 |

## Causa 1 — a libera perdia o olhal porque a captura aceitava a haste meio fora do furo

As três missões com "ferramenta PERDEU O OLHAL" (tent3 run5, homing run2,
tent6 run3) tinham a captura registrada em eixo **+7,3 / +8,6 / −6,7** mm;
as 24 que fecharam, em **−6 … −9** (haste encostada no arame, como o
atravessa deixa, cujo alvo é −8). Nas duas primeiras a descida da captura
escorregou o degrau ~15 mm para FORA do furo pelo arame curvo, sobrou
~5 mm de degrau sob o anel, o destrava não moveu a lingueta (**4,1 e
0,0 mm**, esperado ≥ 12) e a libera puxou com o gatilho travado — o anel
saiu do degrau (deriva −54 e −35 mm). A tolerância de eixo da captura era
**40 mm**, herdada de "o degrau tem 30 mm", e aceitava isso.

Correção: `captura.tol_xyz_m` eixo 40 → **8** mm; `destrava` 40 → **10**
(empurrar só com a haste dentro do furo). A terceira (deriva −15,2 mm com
a lingueta solta, 13,4) é o caso-limite da guarda de 15 mm.

## Causa 2 — o atravessa não fechava porque o servo parava do lado de onde vinha

O servo fechava a postura ao ENTRAR em 6 mm da FK do alvo: a ponta parava
do lado por onde chegou — viés de **+3,3 mm em profundidade e +2,8 em
altura** em 30 atravessas — contra tolerâncias de fase de 6 e 5 mm. Cinco
de trinta não fecharam em 5 iterações. A lei de tarefa (resolved-rate)
chega a 1–3 mm; a tolerância de chegada passou de **6 → 3 mm**, com a
zona morta fina do servo mais fina (0,25 °/s, histerese 2×) para a ponta
não estacionar a 5 mm com todas as juntas em repouso.

## Estado HOME (pedido do orientador, 09:40)

"Vamos incluir o ESTADO de posição de home; na vida real, lá no
laboratório, é essa posição em que o robô deverá sair para o search."
O primeiro estado da missão passa a ser **HOME**: registra a pose de
partida e pede o homing pelos fins de curso ao `arm_joint_servo`
(`/b166er/arm_home_cmd` → `/b166er/arm_homed`); o braço FICA nos switches
(`~apos_homing: home`) — a posição física de descanso do laboratório — e o
SEARCH parte dela. Sem homing no stack (modo posicao, ponte antiga) o
estado recolhe em `stow_home` como o STOW_INIT fazia; `STOW_INIT` segue
aceito como nome antigo em `~estado_inicial`. HUD e launch atualizados.

Dois tropeços no caminho, corrigidos: o preparo de ensaio ainda comparava
com `STOW_INIT` e abortava a missão em 30 s; e o servo publicava `False`
em `arm_homed` ao INICIAR o homing, que o HOME lia como falha — o tópico
passa a carregar só o resultado.

## Bateria (`garra/abortos_fix`, 5 missões)

| run | resultado | lâmina | orienta/aprox/atrav/captura (iterações) | destrava (ponta / lingueta) | captura eixo |
|---|---|---|---|---|---|
| 1 | OK | 31,0° | 1 / 1 / 1 / 1 | 16,7 / 13,1 mm | −5,6 |
| 2 | OK | 31,8° | 1 / 1 / 1 / 1 | 18,6 / 9,5 | −4,8 |
| 3 | OK | 31,7° | 1 / 1 / 1 / 1 | 14,1 / 7,3 | −7,4 |
| 4 | OK | 32,3° | 1 / 1 / 1 / 1 | 18,9 / 9,3 | −7,0 |
| 5 | OK | 31,2° | 1 / 1 / 1 / 1 | 16,2 / 11,8 | −6,8 |

**5/5**, a primeira bateria inteira do braço sem encoder — e todas as
fases de aproximação em UMA iteração (antes, 2 a 5). Erro final do
atravessa: prof +3,5 ± 0,4, alt +2,1 ± 0,9 (o viés de aproximação
continua, agora dentro da tolerância); captura eixo +6,3 ± 1,0 (a haste no
arame). Servo: 24 alcançadas (mediana 2 mm, máx 3), 1 timeout, 35
BLOQUEADA — mediana a 4 mm, isto é, fechamentos por estagnação logo acima
da tolerância de 3 mm, que a missão absorve na iteração seguinte.

## Pendências

- Lingueta a 7–9 mm em três runs (esperado ≥ 12) e a chave abriu mesmo
  assim: conferir se o modelo do gatilho ainda exige 13 mm para soltar, ou
  se a libera está completando a soltura (RELATORIO14/16).
- BLOQUEADA a 4 mm é barulho: o detector de estagnação poderia aceitar
  "parou dentro de 1,5× a tolerância" como alcançada.
- O zero do J2 do modelo (RELATORIO25) continua pendente com o robô.
