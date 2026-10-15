# RELATORIO27 — limites e fins de curso do RV-M2 retrofitado (Cerbaro) no modelo — 2026-10-14

## Por que

O URDF e a cinemática usavam os cursos do manual BFP-A5296 repartidos ao
meio (J1 ±150°, J2 ±65°, J3 ±60°, J4 ±110°, J5 ±180°). Em 09 Out ficou
registrado que o zero do J2 do modelo não batia com o robô: no modelo 0° é o
braço superior na horizontal à frente e 90° a vertical, e o switch de trás
real fica DEPOIS da vertical. O Marco não tinha como medir o ângulo na
bancada (foto de 14 Out: junta a 90° do chão, "alguns graus" até o switch).

A resposta estava na dissertação do Cerbaro (`Dissertacao_Jonathan_Cerbaro_
Oficial (2).pdf`, cap. 3): ele instalou/ajustou os fins de curso e documentou.

| fonte | J1 | J2 | J3 | J4 | J5 |
|---|---|---|---|---|---|
| Tabela 2 (p. 42) — limites de trabalho | ±100° | −30 … +100° | −120 … 0° | ±100° | ±180° |
| Tabela 3 (p. 47) — "configuração home" = onde o homing acha o switch | 0 | **106°** | **−123°** | 0 | 0 |
| modelo até 13 Out (manual ÷ 2) | ±150° | ±65° | ±60° | ±110° | ±180° |

A convenção de zero é a mesma do nosso modelo (DH da Tab. 5: J2 = 0
horizontal à frente, 90 vertical) → o switch de trás do J2 fecha 16° depois
da vertical, como na foto. O cotovelo só dobra para um lado (J3 ≤ 0): o
"ramo do cotovelo" que o estimador tratava (RELATORIO25) **não existe no
robô real**. Chapa do switch do J1 invertida e chapa do J4 movida (p. 42).
Decisão do Marco: "considerar os switches nas posições originais que
Cerbaro adotou" → "pode seguir com a correção do modelo".

## O que mudou (branch `feat/limites-cerbaro`, sobre a ponta da PR #91)

| onde | antes | agora |
|---|---|---|
| URDF `movemaster.urdf.xacro` (batentes) | ±150 / ±65 / ±60 / ±110 / ±180 | ±100 / −31,5…106 / −123…1,5 / ±100 / ±180 (= switches) |
| `kinematics.JOINT_LOWER/UPPER` (trabalho da IK) | = URDF | onde o switch FECHA: ±98,5 / −30…104,5 / −121,5…0 / ±98,5 / ±178,5 |
| `j2_min_deg` / `j2_max_deg` (launches) | −65 / 65 | −30 / 104,5 |
| `arm_switches.yaml` lower/upper | ±150 / ±65 / ±60 / ±110 / ±180 | [−100, −31,5, −123, −100, −180] / [100, 106, 1,5, 100, 180], margem 1,5° |
| `arm_postures.yaml` home | [0, 63,5°, −58,5°, −108,5°, 0] | **[0, 104,5°, −121,5°, 0, 0]** (Tab. 3, J1/J5 em zero) |
| stow_home / travel / reset / fallbacks J4 | −1,8 rad (−103°) | −1,70 rad (−97,4°) |
| `state_estimator._HOME_Q` | [0, −0,5, +0,8, 0, 0] (J3 > 0 inválido) | [0, 0,5, −0,8, −0,5, 0] |
| `state_estimator ~ramo_sinal` | True | **False** (não há espelho) |
| homing `home_ordem` | [4, 3, 2, 5, 1] | **[2, 4, 3, 5, 1]** + `home_park_deg` J4 → 0 após o switch |

Regra única: a IK pode ir até onde o firmware corta o sentido, nunca além;
a postura home (J2/J3 nos switches) fica dentro da faixa. O Cerbaro apertava
J2/J3 por software a 100/−120 — cabe em `j2_max_deg` se for preciso.

## Alcance da tarefa com os limites reais

`scratchpad/scan_standoff.py`, `scan_j4.py`, `fk_check.py` (olhal a 0,805 m,
standoff 0,62 m, poses da tarefa de `chave_seccionadora_task.yaml`):

- Sem semente, a IK cai num ramo PINADO: J3 = 0 com J4 +96° (standoff 0,62)
  ou J4 = −100° (standoffs mais curtos) — resíduo 0–2 mm mas sem folga.
- Com a semente do `deploy` (como a missão faz, semente única = continuidade):
  captura em q = [2,7, 50,4, −12,0, −88,7, −2,1]°, resíduo 0,1 mm, degrau
  1,7°, **10° de folga no J4**; atravessa e destrava idem.
- Com os limites ANTIGOS a IK usava q = [2,7, −35,9, +31,1, 75,3, 2,5]°:
  cotovelo dobrado para o lado que o robô real não tem.
- Posturas (FK): home ponta (0,369, 0, 0,637), stow 0,512 m à frente, search,
  deploy, ACORDA — todas dentro da faixa, nenhum ponto abaixo da base do braço.

## O choque no homing (run diagnóstica)

Primeira missão com os switches novos (`garra/limites_cerbaro/diag_ordem_antiga/run1`,
MISSION_OK, lâmina aberta) — o Marco viu: "o movimento da J3 chocou com o
manipulador … acho que foi por causa da posição da J4 também". Na ordem
antiga (J4 → J3 → J2) o J3 ia a −121,5° com o J2 ainda em 20° (ACORDA) e o
J4 em −98,5°: antebraço para dentro da base, ferramenta apontada para o
braço superior. Varredura por FK da sequência (`scratchpad/homing_seq.py`,
partidas ACORDA/stow/search/deploy/J4 = +98°):

| ordem | ponto mais baixo da sequência (acima da base do braço) |
|---|---|
| 4-3-2-5-1 (antiga) | 0,21 m (ACORDA), 0,25 (deploy), 0,33 (stow) — dentro do corpo |
| 4-2-3-5-1, J4 volta a 0 | 0,38 m (ACORDA: J4 no switch aponta a ferramenta para baixo à frente) |
| **2-4-3-5-1, J4 volta a 0** | **0,44–0,47 m** em todas as partidas (o mínimo é a própria postura de partida) |

Com o J2 já em 104,5° o antebraço está em pé, o punho gira no alto, volta a
zero (ferramenta alinhada ao antebraço, como na home do Cerbaro) e só então
o J3 dobra, levando o antebraço à frente, 17° abaixo da horizontal. O servo
ganhou a fase `estaciona` (anda pela estimativa recém-ancorada no switch).
Mesma ordem vale para a bancada: **nunca dobrar o J3 até o switch com o J2
baixo**.

## Bateria (garra fechada, âncora ligada) — INTERROMPIDA na run1

`garra/limites_cerbaro/fechada/run1`: homing na ordem nova OK (J2 16 s, J4
8 s + 20 s de volta a zero, J3 20 s, J5 14 s, J1 13 s), HOME_POSITION, SEARCH,
APPROACH, REFINE, DEPLOY e as fases orienta / aproxima_lateral / atravessa /
captura fechadas em 1 iteração — e **ABORT no destrava**: whole-body não
fechou em 40 s, melhor 11,9 mm. A descida aconteceu (alt −3 mm do alvo); o
que não fechou foi o **eixo: +11,5 mm** constantes, acima da régua de 10 mm.

O que a IK pediu por fase mostra o motivo — o braço trabalha ENCOSTADO nos
limites reais:

| fase | IK pediu (J1..J5, °) | encostado em |
|---|---|---|
| DEPLOY (pré-posição) | [−10,9, 76,6, −33,2, **−98,5**, 9,0], resíduo 9,5 mm | J4 no switch |
| orienta | [−20,8, 62,5, −14,8, **−98,5**, 13,0] | J4 no switch |
| aproxima_lateral | [−16,8, 43,7, **0,0**, −79,5, 7,9] | J3 no switch (cotovelo reto) |
| atravessa | [−9,9, 45,4, **−0,1**, −83,6, 4,0] | J3 |
| captura | [−9,8, 43,3, **0,0**, −83,7, 4,1] | J3 |

O servo registrou "postura BLOQUEADA" a 3–5 mm em todas essas fases (a IK
pede J3 > 0 ou J4 < −98,5, o firmware emulado corta o sentido no switch e o
resto das juntas não compensa). No destrava, em whole-body, a pseudo-inversa
distribui velocidade também para o J3, que não anda: o movimento realizado
difere do planejado e a ponta deriva pelo eixo do furo em vez de só descer.
Com os limites antigos nada disso aparecia porque a IK dobrava o cotovelo
para +31° — o lado que o robô real não tem.

Offline (`fk_check.py`, base a 0,62 m do olhal, semente = deploy) existe
solução com folga: captura em [2,7, 50,4, −12,0, −88,7, −2,1]° (10° de folga
no J4). A missão não chega nela porque parte do DEPLOY já pinado no J4 e, com
semente única (continuidade, decisão de 2026-09-01 contra saltos de 142°),
escorrega para o ramo J3 = 0.

### Decisão do Marco: J2 na vertical

"Acho um risco fazer o HOME com o J2 no zero, sugiro 90 graus" → "vamos nos
dois em 90 graus". HOME POSITION = [0, 90°, −121,5°, 0, 0] (o homing acha o
switch em 104,5° e o servo traz o J2 14,5° à frente, como J1/J5 voltam a
zero) e o reset acorda o braço com o J2 vertical. Homing do J2 passa a
durar 2 s em vez de 16.

## 15 Out — resolved-rate ciente da saturação e barreira na IK

**`kinematics.resolve_com_saturacao`**: junta no limite de trabalho (ou com o
fim de curso fechado em `/b166er/arm_limit_switch`) e velocidade pedida para
fora perde a coluna da Jacobiana e o sistema é resolvido de novo; as outras
juntas e a base compensam. Usado no `fuzzy_wb_controller` (modo ponta com e
sem base travada, whole-body 8-DOF) e no `arm_joint_servo` (lei de tarefa).
`~sat_margem_deg` = 0,3. Nas baterias aparece como "J3 saturada — resolvido
sem" na HOME POSITION (J3 no próprio switch) e "J4 saturada" na saída do
olhal; em nenhum caso bloqueou uma fase.

**Barreira na `ik_tooltip_com_degrau`** (Chan & Dubey projetada no espaço nulo
das linhas de posição, `IK_PESO_LIM` 2,5e-4, passo ≤ 0,5°/it). Primeira versão
agia em todo o curso e **derrubou a run1 da bateria v2 na atravessa**: a
barreira gastou o grau de liberdade livre da tarefa — a rotação da haste em
torno do eixo do furo, que é o J4 — e inclinou a haste 12° para dentro da
parede (J4 −83 → −71°); a ponta encostava a 10,5 mm no eixo e o recuo por
contato não resolveu. Com **zona morta de 6°** (`IK_LIM_ZONA`, rampa linear) a
barreira só tira a junta do switch por alguns graus e não escolhe postura:
offline, do deploy a captura fica em J3 −10°/J4 −90° (era −12/−89) e a
solução pinada em J3 = 0 vai só a −3,3°.

### Bateria v3 (garra fechada, âncora ligada, J2 em 90°, saturação + barreira com zona morta)

| run | homing | fases da manipulação | lâmina | resultado |
|---|---|---|---|---|
| 1 | OK (J2 2,1 s, J4 8,2 + 19,9 s, J3 20,3 s) | todas em 1 iteração | 32,8° | **OK** |
| 2 | OK | 1 iteração | 30,6° | **OK** |
| 3 | OK | 1 iteração | 32,1° | **OK** |
| 4 | OK | 1 iteração; destrava fechou por estagnação a 17,7 mm, eixo +8,6 | 30,7° | **OK** |
| 5 | OK | 1 iteração até a captura; **destrava estagnou a 16 mm com eixo +10,8** (régua 10) | — | ABORT |

IK por fase na v3 (run1): orienta [−19,5, 59,3, −12,5, −98,5, 12,0]°,
aproxima_lateral [−16,2, 48,9, −4,7, −84,1, 7,7]°, atravessa [−9,0, 50,7,
−5,6, −86,9, 3,1]° — J3 fora do switch, J4 com 11° de folga; só a orienta
ainda encosta o J4 (o DEPLOY pré-posiciona em −98,5 e a orienta é a fase
mais longe do olhal).

**A run5 não é o braço: é o alvo.** A captura deixa a ponta 8 mm para dentro
do furo de propósito (offset −8, tolerância ±8) e o destrava pedia eixo = 0.
O erro de eixo nascia em 0–16 mm por construção, a ponta presa no olhal não
o corrige (o whole-body não move a haste pelo furo), e a régua de 10 mm
decidia a missão no cara ou coroa: run4 8,6 mm (passou), run5 10,8 (caiu),
09 Out 9,0–9,3 (passaram), run1 de 14 Out 11,5 (caiu). Fix (commit d368ec4):
o alvo em eixo do destrava passa a ser onde a captura registrou a ponta,
como já era em altura; a régua mede deriva desde a captura, que era a
intenção ("empurrar só com a haste dentro do furo").

### Bateria v4 (idem + eixo do destrava relativo à captura)

| run | homing | destrava | lâmina | resultado |
|---|---|---|---|---|
| 1 | OK | estagnou a 16,3 mm | 34,2° | **OK** |
| 2 | OK | estagnou a 14,9 mm | 30,8° | **OK** |
| 3 | OK | 2915 [mission] fase "destrava" alcançada por whole-body em 0.8s (0.0011 m) — no frame da parede eixo=+0.1 prof=+0.7 alt=+0.8 mm (tolerân | 34,4° | **OK** |
| 4 | OK | teto de descida a 22,3 mm, eixo −0,1 mm desde a captura | 34,4° | **OK** |
| 5 | OK | estagnou a 16,5 mm | 34,4° | **OK** |

**5/5**, 00:40–01:12, ~6,5 min por missão, zero timeouts, sem reiniciar o stack
entre a v3 e a v4 (só a missão é relançada). Eixo herdado da captura entre
−6,2 e −8,1 mm. Logs em `garra/limites_cerbaro_v4/fechada/run1..5`.

## Conclusão (15 Out, 01:15)

O modelo tem os limites do robô que existe e a missão fecha 5/5 com eles —
mas só depois de três coisas que os limites antigos escondiam:

1. **Ordem do homing**: ombro para trás, punho ao switch e de volta a zero,
   cotovelo por último. Com a ordem antiga o antebraço entrava na base.
2. **Saturação explícita** no resolved-rate (junta no switch perde a coluna)
   e **barreira com zona morta** na IK (tira a junta do switch por alguns
   graus, sem escolher postura — a primeira versão inclinou a haste e caiu).
3. **Eixo do destrava relativo à captura**: o erro de 0–16 mm era de
   construção e decidia a missão no cara ou coroa desde 08 Out.

A tarefa continua perto da borda do espaço de trabalho (orienta com J4 em
−98,5°, 11° de folga nas demais fases). Standoff ficou como estava (0,62 m).
Para a bancada: a ordem do homing e a saturação são exatamente o que as
placas fazem (cortam o sentido no switch); o que a simulação ensaia agora é
o que o robô vai fazer.
### Próximos passos (escritos em 14 Out, 18:50 — executados acima)

1. **Jacobiana ciente da saturação** no whole-body e no servo: junta com o
   switch fechado (ou no limite de software) e velocidade pedida para fora →
   coluna zerada, as outras compensam. É o que o robô real exige de qualquer
   jeito (as placas cortam o sentido no switch).
2. **Afastamento de limites na IK da tarefa** (termo fraco no espaço nulo da
   tolerância do degrau), para a solução não encostar em J3 = 0 / J4 = −98,5
   quando há folga disponível.
3. Rever o standoff (0,58 m dá 3° de folga no ramo cotovelo-para-cima,
   `scan_standoff.py`) só se 1 e 2 não bastarem — número calibrado numa pose
   só é suspeito.
4. Decisão do Marco pendente: HOME POSITION e postura de acordar com o J2 em
   90° ("acho um risco fazer o HOME com o J2 no zero").
5. Bateria de 5 com tudo isso.
