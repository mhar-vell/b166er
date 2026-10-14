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

### Próximos passos (depois da pausa de 14 Out)

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
