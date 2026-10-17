# RELATORIO31 — bateria de poses de partida com o stack de outubro — 2026-10-16

Marco: "quer iniciar com a bateria para a partida?". Mesmas 8 poses do
RELATORIO9 (04 Set), 2 execuções cada, stack atual (limites do Cerbaro,
J2 em 90°, standoff 0,68, arco deslizante, desengata em whole-body,
orientação nas fases engatadas). Roteiro `scratchpad/bateria_poses2.sh`
→ `run_garra.sh` com `RESET_ARGS`/`POSE_CHECK` (`checa_pose.py`), saída
em `docs/baterias/poses_out/<pose>/runN`.

## Resultado: 12/16

| pose | x, y (m), yaw | 04 Set | hoje | falhas de hoje |
|---|---|---|---|---|
| ref | 0, 1,0, 0° | 3/3 | 2/2 | — |
| perto | 0, 1,5, 0° | 3/3 | 2/2 | — |
| longe | 0, 0,5, 0° | 2/3 | 2/2 | — |
| esq | −0,5, 1,0, 0° | 2/3 | 1/2 | run8: libera não fechou (trinco) |
| dir | +0,5, 1,0, 0° | 3/3 | 1/2 | run10: arco2 perdeu o olhal (batente) |
| frente | 0, 1,0, 90° | **0/3** | **2/2** | — |
| costas | 0, 1,0, 180° | 3/3 | 1/2 | run13: libera não fechou (trinco) |
| diag | 0,5, 1,5, −45° | 3/3 | 1/2 | run15: arco2 perdeu o olhal (batente) |

Percepção: zero falhas. O viés de yaw do SEARCH ficou entre 7° e 21° (a
busca oblíqua da PR #53 resolveu o caso "frente", 0/3 em setembro). Nenhuma
run "passou ao lado" do alvo no DEPLOY (ramo da #94 não exercitado).
Lâmina 34,4° em todas as 12 que abriram. Manipulação: 4 falhas, duas causas,
nenhuma delas da pose de partida.

## Falha 1 — trinco do "alcançado" no controlador (esq2, costas1)

Bag de esq2 (`poses_out/esq/run2/return.bag`, 9394–9434 s): na libera as
juntas ficaram CONGELADAS 30 s em [−11,6, 36,2, −10,1, −48,0]°, comando de
base zero, e a base foi arrastada 3,4 cm em direção à parede pelo contato
do dedo com o anel. No rosout, a cada 2 s: `[fuzzy_wb_ctrl] alvo alcançado
— pos=0,2001 m`. O IK do estimador convergiu em 800/800 amostras — não é
estimativa.

Causa: no modo servo_tooltip o controlador marca `_reached` quando a ponta
passa a menos de `tol_pos` (2 mm) do alvo e PARA de atuar; o ramo de volta
da histerese (erro > 1,5 × tol) só existia no modo 6D. A ponta passou a
< 2 mm num transiente (a libera chegou a 7–9 mm e oscilou), o trinco
fechou, o contato puxou a ponta a 15 mm do alvo e a fase estourou o
timeout. Mesmo padrão em costas1 (11268 s). Fix: ramo de volta nos dois
lugares que setam `_reached` (commit 4d4f0ea). O bug é antigo; a orientação
nas fases engatadas (#95) mudou os transientes o bastante para ele aparecer
duas vezes em 16 runs.

## Falha 2 — fim do arco2 além do batente da lâmina (dir2, diag1)

O arco progride normal até ~30° (deriva de eixo ≤ 2 mm) e, no fim, a ponta
passa de 35° enquanto a lâmina para em **34,4° em todas as runs** — é o
batente dela. Com o alvo a 35°, o whole-body empurra contra o batente até
o erro de profundidade cair abaixo de 10 mm; nas 12 runs boas isso fechou
a 6,7–9,9 mm, no limite; nas duas ruins o dedo escapou do anel antes
(salto de −52,7 e −32,6 mm pelo eixo em um ciclo, "PERDEU O OLHAL"). Fix:
fim do arco2 em 33° (y 108,9, z −57,3 mm), alcançável antes do batente;
com o desengata em whole-body a lâmina não recua ao sair (commit 53a04f6).

## Confirmação (esq, dir, costas, diag × 2, com os dois fixes)

`poses_out_fix/<pose>/runN`, stack reiniciado com a histerese; fim do arco2 em 33°.

| pose | run1 | run2 |
|---|---|---|
| esq | **ABORT** — arco1: deriva de eixo −19,2 mm, "NÃO SOLTA", lâmina 11° no waypoint de 15°; o dedo perdeu o anel ao puxar | OK |
| dir | OK | OK |
| costas | OK | OK |
| diag | OK | OK |

**7/8.** O trinco não voltou a aparecer ("alcançado" agora solta quando o
erro passa de 1,5 × tol), e nenhuma run perdeu o olhal no fim do arco
(lâmina 34,4° nas sete, o batente; o alvo a 33° fecha pela régua sem
disputar com ele). A falha restante é do ENGATE: a ponta sai do arame
logo na libera/arco1 — o mesmo tipo de evento raro das baterias de
setembro (RELATORIO9 run8) — e não tem correlação com o ângulo do degrau
na captura (2–5,5° tanto nas boas quanto nas ruins), com a postura do
braço (J1 −8…−10, J4 −50…−63 em todas) nem com a altura da captura.

## Leitura

1. Percepção e navegação: 24/24 hoje contra 20/24 em setembro — a pose
   "frente" deixou de derrubar a missão (busca oblíqua) e o DEPLOY chegou
   direto em todas.
2. Dois defeitos antigos do controlador/tarefa ficaram visíveis só porque
   a bateria saiu da pose de referência: o trinco do "alcançado" e o alvo
   do arco além do batente. Corrigidos nesta PR.
3. Engate: 3 perdas em 24 runs (esq2 nas poses, esq1 na confirmação, mais
   as duas de arco que o fim em 33° cobre). Causa não fechada; o próximo
   passo é instrumentar o contato (força no anel, posição do degrau em
   relação ao arame no instante da perda) — a bancada vai dizer se é da
   física do ODE ou do dedo.
