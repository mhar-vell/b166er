# RELATÓRIO 20 — abortos no atravessa: degrau na aba da lingueta; alvo +8 mm e recuo por contato

Data: 2026-09-30, 15:39–17:03, shiroi (Gazebo com GUI), garra fechada,
código da `main` (#78/#79) + branch `fix/atravessa-recuo-aba` (PR #80).

## O aborto

Missão das 15:39 (primeira na `main` com a garra fechada): MISSION_ABORTED
na fase "atravessa", "5 iterações sem fechar", ponta parada a 21,5 mm do
alvo no eixo do furo, J1 em −10,7° com o torque subindo de 3,9 a 13,3 N·m
(bloqueado). Reproduzido às 16:05 com a captura de contatos do Gazebo
(`gz topic -e /gazebo/default/physics/contacts`, filtrada por bloco):

- it0 eixo +37,9 mm (J1 3,2° aquém), it1 sem mover (5,7°), depois
  contato **arame superior do anel (`chave_olhal_link_collision_5`,
  lado da parede) × degrau (`tool_tip_collision_3`)** até 771 N e
  **degrau × batente do laço (`chave_laco_collision_5`)**; J1 saturado
  em 30 N·m.
- Experimento isolado (`exp_j1.py`): longe da parede o J1 obedece exato
  (−9,70° pedindo −9,7°, torque 0).
- Base teleportada à pose do DEPLOY (`exp_j1_parede.py`): J1 para em
  −12,8° e rasteja a 5 N·m; contato contínuo **aba da lingueta
  (`chave_lingueta_collision`) × degrau**, ponto de contato na face da
  aba (x = olhal +18 mm, 14 mm para a parede, 5 mm acima do centro).
  Aba no mundo, relativa ao olhal: x ±18, y +24..+40 (lado da parede),
  z +6..+11 mm.

Por que a ferramenta chega lá: entra 4 a 7 mm alta (alvo +10 ± 5) e 2 a
5 mm para a parede, e a REFINE estima a parede 4 a 17 mm mais funda que a
real (3,004–3,017 contra 3,000). Em 7 de 16 execuções do dia o it0 já
parou em ≈ +20 mm (aberta e fechada); 5 recuperaram na iteração seguinte
ou por uma correção da âncora (≤ 2 mm), 2 abortaram. Não é efeito da
garra fechada.

`reset_sim.py` zera lingueta e lâmina (eu havia dito o contrário: erro
meu de leitura); a lâmina a 6° vista às 16:45 veio dos meus testes.

## Correções (decisão do Marco: 1 e 3)

1. `aproxima_lateral` e `atravessa`: `offset_xyz_m[1] = +0.008` (frame
   da parede, +y = lado do robô). 10 mm a mais até a aba, 2 mm de folga
   no furo (30 mm) para o degrau (10 mm).
2. `_reach_by_iterative_ik`: recuo por contato — erro caiu < 3 mm e
   |faltou J1| cresceu > 0,5° entre iterações ⇒ recua 20 mm pelo eixo e
   5 mm para baixo, alvo 5 mm mais baixo, máx. 2 recuos/fase, sem
   consumir iteração (`~ik_recuo_*`).

## Verificação: 4 missões, garra fechada

| run | resultado | atravessa it0 (eixo/prof/alt mm) | iterações | recuo | lâmina |
|---|---|---|---|---|---|
| 1 | MISSION_OK | +0,4 / +4,3 / +4,4 | 1 | não | 29,1° |
| 2 | MISSION_OK | +1,9 / +3,9 / +4,6 | 1 | não | OK |
| 3 | MISSION_OK | +1,1 / +4,0 / +5,4 | 2 | não | OK |
| 4 | MISSION_OK | +4,6 / +6,2 / +3,8 | 3 | **sim, 1×** | OK |

Na run 4 o it1 ficou preso com altura +5,8/+6,6 mm (fora da tolerância de
5) e o J1 aquém (−0,4 → −1,6°): o recuo disparou, voltou 20 mm, desceu
5 mm, e a fase fechou na iteração seguinte com alt +3,9. Antes da
correção 1 o it0 do atravessa parava em +19 a +38 mm em 7 de 16; agora
+0,4 a +4,6 em 4 de 4.

Quatro execuções não são uma bateria; o próximo passo é repetir a 5×5.
