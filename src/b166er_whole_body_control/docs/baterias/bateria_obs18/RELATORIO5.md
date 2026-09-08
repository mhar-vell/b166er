# Critério observável do destrava — baterias 41–45 (curso 13) e 46–50 (curso 18) — 2026-09-03

Item 1 do Marco: "criar critério de destrava observável (soltura é no libera)".
A fase deixa de fechar por altura-alvo (relativa ao olhal estimado) e passa a
fechar quando a PONTA, medida pela T265, desceu ≥ `curso_min_m` desde o
instante em que a captura fechou; eixo e profundidade seguem por tolerância; o
comando em altura vira relativo (captura − curso − 5 mm). Híbrido por fase.

## Curso 13 mm (runs 41–45): 4/5

| run | resultado | destrava | ponta desceu | lingueta ao fim |
|---|---|---|---|---|
| 41 | OK 28,6° | 3,6 s | 13,8 mm | 8,4 mm |
| 42 | OK 29,6° | 19,1 s | 13,5 mm | 7,9 mm |
| 43 | OK 30,1° | 19,8 s | 21,3 mm | 16,7 mm |
| 44 | OK 28,7° | 3,7 s | 14,4 mm | 11,2 mm |
| 45 | ABORTED | timeout (profundidade −9…−10 mm, tol 6) | – | – |

A ponta desce mais do que o anel: perdem-se 3–5 mm (deformação de contato do
ODE e/ou o degrau deslizando no arame curvo). 13 mm de ponta não garantem os
13 mm de lingueta.

## Curso 18 mm (runs 46–50): 5/5

| run | resultado | destrava | ponta desceu | lingueta ao fim |
|---|---|---|---|---|
| 46 | OK 27,9° | 38,2 s | 24,2 mm | 19,0 mm |
| 47 | OK 28,7° | 13,6 s | 18,9 mm | 20,0 mm |
| 48 | OK 30,4° | 4,4 s | 20,3 mm | 16,2 mm |
| 49 | OK 28,7° | 29,1 s | 25,1 mm | 20,0 mm |
| 50 | OK 29,7° | 36,8 s | 24,2 mm | 20,0 mm |

Pela primeira vez em 50 missões a lingueta passa de 12 mm ao fim do destrava
em TODAS as execuções: a fase de destravamento destrava. Custo: o destrava
demora mais (4–38 s), porque em três runs o anel foi ao fim de curso de 20 mm
e a ponta ainda tinha de completar 18 mm medidos (a diferença é a perda de
contato). A run45 caiu no estacionamento em profundidade de sempre, que é o
item 2 (piso da base na direção do resíduo).

## Nota para a bancada

O critério é a mesma medida que a T265 dá no braço real: descida da ponta
desde o contato da captura. O número (18 mm) foi calibrado contra a lingueta
da simulação; na bancada a perda ponta→anel será outra (aço, sem penetração)
e o curso real é ~15 mm — recalibrar lá, com o gatilho audível como gabarito.
