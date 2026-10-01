# RELATÓRIO 23 — teto de descida do destrava: aplicado, não exercitado; e a profundidade que o atravessa não perdoa

Data: 2026-09-30, 20:43–21:01, shiroi, garra fechada, `main` 2c7132f +
`curso_teto_m` (branch `fix/destrava-teto-descida`). Cinco missões.

## Fase destrava

### destrava_teto/fechada (5 runs)

| run | resultado | captura prof mm | captura alt mm | alvo mm | descida mm | lingueta mm | estagnou | teto | reassentou | perdeu olhal | duração s | J4 ° | J4 méd N·m | J4 máx | sat % | J2 máx |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| run1 | MISSION_OK | -2.2 | -14.2 | -34.2 | 13.5 | 10.5 | 1 | 0 | 0 | 0 | 3.6 | 78 | 0.87 | 2.24 | 0 | 30.2 |
| run2 | MISSION_ABORTED | — | — | — | — | — | 0 | 0 | 0 | 0 | — | — | — | — | — | — |
| run3 | MISSION_OK | -1.5 | -12.4 | -32.4 | 11.7 | 8.4 | 1 | 0 | 0 | 0 | 3.6 | 77 | 0.95 | 2.61 | 0 | 30.1 |
| run4 | MISSION_OK | -3.7 | -13.6 | -33.6 | 15.2 | 11.7 | 1 | 0 | 0 | 0 | 4.2 | 79 | 0.81 | 1.95 | 0 | 30.1 |
| run5 | MISSION_OK | -3.0 | -12.9 | -32.9 | 14.6 | 10.8 | 1 | 0 | 0 | 0 | 4.1 | 85 | 0.75 | 1.92 | 0 | 29.0 |
| **média [mín; máx]** | 4/5 OK | -2.6 [-3.7; -1.5] | -13.3 [-14.2; -12.4] | -33.3 [-34.2; -32.4] | 13.8 [11.7; 15.2] | 10.3 [8.4; 11.7] | 1 [0; 1] | 0 [0; 0] | 0 [0; 0] | 0 [0; 0] | 3.9 [3.6; 4.2] | 80 [77; 85] | 0.85 [0.75; 0.95] | 2.18 [1.92; 2.61] | 0 [0; 0] | 29.8 [29.0; 30.2] |

## Leitura

- **4/5 MISSION_OK.** Nas quatro que chegaram ao destrava a fase fechou
  por estagnação em 11,7–15,2 mm (lingueta 8,4–11,7), J4 a 0,75–0,95 N·m
  sem saturar, libera em 2,3–5,2 s, soltura com deriva ≤ 0,6 mm. **O teto
  de 22 mm não foi acionado em nenhuma**: o caso que o motivou (captura
  alta, descida de 29,5 mm) não se repetiu nestas cinco. O teto fica
  como proteção, verificada só em sintaxe e por leitura do laço.
- **O aborto (run2) foi no atravessa, não no destrava**, e ensina algo
  novo sobre o recuo. A ponta parou a 39 mm do alvo no eixo; o recuo
  disparou duas vezes (20 mm para trás, 5 mm para baixo) e a ponta
  voltou ao mesmo lugar. No bag o J1 anda livre para trás (−2,5 N·m) e
  trava para a frente em ≈ −10,2° com 10 N·m: contato unilateral. A ponta
  estava em y = 2,887 — **17 mm para o lado da parede** do plano do anel
  (2,870) — com a aresta do degrau a 2 mm da face da aba da lingueta.
- **A variável que separa aborto de sucesso é a profundidade estimada
  da parede na REFINE** (real: 3,000):

  | execução | parede estimada (y) | ponta no atravessa it0 (y) | resultado |
  |---|---|---|---|
  | destrava_teto run1 | 3,005 | 2,868 | OK |
  | destrava_fechada run3 | 3,002 | 2,871 | OK no atravessa |
  | main_fechada (15:39) | 3,009 | 2,879 | ABORT |
  | caca_abort tent1 | 3,017 | 2,879 | ABORT |
  | destrava_teto run2 | **3,019** | **2,887** | ABORT |

  O deslocamento de +8 mm do alvo (PR #80) cobre vieses até ~10 mm; com
  17–19 mm o degrau chega à aba de qualquer jeito, e o recuo reaproxima
  pela mesma profundidade, logo não ajuda. Aumentar o offset não
  resolve: com +12 a aresta do degrau do lado do robô já encosta no arame
  (furo de 30 mm, degrau de 10).

## O que fica

A causa de fundo dos abortos do atravessa é o viés de profundidade da
estimativa da parede pela tag (+2 a +19 mm no dia, sempre para dentro da
parede). Duas saídas, sem aplicar, decisão do orientador:

1. **Profundidade pelo laser na REFINE.** O Hokuyo mede a parede a
   milímetros e já publica `front_clearance`; a tag continua dando
   lateral e yaw. Tentei conferir pelo bag e a conta com `front_clearance`
   deu 3,29 m em dois instantes (base + 0,3189 + folga), 29 cm além da
   parede: ou a folga é de outro setor/referência, ou o offset do laser
   não é o que assumi. Precisa de uma verificação própria antes de usar.
2. **Rejeitar a REFINE com viés grande**: repetir a coleta se a
   profundidade estimada discordar da anterior/do laser por mais de
   10 mm. Barato, mas trata o sintoma.
