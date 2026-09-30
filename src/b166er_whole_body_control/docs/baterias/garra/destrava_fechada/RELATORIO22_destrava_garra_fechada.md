# RELATÓRIO 22 — o destrava com a garra fechada desce menos? (bateria de 5 + reanálise de 20)

Data: 2026-09-30, 18:20–18:39, shiroi. Pergunta do orientador depois do
RELATORIO21: a lingueta ficou em 6,7 mm de média com a garra fechada
contra 12,1 com a aberta. É sistemático? Cinco missões completas com a
garra fechada (`garra_dx` 0,0, código da `main` 1d1e246), analisadas só
na fase destrava, mais a reanálise das 20 missões anteriores do dia.

## Fase destrava, por execução

### destrava_fechada/fechada (5 runs)

| run | resultado | captura prof mm | captura alt mm | alvo mm | descida mm | lingueta mm | estagnou | reassentou | perdeu olhal | duração s | J4 ° | J4 méd N·m | J4 máx | sat % | J2 máx |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| run1 | MISSION_OK | -1.5 | -11.8 | -31.8 | 11.6 | 6.2 | 1 | 0 | 0 | 3.8 | 85 | 1.15 | 3.66 | 0 | 28.5 |
| run2 | MISSION_OK | -1.7 | -12.3 | -32.3 | 13.4 | 7.1 | 1 | 0 | 0 | 3.9 | 85 | 0.72 | 2.98 | 0 | 29.0 |
| run3 | MISSION_ABORTED | +2.7 | -6.8 | -26.8 | 29.5 | 19.1 | 0 | 0 | 1 | 17.1 | 102 | 3.86 | 4.20 | 88 | 38.8 |
| run4 | MISSION_OK | -3.1 | -13.3 | -33.3 | 12.0 | 9.8 | 1 | 0 | 0 | 3.7 | 84 | 0.72 | 2.48 | 0 | 29.8 |
| run5 | MISSION_OK | -0.9 | -11.8 | -31.8 | 11.2 | 5.7 | 1 | 0 | 0 | 3.6 | 84 | 0.99 | 2.02 | 0 | 29.2 |
| **média [mín; máx]** | 4/5 OK | -0.9 [-3.1; +2.7] | -11.2 [-13.3; -6.8] | -31.2 [-33.3; -26.8] | 15.5 [11.2; 29.5] | 9.6 [5.7; 19.1] | 1 [0; 1] | 0 [0; 0] | 0 [0; 1] | 6.4 [3.6; 17.1] | 88 [84; 102] | 1.49 [0.72; 3.86] | 3.07 [2.02; 4.20] | 18 [0; 88] | 31.1 [28.5; 38.8] |

### bateria5x5_fix/fechada (5 runs)

| run | resultado | captura prof mm | captura alt mm | alvo mm | descida mm | lingueta mm | estagnou | reassentou | perdeu olhal | duração s | J4 ° | J4 méd N·m | J4 máx | sat % | J2 máx |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| run1 | MISSION_OK | -5.1 | -14.2 | -34.2 | 14.2 | 12.4 | 1 | 0 | 0 | 4.1 | 82 | 0.89 | 2.16 | 0 | 30.1 |
| run2 | MISSION_OK | -0.2 | -11.4 | -31.4 | 10.9 | 5.0 | 1 | 0 | 0 | 3.4 | 85 | 0.88 | 2.79 | 0 | 28.8 |
| run3 | MISSION_ABORTED | -1.4 | -11.2 | -31.2 | 12.3 | 5.7 | 1 | 0 | 1 | 3.9 | 83 | 1.04 | 3.04 | 0 | 30.8 |
| run4 | MISSION_OK | +0.8 | -8.9 | -28.9 | 11.8 | 3.2 | 1 | 0 | 0 | 4.0 | 88 | 1.61 | 4.20 | 4 | 28.0 |
| run5 | MISSION_OK | -0.1 | -12.6 | -32.6 | 10.2 | 7.1 | 1 | 0 | 0 | 3.4 | 74 | 1.13 | 2.69 | 0 | 30.0 |
| **média [mín; máx]** | 4/5 OK | -1.2 [-5.1; +0.8] | -11.7 [-14.2; -8.9] | -31.7 [-34.2; -28.9] | 11.9 [10.2; 14.2] | 6.7 [3.2; 12.4] | 1 [1; 1] | 0 [0; 0] | 0 [0; 1] | 3.8 [3.4; 4.1] | 82 [74; 88] | 1.11 [0.88; 1.61] | 2.98 [2.16; 4.20] | 1 [0; 4] | 29.5 [28.0; 30.8] |

### bateria5x5_fix/aberta (5 runs)

| run | resultado | captura prof mm | captura alt mm | alvo mm | descida mm | lingueta mm | estagnou | reassentou | perdeu olhal | duração s | J4 ° | J4 méd N·m | J4 máx | sat % | J2 máx |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| run1 | MISSION_OK | -4.2 | -14.2 | -34.2 | 12.3 | 8.9 | 1 | 0 | 0 | 3.6 | 79 | 1.29 | 3.86 | 0 | 30.1 |
| run2 | MISSION_OK | -0.3 | -11.5 | -31.5 | 12.1 | 8.8 | 1 | 0 | 0 | 3.2 | 75 | 1.05 | 2.66 | 0 | 29.7 |
| run3 | MISSION_OK | +0.7 | -6.8 | -26.8 | 19.5 | 16.7 | 1 | 2 | 0 | 31.8 | 102 | 2.73 | 4.20 | 50 | 54.5 |
| run4 | MISSION_OK | -4.4 | -14.1 | -34.1 | 15.4 | 12.5 | 1 | 0 | 0 | 3.9 | 79 | 0.86 | 2.48 | 0 | 30.1 |
| run5 | MISSION_OK | -1.8 | -14.6 | -34.6 | 16.0 | 13.6 | 1 | 0 | 0 | 3.6 | 72 | 1.09 | 2.63 | 0 | 30.2 |
| **média [mín; máx]** | 5/5 OK | -2.0 [-4.4; +0.7] | -12.2 [-14.6; -6.8] | -32.2 [-34.6; -26.8] | 15.1 [12.1; 19.5] | 12.1 [8.8; 16.7] | 1 [1; 1] | 0 [0; 2] | 0 [0; 0] | 9.2 [3.2; 31.8] | 82 [72; 102] | 1.41 [0.86; 2.73] | 3.17 [2.48; 4.20] | 10 [0; 50] | 34.9 [29.7; 54.5] |

### bateria5x5/fechada (5 runs)

| run | resultado | captura prof mm | captura alt mm | alvo mm | descida mm | lingueta mm | estagnou | reassentou | perdeu olhal | duração s | J4 ° | J4 méd N·m | J4 máx | sat % | J2 máx |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| run1 | MISSION_OK | -0.7 | -12.8 | -32.8 | 10.5 | 6.8 | 1 | 0 | 0 | 3.4 | 79 | 1.05 | 2.70 | 0 | 29.6 |
| run2 | MISSION_OK | -0.2 | -11.2 | -31.2 | 10.5 | 4.7 | 1 | 0 | 0 | 3.6 | 83 | 1.26 | 3.35 | 0 | 28.8 |
| run3 | MISSION_OK | -3.3 | -12.8 | -32.8 | 19.0 | 17.0 | 0 | 0 | 0 | 6.3 | 87 | 0.97 | 3.21 | 0 | 29.4 |
| run4 | MISSION_OK | -3.8 | -14.9 | -34.9 | 10.9 | 7.9 | 1 | 0 | 0 | 3.4 | 84 | 1.34 | 3.03 | 0 | 30.3 |
| run5 | MISSION_OK | +0.7 | -7.3 | -27.3 | 18.7 | 14.4 | 1 | 2 | 0 | 17.1 | 94 | 2.26 | 4.20 | 32 | 37.5 |
| **média [mín; máx]** | 5/5 OK | -1.5 [-3.8; +0.7] | -11.8 [-14.9; -7.3] | -31.8 [-34.9; -27.3] | 13.9 [10.5; 19.0] | 10.2 [4.7; 17.0] | 1 [0; 1] | 0 [0; 2] | 0 [0; 0] | 6.8 [3.4; 17.1] | 86 [79; 94] | 1.38 [0.97; 2.26] | 3.30 [2.70; 4.20] | 6 [0; 32] | 31.1 [28.8; 37.5] |

### bateria5x5/aberta (5 runs)

| run | resultado | captura prof mm | captura alt mm | alvo mm | descida mm | lingueta mm | estagnou | reassentou | perdeu olhal | duração s | J4 ° | J4 méd N·m | J4 máx | sat % | J2 máx |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| run1 | MISSION_OK | -1.2 | -12.9 | -32.9 | 11.3 | 8.0 | 1 | 0 | 0 | 3.4 | 77 | 1.06 | 2.11 | 0 | 30.1 |
| run2 | MISSION_OK | -4.6 | -13.1 | -33.1 | 17.9 | 15.1 | 1 | 0 | 0 | 4.9 | 77 | 1.85 | 4.20 | 15 | 29.1 |
| run3 | MISSION_OK | -2.4 | -13.1 | -33.1 | 14.1 | 10.0 | 1 | 0 | 0 | 3.6 | 73 | 0.70 | 1.60 | 0 | 30.1 |
| run4 | MISSION_OK | -0.8 | -11.9 | -31.9 | 12.6 | 7.2 | 1 | 0 | 0 | 3.8 | 79 | 1.05 | 2.66 | 0 | 29.7 |
| run5 | MISSION_OK | -1.1 | -13.6 | -33.6 | 12.7 | 8.8 | 1 | 0 | 0 | 3.4 | 87 | 0.92 | 2.55 | 0 | 30.0 |
| **média [mín; máx]** | 5/5 OK | -2.0 [-4.6; -0.8] | -12.9 [-13.6; -11.9] | -32.9 [-33.6; -31.9] | 13.7 [11.3; 17.9] | 9.8 [7.2; 15.1] | 1 [1; 1] | 0 [0; 0] | 0 [0; 0] | 3.8 [3.4; 4.9] | 79 [73; 87] | 1.12 [0.70; 1.85] | 2.63 [1.60; 4.20] | 3 [0; 15] | 29.8 [29.1; 30.1] |

## Leitura

- **Não é sistemático.** Lingueta média nas cinco baterias do dia:
  fechada 10,2 / 6,7 / 9,6 mm, aberta 9,8 / 12,1 mm. A dispersão dentro de
  cada bateria (3 a 19 mm) é maior que a diferença entre condições. O
  6,7 do RELATORIO21 era ruído de cinco amostras.
- **O punho não é a causa.** No destrava o J4 trabalha a 0,7–1,6 N·m
  médios nas duas condições, sem saturação (exceto nos dois casos abaixo).
  A hipótese de o punho ceder mais com o dedo no eixo não se sustenta.
- **A fase fecha por estagnação em 19 de 20 execuções**, entre 10 e 16 mm
  de descida (alvo 20 = curso 18 + margem 2), e a lingueta fica a meio
  curso (5–13 mm) na maioria. Isso vale para as duas garras: é a
  característica do destrava atual (RELATORIO14 já tratava a estagnação
  com o reassentamento). A libera passa mesmo assim em 17 dos 19 casos.
- **Os dois abortos de hoje com a garra fechada são "perdeu o olhal"
  na libera, por dois caminhos diferentes:**
  - `bateria5x5_fix/fechada/run3`: destrava curto (12,3 mm, lingueta
    5,7), libera puxa com o gatilho preso, o J1 gira 6,3° e a ponta
    deriva −20,4 mm pelo eixo → guarda de 15 mm. É o run8 do
    RELATORIO16.
  - `destrava_fechada/fechada/run3`: captura 5 mm mais alta que o normal
    (−6,8 mm), o destrava **passa do alvo** (29,5 mm de descida para um
    alvo de 20), a lingueta chega ao fim do curso (19,1 de 20), o J4
    satura 88 % do tempo e a ponta escorrega −16,5 mm pelo eixo ainda
    durante o destrava; a libera detecta na primeira amostra (1,2 s).
    A aberta run3 da bateria anterior teve o mesmo perfil (19,5 mm,
    J4 50 % saturado, 2 reassentamentos, deriva +8,5) e sobreviveu por
    ficar abaixo dos 15 mm.
- **Hipótese do J5 livre descartada.** Com o dedo no eixo, a rolagem do
  punho não move a ponta e poderia derivar sem o controlador ver. Medido
  na libera de 15 execuções: o J5 varia 0,1–0,6° em todas, inclusive nas
  que perderam o olhal.
- Contagem do dia: perdeu o olhal em 2 de 10 fechadas e 0 de 10 abertas.
  Com esses números não dá para atribuir à garra; antes de hoje o modo
  apareceu com a garra aberta (run8, 05 Set).

## O que fica

O destrava é a fase mais frágil da manipulação nas duas condições: fecha
por estagnação, raramente completa o curso, e quando completa (captura
alta) passa do alvo e escorrega. Dois pontos concretos para uma próxima
rodada, ambos independentes da garra:

1. Teto de descida no destrava pela própria ponta (a medida que a
   bancada tem): parar de empurrar ao atingir o curso (20 mm) em vez de
   mirar 20 mm abaixo e deixar o whole-body passar.
2. Captura alta (alt > −9 mm) como sinal de reassentar antes de
   descer: nos dois casos de escorregão do dia a captura estava 5 mm
   acima das demais.

Nenhum dos dois foi aplicado; decisão do orientador.
