# RELATÓRIO 21 — bateria 5 × 5 com as correções do atravessa (PR #80)

Data: 2026-09-30, 17:08–17:45, shiroi (Gazebo com GUI). Mesmo roteiro do
RELATORIO19 (5 missões por condição, garra aberta 0,03 e fechada 0,0,
mesma pose de partida, reset entre missões), agora com as correções da
PR #80: alvo do aproxima_lateral/atravessa 8 mm para o lado do robô e
recuo por contato na IK iterativa.

## Resultado por execução

### aberta (5 runs)

| run | resultado | atravessa it0 eixo mm | atravessa it | recuos | lâmina ° | destrava desc mm | lingueta mm | libera s | libera erro mm | recuo mm | deriva mm | J4 libera ° | J4 máx libera | sat libera % | J4 máx arco1 | J4 máx arco2 | J5 máx libera |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| run1 | MISSION_OK | -1.4 | 3 | 0 | 30.6 | 12.3 | 8.9 | 5.0 | 5.5 | 54.3 | +0.7 | 80 | 4.20 | 37 | 3.85 | 3.33 | 0.37 |
| run2 | MISSION_OK | +1.9 | 1 | 0 | 28.7 | 12.1 | 8.8 | 8.2 | 10.5 | 48.4 | -0.3 | 74 | 4.20 | 48 | 4.20 | 3.12 | 0.91 |
| run3 | MISSION_OK | +0.7 | 1 | 1 | 29.7 | 19.5 | 16.7 | 3.2 | 11.1 | 51.8 | +8.5 | 104 | 3.26 | 0 | 4.20 | 4.20 | 6.61 |
| run4 | MISSION_OK | +1.9 | 2 | 0 | 30.4 | 15.4 | 12.5 | 5.1 | 7.3 | 52.1 | +0.3 | 81 | 4.20 | 35 | 4.20 | 4.20 | 0.34 |
| run5 | MISSION_OK | +2.3 | 2 | 0 | 30.3 | 16.0 | 13.6 | 2.5 | 10.2 | 57.1 | -0.3 | 73 | 4.20 | 6 | 4.20 | 4.20 | 0.68 |
| **média [mín; máx]** | 5/5 OK | +1.1 [-1.4; +2.3] | 2 [1; 3] | 0 [0; 1] | 29.9 [28.7; 30.6] | 15.1 [12.1; 19.5] | 12.1 [8.8; 16.7] | 4.8 [2.5; 8.2] | 8.9 [5.5; 11.1] | 52.7 [48.4; 57.1] | +1.8 [-0.3; +8.5] | 82 [73; 104] | 4.01 [3.26; 4.20] | 25 [0; 48] | 4.13 [3.85; 4.20] | 3.81 [3.12; 4.20] | 1.78 [0.34; 6.61] |

### fechada (5 runs)

| run | resultado | atravessa it0 eixo mm | atravessa it | recuos | lâmina ° | destrava desc mm | lingueta mm | libera s | libera erro mm | recuo mm | deriva mm | J4 libera ° | J4 máx libera | sat libera % | J4 máx arco1 | J4 máx arco2 | J5 máx libera |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| run1 | MISSION_OK | +1.8 | 3 | 0 | 31.3 | 14.2 | 12.4 | 2.4 | 3.2 | 58.5 | +1.4 | 84 | 3.92 | 0 | 3.75 | 2.73 | 0.13 |
| run2 | MISSION_OK | +0.3 | 1 | 0 | 29.3 | 10.9 | 5.0 | 3.2 | 8.4 | 53.7 | +0.3 | 86 | 4.20 | 30 | 3.92 | 3.98 | 0.23 |
| run3 | MISSION_ABORTED | +1.0 | 2 | 1 | — | 12.3 | 5.7 | — | — | — | — | 81 | 4.20 | 64 | — | — | 0.18 |
| run4 | MISSION_OK | +2.0 | 1 | 0 | 29.2 | 11.8 | 3.2 | 3.6 | 11.3 | 53.1 | -0.3 | 88 | 4.20 | 28 | 4.20 | 4.20 | 0.21 |
| run5 | MISSION_OK | +5.3 | 2 | 0 | 28.9 | 10.2 | 7.1 | 7.1 | 13.5 | 51.9 | -1.1 | 73 | 4.20 | 40 | 4.20 | 3.58 | 0.49 |
| **média [mín; máx]** | 4/5 OK | +2.1 [+0.3; +5.3] | 2 [1; 3] | 0 [0; 1] | 29.7 [28.9; 31.3] | 11.9 [10.2; 14.2] | 6.7 [3.2; 12.4] | 4.1 [2.4; 7.1] | 9.1 [3.2; 13.5] | 54.3 [51.9; 58.5] | +0.1 [-1.1; +1.4] | 82 [73; 88] | 4.14 [3.92; 4.20] | 32 [0; 64] | 4.02 [3.75; 4.20] | 3.62 [2.73; 4.20] | 0.25 [0.13; 0.49] |

## Leitura

- **O atravessa deixou de travar.** Erro no eixo na primeira iteração:
  +1,1 mm [−1,4; +2,3] aberta e +2,1 mm [+0,3; +5,3] fechada, contra
  +11,9 [−0,3; +21,8] e +12,0 [−0,9; +21,5] na bateria anterior
  (RELATORIO19, 7 de 10 execuções paradas em ≈ +20 mm). Nenhuma das 10
  chegou perto da aba. O recuo por contato disparou uma vez (aberta
  run3) e a fase fechou em seguida.
- **9/10 MISSION_OK.** O aborto (fechada run3) é OUTRO modo de falha, já
  conhecido: destrava fechou com a lingueta em 5,7 mm ("gatilho pode não
  ter soltado") e na libera a ponta derivou −20,4 mm pelo eixo desde a
  captura, acima da guarda de 15 mm ("perdeu o olhal", RELATORIO16, PR
  #54) → ABORT_SAFE limpo (saiu em degraus, voltou à partida). Não é o
  atravessa: ali fechou em 2 iterações com +1,0 mm.
- **Efeito colateral corrigido nesta rodada:** no ABORT dessa execução o
  recuo disparou na fase de saída ('saida_sobe'), onde recuar pelo eixo
  e descer 5 mm é o oposto do desejado (não impediu a saída, mas é
  errado). O recuo passa a valer só em `~ik_recuo_fases` =
  [aproxima_lateral, atravessa].
- Punho, fechada × aberta, mesma leitura do RELATORIO19: J5 máximo na
  libera 0,13–0,49 N·m contra 0,34–6,61; saturação do J4 na libera
  0–64 % contra 0–48 % (uma execução fechada, a que abortou, saturou 64 %
  puxando um olhal que já tinha escapado).
- Lingueta no destrava: aberta 12,1 mm [8,8; 16,7], fechada 6,7 mm
  [3,2; 12,4]. Na bateria anterior eram 9,8 e 10,2. Cinco execuções não
  separam isso do ruído, mas fica anotado: se o destrava com a garra
  fechada desce menos de forma sistemática, é o próximo alvo (o critério
  aceita 10 mm de descida e a libera passa com a lingueta a meio curso
  em 8 de 9 casos, mas foi ela que perdeu o olhal aqui).

## Conclusão

As correções 1 e 3 resolvem o que se propuseram: 0 travamentos no
atravessa em 10 execuções (7 em 10 antes). O aborto restante é do
destrava/libera (lingueta a meio curso → deriva de eixo), independente da
mudança, e merece uma bateria própria do destrava com a garra fechada.
