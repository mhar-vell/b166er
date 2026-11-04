# RELATORIO50 — atravessa com eixo +8: engate por parada no eixo — 04 Nov 2026

Marco: "pode seguir com a atravessa do eixo +8". Branch `feat/engate-por-parada-no-eixo` (commit 2e693a48), a partir
da main com o refinamento vendo a tag (PR #116).

## Diagnóstico (bateria `refine_sem_garra`, RELATORIO49)

Nas três falhas de eixo +8, a parede fica estimada 8 mm além da real ao longo do eixo. A haste encosta no arame antes do
alvo, e a ponta para com erro de +6,2 a +7,2 mm em todas as iterações, logo acima da tolerância de 6. O engate por
contato existente não fechava esses casos, por dois motivos:

- **O travamento quase nunca é detectado.** O sinal esperado, a ponta parada com o ombro cada vez mais aquém do pedido, só apareceu em 1 das 3 execuções.
- **O alinhamento é exigido no instante da trava.** Empurrando o arame, a ponta escorrega em profundidade e altura de −6 a +9 mm entre iterações, e o critério pedia as duas dentro da tolerância naquele instante.

## A mudança

Novo critério na atravessa, o engate por parada no eixo (`_engate_por_parada`). A fase fecha por contato quando as
últimas 3 iterações têm a ponta no mesmo ponto do eixo, com espalhamento de até 1,5 mm, todas antes do alvo, com erro
positivo, e a menos de 15 mm dele. A borda do furo bloqueia a uns 16,6 mm, então fica de fora. Além disso, a MEDIANA da
profundidade e da altura precisa estar dentro da tolerância. A parede é deslocada pelo eixo como no engate por contato,
e a captura parte do ponto do contato. O critério usa só a ponta medida pela T265 e vale na bancada. Os parâmetros são
`~engate_parada_n` (3, e 0 desliga) e `~engate_parada_tol_mm` (1,5).

Antes de rodar, simulamos o critério sobre os registros das baterias refine_sem_garra, entrada_centro2, entrada_centro e
destrava_fim_curso. Ele dispararia em 5 execuções, todas abortadas, e em nenhuma que tinha dado certo.

## Bateria `parada_no_eixo` (5 vieses × 5): 17/25

| viés | parada no eixo | main (RELATORIO49) |
|---|---|---|
| controle | 5/5 | 5/5 |
| eixo −13 | 2/5 | 4/5 |
| eixo +8 | **3/5** | 2/5 |
| profundidade +12 | 4/5 | 4/5 |
| altura +10 | 3/5 | 3/5 |
| **total** | **17/25** | **18/25** |

- **O critério disparou 2 vezes, nas execuções 4 e 5 de eixo +8, e as duas terminaram com sucesso.** As paradas foram em 6,2/5,7/6,5 mm e 7,0/6,4/7,4 mm. Não disparou em nenhum outro viés.
- **As outras duas falhas de eixo +8 não passam pela atravessa.** A execução 3 perdeu a tag na aproximação: o laser viu só 13 pontos de parede, e a tag não reapareceu em 30 s de giro. A execução 1 falhou na captura e depois no reassentamento.
- **O eixo −13 caiu de 4 para 2 sem o critério ter disparado.** Nas três falhas o degrau engatou, com a lingueta chegando a 20 mm no destrava. O que falhou veio depois: duas vezes o limite de descida no destrava, a 49 e a 31 mm, uma lâmina parada em 19,8°, abaixo dos 25° exigidos, e uma perda do olhal na libera. Nas baterias anteriores esse viés variou entre 2 e 4 em 5.
- O refinamento viu a tag em 24 das 25 execuções.

## Conclusão

O critério resolve o caso para o qual foi feito: as duas atravessas que ficariam paradas a 6–7 mm do alvo fecharam, e as
duas missões deram certo. Ele não disparou em nenhuma execução fora desse caso. O total ficou empatado com a main, e a
queda no eixo −13 acontece depois do engate, em fases que a mudança não toca. **Recomendo mergear.** O próximo ponto é o
eixo −13 depois do engate, com o degrau pegando o arame perto da ponta, o que leva ao limite de descida no destrava e à
perda do olhal na libera.
