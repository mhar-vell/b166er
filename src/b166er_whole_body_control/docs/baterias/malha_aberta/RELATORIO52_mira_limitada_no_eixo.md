# RELATORIO52 — ponta que passa da haste no eixo +8: mira limitada no eixo — 04 Nov 2026

Marco: "pode seguir com a ponta que passa da haste no eixo +8". Branch `feat/mira-limitada-no-eixo` (commit c1a1f3de),
a partir da main com a busca da haste (PR #118).

## Diagnóstico (execução 5 de eixo +8 da bateria `busca_haste`)

Nas duas primeiras iterações, a haste estava no arame, com erro de +6,2 e +5,4 mm. Pela sonda, a ponta estava a 9 a
10 mm do centro, como no controle. Na terceira, a ponta saltou para −31 a −39 mm pela sonda, ou seja, passou da haste.
A partir daí vieram recuos e busca lateral, e a missão terminou com a ponta uns 9 cm fora em profundidade, sem perceber.
A hipótese foi a mira além do alvo, que a IK iterativa acumula a cada iteração bloqueada, uns 5 mm por iteração. Na
terceira, o ombro empurraria a haste por cima do arame. Esse salto, de um batente estável para além da haste, apareceu
em 3 de 15 execuções de eixo +8 e em nenhuma dos outros vieses.

## A mudança

Na atravessa, a componente da mira além do alvo ao longo do eixo do furo fica limitada a 2 mm
(`~mira_eixo_max_m`, e 0 desliga). A compensação em profundidade e em altura continua livre.

## Bateria `mira_limitada` (5 vieses × 5): 19/25

| viés | mira limitada | main (RELATORIO51) |
|---|---|---|
| controle | 4/5 | 5/5 |
| eixo −13 | 5/5 | 5/5 |
| eixo +8 | 4/5 | 4/5 |
| profundidade +12 | 4/5 | 4/5 |
| altura +10 | 2/5 | 4/5 |
| **total** | **19/25** | **22/25** |

| atravessas com salto de mais de 30 mm entre iterações | |
|---|---|
| mira limitada | **5/23** |
| busca_haste (main) | 2/24 |
| parada_no_eixo | 1/24 |
| refine_sem_garra | 3/25 |

- **O eixo +8 ficou igual, em 4/5.** A falha restante, a execução 3, saltou para −47,9 mm já na primeira iteração.
- **A hipótese estava incompleta.** Os saltos grandes acontecem na primeira iteração, vindo do passo intermediário, ou de uma iteração para a outra sem acúmulo nenhum. Exemplos: −47,9 e −33 mm no início, e +27,7 → −32,9 mm em seguida. É o salto do ombro em malha aberta, que passa do pedido, descrito no RELATORIO41. A limitação da mira não atua sobre isso.
- **Falhas:** três atravessas com esses saltos, nas execuções de profundidade +12 número 2, eixo +8 número 3 e altura +10 número 3; duas perdas da tag na aproximação, no controle 4 e na altura +10 número 4, sem relação com a mudança; e uma subida na libera, na altura +10 número 5, que também teve salto na atravessa.

## Conclusão

A mudança não reduziu os saltos além da haste: houve 5 em 23 atravessas, contra 1 a 3 por bateria antes. O total caiu
de 22 para 19 em 25. Com cinco execuções por viés não dá para dizer que piorou, mas também não há ganho. **Não recomendo
mergear.** A PR fica como rascunho, para registro.

O próximo passo, se o Marco quiser seguir, é atacar o salto do ombro em malha aberta na própria atravessa. Uma opção é
um passo final ainda menor, perto da haste. Outra é limitar a velocidade do J1 nos últimos milímetros, para o ombro não
passar do pedido quando a haste encosta.
