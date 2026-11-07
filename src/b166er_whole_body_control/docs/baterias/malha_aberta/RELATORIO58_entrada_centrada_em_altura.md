# RELATORIO58 — altura +10: entrada no furo centrada em altura — 07 Nov 2026

Marco: "pode seguir com a altura +10". Branch `feat/entrada-centrada-em-altura` (commit 07e1b387), a partir da main
com os RELATORIO56/57. Baterias com o viés de profundidade de +6 mm, conforme a decisão de 06 Nov.

## Diagnóstico (baterias vies_prof6, laser_angulo, busca_haste e parada_no_eixo)

A referência da ponta é a face de CIMA do degrau, que tem 10 mm de altura. A aproximação lateral e a atravessa entravam
com a ponta a +10 mm, com o degrau entre 0 e +10 mm num furo de ±20 mm: 10 mm de folga em cima e 20 embaixo. A assimetria
vinha da nota "a lâmina tapa a metade de baixo do furo", desatualizada desde 01 Set, quando a barra passou a terminar
abaixo do anel.

| altura verdadeira da ponta no fim da aproximação lateral | mediana | faixa |
|---|---|---|
| controle | 10,4 mm | 7,0 a 14,0 |
| altura +10 | **20,9 mm** | 17,2 a 28,7 |

Com a parede estimada 10 mm acima da real, a face de cima do degrau entrava na borda interna do arame superior, a +20 mm.
A atravessa ou o passo sem fechar respondia por 4 das 9 falhas recentes desse viés.

## A mudança

A aproximação lateral e a atravessa passam a entrar com a ponta a **+5 mm**. O degrau fica entre −5 e +5 mm, com 15 mm de
folga para cada lado. Na elipse, essa altura também dá cerca de 1,4 mm a mais de folga em profundidade.

## Bateria `entrada_alt5` (5 vieses × 5): 23/25

| viés | entrada a +5 | main (RELATORIO57, vies_prof6) |
|---|---|---|
| controle | 5/5 | 5/5 |
| eixo −13 | 5/5 | 5/5 |
| eixo +8 | 5/5 | 5/5 |
| profundidade +6 | 5/5 | 5/5 |
| altura +10 | **3/5** | 2/5 |
| **total** | **23/25** | **22/25** |

| altura verdadeira da ponta na entrada | entrada a +5 |
|---|---|
| controle | 4,1 a 7,1 mm |
| altura +10 | **13,4 a 17,8 mm**, antes 17,2 a 28,7 |

- **Atravessa ou passo sem fechar caiu para 1,** e nenhuma lâmina abriu antes do destrava. Refinamento com tag em 25/25 e nenhuma aproximação sem tag.
- **As duas falhas, ambas de altura +10:**
  - **Execução 3:** subida na libera de 12,1 mm, seguida de reassentamento e destrava sem fechar.
  - **Execução 4:** o passo intermediário ficou com 75 mm de resíduo, e a atravessa não fechou depois de 4 recuos. Um resíduo desse tamanho tem a assinatura do salto do ombro em malha aberta.

## Conclusão

Com a entrada centrada, a face de cima do degrau deixa de entrar no arame superior quando a parede está estimada alta,
e o total chega a 23/25, o melhor das baterias. **Recomendo mergear.** As falhas que restam são o salto do ombro, cuja
emulação depende de uma decisão do Marco (RELATORIO53), e uma subida na libera.
