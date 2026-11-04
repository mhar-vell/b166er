# RELATORIO51 — eixo −13 depois do engate: busca da haste — 04 Nov 2026

Marco: "pode seguir com o eixo −13 depois do engate". Branch `feat/atravessa-ate-a-haste`, a partir da main com o
engate por parada no eixo (PR #117).

## Diagnóstico (baterias `parada_no_eixo` e `refine_sem_garra`)

| | controle | eixo −13 |
|---|---|---|
| eixo verdadeiro da ponta na captura | ~10 mm | 21 a 31 mm |

Com a parede estimada aquém da real, a atravessa fecha no próprio alvo sem tocar em nada, e a haste fica 13 a 20 mm antes
do arame. O arame cai perto da ponta do degrau. Nas falhas, a ponta desliza para fora ao longo do eixo durante o destrava,
de 5 a 12 mm, e o arame escapa pela ponta: a lingueta volta a zero e o destrava cai no vazio, chegando ao limite de
descida a 49 e a 31 mm. No controle, a ponta já termina com a haste no arame. A sonda mostra a parada em 9 a 10,5 mm,
o mesmo ponto em que a haste para quando a parede está 8 mm além.

## Tentativa descartada: alvo 15 mm além da haste (commit 8e7c08be, revertido em 0d3b3a0a)

Mandar a atravessa mirar 15 mm além do batente da haste e deixar a parada no eixo fechar a fase não funcionou. Na missão
de prova sem viés, o erro ao longo do eixo oscilou de −56 a +63 mm, houve 4 recuos e 11 iterações, e a haste passou do
arame, terminando a −7,4 mm pela sonda. A IK iterativa acumula a mira além do alvo a cada iteração, e isso vira força
contra o arame. Com 8 mm de sobra, no viés de +8, a parada tinha sido estável. Com 15 mm, não.

## A mudança: busca da haste (commit 3f650e34)

Depois de uma atravessa que fecha SEM contato, a ponta avança ao longo do eixo em passos de 5 mm, com um único comando
de IK por passo e sem mirar além do alvo. Quando um passo avança menos de 3 mm, a haste encostou. A parede é deslocada
pelo avanço total, e a captura parte do contato. Se a atravessa já fechou por contato ou por parada no eixo, a busca não
roda. O avanço é medido pela ponta da T265, então vale na bancada. Parâmetros: `~busca_haste_passo_m` (0,005, e 0
desliga), `~busca_haste_max_m` (0,025) e `~busca_haste_avanco_min_m` (0,003).

Nas provas, o controle encostou no primeiro passo, com correção de 2,5 mm. Com eixo −13, a ponta avançou 15,3 mm em três
passos e terminou a 9,4 mm pela sonda, contra 10,1 mm no controle.

## Bateria `busca_haste` (5 vieses × 5): 22/25

| viés | busca da haste | main (RELATORIO50) |
|---|---|---|
| controle | 5/5 | 5/5 |
| eixo −13 | **5/5** | 2/5 |
| eixo +8 | **4/5** | 3/5 |
| profundidade +12 | 4/5 | 4/5 |
| altura +10 | **4/5** | 3/5 |
| **total** | **22/25** | **17/25** |

- **A busca encontrou a haste em 22 execuções e nunca chegou ao limite de 25 mm sem encostar.** A parada no eixo fechou outras 2, no eixo +8.
- **O eixo verdadeiro na captura ficou entre 8,9 e 10,9 mm em 21 das 24 capturas.** As exceções foram duas de eixo −13, com 15,9 e 18,6 mm, ainda assim melhores que os 21 a 31 de antes, e as duas deram certo. A terceira foi uma de eixo +8, com −9,7 mm.
- **As três falhas:**
  - **Eixo +8, execução 5:** a ponta passou do ponto, ficando a −9,7 mm na captura, e o olhal se perdeu no segundo arco por deriva ao longo do eixo.
  - **Profundidade +12, execução 1:** a ponta subiu 8,6 mm na libera, e o reassentamento não conseguiu reentrar.
  - **Altura +10, execução 4:** perdeu a tag na aproximação, porque o laser viu só 1 a 4 pontos de parede. Não tem relação com a mudança.
- O refinamento viu a tag em 24 das 25 execuções.

## Conclusão

A busca da haste faz a atravessa terminar com a haste no arame em todas as missões, como o controle já fazia. Ela
corrige o erro da parede ao longo do eixo pelo contato, sem depender da percepção. O eixo −13 foi de 2/5 para 5/5, e o
total, de 17 para 22 em 25, o melhor das baterias até aqui. **Recomendo mergear.** Ficam pendentes a ponta que passa da
haste no eixo +8, a subida na libera com profundidade +12 e a perda da tag na aproximação.
