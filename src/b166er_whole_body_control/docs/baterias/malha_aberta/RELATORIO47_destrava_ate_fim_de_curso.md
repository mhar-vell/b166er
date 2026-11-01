# RELATORIO47 — destrava até o fim de curso da lingueta — 31 Out a 01 Nov 2026

Marco: "pode seguir com a perda do olhal na libera". Branch `feat/destrava-ate-fim-de-curso` (commit e627c5e8), a partir
da main com a atravessa em passos (PR #113).

## Diagnóstico (sonda da bateria `atravessa_passos`, RELATORIO46)

As três perdas de olhal na libera com altura +10 (execuções 1, 2 e 3) têm a mesma assinatura:

| | 17 libera que deram certo | 3 perdas com altura +10 |
|---|---|---|
| lingueta no fim da captura | 2–13 mm (19 de 21 missões) | 0 mm |
| lingueta no início da libera | 17,8–20 mm | 13,5–14,3 mm |
| a aba passa sob o laço a | 13 mm | 13 mm |
| durante a libera | lingueta fica, chave abre | punho cede ~12°, ponta sobe, lingueta volta a 0, lâmina parada em 0,6° |

Com a parede estimada 10 mm acima da real, a captura termina com o degrau só encostado no arame, enquanto nas outras
ela já termina empurrando a lingueta. O destrava descia 20 mm abaixo da captura e fechava ao alcançar o alvo. Na
altura +10, esses 20 mm deixavam a lingueta a 1 mm da trava. Na libera, o punho cedia, a trava fechava de novo e a ponta
escorregava ao longo do eixo do furo até o alarme de perda disparar.

A missão não enxerga esse desvio, porque a captura e a descida estão no mesmo referencial estimado. O que ela enxerga é
a ponta parar quando a lingueta bate no fim de curso. Na mesma bateria, 10 destravas já fechavam por essa estagnação.

A quarta perda (eixo −13, execução 1) é de outro tipo: a ponta escorregou 20 mm ao longo do eixo durante o próprio destrava.

## A mudança

Parâmetro novo `curso_alvo_m` no destrava, que é a descida comandada a partir da captura: de 20 para **28 mm**. O
limite de descida sobe de **22 para 30 mm**. O mínimo de 18 mm para aceitar continua, assim como a estagnação, que fecha
a fase quando a ponta para depois de 10 mm. O objetivo é a fase terminar com a lingueta no batente, e não ao alcançar
um alvo. Sem o parâmetro, o comportamento é o anterior. O critério é a ponta medida pela T265, e por isso vale na bancada.

## Bateria `destrava_fim_curso` (5 vieses × 5): 17/25

| viés | até o fim de curso | atravessa em passos, main (RELATORIO46) |
|---|---|---|
| controle | 5/5 | 5/5 |
| eixo −13 | 3/5 | 3/5 |
| eixo +8 | **5/5** | 4/5 |
| profundidade +12 | 0/5 | 3/5 |
| altura +10 | **4/5** | 1/5 |
| **total** | **17/25** | **16/25** |

- **Lingueta no início da libera:** entre 19,8 e 20 mm nas 19 execuções em que o degrau pegou o olhal. A chave abriu na libera em todas elas.
- **Perda do olhal na libera:** caiu de 4 para 1. A que sobrou é a do eixo −13, execução 2, em que o destrava nunca moveu a lingueta: o degrau não estava no olhal. É o mesmo tipo da execução 4 do eixo −13 na bateria anterior.
- **Como o destrava fechou:** 14 pela estagnação no fim de curso e 1 pelo limite de descida.

## Profundidade +12: variação da atravessa, não efeito da mudança

As cinco falhas de profundidade +12 aconteceram na atravessa ou na captura, antes do destrava. Para conferir, rodamos
mais cinco com a branch e três com a main, no mesmo stack:

| profundidade +12 | sucessos | onde falhou |
|---|---|---|
| branch, bateria principal | 0/5 | atravessa ×4, captura ×1 |
| branch, repetição | 0/5 | atravessa ×4, aproximação perdeu a tag ×1 |
| main, mesmo stack | 1/3 | atravessa ×2 |
| main, RELATORIO46 | 3/5 | atravessa ×2 |

A mudança só altera o alvo e o limite do destrava, e nada dela roda antes da captura. Nas repetições, nenhuma execução
chegou ao destrava, então não há resto de uma execução contaminando a seguinte. A atravessa com profundidade +12 é
instável nas duas versões: a primeira iteração começa uns 25 mm antes do alvo, a ponta encosta e recua várias vezes, e às
vezes não converge. A diferença entre as versões, de 0/10 contra 4/8, não tem caminho pelo código. É o próximo ponto a
atacar, junto com o abaixo.

## Falha nova no arco2

Duas perdas de olhal no arco2 (eixo −13, execução 1; altura +10, execução 4), com a ponta derivando 21 a 22 mm ao longo do
eixo. Nenhuma das duas baterias anteriores teve perda no arco2. Agora mais execuções chegam lá: 19 contra 16. Ainda não
sabemos se o destrava mais fundo contribui. A sonda guarda o olhal na posição de repouso, então não mede o arco.

## Conclusão

- O defeito que motivou a mudança foi resolvido: a libera agora começa com a lingueta no batente em todas as execuções engatadas, e a altura +10 foi de 1/5 para 4/5.
- Excluindo a profundidade +12, que falha antes da fase alterada, foram 17/20, contra 13/20 da main.
- **Recomendo mergear.** Os próximos pontos são a atravessa com profundidade +12 e a deriva de eixo no arco2.
