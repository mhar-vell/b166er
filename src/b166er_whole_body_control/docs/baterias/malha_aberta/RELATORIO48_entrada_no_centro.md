# RELATORIO48 — atravessa com profundidade +12: entrada no centro do furo — 01 Nov 2026

Marco: "pode seguir com a atravessa da profundidade +12". Branch `feat/atravessa-centro-profundidade`, a partir da main
com o destrava até o fim de curso (PR #114).

## Diagnóstico (sonda e contatos da execução 6 de profundidade +12, bateria `destrava_fim_curso_prof`)

- **A ponta entra longe do centro em profundidade.** A aproximação e a atravessa mandam +8 mm em profundidade, para o lado do robô. Essa margem é de 30 Set, quando o refinamento errava a parede em 4 a 17 mm; hoje o laser corrige a profundidade. Sem viés de profundidade, a sonda mede a ponta em média a +5 mm do centro no fim da aproximação, quase sempre entre 0 e 9,5 mm, nas baterias degrau30, atravessa_passos e destrava_fim_curso. Com o viés de +12, ela fica entre 15 e 25 mm.
- **O furo é uma elipse.** Os semieixos internos têm cerca de 14 mm em profundidade e 20 mm em altura. O degrau tem 10 × 10 mm, com a face de cima a +10 na atravessa, e por isso tem só uns ±7 mm de folga em profundidade. Até o controle às vezes encosta do lado do robô.
- **O anel só cede para baixo.** Fora da folga, o degrau bate na curva de cima do arame, a lingueta desce até o fim de curso de 20 mm e o degrau passa POR CIMA do olhal. Os contatos dessa execução mostram a haste e o degrau sobre o arame superior, de 24 a 28 mm acima do centro do anel deslocado. Os recuos de 20 mm ao longo do eixo não desfazem isso, e a busca lateral de ±8 mm parte de um ponto deslocado demais.
- **A nota antiga sobre a metade de baixo está desatualizada.** O YAML ainda diz que a lâmina tapa essa metade do furo, mas desde 1 Set a barra termina abaixo do anel.

## Primeira mudança: profundidade +8 → +3 mm na aproximação e na atravessa (commit 061d3755)

### Bateria `entrada_centro` (5 vieses × 5): 15/25

| viés | entrada a +3 | main (RELATORIO47) |
|---|---|---|
| controle | 3/5 | 5/5 |
| eixo −13 | 3/5 | 3/5 |
| eixo +8 | 3/5 | 5/5 |
| profundidade +12 | **3/5** | 0/5 (0/10 com as repetições) |
| altura +10 | 3/5 | 4/5 |
| **total** | **15/25** | **17/25** |

**A profundidade +12 foi de 0/10 para 3/5**, que era o alvo. Por outro lado, apareceu uma falha nova:

| falha | entrada_centro | destrava_fim_curso | atravessa_passos | degrau30 |
|---|---|---|---|---|
| destrava parou CURTO, abaixo de 10 mm | 3 | 0 | 0 | 1 |
| captura sem fechar | 3 | 1 | 2 | 1 |
| reassentamento não conseguiu subir | 4 | 0 | 1 | 0 |

**A causa é a própria mudança.** Com a entrada centrada, o degrau passa longe da curva do arame de baixo e desce mais fundo na captura. A captura estimada termina entre −10 e −20 mm, contra −7 a −11 mm antes. Pela sonda, ela já empurra a lingueta de 10 a 12 mm, contra 3 a 6 mm antes. No destrava, a lingueta chega ao fim de curso depois de só 7 a 9 mm de descida, e a sonda confirma os 20 mm nas três. A missão exige 10 mm de descida desde a captura para aceitar a parada. Por isso conclui que o degrau escorregou e manda reassentar, mesmo com o destrava cumprido. O reassentamento então falha ao subir.

## Segunda mudança: descida do destrava contada do alvo da captura (commit 48f5b8f3)

Novo `curso_ref_alvo_captura` no destrava. Quando a captura fecha abaixo do próprio alvo (−10 mm), a descida do
destrava é contada a partir do alvo, não de onde a captura parou. Isso vale para o mínimo de aceitação, a parada no fim
de curso, o limite e o alvo da descida. A parte do curso da lingueta já feita pela captura passa a contar. Quando a
captura fecha acima do alvo, com o degrau só encostado, tudo continua como antes. O critério usa a ponta medida pela
T265.

### Bateria `entrada_centro2` (5 vieses × 5): 16/25

| viés | +3 e descida do alvo | só +3 | main (RELATORIO47) |
|---|---|---|---|
| controle | 5/5 | 3/5 | 5/5 |
| eixo −13 | 2/5 | 3/5 | 3/5 |
| eixo +8 | 3/5 | 3/5 | 5/5 |
| profundidade +12 | **2/5** | 3/5 | 0/5 |
| altura +10 | 4/5 | 3/5 | 4/5 |
| **total** | **16/25** | **15/25** | **17/25** |

| falha | +3 e descida do alvo | só +3 |
|---|---|---|
| destrava parou curto | **0** | 3 |
| reassentamento não subiu | **0** | 4 |
| captura sem fechar | 2 | 3 |
| atravessa sem fechar | 4 | 3 |
| chave não abriu | 3 | 1 |

- **O destrava curto sumiu,** junto com os reassentamentos que não subiam.
- **Profundidade +12 nas duas baterias da branch: 5/10.** Com a entrada antiga, no mesmo stack, foram 0/10 com a branch do RELATORIO47 e 1/3 com a main.
- **As três "chave não abriu" são outra falha, que já existe na main.** A atravessa fechou pela estimativa a −2 mm do alvo, mas pela sonda a ponta ficou no último passo intermediário, a 39–43 mm do centro, fora do anel. O ombro não executou o último movimento de uns 25 mm, e o estimador sem encoder contou o comando como feito. A captura e o destrava desceram na frente do olhal, e o destrava foi aceito porque alcançou o alvo em queda livre. Capturas com a ponta a mais de 35 mm do centro, pela sonda: degrau30 0, atravessa_passos 0, destrava_fim_curso (main atual) 2, entrada_centro 2, entrada_centro2 3.

## Conclusão

- A causa da falha com profundidade +12 era geométrica: a entrada deslocada 8 mm para o lado do robô num furo elíptico com ±7 mm de folga. Com a entrada no centro, esse viés foi de 0/10 para 5/10.
- O total ficou empatado com a main: 16/25 contra 17/25. Fora da profundidade, a branch fez 12/20 e 14/20, e a main fez 17/20 na última bateria, 13/20 e 15/20 nas anteriores. A diferença está dentro da variação que temos visto.
- **Recomendo mergear.** O ganho na profundidade é grande e tem causa medida. A próxima falha a atacar é o ombro que não executa o último passo da atravessa sem que o estimador perceba. Ela aparece 2 a 3 vezes por bateria desde a PR #114, e um destrava que desce 28 mm sem estagnar poderia ser o sinal de que o degrau não está no olhal.
