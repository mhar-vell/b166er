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

## Segunda mudança: em andamento

Contar a descida do destrava a partir da altura-alvo da captura (−10 mm) quando a captura fecha mais funda que ela. A
parte do curso da lingueta feita pela captura passa a contar. Resultado na próxima seção, quando a bateria terminar.
