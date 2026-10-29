# RELATORIO42 — referência com o olhal medido no lab — 2026-10-29

Bateria `olhal_real`, 5 vieses × 5, na **main** de 28 Out (2b9237f4): olhal com as medidas do lab (PR #107, furo 40 × 28 mm,
arame Ø 8 mm) e servo sem caçar o alvo (PR #105). Sem as PRs em rascunho: destrava pela lateral (#104), salto do J1
(#106), degrau de 30 mm.

## Resultado: 10/25

| viés | olhal_real (main) | salto_j1 (#106, olhal antigo) | destrava_lateral (#104, olhal antigo) |
|---|---|---|---|
| controle | **4/5** | 2/5 | 3/5 |
| eixo −13 | **2/5** | 0/5 | 0/5 |
| eixo +8 | 2/5 | 2/5 | 2/5 |
| profundidade +12 | **2/5** | 0/5 | 0/5 |
| altura +10 | **0/5** | 2/5 | 3/5 |
| **total** | **10/25** | 6/25 | 8/25 |

É o melhor total das baterias de cinco vieses. A comparação não é limpa: entre elas mudaram o olhal, o servo e o
procedimento de destrava. Mas dois padrões aparecem:

- **Altura +10 cai para 0/5**, e em quatro das cinco a libera "subiu" (9 a 13,5 mm). É exatamente o caso que o destrava
  pela lateral resolvia: a libera da main puxa numa altura fixa e sobe 2 a 9 mm, e a aba volta para o laço. Com a #104
  esse viés deu 3/5.
- **Eixo −13 e profundidade +12 saem do zero** (2/5 cada) pela primeira vez. Na main a captura apoia o degrau no fundo do
  olhal (O2), e o destrava desce só 20 mm; na #104 a captura mira o centro do furo e a encostada da F5 vem antes de
  descer. Com a parede deslocada, o procedimento mais longo da #104 tem mais etapas para errar.

## Onde falhou (15 abortos)

| falha | execuções |
|---|---|
| atravessa sem fechar | 8 (prof +12 ×3, alt +10 ×3, eixo +8 ×2, controle) |
| libera "subiu" | 4 (altura +10) |
| perdeu o olhal (libera, arco1, arco2) | 3 |
| captura sem fechar | 2 |
| chave não abriu | 1 |

A atravessa sem fechar é agora a falha mais comum. A saída final do furo (`saida_sobe`) não fecha em 5 iterações em
quase todas as missões, inclusive nas que deram certo, sem consequência.

## Próximo passo sugerido

Trazer para a main só a **libera que puxa sem subir** (um pedaço pequeno da #104), mantendo a captura e o destrava atuais,
e medir de novo. Se a altura +10 voltar a 3/5 sem perder o resto, o total passa de 12/25.
