# RELATORIO46 — atravessa em passos — 2026-10-31

Marco: "pode seguir com a entrada em passos menores". Branch `feat/atravessa-em-passos` (commit 424573c9), a partir da
main de 30 Out (dedo v4, libera sem subir, olhal do lab, servo sem tremor).

## A mudança

Antes de ir ao furo (X −10 mm), a ponta para em X −60 mm e em X −35 mm, na profundidade e altura da atravessa
(`~atravessa_passos_m`, tolerância 12/8/6 mm). Cada movimento do J1 fica em 25 a 35 mm, em vez de 80 mm de uma vez. Um
passo que não fecha não aborta: a missão segue para o próximo. Motivo (RELATORIO45): o salto da atravessa acontecia na
primeira iteração, quando o J1 em malha aberta recebia os 80 mm de uma vez.

## Bateria `atravessa_passos` (5 vieses × 5): 16/25

| viés | em passos | main, degrau 30 mm (RELATORIO44) |
|---|---|---|
| controle | 5/5 | 5/5 |
| eixo −13 | 3/5 | 3/5 |
| eixo +8 | 4/5 | 4/5 |
| profundidade +12 | **3/5** | 2/5 |
| altura +10 | 1/5 | 3/5 |
| **total** | **16/25** | **17/25** |

**A atravessa sem fechar caiu de 6 para 3** (profundidade +12 ×2, altura +10), que era o alvo. Os passos intermediários
não fecharam 11 vezes, sem abortar a missão, como previsto.

Onde falhou (9 abortos):

| falha | execuções |
|---|---|
| atravessa sem fechar | 3 |
| perdeu o olhal na libera | 3 |
| captura sem fechar | 2 |
| aproximação lateral sem fechar, depois a libera subiu | 1 |
| chave não abriu (limite de descida) | 1 |

## Conclusão

- A falha que motivou a mudança caiu pela metade (6 → 3), e quatro dos cinco vieses ficaram iguais ou melhores.
- A altura +10 caiu de 3/5 para 1/5, com falhas na libera e na captura, que não têm relação direta com a entrada em
  passos. Com cinco execuções por viés, uma diferença de duas está dentro da variação.
- No total, 16/25 contra 17/25: empate.
- **Recomendo mergear**: resolve o defeito que tinha sido medido, sem prejuízo mensurável. A próxima falha mais frequente
  é a perda do olhal na libera.
