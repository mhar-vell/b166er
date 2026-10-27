# RELATORIO37 — o teto de descida do destrava diz que não engatou; reassentar não resolve — 2026-10-26

Marco: "pode seguir com o item 2" (a libera depois de um engate fora do
centro). Branch `fix/destrava-teto-reassenta`, a partir da main (sem as
mudanças da PR #101).

## Diagnóstico: a perda não é na libera, é no destrava

Nas 45 execuções medidas em 23–24 Out (`medida_101`, `medida_101_eixo`)
e nas baterias anteriores, como o destrava fechou contra o resultado:

| destrava fechou por | execuções | abortaram |
|---|---|---|
| **teto de descida** (22 mm sem o mecanismo parar) | 17 | **15** |
| **estagnação** (fim de curso) | 38 | 5 |

A sonda (verdade do Gazebo) mostra o porquê: nas perdas, a lingueta está
em **0 mm no começo da libera** e o degrau a 27–33 mm do anel. A ponta
desceu sem o anel. A libera puxava assim mesmo e a falha aparecia como
"perdeu o olhal", "subiu" ou "chave não abriu", mas o engate já tinha
sido perdido antes. As duas exceções (teto e OK) foram uma com a
lingueta em 14,8 mm e uma que só passou porque reassentou.

**O sinal é observável na bancada**: é a descida da ponta medida pela
T265, sem precisar da lingueta.

## A mudança

`~teto_reassenta` (True): no destrava, chegar ao teto falha a fase com
`teto`, e o MANIPULATE reassenta como no `subiu`: saída pelo eixo do
furo, reentrada pela aproximação lateral, atravessa, captura e destrava
de novo. Commit 2c6e0b20.

## Bateria `teto_reassenta`: 5 vieses × 5 — 5/25

| viés | acertos |
|---|---|
| controle | 3/5 |
| eixo −13 | 0/5 |
| eixo +8 | 1/5 |
| profundidade +12 | 0/5 |
| altura +10 | 1/5 |

O teto disparou em 9 execuções (14 vezes). **Só 1 de 9 se recuperou**
(alt +10 run 2). Nas outras oito a reentrada repetiu o erro: o teto
chegou de novo ("destrava não fechou de novo") ou a recaptura não
fechou. Faz sentido: a reentrada usa a mesma estimativa da parede que
produziu o primeiro engate fora do anel. As duas falhas do controle não
têm a ver com o teto (atravessa sem fechar com 4 recuos; escorregão na
libera).

## Conclusão

- **O detector é válido**: teto no destrava = o dedo não está segurando o
  anel. Isso fica como conhecimento, e o critério de sucesso antigo
  (teto = "cumprida") estava errado.
- **A recuperação por repetição não funciona** (1/9): reentrar pelo
  mesmo caminho, com a mesma parede, leva ao mesmo lugar.
- **Não recomendo mergear.** A troca é uma perda silenciosa na libera
  por um aborto mais cedo no destrava. O resultado final não melhora e
  o controle caiu para 3/5 (n pequeno, e as falhas não são do teto).
- O que resolveria é usar a informação do teto para corrigir a
  estimativa (o anel não está onde a missão pensa, a ponta passou ao
  lado). Em que direção corrigir depende da geometria do degrau no
  furo, e é o mesmo ponto em aberto do viés −13: precisa do desenho
  com o Marco antes de mais código.
