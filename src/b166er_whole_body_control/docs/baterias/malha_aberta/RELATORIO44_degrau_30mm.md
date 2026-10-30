# RELATORIO44 — degrau de 30 mm — 2026-10-30

Marco: "se aumentarmos o tamanho do degrau, isso ajudaria?" e "pode seguir com o degrau de 30 mm". Branch
`exp/degrau-30mm-main`: a main de 29 Out (614faaf8: olhal do lab, servo sem tremor, libera sem subir) mais o degrau de
**30 mm** na colisão do dedo (era 20). A malha visual continua com 20 mm.

## Por que

Na bateria da libera sem subir, o eixo −13 deu 0/5: nas cinco o destrava atingiu o limite de descida sem pegar o olhal. A
sonda das baterias anteriores mostrava a ponta do degrau 2 a 8 mm **antes** do plano do olhal nesses casos. Com 10 mm a
mais de degrau, a ponta passaria do plano.

## Bateria `degrau30` (5 vieses × 5): 17/25

| viés | degrau 30 mm | degrau 20 mm (libera sem subir) |
|---|---|---|
| controle | **5/5** | 3/5 |
| eixo −13 | **3/5** | 0/5 |
| eixo +8 | 4/5 | 4/5 |
| profundidade +12 | 2/5 | 3/5 |
| altura +10 | 3/5 | 4/5 |
| **total** | **17/25** | **14/25** |

- **Eixo −13 de 0/5 para 3/5**: era o caso-alvo. Nenhum dos dois abortos foi pelo limite de descida com a chave sem
  abrir (um na atravessa, um perdendo o olhal na libera).
- **Controle 5/5**, chave aberta a 30,7–33,1°.
- **17/25, o melhor total até agora.** As variações em profundidade +12 e altura +10 (uma execução para menos em cada)
  estão dentro do ruído de cinco execuções.

## Onde falhou (8 abortos)

| falha | execuções |
|---|---|
| atravessa sem fechar | 5 (profundidade +12 ×3, eixo +8, eixo −13, altura +10) |
| destrava não fechou em 40 s, nem depois de reassentar | 1 (altura +10) |
| captura sem fechar e perdeu o olhal na libera | 1 (eixo −13) |

Com o degrau longo, **a atravessa sem fechar passa a ser quase a única falha**.

## Conclusão e o que muda na bancada

- **Recomendo o degrau de 30 mm.** Na simulação ele resolve o caso do degrau curto sem piorar os outros.
- Na bancada isso exige **imprimir uma peça nova** (dedo v4, `gera_dedo.py`), com o degrau de 30 mm e o resto igual à v3.
  Antes de mergear, a malha visual `dedo_fixo_v3.stl` precisa ser regenerada com o degrau longo, para que o desenho e a
  colisão voltem a ser a mesma peça. Decisão do Marco.
- Próximo alvo: a atravessa que não fecha.
