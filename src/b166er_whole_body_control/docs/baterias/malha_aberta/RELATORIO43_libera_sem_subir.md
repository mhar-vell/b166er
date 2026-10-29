# RELATORIO43 — libera puxa sem subir, na main — 2026-10-29

Marco: "pode seguir com a libera sem subir na main". Branch `feat/libera-sem-subir` (commit d363582c), a partir da main
com o olhal do lab (#107) e o servo sem tremor (#105).

## A mudança

A libera pedia altura fixa (−25 mm do centro do olhal), mas o destrava termina onde a lingueta chegou ao fim do curso,
de −24 a −36 mm nesta bateria. Puxando em +Y, a ponta subia, a aba voltava para o laço e a libera "subia". Agora a libera
puxa na altura registrada no fim do destrava, também no reassentar (`~libera_mantem_altura`). É o pedaço da PR #104 que
resolvia a altura +10, sem o resto dela. Marco: "para liberar para a abertura tem que permanecer para baixo e puxar a
alavanca".

## Bateria `libera_sem_subir` (5 vieses × 5): 14/25

| viés | libera sem subir | referência `olhal_real` (main) |
|---|---|---|
| controle | 3/5 | 4/5 |
| eixo −13 | 0/5 | 2/5 |
| eixo +8 | **4/5** | 2/5 |
| profundidade +12 | **3/5** | 2/5 |
| altura +10 | **4/5** | 0/5 |
| **total** | **14/25** | **10/25** |

- **Altura +10 vai de 0/5 a 4/5.** Era o alvo: nenhuma libera "subiu" nesse viés; a quinta perdeu o olhal no arco2.
- **Total 14/25**, o melhor até agora.
- **Eixo −13 cai para 0/5**: nas cinco o destrava atingiu o limite de descida (o degrau curto em X, sem o olhal) e a chave
  não abriu. A libera não tem como ajudar quando o destrava não pegou o olhal. É o caso que o degrau de 30 mm deve atacar.

## Onde falhou (11 abortos)

| falha | execuções |
|---|---|
| eixo −13: limite de descida e chave não abriu | 5 |
| atravessa ou captura sem fechar | 3 |
| perdeu o olhal (libera, arco2) | 2 |
| percepção: tag perdida no APPROACH | 1 |

## Conclusão

**Fica.** Recomendo mergear. Próximo passo sugerido: o degrau de 30 mm em cima disto, para o eixo −13.
