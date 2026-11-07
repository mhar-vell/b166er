# RELATORIO57 — bateria com o viés de profundidade de +6 mm — 06 Nov 2026

Marco: "pode seguir com o viés de +6 mm". Decisão sobre o RELATORIO56: o viés de teste de profundidade passa de +12 para
+6 mm, dentro da folga geométrica do dedo (±9 mm). Bateria `vies_prof6` na main 1514538e (até a PR #121), sem mudança de
código.

## Resultado (5 vieses × 5): 22/25

| viés | vies_prof6 | laser_angulo (mesma main, profundidade +12) |
|---|---|---|
| controle | 5/5 | 4/5 |
| eixo −13 | 5/5 | 4/5 |
| eixo +8 | 5/5 | 5/5 |
| **profundidade +6** | **5/5** | 2/5 (com +12) |
| altura +10 | 2/5 | 2/5 |
| **total** | **22/25** | **17/25** |

- **Profundidade +6: 5/5.** Pela sonda, a ponta chegou ao fim da aproximação lateral a 1,4 a 6,5 mm do centro do furo em profundidade, dentro da folga. Nenhuma lâmina abriu antes do destrava nesse viés.
- **Percepção:** refinamento com tag em 25/25, nenhuma aproximação sem tag, e a busca do ângulo pelo laser disparou em 6 missões.
- **Nenhum alarme de subida na libera.**

## As três falhas, todas com altura +10

| execução | falha |
|---|---|
| 1 | passo intermediário sem fechar, depois perdeu o olhal no segundo arco por deriva de −17,6 mm ao longo do eixo |
| 2 | atravessa sem fechar, com a lâmina aberta antes do destrava |
| 3 | o segundo arco não fechou em 40 s (melhor resíduo 19,4 mm) |

## Conclusão

Com o viés de profundidade dentro da folga do dedo, esse viés passa a 5/5, e controle, eixo −13 e eixo +8 também
fecharam todas. O total, 22/25, empata com o melhor das baterias (busca_haste). A altura +10 concentra agora as falhas,
com 2/5 nesta bateria e na laser_angulo, e é o próximo ponto, especialmente o segundo arco, que aparece em duas das três.
