# RELATORIO55 — subida na libera: alarme com confirmação — 06 Nov 2026

Marco: "pode seguir com a subida na libera". Branch `feat/libera-subida-confirmada` (commit b5160dc3), a partir da main
com a busca do ângulo pelo laser (PR #121).

## Diagnóstico (alarmes de subida de todas as baterias, classificados pela sonda)

O alarme "ponta SUBIU", de 8 mm em 0,3 s na ponta estimada (RELATORIO32, 17 Out), dispara num momento normal da missão.
Depois do destrava, a mola da lingueta devolve o olhal para cima, e ele encosta na face de cima do degrau, a F1, levando a
ponta junto. Pela sonda, a ponta sobe de 1,7 a 9,4 mm na soltura do gatilho em missões que abriram a chave.

Classificando os 15 alarmes de 8 a 12 mm pela lingueta, que só existe na simulação:
- **8 eram a chave abrindo:** a lingueta segurava entre 5 e 16 mm, e a lâmina girava de 5 a 8°.
- **7 eram perdas reais:** a lingueta voltava a zero.

O tamanho da subida no alarme não separa os dois casos, e o reassentamento que vem depois salvou só 3 das 21 missões com
alarme. O que mais separa é a subida seguir ou não. Nos falsos alarmes, ela parou entre 7 e 12 mm em todos, menos um. Nas
perdas, foi de 16 a 40 mm em todas, menos uma.

## A mudança

O alarme passa a abrir uma janela de confirmação de 0,5 s (`~libera_subida_confirma_s`), com a libera seguindo. Só conta
como perda se a subida total desde o início passar de 14 mm (`~libera_subida_total_m`). Se parar antes disso, a libera
continua. O critério usa só a ponta medida pela T265.

## Bateria `subida_confirmada` (5 vieses × 5): 19/25

| viés | subida confirmada | main (RELATORIO54, laser_angulo) |
|---|---|---|
| controle | 5/5 | 4/5 |
| eixo −13 | 5/5 | 4/5 |
| eixo +8 | 5/5 | 5/5 |
| profundidade +12 | 1/5 | 2/5 |
| altura +10 | 3/5 | 2/5 |
| **total** | **19/25** | **17/25** |

| alarmes de subida | |
|---|---|
| abertos | 2 |
| liberados como falsos | **0** |
| confirmados como perda | 2, com 19,0 e 19,5 mm |

- **O caminho novo não foi exercitado.** Nenhum alarme foi liberado. Os 2 confirmados teriam abortado também com o alarme antigo.
- **Execução 3 de altura +10:** perda real. A lingueta voltou a zero e a ponta subiu mais de 37 mm.
- **Execução 1 de altura +10:** a lingueta segurou em 15 mm, mas a ponta subiu 20 mm pela sonda, porque o dedo saiu de cima do arame com o olhal ainda preso. Nem a regra nova nem a lingueta classificam esse caso com clareza.
- **As outras 4 falhas são de profundidade +12 e não passam pelo alarme:** uma chave que não abriu (lâmina em 3,3°), duas perdas do olhal ao longo do eixo na libera (−31 e −32 mm) e uma captura sem fechar.
- Houve só 2 alarmes nesta bateria, contra 3 na laser_angulo e 1 a 2 nas anteriores.

## Conclusão

**Não há evidência de efeito.** O total subiu de 17 para 19, mas nenhum alarme foi liberado, então a diferença não vem da
mudança. **Não recomendo mergear ainda.** A PR fica como rascunho até haver dados do caminho novo. Para exercitá-lo, o
mais direto é uma bateria dirigida só com altura +10, o viés em que os alarmes aparecem, com umas 10 execuções e cerca de
80 minutos. A falha mais frequente desta bateria é a profundidade +12, com 4 falhas em fases variadas.
