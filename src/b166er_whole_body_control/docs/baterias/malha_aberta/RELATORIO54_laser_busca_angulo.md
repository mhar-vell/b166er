# RELATORIO54 — perda da tag na aproximação: o laser procura o ângulo da parede — 05 Nov 2026

Marco: "pode seguir com a perda da tag na aproximação". Branch `feat/laser-busca-angulo-da-parede` (commit d46ad197), a
partir da main com a busca da haste (PR #118). O stack foi reiniciado com o firmware da main antes desta bateria.

## Diagnóstico (baterias `busca_haste` e `parada_no_eixo`)

Nas missões que perderam a tag, a busca inicial, a uns 2,2 m da parede, deu o ângulo da parede 25 a 32° errado: 2,58 a
2,72 rad, quando o certo é cerca de 3,14 rad. É a ambiguidade do PnP de longe. O laser, que deveria corrigir o ângulo, só
aceita pontos numa faixa de 0,15 m em torno do plano estimado pela tag e recusa correções de mais de 0,10 rad, cerca de
5,7°. Com 30° de erro, a faixa pegava de 1 a 13 pontos, abaixo do mínimo de 30, e o ângulo errado ficava valendo. A
aproximação ia a 1,3 m virada para o lado errado e perdia a tag. Nas duas baterias isso aconteceu 14 vezes: o giro de
recuperação reencontrou a tag em 12 e 2 missões abortaram.

## A mudança

Quando a faixa da tag tem pontos de menos, o laser procura ele mesmo a direção da parede (`_laser_busca_angulo`). Ele
gira a normal da tag de −35° a +35° em passos de 1°, em torno da posição da tag. Para cada ângulo, monta um histograma
das distâncias dos pontos à reta, em faixas de 2 cm até ±0,40 m, deixando de fora a região da chave. Fica com a faixa mais
povoada, desde que ela tenha o mínimo de pontos. Depois vale o ajuste de sempre, com as mesmas recusas por resíduo, e
limites mais largos só nesse caso. A posição lateral continua vindo da tag. Os parâmetros são `~laser_busca_angulo_rad`
(35°, e 0 desliga) e `~laser_busca_faixa_m` (0,40).

## Bateria `laser_angulo` (5 vieses × 5): 17/25

| percepção na aproximação | laser_angulo | busca_haste (main) | parada_no_eixo |
|---|---|---|---|
| busca inicial com o laser recusando por falta de pontos | **0** | 7 | 8 |
| aproximação sem nenhuma amostra da tag | **0** | 6 | 8 |
| missão abortada por perder a tag | **0** | 1 | 1 |
| refinamento com tag | 25/25 | 24/25 | 24/25 |

- **A busca do ângulo disparou em 15 das 25 missões.** O ângulo da tag estava sempre errado para o mesmo lado, de +23 a +32°, e o laser encontrou a parede com 60 a 71 pontos na reta. Na primeira missão, a correção levou a estimativa a yaw −3,14 rad e profundidade 3,000 m, que é a verdade da simulação.

| viés | laser_angulo | main (RELATORIO51) |
|---|---|---|
| controle | 4/5 | 5/5 |
| eixo −13 | 4/5 | 5/5 |
| eixo +8 | **5/5** | 4/5 |
| profundidade +12 | 2/5 | 4/5 |
| altura +10 | 2/5 | 4/5 |
| **total** | **17/25** | **22/25** |

**As 8 falhas aconteceram todas depois da aproximação,** em fases que a mudança não toca: três subidas na libera, três
atravessas sem fechar (duas na atravessa e uma no passo intermediário), uma captura em que a chave não abriu e uma no
segundo arco. A taxa de sucesso foi a mesma com e sem a busca disparada: 10 em 15 contra 7 em 10. As poses de chegada da
aproximação também foram as mesmas, (0,37; 1,99; 1,57 rad) na maioria. A variação do total entre baterias com a mesma
base tem sido grande: 22, 19, 15 e 17 nas quatro últimas.

## Conclusão

A mudança resolve o que se propôs: o laser passou a corrigir o ângulo da parede de longe, e a aproximação deixou de
perder a tag (0 contra 6 a 8 por bateria). O total ficou abaixo da última bateria da main, mas as falhas estão em fases
posteriores e não dependem de a busca ter disparado. **Recomendo mergear.** A falha mais frequente agora é a subida na
libera (3), seguida da atravessa sem fechar (3).
