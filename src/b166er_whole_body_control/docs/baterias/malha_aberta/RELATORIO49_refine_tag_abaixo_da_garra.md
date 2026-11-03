# RELATORIO49 — o refinamento não via a tag: a garra a cobria — 03 Nov 2026

Marco: "pode seguir com o último passo da atravessa". Branch `feat/refine-tag-abaixo-da-garra`, a partir da main com a
entrada no centro do furo (PR #115).

## Correção do diagnóstico do RELATORIO48

O RELATORIO48 atribuiu as três "chave não abriu" ao ombro, que não teria executado o último passo da atravessa sem o
estimador perceber. **Estava errado.** Na execução eixo +8 número 3, a ponta estimada e a verdadeira coincidem, em
(0,237; 2,868; 0,800) e (0,2375; 2,8682; 0,7998). O ombro executou o passo e o estimador viu certo. **O alvo é que estava
errado:** a parede estava estimada em x = 0,038 m, contra 0,001 m no controle 1, ou seja, 37 mm fora ao longo do eixo do
furo. É a direção que o laser não corrige. O RELATORIO48 foi corrigido nesta branch.

## Diagnóstico

- **De onde vinha o erro.** O refinamento de perto, feito no ponto de parada, não via a tag em cerca de 4 de cada 5 missões ("só 0 amostras da tag"). A posição ao longo do eixo ficava então com a medida da aproximação, feita de longe.
- **Quanto pesava.** Em 133 missões de sete baterias, as 22 em que o refinamento viu a tag tiveram erro de no máximo 3 mm. As 110 que ficaram com a aproximação tiveram mediana de 5 mm e máximo de 41 mm. Com erro acima de 10 mm, foram 5 sucessos em 16; acima de 20 mm, nenhum em 5. As duas medidas da aproximação com só 5 amostras erraram 40 mm.
- **Por que o refinamento não via a tag.** O registro do localizador mostra detecções e ambiguidades durante a aproximação e silêncio total no ponto de parada. Ou seja, a tag nem era detectada. A sonda de câmera, um quadro da fisheye da T265 durante o refinamento, mostrou o motivo: na postura de busca, com o eixo da câmera horizontal, a tag, 16 cm acima da câmera, cai na faixa de cima do quadro, e a garra, fixa no quadro, cobre a borda de cima dela.

## A mudança

Postura nova `refine: [0, 1,0, −1,0, −0,23, 0]`, a de busca com o punho inclinado 13°, comandada no início do REFINE.
A tag passa a cair no centro do quadro, abaixo da garra. O sinal foi medido na imagem, não pela cinemática direta.
Com +0,23, o chão cresceu no quadro e a tag subiu uns 55 px, em direção à garra. Com −0,23, ela fica no centro, como
mostra a sonda. A pose da câmera usada pelo localizador vem da T265, não das juntas, então inclinar o punho sem encoder
não entra na medida. O mesmo vale na bancada, que usa a mesma câmera.

## Bateria `refine_sem_garra` (5 vieses × 5): 18/25

| viés | refine sem a garra | main (RELATORIO48, entrada_centro2) |
|---|---|---|
| controle | 5/5 | 5/5 |
| eixo −13 | **4/5** | 2/5 |
| eixo +8 | 2/5 | 3/5 |
| profundidade +12 | **4/5** | 2/5 |
| altura +10 | 3/5 | 4/5 |
| **total** | **18/25** | **16/25** |

| | refine sem a garra | main |
|---|---|---|
| refinamento com tag | **25/25** | ~4/25 |
| erro da parede ao longo do eixo, mediana | 2 mm | 4 mm |
| erro da parede ao longo do eixo, máximo | **3 mm** | 38 mm |
| capturas com a ponta fora do anel | **0** | 3 |
| "chave não abriu" | **0** | 3 |

## O que sobrou (7 falhas)

- **Atravessa sem fechar: 4,** sendo três com eixo +8 e uma com profundidade +12. Com o viés de +8, o alvo fica 8 mm além do real ao longo do eixo e a haste encosta no arame antes. O engate por contato deveria fechar esses casos, porque está dentro dos 15 mm que ele aceita, mas não fechou. É o próximo ponto.
- **Libera: 3.** Duas com altura +10 tiveram o destrava parado curto, com 2,7 e 4,7 mm, seguido de reassentamento e subida na libera. A terceira, com eixo −13, perdeu o olhal ao longo do eixo.

## Conclusão

O refinamento agora mede a parede de perto em todas as missões, e o erro ao longo do eixo caiu de até 38 mm para no
máximo 3 mm. As capturas fora do anel e as "chave não abriu" sumiram, e o total foi o melhor das baterias até aqui.
**Recomendo mergear.** O próximo ponto é a atravessa com viés de +8 ao longo do eixo, que deveria fechar por contato.
