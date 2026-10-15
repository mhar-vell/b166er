# RELATORIO29 — o overshoot do DEPLOY/aproxima e o giro que translada — 2026-10-15

Pendência 1 do RELATORIO28: na run3 de `garra/arco_deslizante` a base
"passou do alvo (0,10 m)", realinhou, andou mais 0,30 m e estacionou ~20 cm
fora; a aproxima_lateral abortou com "IK não fechou (22 mm)".

## O que o bag mostra (`return.bag`, odom = `/pioneer3at/odom`, 6043–6078 s)

| t (s) | fase | o que a base fez |
|---|---|---|
| 6045–6047 | TURN_TO (ω 0,4) | girou 97° → 128° e, parada, **transladou ~8 cm** (0,349/1,961 → 0,404/1,946 → 0,400/1,982) |
| 6048–6053 | DRIVE | v 0,15 → 0,04 (frac), ω = 1,2·err saturado em 0,4; rumo 128° → 180°: **órbita** em torno do alvo, passando ao lado a 10 cm |
| 6053 | "passou do alvo" | parou e foi para TURN_TO |
| 6054–6064 | TURN_TO (ω 0,4, v 0) | girou 134° e **transladou 22 cm** (0,183/2,308 → −0,031/2,368) |
| 6065–6068 | DRIVE "0,30 m" | andou até um alvo que já não estava onde ela achava |
| 6068–6075 | TURN_FINAL (ω 0,4, v 0) | girou 100° e **transladou 21 cm** (0,143/2,171 → 0,269/2,001) |

`robot_state.base_odom` = odom em todos os instantes (o estimador copia a
odometria): não é erro de estimativa, é o movimento real do chassi no
Gazebo (odom do plugin skid-steer vem do mundo).

**A base não gira em torno do próprio centro.** Gira em torno de um ponto
~14 cm à frente — o CG do conjunto com o braço (teto por postura: CG
x = 0,135–0,147 m), onde o atrito das rodas se concentra. Corda de 21 cm em
100° → raio ≈ 14 cm. Vale para o Pioneer real com o braço (CG medido a
olho em 01 Out: 0,139 m).

Encadeamento: o giro inicial de 30° deixou ~8–10 cm de erro lateral; o
avanço curto (0,31 m) com ω saturado e v no mínimo orbitou em vez de
corrigir; a "meia-volta" para realinhar custou 22 cm; o giro final mais
21 cm. Nas outras runs o TURN_FINAL é de poucos graus e o efeito some.

## Correção (commit 336577a, `_navigate_to`)

1. **Passar ao lado vale como chegada.** Alvo para trás com erro lateral
   (cross-track) < `nav_cross_tol` (0,12 m): parar e ir ao heading final,
   sem meia-volta. A manipulação tolera isso (estacionamento normal ~2 cm,
   IK fecha com vários cm); a meia-volta custava 2 × 20 cm.
2. **Sem órbita.** Perto do alvo (dist < `nav_slow_dist`) o teto de ω cai
   com a distância, mínimo `nav_w_min_perto` 0,15 rad/s: o avanço vence o
   giro na chegada.

Não corrigido (e anotado): a translação do giro parado em si. Compensá-la
exigiria conhecer o pivô (≈ CG) e prever o deslocamento — na bancada o
primeiro passo é MEDIR quanto o Pioneer com o braço translada num giro de
90° parado.

## Bateria `garra/nav_overshoot`

**5/5** (14:57–15:31, lâmina 34,4° em todas; DEPLOY/aproxima com avanços de
0,36–0,44 m, "chegou" direto nas cinco). O caminho novo ("passou ao lado →
aceito") NÃO foi exercitado: o overshoot é raro (1 em ~15 runs hoje) e a
bateria serve como teste de não-regressão do teto de ω perto do alvo.
Exercitar o ramo de propósito exige uma pose de partida que force erro
lateral no DEPLOY — fica para a bateria de poses (RELATORIO de poses de
partida, `exp_aproximacao`), junto com a medição do pivô na bancada.
