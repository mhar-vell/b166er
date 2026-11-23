# RELATORIO53 — salto do ombro na chegada à haste: folga dinâmica do J1 no firmware simulado — 05 Nov 2026

Marco: "pode seguir com o salto do ombro na chegada à haste". Branch `fix/firmware-sim-j1-sem-divida` (commit 592b0326),
a partir da main com a busca da haste (PR #118). O stack foi reiniciado para carregar o firmware.

## Diagnóstico (sonda da execução 3 de eixo +8 da bateria `mira_limitada`)

Durante o passo intermediário, com a haste já no arame, o J1 estava em −9,3°. Em 0,6 s ele foi a −4,5° sem nenhum
comando novo desse tamanho: a IK seguinte pediu −8,6°. São uns 60 mm de ponta. O RELATORIO41 já tinha registrado um pedido
de 0,6° virando um giro de 4 a 5°.

No firmware simulado (`arm_openloop_sim`, saída em posição), o ângulo de comando integra a velocidade pedida e pode
correr até `folga_deg` à frente do ângulo real. Essa folga existe para cobrir a queda por gravidade do controlador de
posição no J2 e no J3. O J1 gira em torno do eixo vertical e não tem essa queda, mas tinha 4° de folga. Com a haste
bloqueada, o comando corria 4° à frente, e o P de 100 N·m/rad apertava cerca de 7 N·m contra o arame. Quando o contato
escorregava, a junta disparava de uma vez. Isso batia com o tamanho dos saltos medidos.

## A mudança

No J1, a folga passou a ser o atraso dinâmico do PID na velocidade pedida, |v|·d/p, mais 0,5°, sem passar dos 4°
(`~folga_dinamica_juntas`, `~folga_dinamica_margem_deg`). A ideia era que o motor real, comandado por PWM de velocidade,
não guarda posição.

## Bateria `j1_sem_divida` (5 vieses × 5): 15/25

| viés | folga dinâmica | main (RELATORIO51) |
|---|---|---|
| controle | 2/5 | 5/5 |
| eixo −13 | 4/5 | 5/5 |
| eixo +8 | 3/5 | 4/5 |
| profundidade +12 | 3/5 | 4/5 |
| altura +10 | 3/5 | 4/5 |
| **total** | **15/25** | **22/25** |

| | folga dinâmica | busca_haste (main) |
|---|---|---|
| missões com o J1 girando mais de 3° em menos de 1 s, pela sonda | 2 | 3 |
| atravessas com salto de mais de 30 mm | 3/25 | 2/24 |
| desengate que não fechou em 40 s | **4** | 0 |

- **Os saltos não diminuíram.** O J1 ainda gira mais de 3° em menos de 1 s em 2 missões, contra 3 na main. A dívida de posição explica parte do fenômeno, mas não todo. O resto vem do contato no simulador, com a haste escorregando sobre o arame, e da ação dos outros eixos.
- **A mudança quebrou o desengate.** Em 4 missões, três de controle e uma de eixo +8, a última fase não fechou em 40 s, depois de a chave já abrir. No firmware simulado, a força do J1 é proporcional à dívida de posição. Ao limitar a dívida, o ombro fica sem força para tirar o dedo do anel com atrito. O motor real dá torque de travamento independentemente da posição, e é exatamente isso que essa emulação não representa.
- **Outras falhas:** três atravessas sem fechar com saltos; uma captura sem fechar; um destrava no limite seguido de subida na libera; e duas perdas da tag na aproximação, que depois falharam na atravessa.

## Conclusão

**Não recomendo mergear.** A folga dinâmica tira força do J1 justamente quando ele precisa, e não acaba com os saltos.
O stack foi reiniciado com o firmware da main ao final desta bateria.

Uma emulação mais fiel do motor separaria força de posição. O controlador aplicaria esforço limitado, como o torque de
travamento proporcional ao PWM, em vez de perseguir um ângulo que acumula. Isso mexe no núcleo do modo malha aberta e
pede uma decisão do Marco antes de seguir.
