# RELATORIO40 — o servo das juntas parou de caçar o alvo — 2026-10-28

Marco, vendo a simulação: "o robô está com muito shaking". Branch `fix/servo-sem-cacar` (commit 593beacc), empilhada sobre
a PR #104.

## A causa

Ensaio controlado: uma postura 2° ao lado da atual no J1, registrando os comandos do servo e o ângulo verdadeiro.

- Perto do alvo, o servo manda a junta no **piso de 3°/s** (abaixo de ~1,5°/s o motor real não vence o atrito).
- A medida da ponta pela T265 volta ~0,2 s depois. Nesse tempo a junta passa ~0,5° do alvo, ≈5 mm na ponta, mais que a
  tolerância de 3 mm.
- O comando inverteu a cada 0,2 a 0,4 s, e o J1 oscilou ±0,5° por **4 s** até cair na tolerância por acaso.

Com o braço parado, a T265 simulada e as juntas não têm ruído: o tremor não vem do sensor, vem dessa caça ao alvo.

## A mudança

Uma junta movida pelo piso que **cruza o alvo** (o pedido troca de sinal) descansa até a próxima postura. Com todas
paradas e a ponta a menos de `~tol_ee_passo` (8 mm, o menor passo que o braço dá), a postura fecha como "alcançada (dentro
do passo mínimo)", e a iteração da missão mede e corrige o resíduo. `~parar_ao_cruzar` desliga.

Ensaios repetidos (J1 +2° e −2°, J2 +3°, J4 −3°): a postura fecha em 0,8 a 1,0 s, **sem nenhuma inversão**.

## Nas missões (5 controles, `servo_sem_cacar`)

Inversões de sentido por segundo, somando J1 a J4, nos controles antes (`destrava_lateral_mais2`) e depois:

| fase | antes | depois | quem move o braço |
|---|---|---|---|
| orienta | 3,17 | **0,66** | postura (IK) |
| aproxima lateral | 1,68 | 1,15 | postura (IK) |
| atravessa | 1,67 | **0,44** | postura (IK) |
| captura | 1,95 | 1,43 | postura (IK) |
| desengata | 3,34 | **0,88** | whole-body |
| libera | 2,55 | **0,51** | whole-body |
| destrava | 1,56 | 2,07 | whole-body |
| arco1 | 1,19 | 1,35 | whole-body |
| arco2 | 1,25 | 0,65 | whole-body |

O tremor caiu muito nas fases de postura, onde a mudança atua: de 3 para 0,7 inversões por segundo no orienta, de 1,7
para 0,4 na atravessa. Nas fases whole-body o caminho é outro (o controlador manda velocidade direto), e o destrava e o
arco1 não melhoraram. As quedas na libera e no desengata vêm, provavelmente, de chegarem a elas sem a caça das fases
anteriores.

Resultado das missões: **3/5** (chave aberta a 32,4–34,4°), o mesmo 3/5 dos controles antes da mudança. As duas falhas
foram uma captura que não fechou e uma perda do olhal no arco1. Nas três que deram certo a saída final do furo levou
mais iterações (5 sem fechar no `saida_sobe`, sem consequência).

## Conclusão

- **Fica.** O braço para de tremer nas fases de postura sem piorar o resultado.
- Falta tratar o tremor nas fases whole-body (destrava e arco1), que usam outro caminho até o firmware.
