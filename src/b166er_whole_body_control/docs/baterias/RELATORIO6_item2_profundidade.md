# Item 2 — profundidade no whole-body: o que era, o que não era — 2026-09-03

Marco: "investigar profundidade no whole-body, keepout".

## Cronologia das hipóteses
1. **Keepout** (plano a 0,55 m): descartado pela sonda — base nunca < 0,64 m.
2. **J4 no batente + base pesada (12×) no piso de 10 mm/s** (sonda da run33):
   base avança 11 cm, braço recua 6 cm. → Chan & Dubey (pesos das juntas):
   4/5, sem ganho. → Base barata quando junta no batente: nunca disparou
   (51–55: 2/5; 56–60: 4/5), mesmo com margem 5° e comando integrado.
3. **Diagnóstico (run61/62)**: o controlador manda o J4 DESCER (q̇ < 0,
   setpoint 79,5 → 65,8°) e o J4 real SOBE até 110° com esforço saturado em
   −20 N·m (limite), J2/J3 a −20…−23 N·m, (pos − setpoint) do J4 = 45°.
   **O punho é retro-acionado pela força de contato**: o destrava empurra o
   olhal contra o fim de curso da lingueta com controle de posição, o
   braço acumula força até o limite de esforço e a junta mais fraca (J4,
   20 N·m, P=100) cede. A profundidade "foge" porque o punho colapsa.
   É artefato do braço simulado (o RV-M2 não retro-aciona), mas a causa —
   empurrar contra um batente com controle de posição — valeria na bancada.

## Remédio e resultado
- **Estagnação da descida** (fase fecha quando a descida medida para de
  crescer): 1,5 s / 0,5 mm / mín 10 mm → bateria 63–67 **5/5**, destrava
  ~4 s, esforço do J4 2–3 N·m (punho íntegro). Custo: fecha com o anel em
  2–9 mm; a libera termina a soltura.
- Janela 3 s / 0,3 mm / mín 15 mm (run68): **punho colapsou de novo** (J4
  saturado 60 %, 43° atrás) — com o anel no fim de curso a ponta continua
  descendo pelo colapso do J4, a estagnação nunca aparece; robô tombou.
  Revertido para a janela curta.

## Resumo das baterias do dia (mesmo fixture com gatilho)
| config | runs | OK | destrava | lingueta fim destrava | punho |
|---|---|---|---|---|---|
| IK pura | 19–20 | 2/2 | 1,5 s | ~5 mm | – |
| wb esfera 20 mm | 21–25 | 5/5 | ~1 s | 0–7 | – |
| wb régua por eixo | 26–30 | 1/5 | timeouts | – | colapso (não medido) |
| híbrido | 31–35 | 5/5 | 2–23 s | 4–7,5 | colapso parcial |
| híbrido + pesos Chan-Dubey | 36–40 | 4/5 | 2,5–19 s | 3–14 | – |
| híbrido + critério observável 13 mm | 41–45 | 4/5 | 3,6–20 s | 8–17 | – |
| híbrido + critério 18 mm | 46–50 | 5/5 | 4–38 s | **16–20** | colapso (J4 a 110°) |
| + base barata (não disparou) | 51–55, 56–60 | 2/5, 4/5 | … | … | colapso |
| + estagnação curta | 63–67 | 5/5 | ~4 s | 2,5–9 | **íntegro** |
| + estagnação longa | 68 | 0/1 | timeout | – | colapso, robô tombou |

## Próximo passo (decisão do Marco)
Modelar o punho simulado com a rigidez/torque do RV-M2 real (J4 hoje: 20
N·m, P=100). Com um punho que não cede, o critério de 18 mm (46–50) dá
lingueta 16–20 sem colapso — é a combinação que a bancada deve reproduzir.
