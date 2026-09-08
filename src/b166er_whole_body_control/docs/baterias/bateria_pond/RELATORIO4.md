# Bateria: híbrido + ponderação ciente de batente — 2026-09-03 (runs 36–40)

Marco: "implementa a ponderação ciente de batente e roda a bateria".
Chan & Dubey (1995) na variante de ponta do fuzzy_wb_controller: w_i = 1 +
|dH/dq_i| quando a junta vai para o batente (cap 100), nos dois ramos
(base travada / base livre). Stack reiniciado; híbrido por fase; mesma régua.

| run | resultado | lâmina | destrava (wb) | libera | lingueta fim destrava / máx |
|---|---|---|---|---|---|
| 36 | OK | 28,7° | 4,9 s | 4,5 s | 3,3 / 16,2 mm |
| 37 | ABORTED (destrava timeout 40 s) | – | timeout | – | – / 20,2 mm |
| 38 | OK | 29,4° | 2,5 s | 5,9 s | 5,1 / 16,8 mm |
| 39 | OK | 30,7° | 2,8 s | 6,8 s | 3,3 / 15,5 mm |
| 40 | OK | 28,6° | 19,2 s | 9,1 s | **13,9 mm — gatilho solto** / — |

**4/5**, contra 5/5 do híbrido sem os pesos (runs 31–35; destrava 2,3–22,6 s).
Sem ganho mensurável.

## O que a sonda diz

- Os pesos atuaram: 48 avisos, J4 com peso 100 (às vezes J2 ~2).
- Na run37 (timeout, 45 s de destrava) o J4 ainda ficou no batente de 110°
  em 26 % das amostras e a base avançou 0,759 → 0,629 m a ~7 mm/s. O
  |cmd_v| máximo em TODAS as runs foi 0,010 m/s = exatamente o piso
  MIN_BASE_LIN: a base nunca recebe mais que o piso.
- Explicação: perto do alvo o escalonador Fuzzy está na banda NEAR (k_pos
  0,1–0,35); com erro de ~1 cm a velocidade cartesiana pedida é de 1–4 mm/s;
  a fração da base, mesmo com o J4 caro, fica abaixo do piso. O gargalo é o
  ganho na banda NEAR somado à fração da base, não só o custo das juntas.

## Decisão

Ponderação fica implementada e DESLIGADA por padrão (`~limit_aware_weights`
false) até haver evidência de ganho. Próximo passo com mais chance: piso de
velocidade da base maior e na direção do resíduo quando uma junta está no
batente, ou banda NEAR do k_pos menos conservadora na variante de ponta.
