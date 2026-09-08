# Bateria whole-body × postura — 2026-09-03

Pedido do Marco: "merge realizado, roda a bateria com use_wholebody:=true".
Mesmo fixture (gatilho físico, PR #42), mesmo YAML, 5 execuções seguidas
(runs 21–25) com `use_wholebody:=true`; comparação com as runs 19 e 20 do
modo postura (IK iterativa), do mesmo dia. Análise: `analisa_bateria.py`.

## Whole-body (Fuzzy, laço de velocidade), runs 21–25

| run | resultado | lâmina | dur | destrava | libera | arco1 | arco2 | lingueta máx (sonda) | lingueta ao fim do destrava | arm_vel_cmd ≠ 0 na manipulação |
|---|---|---|---|---|---|---|---|---|---|---|
| 21 | OK | 29,2° | 154 s | 1,3 s / 17,9 mm | 4,4 s / 18,6 | 0,4 s / 15,8 | 0,4 s / 16,2 | 16,2 mm | 0,0 mm | 413/421 |
| 22 | OK | 30,7° | 144 s | 1,9 s / 15,9 | 6,3 s / 15,0 | 0,4 s / 10,3 | 0,4 s / 13,9 | 16,1 mm | 6,6 mm | 470/479 |
| 23 | OK | 30,3° | 145 s | 0,9 s / 16,8 | 2,6 s / 6,6 | 0,4 s / 11,7 | 0,7 s / 18,0 | 16,1 mm | 1,4 mm | 349/358 |
| 24 | OK | 30,5° | 164 s | 1,0 s / 16,6 | 6,5 s / 13,9 | 0,4 s / 9,6 | 0,4 s / 15,0 | 15,4 mm | 0,0 mm | 437/446 |
| 25 | OK | 29,8° | 150 s | 1,0 s / 17,6 | 3,7 s / 16,1 | 0,6 s / 15,4 | 0,4 s / 17,6 | 16,0 mm | 0,0 mm | 363/372 |

(tempo da fase / resíduo esférico ao fechar; tolerância do modo: 20 mm esféricos, 5 amostras)

## Postura (IK iterativa), runs 18–20

| run | resultado | lâmina | destrava | libera | arco1 | arco2 | lingueta ao fim do destrava |
|---|---|---|---|---|---|---|---|
| 18 | ABORTED em destrava (degrau em cima da barra da lâmina — corrigido antes da 19) | – | FALHOU | – | – | – | – |
| 19 | OK | 29,6° | 1,5 s / 8,8 mm | 5,1 s / 11,5 | 1,4 s / 9,0 | 1,4 s / 9,9 | 4,8 mm |
| 20 | OK | 30,3° | 1,5 s / 7,4 | 6,4 s / 9,6 | 1,5 s / 7,8 | 1,6 s / 8,1 | 5,5 mm |

(tolerância do modo: por eixo no frame da parede, 4–15 mm conforme a fase)

## O que a bateria diz

1. **O Fuzzy moveu o braço.** 97–98 % das amostras de `/b166er/arm_vel_cmd`
   durante a manipulação tinham velocidade não nula. Em 24 Ago esse número
   era 0 de 21 396. O caminho whole-body existe e funciona ponta a ponta:
   5 de 5 missões OK, chave aberta entre 29,2° e 30,7°.
2. **A comparação de resíduos não é justa ainda.** O modo whole-body fecha
   a fase por tolerância ESFÉRICA de 20 mm; o modo postura, por tolerância
   por eixo (4–15 mm). Por isso os resíduos do whole-body são 10–19 mm e os
   da postura 7–12 mm. Para comparar de verdade, `_reach_by_wholebody`
   precisa usar o mesmo `_fase_fechou` por eixo.
3. **A fase `destrava` não destrava sozinha em nenhum dos modos.**
   - Postura: fecha com a lingueta em ~5 mm porque o REFINE estima o olhal
     ~8 mm acima do real (achado de ontem).
   - Whole-body: fecha em ~1 s com a lingueta em 0–7 mm porque o alvo
     (15 mm abaixo da captura) já cabe na esfera de 20 mm — a fase é
     satisfeita antes de empurrar. A lingueta chega a 15–16 mm só depois,
     na `libera`, que empurra para baixo e para fora ao mesmo tempo.
   Nos dois modos é a `libera` que solta o gatilho. A missão abre a chave,
   mas o relato honesto é esse.
4. **arco1/arco2 em 0,4 s** no whole-body: o creep com as rodas antes da
   fase já leva a ponta para dentro da esfera de 20 mm; o braço quase não
   participa dos arcos nesse modo.
5. Orientação da ferramenta não é controlada no modo whole-body (servo só
   de posição) e mesmo assim o degrau atravessou o furo 5 de 5 — a atitude
   herdada da postura de saída bastou. Não contar com isso na bancada.

## Próximos passos sugeridos

- Tolerância por eixo no `_reach_by_wholebody` (mesma régua dos dois modos).
- Critério de `destrava` pela própria lingueta em simulação e por algo
  observável na bancada (deslocamento vertical da ponta medido pela T265
  enquanto a força cresce), em vez de só posição-alvo.
- Repetir a bateria depois disso; só então os números valem para o artigo
  (Seção V-A, "qual modo produziu qual resultado").
