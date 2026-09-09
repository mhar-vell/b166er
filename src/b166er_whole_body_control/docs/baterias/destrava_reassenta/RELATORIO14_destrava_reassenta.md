# Destrava com a captura descentrada: estagnação curta e reassentamento — 2026-09-09

Marco: "segue com o destrava da captura descentrada". Origem: o ensaio
E5 do roteiro de bancada na madrugada de 09 Set (`bancada.sh E5 --sim`)
abortou no destrava: "whole-body não fechou em 40 s (melhor 0,0142 m)".

## 1. O que aconteceu (rosbag `E5_run1`, `/b166er/mission_status`)

Captura registrada em eixo +12,3 / prof +0,8 / alt +3,5 mm. No destrava:

| t desde o início | descida (mm) | prof (mm) | alt (mm) |
|---|---|---|---|
| 0,5 s | 0,6 | +0,3 | −19,4 |
| 1,5 s | 6,3 | −4,5 | −13,7 |
| 2,0 s | **8,2** | −5,2 | −11,9 |
| 2 → 40 s | 7,8–8,3 (parada) | −5 → **−10** | −12 → −2 |

A ponta desceu 8 mm em dois segundos e **parou**, com a profundidade
escorregando de +0,8 para −10 mm ao longo dos 40 s: a ponta saiu do
arame e ficou empurrando o vazio. O critério de estagnação existente
só aceita a partir de `curso_estagna_m` = 10 mm (e com a profundidade
dentro da régua de 6 mm); o curso mínimo observável (18 mm) nunca
chegou; a fase ficou 40 s no timeout — exatamente o regime que a
sonda do J4 mostrou colapsar o punho (RELATORIO6/11). Numa execução
boa a descida é de 12–18 mm com a lingueta em 6–15 mm; no modelo a
lingueta tem 20 mm de curso, então parar em 8 mm não é fim de curso.

## 2. Reproduzir de propósito: não reproduziu

`chave_mission.launch` ganhou `task_yaml:=` e a missão começou em
REFINE (base a 0,88 m do olhal) com a captura deslocada no YAML:

| deslocamento da captura | captura registrada (eixo / prof / alt) | descida | resultado |
|---|---|---|---|
| eixo +12 mm, run1 | −7,6 / −3,4 / −11,4 | 17,1 mm | OK |
| eixo +12 mm, run2 | −10,1 / −3,3 / −11,4 | 13,7 | OK |
| eixo −12 mm, run1 | −11,8 / −5,2 / −13,8 | 18,6 | OK |
| eixo −12 mm, run2 | −11,2 / −5,3 / −13,9 | 12,9 | OK |
| prof +4 mm, run1 | −7,9 / **−0,1** / −12,9 | 11,6 | OK |
| prof +4 mm, run2 | −8,1 / **+0,4** / −12,6 | 15,8 | OK |
| prof −4 mm, run1 | −7,5 / −5,8 / −10,8 | 17,0 | OK |
| prof −4 mm, run2 | −7,3 / −7,4 / −13,0 | 15,0 | OK |

Duas leituras. O eixo registrado fica em −7..−12 mm seja qual for o
deslocamento pedido: a captura é IK com tolerância de 40 mm no eixo e
não corrige essa direção — o valor vem do atravessa. E nem o eixo nem a
profundidade rasa (o E5 tinha +0,8; aqui +0,4 e −0,1) reproduzem a
saída do arame: 8/8 abriram. O escorregão é um evento raro (1 em ~15
missões de 08–09 Set) que não se controla pelo offset da captura.

## 3. O que mudou

Como não dá para evitar o evento pelo alvo, a missão passa a
**reconhecê-lo em segundos e a se recuperar**, em vez de empurrar 40 s:

1. **Estagnação curta** (laço whole-body): se a descida parou (< 0,5 mm
   em 4 s) tendo descido ≥ 3 mm, e ou está abaixo de `curso_estagna_m`
   ou a profundidade saiu da régua, a fase devolve `falha_fase =
   'estagnou_curto'`. Antes, esse caso só terminava por timeout.
2. **Reassentar** (MANIPULATE): para o destrava com essa flag, sobe
   `~reassenta_sobe_m` (12 mm) acima da captura por IK, refaz a captura
   por IK (o degrau volta a assentar no arame e o zero da descida é
   registrado de novo) e tenta o destrava outra vez, até
   `~reassenta_max` (1) vezes. É o que um operador faria: levanta, encaixa
   de novo, empurra.
3. `~ensaio_forca_reassenta` (só ensaio) força a primeira estagnação
   curta ao passar de 3 mm, para exercitar o caminho sem depender de a
   ponta escorregar de verdade.

## 4. Validação

Caminho forçado (`bateria_reassenta.sh`, `~ensaio_forca_reassenta`,
missão a partir de REFINE a 0,88 m):

| run | estagnação forçada em | reassenta | recaptura registrada (eixo / prof / alt) | descida depois | resultado |
|---|---|---|---|---|---|
| 1 | 3,2 mm | sobe 12 mm, recaptura | −6,9 / −4,0 / −12,8 | 15,5 mm | OK, chave aberta |
| 2 | 3,0 mm | sobe 12 mm, recaptura | −5,7 / −3,5 / −13,2 | 17,7 mm | OK, chave aberta |

A sequência inteira roda em ~8 s (subir, recapturar por IK, registrar,
descer), contra os 40 s que o timeout custava.

Sem forçar, a regra de estagnação curta esteve ativa nas 3 últimas
missões da bateria de profundidade (11,6–17,0 mm de descida) sem
disparar — a descida normal não para 4 s antes do mínimo.
Regressão com o YAML padrão, a partir de REFINE (`bateria_descentrada.sh
normal`): 2/2 OK, descida 13,6 e 15,5 mm, sem disparo da estagnação
curta. Somando as baterias do dia com a regra ativa: 7 missões, 0
disparos indevidos.

## 5. O que fica

- O escorregão real não foi reproduzido; a correção é de recuperação,
  não de prevenção. Se a bancada mostrar o mesmo (a T265 vê a ponta
  parar cedo e a profundidade fugir), o reassentamento é o que o
  operador faria à mão.
- `curso_estagna_m` (10 mm) fica como está: baixá-lo fecharia o destrava
  com a lingueta em 3–5 mm e entregaria à libera um gatilho preso.
- A captura não corrige o eixo (tolerância 40 mm); o valor registrado
  (−7..−12 mm) vem do atravessa. Não é problema para o furo de 40 mm,
  mas é bom saber ao ler `captura: ponta registrada`.
