# Saída do ABORT depois de um timeout no destrava — 2026-09-09

Marco: "segue com o retorno do ABORT após timeout no destrava".
Pendência aberta desde o E5 do roteiro de bancada (madrugada de 09 Set),
em que o destrava estourou 40 s e o ABORT_SAFE fez uma saída ruim.

## 1. O que aconteceu no E5

`_sai_pelo_eixo` mirava um alvo único a 120 mm pelo eixo do furo, na
mesma altura da ponta. Depois de 40 s empurrando (J2/J3 saturados, J4
retro-acionado), a primeira iteração de IK saiu do ramo:

| it | ponta (x, y, z) | erro | IK pediu |
|---|---|---|---|
| 0 | (0,371, **2,991**, **0,616**) | 0,209 m | J3 0°, J4 95° |
| 1 | (0,336, 2,924, 0,759) | 0,066 | — |
| 2 | (0,304, 2,865, 0,762) | 0,027 | — |
| 3 | (0,381, 2,912, 0,750) | 0,078 | — |

A ponta foi 20 cm para baixo e para dentro do plano da parede (y 2,99,
placa em 2,92), cinco iterações não fecharam (98 mm) e a base recuou
0,25 m mesmo assim. Com o degrau ainda sob o arame, "pelo eixo" na
mesma altura arrasta o anel.

## 2. Reproduzir: o timeout fresco não reproduz

`bateria_abort_destrava.sh` (missão a partir de REFINE, base a 0,88 m,
`~wb_phase_timeout` curto, `~reassenta_max` 0):

| timeout | o que estourou | saída antiga | erro |
|---|---|---|---|
| 6 s, run1 | libera (o destrava fechou pela estagnação) | 1 iteração | 8,3 mm |
| 6 s, run2 | libera | 1 iteração | 9,4 mm |
| 2 s, run1 | **destrava** (ponta a 24 mm do olhal, ~10 mm descida) | 1 iteração | 8,1 mm |
| 2 s, run2 | destrava | 1 iteração | 8,8 mm |

Com o dedo pressionando o arame mas a postura íntegra, a saída antiga
fecha. O que quebrou a saída no E5 foi a **postura colapsada por 40 s
de empurrão** — que a estagnação curta do RELATORIO14 (PR #64) corta em
4 s. Ou seja: a causa direta já não acontece; o que fica é fazer a
saída certa mesmo quando ela acontecer.

## 3. O que mudou: saída em degraus, na ordem inversa da entrada

A entrada é orienta → aproxima_lateral → atravessa → captura. A saída
de emergência faz o caminho de volta a partir de onde a ponta está:

1. **sobe** `~saida_sobe_m` (30 mm) — descarrega o arame, inverso da captura;
2. **eixo** `~saida_curso` (120 mm) — sai do anel pela direção em que entrou;
3. **fora** `~saida_fora_m` (100 mm em profundidade, para o lado do robô)
   — afasta da placa antes de o stow varrer o punho perto dela.

Cada degrau é pequeno (a IK fica no ramo) e é melhor esforço: se um
falhar, os seguintes ainda tentam, e o recuo da base pelas rodas vem
depois de qualquer jeito. As tolerâncias por eixo dos três degraus
estão no YAML (`saida_sobe`, `saida_eixo`, `saida_fora`); sem elas a
régua era esférica de 20 mm e "subir 30 mm" fechava sem sair do lugar.

## 4. Validação

Mesmo cenário da linha de base de 2 s (destrava estoura com a ponta a
24 mm do olhal, ~10–12 mm de descida), saída em degraus:

| run | sobe 30 mm | eixo 120 mm | fora 100 mm | lâmina depois |
|---|---|---|---|---|
| 1 | 1 it, 7,2 mm | 1 it, 8,5 mm | 1 it, 3,8 mm | — |
| 2 | 1 it, 6,3 mm | 1 it, 7,1 mm | 1 it, 4,4 mm | 0,00° (lingueta em repouso) |

Os três degraus fecham na primeira iteração; a ponta sai do anel
subindo, depois pelo eixo, depois para fora, e a chave fica como
estava. O recuo da base (0,25 m) e o stow seguem como antes.

Regressão do caminho de sucesso (a saída também é chamada no RETRACT,
depois do desengata): 2/2 MISSION_OK, os três degraus fechando na
primeira iteração (run1: eixo 9,4 mm, fora 5,9 mm), ~6 s a mais no fim
da missão.

## 5. O que fica

- A causa direta da saída ruim do E5 (postura colapsada por 40 s de
  empurrão) já não acontece com a estagnação curta; a saída em degraus
  é a segunda barreira, e a que o operador esperaria ver.
- Na bancada, se um degrau não fechar, o log diz qual; o recuo da base
  vem depois de qualquer jeito — com a parede atrás livre, como o plano
  já pede para o ABORT.
