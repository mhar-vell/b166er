# RELATORIO28 — standoff 0,68 m e arco deslizante — 2026-10-15

Continuação do RELATORIO27 (limites do Cerbaro). Branch `feat/standoff-limites-reais`.

## 1. Punho no switch no pré-engate → standoff 0,68 m

Pendência do RELATORIO27: o DEPLOY pré-posicionava com o J4 em −98,5° (no
switch) e resíduo 0,9–4,4 mm. Varredura por IK (`scratchpad/deploy_ik2.py`,
`deploy_prof.py`, `standoff_fases.py`, `standoff_q.py`):

- O ponto de pré-engate (15 cm à frente do olhal) fica PERTO DEMAIS DA BASE:
  para alcançá-lo com a ferramenta horizontal o punho precisa de mais de
  98,5° em qualquer standoff. A 10 cm sobra 4° com a base a 0,68 m.
- Nas fases de manipulação a folga do J4 cresce linearmente com a distância
  da base ao olhal: 6° em 0,62 m, 10,7° em 0,64, 15,5° em 0,66, 20,5° em
  0,68, 25,7° em 0,70 (resíduo ≤ 0,2 mm, degrau ≤ 4,7° em todas). O J3 fica
  a ~6° de zero (zona da barreira) em todas: o cotovelo quase reto é a
  geometria da tarefa com os limites reais.
- O "degrau de 0,66" que levou o standoff a 0,62 em 2026-09-01 era do
  modelo antigo (J3 ±60°).

Aplicado: `standoff_distance` 0,62 → **0,68 m**; `orienta` prof 0,15 → 0,10.

**Bateria `garra/standoff_068`: 5/5** (lâminas 31,2 / 29,2 / 32,3 / 32,8 /
run5 OK). DEPLOY com resíduo 0,3 mm em J4 −94,9; fases com J4 −53…−62,
J3 −4…−6, destrava por estagnação em 13–17 mm.

## 2. "No segundo arco o movimento começa com um movimento para baixo"

Observação do Marco. Log da run3 de standoff_068: início do arco2 com prof
+7,6 / alt −15,1 mm — o alvo do arco2 é o ponto da circunferência a 30°,
48 mm à frente e 20 mm abaixo do de 15°, o whole-body vai em linha reta, o
braço resolve a altura num ciclo e a base leva segundos: a fase começava
empurrando o olhal contra o mecanismo, como no destrava.

**Arco deslizante** (`arco_raio_m` 0,200 e `arco_antecipacao_m` 0,015 nas
fases de arco): a cada ciclo a missão lê a profundidade da ponta pela T265,
toma o ponto da circunferência 15 mm à frente e publica esse ponto com a
altura DO ARCO nele. Régua de fechamento = ponto final. Só o que o robô
real tem (ponta pela T265, raio da lâmina).

Quatro rodadas até fechar — cada uma expôs uma coisa que o recuo com as
rodas escondia:

| rodada | o que apareceu | correção |
|---|---|---|
| v1 | o recuo prévio com as rodas (`pull_with_base`) já fazia o arco na horizontal com o braço parado; a ponta entrava na fase a 27,7° e o whole-body só mergulhava os 10 mm que sobravam | sem recuo prévio nas fases com `arco_raio_m` — o whole-body faz o arco inteiro, base incluída |
| v2 | arco tangente (ponta 16 → 27°, erro sempre prof ≈ 2,6 × alt), mas a régua de profundidade (10 mm ≈ 3°) fecha a 27° e a lâmina recua ~3° no desengate: 25,6° contra o limiar de 25 | fim do arco2 em **35°** (y 114,7, z −61,2 mm); desengata no mesmo ponto |
| v3 | run2 "PERDEU O OLHAL" (deriva +16,8 mm): o alvo dos arcos pedia eixo 0 com a captura a −10; fazendo o arco inteiro o whole-body puxou a ponta 10 mm pelo furo | libera e arcos herdam o eixo da captura (como o destrava desde a v4 do R27) |
| v4 | runs 1–3 OK (deriva 0,3–3 mm); run4 "CHAVE NÃO ABRIU": o desengata por IK re-alinhou o degrau com um giro de punho de 20–25° (J4 −61 → −84) com o dedo no anel e a lâmina voltou a 0° | **desengata em whole-body** (só braço, eixo = captura −90 mm, tol 15/20/20) + continuidade com q_current no espaço nulo da posição da IK (`IK_PESO_CONT` 0,05) |

Causa de fundo da v4, que fica como pendência: as fases whole-body são SÓ DE
POSIÇÃO e deixam a orientação da ferramenta livre durante o arco; a primeira
fase por IK depois delas re-alinha o degrau de uma vez. O whole-body com
orientação (6D com peso angular, como a lei de tarefa do servo) resolveria
na raiz.

**Bateria `garra/arco_deslizante` (v5): 4/5**

| run | arco2 | desengata (wb) | lâmina final | resultado |
|---|---|---|---|---|
| 1 | 34,4° | 3 s | 34,4° | **OK** |
| 2 | 34,4° | 4,1 s | 34,4° | **OK** |
| 3 | — | — | — | ABORT na aproxima_lateral: IK não fechou (22 mm). **Navegação**: no DEPLOY a base "passou do alvo (0,10 m)", realinhou, andou mais 0,30 m e estacionou deslocada (J1 da orienta +3° contra −13…−19° nas outras; âncora corrigiu 10×). Não é o arco. |
| 4 | 34,4° | ~2 s | 34,4° | **OK** |
| 5 | 34,4° | ~3 s (ponta 29 mm abaixo no início, corrigido) | 34,4° | **OK** |

Deriva de eixo nos arcos: 0,3 / 3,0 / — / 0,7 / ~1 mm (guarda 15). O arco2
agora sai tangente à circunferência; a lâmina termina em 34,4° em todas as
que chegaram lá, e o desengate não a arrasta de volta.

## Pendências

1. Overshoot do DEPLOY/aproxima (run3): "passou do alvo (0,10 m)" seguido de
   novo avanço de 0,30 m — investigar o estimador/drive nessa transição.
2. Whole-body com orientação nas fases engatadas (ver causa de fundo da v4).
3. Bancada: standoff 0,68 m, pré-engate a 10 cm, arco pela circunferência e
   desengate sem re-alinhar são os parâmetros a levar.
