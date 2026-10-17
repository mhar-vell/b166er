# RELATORIO32 — sonda do engate: como o dedo perde o anel — 2026-10-16

Pendência 2 de 16 Out: nas poses laterais da bateria de poses o dedo
perdeu o anel em 1 de 8 runs (RELATORIO31), sem correlação com o ângulo do
degrau, a postura do braço ou a altura da captura. Marco: "pode seguir com
o item 2" → instrumentar o contato.

## A sonda (`docs/baterias/engate/`, só simulação)

- `sonda_engate.py`: a 20 Hz, a ponta da ferramenta (T265 ⊕ tooltip, o que o
  controle vê) contra o centro do anel pela VERDADE do Gazebo (ângulo da
  lâmina `chave_blade_joint`, pivô em (0,200, 2,870, 0,605), raio 200 mm),
  nos eixos do mundo (eixo = x, prof = −y, alt = z), mais a lingueta, a
  fase e as juntas estimadas. CSV por run.
- `filtra_contatos.awk` sobre `gz topic -e /gazebo/default/physics/contacts`:
  só os contatos em que uma colisão é da ferramenta (tool_tip = degrau,
  tool_rod = haste), com |F| e posição. O Gazebo agrupa as colisões dos
  links fixos: o olhal aparece como `chave_lingueta…lump__chave_olhal_link`
  e o dedo como `L5…lump__tool_tip`.
- `analisa_engate.py`: da captura ao fim, a 0,5 s, com a janela da perda a
  20 Hz.
- Gancho em `run_garra.sh` (`SONDA=1`). Bateria `docs/baterias/engate/esq/run1..4`
  (pose esq, a que mais perdia).

Viés conhecido desta primeira sonda: ela rodou sem `B166ER_GARRA_DX=0`,
então a tooltip da cinemática ficou 30 mm deslocada pelo eixo (GARRA_DX
padrão 0,03). Os números de eixo abaixo já estão corrigidos (−30 mm); as
diferenças entre runs não dependem do viés.

## O que uma run boa mostra (esq run1)

Da captura ao arco2, com o contato sempre olhal × tool_tip (40–190 N):

| fase | eixo (anel → ponta) | prof | alt (ponta − anel) | lingueta |
|---|---|---|---|---|
| captura | 9–10 mm | −4 | −12 | 6,6 |
| destrava | 9 | −4…0 | −25…−29 | 20 (solto) |
| libera | 10 | +2…+6 | −30 | 20 |
| arco1/arco2 | 10–11 | −3…−13 | −27…−30 | 20 |

O degrau tem 20 mm atravessados no furo (a partir da ponta da haste, para
o anel): o arame fica a ~10 mm da quina haste–degrau, no meio do degrau,
e lá fica a missão inteira.

## O que a perda mostra (esq run2, abortada no arco2 por "PERDEU O OLHAL")

Captura com o arame a **12–13 mm** da quina (3 mm mais para a ponta do
degrau que na run boa), dentro da tolerância de eixo da captura (±8 mm).
Destrava igual. **Libera, lâmina a 7,4°, em 0,35 s (26825,85–26826,19 s):**

| t (s) | eixo | alt | lingueta | contato |
|---|---|---|---|---|
| 25,79 | 12,1 | −27,6 | 20,0 | olhal × tip 29–37 N |
| 25,94 | 9,1 | −27,5 | 17,2 | |
| 26,00 | 9,2 | −25,1 | 13,5 | |
| 26,09 | 9,3 | −18,5 | 7,4 | |
| 26,19 | 9,4 | **−13,7** | **5,9** | olhal × tip 54 **+ olhal × rod 37** |

O arame corre 3 mm pelo degrau em direção à quina, a lingueta volta de
20 para 6 mm (a mola devolve o anel), o anel **sobe 14 mm e leva o dedo
junto**: o arame passou por cima do degrau. Dali em diante o contato
inclui a HASTE (olhal × tool_rod): o anel está apoiado na haste, não
pendurado no degrau. O whole-body volta a pressionar (alt −20…−27,
lingueta 12–19 mm, nunca mais os 20) e o arco1 "fecha" com deriva de eixo
−10,6; no arco2 o anel desliza pela haste (eixo 13 → 52 mm em 1,5 s) e
cai da ponta — a lingueta vai a 0 (gatilho rearmado), a lâmina para em
23,1°, "PERDEU O OLHAL −47 mm".

Ou seja: **a perda não é no arco, é um escorregão de 14 mm na libera**, a
10 mm de lâmina aberta, e o arco só revela o estrago.

## Hipótese e o que medir na bancada

O degrau tem 10 mm de altura e o arame 6 mm de bitola; a libera puxa o
anel 30 mm para fora com a ponta pressionando −25 mm. A mola da lingueta
(200 N/m × 20 mm) empurra o anel para cima o tempo todo. Se a ponta se
adianta ao anel (prof +6…+9 mm na run ruim contra +2…+6 na boa) a face
do degrau que segura o arame deixa de ser a face vertical e passa a ser o
canto; com o arame 3 mm mais perto da ponta do degrau, ele passa por
cima. Na bancada: filmar a libera de perfil (câmera na altura do olhal) e
medir quanto o anel sobe quando a lingueta é liberada.

## Propostas (decisão do Marco, nenhuma aplicada)

1. **Detectar o escorregão pela T265 e reassentar**: subida da ponta
   > 8 mm em < 0,3 s durante a libera, com a lingueta ainda a soltar
   (na bancada: sem lingueta, só a subida) → `falha_fase = 'preso'` e o
   reassentar que já existe (sobe, recaptura, destrava de novo). É o que
   o robô real pode ver.
2. **Inserção mais funda e tolerância de eixo mais apertada na captura**
   (−8 ± 8 → −10 ± 4 mm): põe o arame no meio do degrau em todas as runs.
   Custo: a captura pode exigir mais iterações.
3. Libera mais lenta ou com a ponta um pouco mais baixa (alt −30 em vez de
   −25) durante o puxão, para o arame não subir o canto.

## Bateria com a sonda (esq × 4)

| run | captura: arame a (mm da quina) | resultado | o que a sonda viu |
|---|---|---|---|
| 1 | ~10 | **OK** | arame no meio do degrau a missão inteira; contato só olhal × tip |
| 2 | 12–13 | ABORT (arco2, −47 mm) | **fim da libera, lâmina 7,4°**: arame corre 3 mm para a quina, lingueta 20 → 6 mm, anel sobe 14 mm, contato vira olhal × rod; arco1 "solta" com deriva −10,6; arco2 o anel desliza pela haste e cai |
| 3 | ~10 | **OK** | igual à run1 (deriva +1,2 no arco1) |
| 4 | 12–13 | ABORT (arco2, −37 mm) | **fim da libera, lâmina 7,6°**: lingueta 20 → 9, alt −29 → −16, olhal × rod; deriva −7,5 no arco1; mesma sequência da run2 |

**2/4, e as duas perdas são o mesmo evento, no mesmo instante** (o fim do
puxão da libera, lâmina a ~7,5°), com a mesma condição inicial: na
captura o arame ficou 2–3 mm mais perto da ponta do degrau do que nas
runs boas. A tolerância de eixo da captura (±8 mm) permite isso; o degrau
tem 20 mm e o arame 6: a margem útil é de poucos milímetros e a captura
não a controla.

Os `contatos.txt` (5–7 MB por run) ficam fora do git; regenerar com
`SONDA=1` no `run_garra.sh`. Os `sonda_engate.csv` estão versionados. A
sonda agora roda com `B166ER_GARRA_DX=$GARRA` (viés corrigido para as
próximas).

## Conclusão

A perda do engate nas poses laterais não é da pose: é uma margem de
2–3 mm no eixo do furo que a captura deixa ao acaso. A proposta 2
(inserção −10 ± 4 mm) ataca a causa; a proposta 1 (detectar a subida da
ponta na libera e reassentar) cobre o resto e é o que o robô real
enxerga. Nenhuma aplicada — decisão do Marco.
