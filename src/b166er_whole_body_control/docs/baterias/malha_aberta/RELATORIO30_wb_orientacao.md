# RELATORIO30 — orientação da ferramenta nas fases whole-body engatadas — 2026-10-16

Pendência 2 do RELATORIO28: as fases destrava, libera, arco1, arco2 e
desengata eram whole-body SÓ de posição; a atitude da ferramenta ficava
livre durante o engate e a primeira fase por IK depois delas re-alinhava o
degrau de uma vez (giro de punho de 20–25°, J4 −61 → −84, com o dedo no
anel; na run4 de `arco_deslizante` a lâmina voltou de 34,4° a 0°). O
desengate em whole-body (#93) contornou; isto resolve na raiz.

## O que mudou (branch `feat/wb-orientacao`)

**`fuzzy_wb_controller`, modo ponta.** O alvo (`/b166er/ee_target`) passa a
poder carregar orientação: identidade (w = 1) = só posição, como antes;
qualquer outra = as três linhas angulares da Jacobiana (J6[3:]) entram
com peso `~ori_w_ang` (0,10 m/rad) e ganho `~ori_k` (1/s), saturadas em
`~ori_wmax` (0,5 rad/s). É PREFERÊNCIA, não restrição: 5 DOF não fecham
pose 6D com a base travada, e o peso baixo garante que a posição manda.
O DLS ponderado passou a usar `eye(m)` com m = número de linhas.

**`chave_mission`.** A atitude do T265 registrada no ENGATE (fim da
captura e no reassenta, `ctx.ori_captura`; antes dela, a do início do
MANIPULATE) vai no alvo das fases whole-body e no alvo deslizante dos
arcos. Por fase, `wb_orientacao: false` no YAML desliga; global
`~wb_orientacao`.

**Desengata SEM orientação** (run1 da primeira bateria): com a base travada
o braço tem 5 DOF e não sai 90 mm pelo eixo mantendo a atitude completa —
a preferência brigou com a posição e a ponta oscilou 30 s entre −24 e
−48 mm no eixo. Onde o dedo está no anel e a base participa (destrava,
libera, arcos) a orientação vale; na saída, não.

## Bateria `garra/wb_orientacao` (garra fechada, âncora ligada, standoff 0,68): 5/5

| run | destrava | libera | arco1 | arco2 | desengata | lâmina |
|---|---|---|---|---|---|---|
| 1 | 1,3 s | 1,8 s | 1,4 s | 7,7 s | 2,4 s | 34,4° |
| 2 | estagnação | 1,6 s | 1,3 s | 6,2 s | 2,2 s | 34,4° |
| 3 | 2,0 s | 1,2 s | 1,1 s | 7,9 s | 2,2 s | 34,4° |
| 4 | estagnação | 3,7 s | 1,1 s | 9,3 s | 2,5 s | 34,4° |
| 5 | 1,2 s | 2,4 s | 1,6 s | 7,4 s | 2,8 s | 34,4° |

Erro angular máximo registrado nas fases com orientação: **4,9°** (típico
1–3°). Deriva de eixo nos arcos ≤ 3 mm. A lâmina fica em 34,4° depois do
desengate nas cinco: nenhum giro de punho com o dedo no anel.

Comparação com a bateria anterior (`arco_deslizante`, sem orientação):
mesmos tempos de arco (6–9 s), mesma lâmina final; a diferença é que a
atitude agora é observada e mantida, em vez de deixada à sorte — o que o
desengate por IK pagava.

## Para a bancada

- A atitude de referência é a do T265 no engate: o robô real tem exatamente
  isso (pose do T265), nada de simulador no caminho.
- O peso de 0,10 m/rad foi escolhido para não disputar com a posição; se
  na bancada a haste girar no anel, subir para 0,2 antes de mexer em ganho.
