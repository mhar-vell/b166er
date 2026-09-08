# Aproximação de longe: Fuzzy × ganhos fixos no rastreamento whole-body — 2026-09-03/04

Marco: "segue com a bateria de aproximação de longe". Experimento
`exp_aproximacao.py` (fora do repo, em `aproximacao/`): da pose de partida
da missão (0, 1, yaw 0 — chave em +y, alinhamento obrigatório), braço na
postura de viagem, plano de exclusão na parede, alvo = pose 6D da T265
quando a base está a 0,8 m da parede virada para ela; lei whole-body
GERAL (servo_tooltip=false, tetos 0,3 m/s e 0,5 rad/s). Sucesso = 30 mm e
0,1 rad sustentados 1 s; também t_50mm, melhor erro, eventos ALIGN, tombo.

## Descobertas de montagem (2 testes)
1. A **variante de ponta** (a da manipulação) não serve para longe: base
   12× mais cara + teto de 0,06 m/s → chassi a 1–2 cm/s, 1,8 → 0,84 m em
   90 s sem ALIGN.
2. Com o braço na **postura de busca** (à frente), a lei geral a 0,3 m/s e
   0,5 rad/s **tombou o robô** (modo de falha de 13 Ago), depois de 5
   trocas ALIGN↔ADVANCE em 25 s. Com o braço recolhido (viagem) fica de pé.

## Bateria 1 (align_min_planar_dist = 0,05 m), 4 regimes × 5

| regime | ok/5 | t_50mm | melhor erro | ALIGN/exec | tombou |
|---|---|---|---|---|---|
| Fuzzy (Mamdani) | 0 | 19,9 ± 1,9 s | 9–22 mm | 14–21 | 0 |
| fixo conservador 0,3 | 0 | 19,5 ± 0,8 s | 9–31 mm | 3 | 0 |
| fixo médio 0,8 | 0 | 34,6 ± 2,9 s | 7–26 mm | 1–4 | 0 |
| fixo agressivo 1,4 | 0 | nunca chegou | 58–261 mm | 1–6 | 0 |

- **De longe o ganho importa** (ao contrário das fases de contato): o
  agressivo trava a ~26 cm pedindo velocidade lateral da base (base=[−0,34,
  +0,10] com o robô virado para +y) sem que ALIGN dispare, porque o rumo
  até o alvo já aponta para a frente — a limitação lateral do artigo por
  outro caminho. O médio é quase 2× mais lento que o conservador.
- **O Fuzzy chega tão rápido quanto o melhor fixo** mas oscila
  ALIGN↔ADVANCE 14–21 vezes perto do alvo e nunca se estabiliza; o
  conservador alinha 3 vezes e para.
- Ninguém cumpre o critério de estabilização: o controlador segue caçando
  (tol interno 2 mm).
- Causa do chatter (`_update_maneuver_state`): rumo = direção da base ao
  ponto XY do alvo do EE, confiado até 5 cm; quando a base já está no
  lugar o alvo do EE fica 0,5–0,8 m à frente/lado e o rumo gira a cada
  avanço. Correção: `align_min_planar_dist` 0,05 → 0,90 m (alcance +
  margem), commit na branch `fix/manobra-histerese-perto`.

## Bateria 2 (align_min_planar_dist = 0,90 m) — Fuzzy ×5, conservador ×5
Mesmo experimento, mesmas 5 partidas, só o piso do controlador mudou
(`rosparam get /fuzzy_wb_controller/align_min_planar_dist` = 0,9 conferido
depois do restart). Dados em `resultados_bateria2_piso090.jsonl`.

| regime | ok/5 | t_50mm | melhor erro | ALIGN/exec | tombou |
|---|---|---|---|---|---|
| Fuzzy (Mamdani) | 0 | 21,7 ± 1,8 s | 13–27 mm | **1** (era 14–21) | 0 |
| fixo conservador 0,3 | 0 | 19,6 ± 1,1 s | 15–28 mm | **1** (era 3) | 0 |

- **O chatter sumiu**: um único ALIGN por execução nos dois regimes — o
  giro inicial de 90° para a parede — e daí em diante só ADVANCE com o DLS
  holonômico fechando o resto. Confirma a causa apontada na bateria 1 (rumo
  ao alvo do EE girando dentro do alcance), não o escalonador Fuzzy.
- **Tempo até 50 mm praticamente igual** ao da bateria 1 (Fuzzy 21,7 vs
  19,9 s; conservador 19,6 vs 19,5 s): as trocas de estado custavam
  estabilidade, não tempo. Fuzzy e conservador continuam equivalentes em
  velocidade (diferença de 2 s dentro de 1σ+1σ).
- **Melhor erro continua em 13–28 mm** e ninguém fecha 30 mm + 0,1 rad
  sustentados por 1 s: o critério do experimento é mais duro que o da
  missão (que fecha a fase de aproximação pelo standoff da base, não pela
  pose 6D da câmera), e o controlador segue caçando o alvo com tolerância
  interna de 2 mm. Não é regressão: os melhores erros da bateria 1 (9–22
  mm) foram tocados de passagem durante o chatter, não sustentados.
- Nenhum tombo em 10 execuções com a postura de viagem.

## Conclusões para o artigo (Seção V-A / IV-D)
1. Na aproximação de longe (1,9 m), o ganho importa: agressivo trava
   lateralmente, médio é 2× mais lento, conservador e Fuzzy empatam.
2. O Fuzzy não é mais rápido que o melhor ganho fixo neste cenário, mas
   também não é pior — o argumento a favor dele é não precisar ESCOLHER o
   ganho (o agressivo, plausível a priori, falha; o médio custa 2×).
3. O chatter ALIGN↔ADVANCE era da máquina de manobra (piso de 5 cm), não do
   escalonador; corrigido no piso de 0,90 m (PR desta branch).

## Achados a atacar depois
- Tombamento com braço à frente a 0,3 m/s: o teto seguro depende da
  postura — candidato a regra do Fuzzy (inclinação/postura → teto da base).
- A variante de ponta é inútil para longe (documentar como limite, não
  corrigir).
- Agressivo trava lateral sem ALIGN: o rumo ao alvo do EE não vê a
  componente lateral que o DLS pede — outro sintoma da limitação lateral.
