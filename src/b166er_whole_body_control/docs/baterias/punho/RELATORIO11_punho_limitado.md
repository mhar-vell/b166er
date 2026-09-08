# Punho simulado limitado pela especificação do RV-M2 — 2026-09-08

Marco, sobre o punho: "não tenho como medir o torque", mas trouxe a spec
oficial — sem torque de junta nos manuais; carga nominal 1,0 kg (máx. 2,0),
J4 ±110°, J5 ±180°; estimativa estática 12 N a 0,1 m ≈ **1,2 N·m** no punho.
Decisão: "vai pela segunda opção, limita o J4 e roda as baterias e a opção
que já tinha em mente" (= também declarar o risco no artigo/plano de bancada).

## O limite escolhido: 4,2 N·m (era 20)

O `PositionJointInterface` do gazebo_ros_control satura o esforço do PID no
`effort` do URDF. Os 1,2 N·m da spec são o que o punho suporta ALÉM do
próprio peso, então o atuador simulado precisa cobrir o peso próprio na pior
postura (horizontal) mais uma margem:

| parcela | valor |
|---|---|
| L5 1,7 kg a 0,08 m | 1,33 N·m |
| ferramenta 0,16 kg a ~0,25 m | 0,40 N·m |
| L4 2,5 kg (CG no eixo), câmera e suporte (sem inércia) | 0 |
| margem 2 × 1,2 N·m | 2,40 N·m |
| **limite do J4** | **4,2 N·m** |

Commit 17bc76b (`exp/punho-limitado`), comentário completo no URDF.
Conferido no `/robot_description` carregado (`effort="4.2"`, o 20 sumiu).

## Bateria: pose ref × 6, híbrido — 6/6

| run | destrava (descida / lingueta) | soltura no arco1 | lâmina |
|---|---|---|---|
| 1–5 | estagna a 11–13,5 mm / 6–9 mm | SOLTA, recuo 57–58 mm, deriva ≤ 1 mm | 29° |
| 6 (com sonda) | idem | SOLTA, 57,4 mm, −0,6 mm | 29,4° |

Mesmos números da bateria com 20 N·m: o punho limitado **não impediu a
abertura**.

## Onde o punho trabalha no limite (sonda da run6, `tabela_j4_por_fase.txt`)

| fase | |J4| máx | média | saturado (≥ 3,99) | J4 pos |
|---|---|---|---|---|
| destrava | 2,53 N·m | 0,78 | 0 % | 84–85° |
| **libera** | **4,20** | **2,77** | **48 %** | 74–86° |
| arco1 | 4,08 | 1,74 | 4 % | 82–85° |
| arco2 | 2,86 | 1,55 | 0 % | 79–84° |
| DEPLOY / RETRACT (trocas de postura) | 4,20 | 0,8–1,6 | 4–10 % | — |

- O **destrava** (empurrar o anel para baixo) exige 2,5 N·m — dentro do
  limite, sem saturar. A conta de 10:05 (2–3 N·m) confirmada.
- O **libera** (puxar 30 mm para fora segurando embaixo) satura o J4 em
  metade das amostras e o punho cede 12° (86 → 74°) sem perder o anel — a
  base ajuda no puxão e o arco fecha mesmo assim. É a fase que decide na
  bancada: se o punho real ceder mais que isso, o anel escapa (é o modo de
  falha da run8, deriva de eixo).
- Trocas de postura saturam por instantes (transitório do PID a 0,3 rad/s),
  sem consequência.

## O que isso muda

1. O risco de bancada do punho existe, mas está no **libera**, não no
   destrava: o ensaio físico deve começar pelo puxão de 30 mm com o punho
   segurando, medindo a deriva de eixo da ponta (guarda de 15 mm).
2. Se o punho real não segurar o libera, o redesenho é fazer o puxão só
   com a base (o `puxa` já existe antes do arco1) e o punho travado, em vez
   de pedir ao J4 que segure o anel embaixo enquanto o braço puxa.
3. A margem de 2 × 1,2 N·m é uma escolha; com 1 × 1,2 (limite 2,9 N·m) o
   destrava ainda cabe (2,5) mas o libera não — vale rodar essa variante
   antes da bancada se o Marco quiser o pior caso.
