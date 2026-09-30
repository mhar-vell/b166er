# RELATÓRIO 19 — bateria 5 × 5: garra aberta × garra fechada

Data: 2026-09-30, 14:53–15:31, shiroi (Gazebo com GUI). Continuação do
RELATORIO18 (uma missão por condição), a pedido do Marco: "usa o STL da
v3 no visual e roda a bateria 5×5".

## Condições

- Código: main com a #76 (dedo v3) e a #77/#78 (`garra_dx`), mais o
  visual do dedo em STL (`dedo_fixo_v3.stl` no `tool_rod`; colisões em
  caixas **idênticas** às de antes, de propósito, para que a única
  diferença entre as condições seja a garra). `dedo_v3_gazebo_aberta.png`
  é o recorte da tela do Gazebo com a peça.
- Aberta = `garra_dx:=0.03` (dedo 30 mm fora do eixo do J5, valor
  histórico); fechada = `garra_dx:=0.0` (dedo no eixo do J5).
- Cinco missões completas por condição (STOW_INIT → RETURN), mesma pose
  de partida (0, 1, 0°), `phase_timeout:=60 tol_pos:=0.020`, reset
  entre missões (`reset_sim.py`), stack reiniciado uma vez por condição.
  Roteiro: `nuc_smoke_run2.sh` + `garra_dx` + `/joint_states` no bag.
  Logs em `aberta/runN` e `fechada/runN` (bags fora do git).
- Torques de `/joint_states` (limite do J4 simulado: 4,2 N·m, RELATORIO11);
  "sat" = fração das amostras da fase com |τ_J4| ≥ 4,1 N·m.

## Resultado por execução

### aberta (5 runs)

| run | resultado | lâmina ° | destrava desc mm | lingueta mm | libera s | libera erro mm | recuo mm | deriva mm | J4 libera ° | J4 máx libera | sat libera % | J4 máx arco1 | J4 máx arco2 | J5 máx libera |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| run1 | MISSION_OK | 29.1 | 11.3 | 8.0 | 8.2 | 9.6 | 48.6 | -0.4 | 76 | 4.20 | 48 | 4.20 | 3.53 | 1.08 |
| run2 | MISSION_OK | 33.4 | 17.9 | 15.1 | 1.6 | 3.2 | 56.8 | +1.8 | 79 | 3.02 | 0 | 3.95 | 2.45 | 0.11 |
| run3 | MISSION_OK | 29.5 | 14.1 | 10.0 | 6.2 | 10.5 | 59.6 | -0.8 | 71 | 4.20 | 66 | 4.20 | 3.13 | 0.38 |
| run4 | MISSION_OK | 28.3 | 12.6 | 7.2 | 8.9 | 12.8 | 47.5 | -0.6 | 78 | 4.20 | 52 | 4.20 | 4.20 | 1.09 |
| run5 | MISSION_OK | 30.6 | 12.7 | 8.8 | 5.3 | 6.3 | 51.1 | +0.2 | 88 | 4.20 | 36 | 4.20 | 3.73 | 0.50 |
| **média [mín; máx]** | 5/5 OK | 30.2 [28.3; 33.4] | 13.7 [11.3; 17.9] | 9.8 [7.2; 15.1] | 6.0 [1.6; 8.9] | 8.5 [3.2; 12.8] | 52.7 [47.5; 59.6] | +0.0 [-0.8; +1.8] | 78 [71; 88] | 3.96 [3.02; 4.20] | 41 [0; 66] | 4.15 [3.95; 4.20] | 3.41 [2.45; 4.20] | 0.63 [0.11; 1.09] |

### fechada (5 runs)

| run | resultado | lâmina ° | destrava desc mm | lingueta mm | libera s | libera erro mm | recuo mm | deriva mm | J4 libera ° | J4 máx libera | sat libera % | J4 máx arco1 | J4 máx arco2 | J5 máx libera |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| run1 | MISSION_OK | 29.4 | 10.5 | 6.8 | 4.6 | 11.6 | 48.0 | -0.9 | 80 | 4.20 | 30 | 4.03 | 3.93 | 0.24 |
| run2 | MISSION_OK | 28.6 | 10.5 | 4.7 | 4.8 | 11.4 | 58.0 | -0.8 | 83 | 4.20 | 22 | 4.20 | 4.20 | 0.24 |
| run3 | MISSION_OK | 30.1 | 19.0 | 17.0 | 2.8 | 3.1 | 57.6 | +2.9 | 88 | 3.62 | 0 | 2.76 | 2.38 | 0.16 |
| run4 | MISSION_OK | 30.3 | 10.9 | 7.9 | 5.3 | 6.5 | 58.1 | +0.6 | 86 | 4.20 | 18 | 3.99 | 3.70 | 0.21 |
| run5 | MISSION_OK | 29.8 | 18.7 | 14.4 | 2.4 | 8.4 | 50.0 | +3.1 | 99 | 3.43 | 0 | 2.88 | 2.91 | 0.16 |
| **média [mín; máx]** | 5/5 OK | 29.6 [28.6; 30.3] | 13.9 [10.5; 19.0] | 10.2 [4.7; 17.0] | 4.0 [2.4; 5.3] | 8.2 [3.1; 11.6] | 54.3 [48.0; 58.1] | +1.0 [-0.9; +3.1] | 87 [80; 99] | 3.93 [3.43; 4.20] | 14 [0; 30] | 3.57 [2.76; 4.20] | 3.42 [2.38; 4.20] | 0.20 [0.16; 0.24] |

## Leitura

- **Sucesso igual: 10/10.** Lâmina final 30,2° [28,3; 33,4] aberta contra
  29,6° [28,6; 30,3] fechada — a fechada é até mais repetível.
- **A preocupação do RELATORIO18 com o destrava não se confirma.** A
  descida média é a mesma (13,7 mm contra 13,9 mm) e a lingueta ficou
  abaixo dos 12 mm em **4 de 5 aberta e 3 de 5 fechada** — é um
  comportamento do mecanismo/critério, não da garra. Em todos os 10 casos
  a libera e os arcos passaram mesmo assim: o gatilho a meio curso não
  impediu a abertura, o que sugere que o critério "lingueta ≥ 12 mm" é
  conservador para o modelo atual (fica anotado; não é o assunto desta
  bateria).
- **Fechada alivia o punho, de forma consistente:**
  - saturação do J4 no libera: **41 % [0; 66] → 14 % [0; 30]**;
  - J4 máximo no arco1: 4,15 → 3,57 N·m (2 de 5 execuções fechadas
    não chegaram a 3 N·m);
  - J5 máximo no libera: **0,63 → 0,20 N·m** (força de contato passa
    pelo eixo de rolagem);
  - libera mais rápida: 6,0 s → 4,0 s, com o mesmo erro final (8,5 →
    8,2 mm).
  Mecanismo: com o dedo coaxial ao J5 o punho chega ao olhal mais
  vertical (J4 78° → 87° na libera) e a força de tração não gera momento
  de rolagem; o J4 trabalha com braço de alavanca menor.
- **Custo:** deriva de eixo um pouco maior na soltura (+1,0 mm [−0,9;
  +3,1] contra 0,0 [−0,8; +1,8]), muito abaixo da guarda de 15 mm. Com o
  dedo no eixo, J5 sai do espaço de trabalho da ponta (rolar o punho
  não move a ponta), o que a IK já trata — nenhuma fase acusou.
- O que a simulação NÃO diz: o punho real não tem torque de junta
  publicado (estimativa ~1,2 N·m estático, ver plano de bancada E2). A
  redução de carga vale como argumento qualitativo para a bancada, não
  como número.

## Recomendação

Adotar a garra **fechada** como padrão (`garra_dx` 0,0 nos dois launches
e no `GARRA_DX`), mantendo a aberta disponível pelo argumento. Motivos:
mesmo sucesso, menos carga no punho (o risco de bancada E2), libera mais
curta e lâmina mais repetível. Fisicamente é também a posição mais
segura para a garra ficar travada (castanhas encostadas, sem vão).

Quem decide é o Marco. Se aprovar: trocar os defaults, regravar a
RELATORIO/artigo (Seção IV descreve a garra) e repetir a validação nas
poses de partida do RELATORIO (spawn em várias posições).
