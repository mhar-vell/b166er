# RELATÓRIO 18 — garra aberta × garra fechada na missão da chave

Data: 2026-09-30, shiroi (Gazebo, stack completo com GUI). Pedido do Marco
(29 Set): "o gripper sempre está na posição aberta, mas acho que seria
melhor estar na posição fechada; para testar vamos rodar a simulação com
o gripper na posição fechada e outra na posição aberta".

## O que "aberta/fechada" significa no modelo

A garra não tem junta na simulação (JGrip1 e JTool são `fixed`; a
castanha oposta, nua, foi removida em 27 Ago). O que muda é **onde a
castanha com o dedo fica travada**, no eixo transversal do GripCube:

| | castanha (folheto Mitsubishi) | modelo (`garra_dx`) |
|---|---|---|
| aberta | face interna a 30 mm do eixo (curso 0–60) | **0,030 m** (valor histórico) |
| fechada | face interna no eixo | **0,000 m** (dedo coaxial ao J5) |

Fonte: `~/Downloads/dutfpr/movemaster_documentation/RV-M2.pdf`, "DC
motor hand": stroke 0–60 mm entre faces internas, (20–80) externas, e
furação da castanha 2×4-M3 em grade 10 × 15 mm (confirma o 10 × 15
assumido em `gera_dedo.py` v2/v3). O modelo ignora os 5 mm até o centro
da castanha (na verdade 5 mm fechada / 35 mm aberta).

Parametrização feita nesta rodada: `xacro:arg garra_dx` em
`movemaster.urdf.xacro` (JTool e visual Grip1), `GARRA_DX` lido de
`B166ER_GARRA_DX` em `kinematics.py`, arg `garra_dx` em
`b166er_wb.launch` (xacro + env + repasse a `b166er_gazebo.launch`, que
recarrega o `robot_description` e sobrescrevia o valor) e em
`chave_mission.launch` (env). `chave_mission.py` confere o JTool do
`robot_description` contra a cinemática e aborta se discordarem — pegou
exatamente o caso do `b166er_gazebo.launch` na primeira tentativa.

Como rodar:

    sim_stack.sh restart mode:=gazebo gui:=true rviz:=true garra_dx:=0.0
    roslaunch b166er_whole_body_control chave_mission.launch garra_dx:=0.0 ...

## Duas missões, uma por condição

Mesmo roteiro (`nuc_smoke_run2.sh` + `garra_dx` + `/joint_states` no
bag), mesma pose de partida (0, 1, 0°), `phase_timeout:=60
tol_pos:=0.020`. Logs em `aberta/` e `fechada/` (bags fora do git).

| | aberta (0,03) | fechada (0,0) |
|---|---|---|
| resultado | MISSION_OK | MISSION_OK |
| duração (relógio) | 2 min 54 s | 3 min 06 s |
| base na captura (x, y) | 0,129, 2,193 | 0,157, 2,209 |
| captura: ponta acima do alvo | −14,2 mm | −9,3 mm |
| destrava: descida da ponta | 15,1 mm (estagnou em 14,5) | 11,3 mm (estagnou em 10,1) |
| destrava: lingueta (só sim) | 13,1 mm — gatilho solto | **5,9 mm — "pode não ter soltado"** |
| libera: tempo, erro final | 6,3 s, 7,5 mm | 3,1 s, 1,5 mm |
| arco1: soltura (recuo, deriva) | 56,4 mm, +0,1 mm | 55,2 mm, +0,8 mm |
| lâmina 15° / 30° / final | 14,6 / 29,3 / 29,8° | 14,4 / 29,5 / 29,5° |
| J4 na manipulação | 71–76° | 82–87° |
| J5 na manipulação | 85° | 85° |

Torque do J4 (limite 4,2 N·m, RELATORIO11) e do J5 pelo `/joint_states`:

| fase | J4 aberta méd / máx / sat | J4 fechada méd / máx / sat | J5 máx aberta → fechada |
|---|---|---|---|
| destrava | 0,90 / 2,46 / 0 % | 1,50 / 4,20 / 1 % | 0,24 → 0,08 |
| libera | 2,10 / 4,20 / **19 %** | 1,41 / 2,90 / **0 %** | 0,47 → 0,13 |
| arco1 | 2,47 / 4,20 / 6 % | 1,19 / 3,09 / 0 % | 0,59 → 0,09 |
| arco2 | 2,88 / 4,20 / 1 % | 1,09 / 2,64 / 0 % | 0,49 → 0,09 |

## Leitura

- **As duas abrem a chave.** Lâmina, soltura e RETURN iguais dentro do
  ruído de uma execução.
- **Fechada alivia o punho.** Com o dedo no eixo do J5 a força de contato
  não gera momento de rolagem (J5 cai de ~0,5 para ~0,1 N·m) e o braço
  chega ao mesmo ponto com o punho mais vertical (J4 82–87° em vez de
  71–76°), o que baixa o torque de J4 no libera e nos arcos: **zero
  saturação, contra 19 % das amostras no libera com a garra aberta**.
  É justamente o risco de bancada apontado nos RELATORIOS 11/12 (o punho
  cede no libera) e o ensaio E2 do plano de bancada.
- **Fechada piorou o destrava nesta execução.** A captura ficou 5 mm mais
  alta e a descida estagnou em 10,1 mm (mínimo aceito 10) com a lingueta
  em 5,9 mm; a libera passou mesmo assim. Não dá para dizer, com uma
  execução, se o gatilho realmente soltou ou se o modelo do gatilho
  tolera a libera com a lingueta a meio curso (run8 do RELATORIO16 era
  o caso oposto). Com o dedo coaxial, empurrar para baixo é empurrar ao
  longo do J5 — o J4 responde com torque maior (méd 1,50 contra 0,90).
- Diferença de partida: a base estacionou 16 mm mais perto da parede e
  28 mm mais à direita na condição fechada — a IK do REFINE/DEPLOY muda
  com o offset, então os dois casos não partem exatamente do mesmo
  lugar.

## Recomendação

A garra fechada parece melhor para o punho, mas uma execução por
condição não decide. Antes de mudar o padrão (`garra_dx` 0,03 → 0,0):

1. Bateria 5 × 5 na pose de referência, e depois nas poses de partida
   do RELATORIO (spawn em várias posições, como o plano de validação
   pede).
2. Olhar o destrava com a garra fechada: taxa de "lingueta < 12 mm" e se
   o critério de 10 mm mínimos ainda vale.
3. Se confirmar, trocar o default nos dois launches e no `GARRA_DX`, e
   registrar no artigo (Seção IV) que a garra opera fechada.

Quem decide é o Marco; o modelo fica com 0,03 por padrão até lá.
