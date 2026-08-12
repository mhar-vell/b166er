# Tarefa: abertura da chave seccionadora

Fonte: documento "Descrição do movimento de abertura da chave
seccionadora" (Google Docs, compartilhado pelo Marco em 2026-08-12).
Config correspondente: `../config/chave_seccionadora_task.yaml`.

## Contexto

Para os testes de bancada e para a simulação, **o isolador (bucha de
porcelana) é desconsiderado** — só a chave seccionadora em si é
montada na parede. O fixture Gazebo já reflete isso (ver nota "SEM
ISOLADOR" em `urdf/fixtures/chave_seccionadora_lf.urdf.xacro`).

![Diagrama do documento do Marco, com eixos X/Y, dimensões A-D do
datasheet STI, e o arco de abertura (posições 1 e
2)](img/chave_movimento_diagrama.png)

![Foto da bancada real — chave fixada na parede sem isolador, tag
AprilTag logo acima](img/chave_bancada_teste_sem_isolador.jpg)

## Descrição da tarefa (texto original do Marco)

> Na chave seccionadora, temos um olhal onde será utilizado para
> realizar o movimento. Uma ferramenta no end-efector do manipulador
> realizará o engate no olhal e o robot deverá realizar uma força
> perpendicular ao eixo X (este movimento fará com que a chave seja
> liberada para realizar o movimento seguinte). Logo após este
> movimento, o robot deverá realizar um movimento paralelo ao eixo X,
> ou seja o olhal descerá um pouco para a posição um e depois irá
> para a posição 2, realizando dessa forma a abertura da chave.

## Sequência de tarefa modelada

1. **Engate** — ferramenta do end-effector acopla no `chave_olhal_link`.
2. **Fase 1 (liberação)** — força perpendicular ao eixo X (mapeado
   para -Z no frame do fixture): destrava a catraca/trava mecânica.
   Magnitude: ~15mm (Marco confirmou faixa de 10-20mm aceitável).
3. **Fase 2 (abertura)** — movimento com componente dominante
   paralela ao eixo X, passando pela "posição 1" (intermediária) até
   a "posição 2" (chave totalmente aberta).

## Mapeamento de eixos: documento → fixture

O documento usa X horizontal / Y vertical no desenho técnico. No
frame local do fixture Gazebo (`wall_link`), isso corresponde a:

| Documento | Fixture (`wall_link`) |
|---|---|
| X (horizontal) | X (mesmo eixo de `chave_x_offset`/`tag_x_offset`) |
| Y (vertical) | Z (gravidade) |

Frame escolhido por ser invariante ao yaw de spawn do fixture no
mundo Gazebo (`spawn_chave_fixture.launch`) — só depende da geometria
da própria chave, não de onde ela está no mundo.

## Em aberto — precisa de dados reais da bancada

Os `offset_xyz_m` de `pos1`/`pos2` no YAML são **placeholders**,
calculados a partir da geometria de pivô já usada no xacro
(`blade_length=0.200m`, ângulo fechado 20°/0.349rad), assumindo
ângulos de abertura de 45° e 95° — só uma estimativa de proporção
visual do diagrama, não uma medição.

Dois pontos específicos meu cálculo não conseguiu fechar com
confiança, e por isso não travei nenhum valor definitivo:

1. **Curso real de abertura** (quantos graus/mm a chave realmente
   percorre até abrir) — só o Marco, com a chave física em mãos, tem
   esse número.
2. **Sentido do arco.** Pela fórmula de pivô já modelada no xacro
   (`olhal = pivô - blade_length·(sin θ, 0, cos θ)`), um ângulo θ
   *maior* faz o olhal *subir* (Z cresce, lâmina saindo de
   quase-vertical para mais horizontal). O desenho à mão do documento,
   por outro lado, mostra a posição 2 mais *baixa* que a posição do
   olhal fechado. Além disso, a foto real da bancada (acima) mostra um
   mecanismo com aparência de dobradiça/mola vertical, visualmente
   diferente do desenho técnico do datasheet (que inclui o isolador,
   removido na bancada) — não é certo que o mesmo ponto de pivô e o
   mesmo sentido de giro se apliquem 1:1 ao rig de teste real.

**Ação pendente:** Marco vai medir o curso real na bancada física e
passar os valores (graus de rotação da lâmina, ou deslocamento
X/Z medido do olhal) para substituir os placeholders no
`chave_seccionadora_task.yaml`.

## Próximo passo (não implementado ainda)

Com os números confirmados, o próximo incremento é um nó
sequenciador (`task_sequencer.py`, seguindo o padrão SMACH/BT já
previsto na arquitetura do b166er) que carrega este YAML e publica
os waypoints em sequência para `/b166er/ee_target`, avançando para o
próximo waypoint quando o `whole_body_planner` reportar convergência
(`err_norm_pos < 0.005` e `err_norm_orient < 0.02`, mesmos limiares
já usados em `whole_body_planner.py`).
