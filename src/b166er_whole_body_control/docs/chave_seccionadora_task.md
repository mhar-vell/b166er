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

## Curso confirmado

Marco confirmou em 2026-08-12: **30° de giro total** da lâmina, a
partir do ângulo fechado (20°/0.349rad) — posição 2 (totalmente
aberta) = 50° absoluto. Posição 1 (intermediária) fica na metade
desse curso (35° absoluto) — divisão escolhida por mim para ter um
waypoint intermediário, já que o documento não especifica essa
proporção; ajustar se o Marco quiser outra divisão.

Os `offset_xyz_m` de `pos1`/`pos2` no YAML usam a *magnitude*
calculada pela geometria de pivô do xacro (`blade_length=0.200m`)
para esse curso de 30°/15°.

## Sentido do arco — confirmado

Marco confirmou em 2026-08-12: **o olhal desce** para abrir a chave.
Isso é o oposto do que a fórmula de pivô já modelada no xacro previa
(`olhal = pivô - blade_length·(sin θ, 0, cos θ)`, que dá Z crescente
para θ maior — a lâmina saindo de quase-vertical para mais
horizontal) — ou seja, o pivô/sentido de giro real da bancada não é
o mesmo que o desenho técnico do datasheet (que inclui o isolador,
removido na bancada) sugeria. A foto real da bancada (acima), com seu
mecanismo de dobradiça/mola vertical, é visualmente diferente do
desenho técnico do datasheet — consistente com essa divergência.

Os offsets no YAML já refletem isso: mesma magnitude calculada pela
fórmula de pivô, sinal do Z invertido para o sentido real (desce).
`blade_angle_deg` nos waypoints ficou só como referência da magnitude
do curso usada no cálculo, não corresponde mais a um ângulo de pivô
literal.

## Próximo passo (não implementado ainda)

Com os números confirmados, o próximo incremento é um nó
sequenciador (`task_sequencer.py`, seguindo o padrão SMACH/BT já
previsto na arquitetura do b166er) que carrega este YAML e publica
os waypoints em sequência para `/b166er/ee_target`, avançando para o
próximo waypoint quando o `whole_body_planner` reportar convergência
(`err_norm_pos < 0.005` e `err_norm_orient < 0.02`, mesmos limiares
já usados em `whole_body_planner.py`).
