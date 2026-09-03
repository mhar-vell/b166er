# Malha do Hokuyo UST-05LX

## `hokuyo_ust05lx.stl`

Forma da família URG/UST da Hokuyo, a partir de
`gazebo_plugins/test/multi_robot_scenario/meshes/laser/hokuyo.dae`
(gazebo_plugins, BSD). O envelope do arquivo já é **50 × 50 × 70 mm** —
o mesmo do UST-05LX (50 × 50 × 70 no datasheet), então não houve escala.

O `.dae` está deitado (o eixo de 70 mm é o Y). Foi girado para ficar em
pé (+Z para cima), com a base em z = 0 e centrado em XY. O lado de cima
foi decidido primeiro por uma heurística (faixa branca oposta ao volume
maior do corpo) — e ela errou: o Marco, com o sensor real na mão, viu a
malha de ponta-cabeça no Gazebo. Regravada invertida. A janela fica a
**34 mm** da base.

STL (sem materiais) de propósito: com os materiais embutidos do `.dae`
o Gazebo ignora o `<material>` do link, e a cor é justamente o pedido do
Marco (2026-09-02): "renderizado/modelado, e mudado de cor para que
possamos enxergar na simulação". O link usa `Gazebo/Orange`.

No URDF a origem do `laser_hokuyo_link` é o **plano de varredura** (o
sensor `ray` emite dali), então a malha desce 34 mm para a base assentar
na placa, e a instanciação sobe o link para z = 0,034 acima do
`top_plate` (era 0,025 para um cubo de 25 mm). A altura real da janela
na bancada não foi medida — está assumida pela malha.

Parâmetros do sensor, já os do UST-05LX antes desta mudança: 0,06–5,0 m,
±135°, 1081 passos (0,25°), 40 Hz. Só os comentários ao lado deles
falavam de outro modelo (30 m, 720 passos) e foram corrigidos.
