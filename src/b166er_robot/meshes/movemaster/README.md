# Malhas do conjunto punho + T265

## `t265.stl`

Intel RealSense T265. Convertida para STL binário a partir de
`common/res/sources/T265.obj` do
[librealsense](https://github.com/yujinrobot/librealsense_framos), licença
**Apache-2.0** (verificada no `LICENSE` do repositório em 2026-09-02).

Mede **108,00 × 24,49 × 12,50 mm**, as dimensões da peça real. Substituiu a
`realsense.stl` do submódulo `movemaster_control`, que mede 148 × 30 × 28 mm
e não é uma T265 — a diferença apareceu quando o Marco comparou a simulação
com a câmera da bancada.

No `t265_link` a malha é posicionada pelo **centro de rastreamento**, que a
documentação da Intel define como o ponto médio entre as duas fisheyes (não
o centro do corpo). As fisheyes estão na malha em X = −30,86 e X = +33,88 mm,
separadas por 64,74 mm — a baseline nominal de 64 mm.

Conferência independente: com a pose usada no URDF, as fisheyes caem em
y = ±0,03237 m no `t265_link`, contra os +0,032 m que o `JFisheye1` usa
desde 2026-08-26, medidos na bancada com o driver real. Duas origens
diferentes, 0,4 mm de diferença.

## `suporte_movemaster.stl`

O suporte do punho **sem as duas câmeras decorativas**.

A malha original (`suporte_movemaster_com_cameras.stl`, no submódulo
`movemaster_control`, preservada intacta) traz duas D435 desenhadas sobre a
chapa — 90 × 25 × 25 mm, três lentes espaçadas de 50 mm cada. Nenhuma das
duas existe na bancada. Câmera desenhada que não é sensor atrapalha quem
olha a simulação justamente para conferir a montagem: uma delas foi
confundida com a T265 nesta sessão.

Gerada por separação em componentes conectadas, removendo toda componente
inteiramente contida em `y_stl > 220` (a faixa das duas D435): 268.832 dos
269.936 triângulos saíram. **Os furos de fixação foram preservados** — são
eles que posicionam a T265 (dois furos M3, ⌀3,5 mm, espaçados 50,50 mm,
sobre o plano de apoio em `z_stl = 6,50`).

O `<visual>` do `CameraSupport` continua com a mesma origem e escala: a
malha nova está no sistema de coordenadas da original.

## `dedo_afunilamento.stl`

O afunilamento **inclinado** do dedo fixo: tronco de pirâmide de 20 × 25 mm
(topo, encostado no garfo) para 10 × 10 mm (base, a seção da haste), com
10 mm de altura. Gerado em código (12 triângulos), **em metros e já no
frame do `tool_rod`** — topo em z = −0,025, base em z = −0,035 — por isso
o `<visual>` o usa com origem zero e sem `scale`.

Substituiu um bloco 15 × 22 de cantos vivos que estava no URDF: o Marco
apontou que a peça real é uma transição contínua ("o afunilamento é
inclinado, não algo como canto vivo"). Se `dedo_garfo_h` ou `dedo_afun_h`
mudarem, a malha tem que ser regerada — ela não lê as propriedades.

Só o visual usa a malha; a colisão é uma caixa 10 × 10, porque a rampa
nunca encosta em nada e caixa é mais estável no ODE que trimesh.

## `dedo_fixo.stl` — a ferramenta de manobra, para IMPRIMIR

Gerado por `scripts/gera_dedo.py` (numpy; sem FreeCAD no shiroi), em
**mm**, origem no topo do garfo e Z para baixo como no URDF. Geometria do
bloco `dedo_*` do `movemaster.urdf.xacro` (revisada com o Marco em
2026-09-02): garfo em U 20 × 30,4 × 25 (duas abas de 10 e vão de 10,4 que
abraça a aba original de 10 mm da castanha da HM-01), rampa 20 × 30,4 →
10 × 10, haste 10 × 10 × 80, degrau 30 × 10 × 10 (20 de projeção em −X);
125 de altura total, 25,5 cm³.

O que o STL **não** traz: os 4 furos M3 (⌀3,5) por aba em grade 2×2 — o
espaçamento da grade da castanha não foi medido (padrão 10 × 10 no
desenho, a conferir). Passe o medido a `gera_dedo.py --furo-dx --furo-dz`
e regenere. Desenho cotado em `docs/dedo_fixo_desenho.pdf`.

Nota: o URDF desenha o garfo como caixa de 20 × 25; a peça real é o U de
30,4. A caixa é aproximação visual; a colisão que importa é a da haste e
do degrau.
