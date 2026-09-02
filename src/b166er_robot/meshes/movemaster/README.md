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
