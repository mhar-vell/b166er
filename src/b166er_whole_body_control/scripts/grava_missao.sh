#!/usr/bin/env bash
# Grava UMA missão da chave em vídeo, com as quatro janelas do stack numa
# grade 2×2: Gazebo, RViz, imagem da câmera (rqt_image_view) e o painel
# da missão (mission_hud). Pedido do Marco em 2026-09-08 ("vc consegue
# gerar um video de uma missão com o Gazebo, RViz, painel e a camera?").
#
# Uso:
#   scripts/grava_missao.sh SAIDA.mp4 [--leve] [--x X --y Y --yaw GRAUS]
#                           [--sem-layout] [--regiao X,Y,W,H] [-- args do launch]
#
#   --leve        gera também SAIDA_leve.mp4 (1280 de largura, 900 kbps,
#                 ~10 MB por missão) para mandar por chat/e-mail
#   --sem-layout  não mexe nas janelas (você já as arrumou à mão)
#   --regiao      região da tela a capturar; padrão = o maior monitor
#                 conectado (xrandr)
#   -- ...        argumentos extras para chave_mission.launch
#
# Pré-requisitos: stack de pé (sim_stack.sh preflight), sessão X11 e o
# GStreamer DO SISTEMA (/usr/bin/gst-launch-1.0 com ximagesrc e x264enc).
# O gst-launch do conda (ros_env) NÃO tem ximagesrc — por isso o binário
# é chamado com env -i e caminho absoluto. Sem ffmpeg de propósito: não
# está instalado no shiroi.
#
# O que sai: MP4 H.264 a 15 fps, 1920 px de largura (altura proporcional
# à região), ~50 MB por missão de ~4 min. A gravação começa depois do
# reset e termina 4 s depois de "resultado: MISSION_*".
set +u   # os setup.sh do conda/gazebo têm variáveis não definidas; set -u aborta o source
OUT=""; LEVE=0; LAYOUT=1; REGIAO=""; X=0.0; Y=1.0; YAW=0; EXTRA=""
while [ $# -gt 0 ]; do
    case "$1" in
        --leve) LEVE=1 ;;
        --sem-layout) LAYOUT=0 ;;
        --regiao) REGIAO="$2"; shift ;;
        --x) X="$2"; shift ;;
        --y) Y="$2"; shift ;;
        --yaw) YAW="$2"; shift ;;
        --) shift; EXTRA="$*"; break ;;
        -*) echo "argumento desconhecido: $1"; exit 2 ;;
        *) OUT="$1" ;;
    esac
    shift
done
[ -n "$OUT" ] || { sed -n 2,25p "$0"; exit 2; }
[ -n "${DISPLAY:-}" ] || { echo "[grava] sem DISPLAY — precisa de sessão X11"; exit 1; }
[ -x /usr/bin/gst-launch-1.0 ] || { echo "[grava] /usr/bin/gst-launch-1.0 não existe"; exit 1; }
command -v wmctrl >/dev/null || { echo "[grava] wmctrl não instalado"; exit 1; }
S="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$(dirname "$OUT")"
LOG="${OUT%.mp4}.missao.log"

# ---- região de captura: o maior monitor conectado ---------------------
if [ -z "$REGIAO" ]; then
    REGIAO=$(xrandr | awk '/ connected/ { for (i=1;i<=NF;i++) if ($i ~ /^[0-9]+x[0-9]+\+[0-9]+\+[0-9]+$/) print $i }' \
        | sed 's/[x+]/ /g' | awk '{ if ($1*$2 > a) { a=$1*$2; r=$3","$4","$1","$2 } } END { print r }')
fi
IFS=, read -r RX RY RW RH <<< "$REGIAO"
echo "[grava] região $RX,$RY ${RW}x${RH}"

# ---- grade 2×2 --------------------------------------------------------
if [ "$LAYOUT" = 1 ]; then
    W2=$((RW / 2)); H2=$((RH / 2))
    coloca() {  # título x y
        local id
        id=$(wmctrl -l | grep -i -- "$1" | head -1 | awk '{print $1}')
        if [ -z "$id" ]; then echo "[grava] janela '$1' não encontrada — segue sem ela"; return; fi
        wmctrl -i -r "$id" -b remove,maximized_vert,maximized_horz
        wmctrl -i -r "$id" -b remove,fullscreen
        wmctrl -i -r "$id" -e 0,"$2","$3","$W2","$H2"
        wmctrl -i -a "$id"
    }
    coloca " Gazebo"          "$RX"            "$RY"
    coloca "RViz"             "$((RX + W2))"   "$RY"
    coloca "rqt_image_view"   "$RX"            "$((RY + H2))"
    coloca "painel da missão" "$((RX + W2))"   "$((RY + H2))"
    sleep 2
fi

# ---- stack e reset ----------------------------------------------------
"$S/sim_stack.sh" preflight || { echo "[grava] PREFLIGHT REPROVADO"; exit 1; }
source /home/marco/miniforge3/etc/profile.d/conda.sh
conda activate ros_env
source /home/marco/b166er/devel/setup.bash
for tent in 1 2 3; do
    python3 "$S/reset_sim.py" --x "$X" --y "$Y" --yaw "$YAW" | grep -q -- "-> OK" && break
    echo "[grava] reset falhou (tentativa $tent)"; sleep 3
done
sleep 2

# ---- gravação ---------------------------------------------------------
OW=1920; OH=$(( (OW * RH / RW) / 2 * 2 ))
env -i DISPLAY="$DISPLAY" HOME="$HOME" PATH=/usr/bin:/bin /usr/bin/gst-launch-1.0 -e \
    ximagesrc startx="$RX" starty="$RY" endx="$((RX + RW - 1))" endy="$((RY + RH - 1))" use-damage=0 \
    ! video/x-raw,framerate=15/1 ! videoconvert ! videoscale \
    ! video/x-raw,width="$OW",height="$OH",format=I420 \
    ! x264enc speed-preset=veryfast tune=zerolatency bitrate=4000 \
    ! video/x-h264,stream-format=avc ! mp4mux ! filesink location="$OUT" \
    > "${OUT%.mp4}.gst.log" 2>&1 &
GST=$!
sleep 3
kill -0 "$GST" 2>/dev/null || { echo "[grava] gst-launch morreu na largada:"; head -3 "${OUT%.mp4}.gst.log"; exit 1; }
echo "[grava] gravando (${OW}x${OH} @ 15 fps) -> $OUT"

# ---- missão -----------------------------------------------------------
roslaunch b166er_whole_body_control chave_mission.launch phase_timeout:=60 $EXTRA > "$LOG" 2>&1 &
RL=$!
for _ in $(seq 1 180); do
    grep -qE "resultado: MISSION" "$LOG" 2>/dev/null && break
    sleep 4
done
grep -E "resultado: MISSION" "$LOG" || echo "[grava] SEM RESULTADO em 12 min"
sleep 4
kill -INT "$GST"; wait "$GST" 2>/dev/null
kill -INT "$RL" 2>/dev/null; sleep 3
echo "[grava] vídeo: $OUT ($(du -h "$OUT" | cut -f1))"

# ---- versão leve ------------------------------------------------------
if [ "$LEVE" = 1 ]; then
    LW=1280; LH=$(( (LW * RH / RW) / 2 * 2 ))
    env -i PATH=/usr/bin:/bin HOME="$HOME" /usr/bin/gst-launch-1.0 -e \
        filesrc location="$OUT" ! qtdemux ! avdec_h264 ! videoconvert ! videoscale \
        ! video/x-raw,width="$LW",height="$LH",format=I420 \
        ! x264enc speed-preset=medium bitrate=900 ! video/x-h264,stream-format=avc \
        ! mp4mux ! filesink location="${OUT%.mp4}_leve.mp4" > /dev/null 2>&1
    echo "[grava] leve: ${OUT%.mp4}_leve.mp4 ($(du -h "${OUT%.mp4}_leve.mp4" | cut -f1))"
fi
echo "[grava] fim"
