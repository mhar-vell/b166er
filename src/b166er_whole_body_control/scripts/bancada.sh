#!/usr/bin/env bash
# ROTEIRO DE BANCADA no terminal — um comando por ensaio do
# docs/plano_de_bancada.md (E0…E5). Pedido do Marco em 2026-09-09:
# "vc pode criar um tutorial via terminal dos testes de bancada?".
#
# Uso:
#   scripts/bancada.sh pre|E0|E1|E2|E3|E4|E5 [--sim] [--auto] [--dir DIR] [--n N]
#
#   pre     pré-requisitos do item 0 (ModemManager, portas, T265, firmware, medidas)
#   E0      energização, freios, trava de inclinação, laser
#   E1      percepção estática com a T265 (4 distâncias × 3 atitudes)
#   E2      punho no libera (missão a partir de REFINE, fase a fase)
#   E3      manipulação completa a partir do standoff (N execuções)
#   E4      aproximação de 2,0 m sem manipulação (estado_final=REFINE)
#   E5      missão completa (N execuções)
#   --sim   ensaio na SIMULAÇÃO (namespace /camera, sem portas/freios,
#           reset pelo reset_sim.py): serve para ensaiar o roteiro
#   --auto  não espera Enter nos passos manuais (para testar o script)
#   --dir   pasta dos resultados (padrão ~/bancada/AAAA-MM-DD)
#   --n     repetições em E3/E5 (padrão 3 e 5)
#
# O que ele faz por você: imprime cada passo, roda as verificações que
# dá para automatizar (serviços, portas, tópicos publicando, taxas),
# espera o seu Enter nos passos manuais, grava o rosbag com os tópicos
# do plano (seção 2) e anota tudo, com hora e as suas medidas de trena,
# em DIR/bancada.md. O que ele NÃO faz: sudo (imprime o comando e
# espera), regravar firmware, e qualquer comando de movimento sem o seu
# Enter. Cada ensaio começa conferindo o critério de parada do anterior.
#
# Regras que vêm da memória da bancada (2026-08-07/26):
#   · ModemManager corrompe o rosserial: `sudo systemctl stop ModemManager`
#     ANTES de ligar as placas — só o Marco (sudo interativo);
#   · portas dos Arduinos SEMPRE pelos links do udev /dev/arduino_{1,2,3};
#     os Arduino_*.py têm porta fixa e trocada — passar a porta por
#     argumento, nunca editar os arquivos;
#   · T265 aparece como Movidius no lsusb e isso é NORMAL: conferir com
#     rs-enumerate-devices; as DUAS fisheyes habilitadas; tópicos /t265/…;
#   · códigos de freio (firmware de TESTE): J2 = GoHome 7 na placa 1
#     (30 s solto), J3 = GoHome 9 na placa 2 (5 ciclos de 2 s). Com o
#     firmware OPERACIONAL, GoHome é flag de homing — NÃO mandar códigos.
set +u   # os setup.sh do conda/gazebo têm variáveis não definidas
ENSAIO=""; SIM=0; AUTO=0; DIR=""; N=""
while [ $# -gt 0 ]; do
    case "$1" in
        --sim) SIM=1 ;;
        --auto) AUTO=1 ;;
        --dir) DIR="$2"; shift ;;
        --n) N="$2"; shift ;;
        -*) echo "argumento desconhecido: $1"; exit 2 ;;
        *) ENSAIO="$1" ;;
    esac
    shift
done
case "$ENSAIO" in pre|E0|E1|E2|E3|E4|E5) ;; *) sed -n 2,30p "$0"; exit 2 ;; esac
[ -n "$DIR" ] || DIR="$HOME/bancada/$(date +%Y-%m-%d)"
mkdir -p "$DIR"
REG="$DIR/bancada.md"
S="$(cd "$(dirname "$0")" && pwd)"
source "${CONDA_SH:-$HOME/miniforge3/etc/profile.d/conda.sh}"
conda activate "${CONDA_ENV:-ros_env}"
source "${B166ER_WS:-$(cd "$S/../../.." && pwd)}/devel/setup.bash"
# Nomes que mudam entre simulação e hardware (conferidos em 2026-09-09):
#   fisheye  sim /camera/fisheye1/*      hw /t265/fisheye1/*
#   odom     sim /pioneer3at/odom        hw /pioneer/pose (RosAria remapeado)
#   laser    /pioneer3at/laser_hokuyo/scan nos dois: é o ~scan_topic padrão do
#            laser_safety, sem remap no hardware — o driver do Hokuyo real
#            precisa publicar aí (ou passar ~scan_topic ao laser_safety)
#   T265     /t265/odom/sample nos dois
if [ "$SIM" = 1 ]; then CAM=/camera; ODOM=/pioneer3at/odom; SCAN=/pioneer3at/laser_hokuyo/scan
else CAM=/t265; ODOM=/pioneer/pose; SCAN=/pioneer3at/laser_hokuyo/scan; fi
T265ODOM=/t265/odom/sample
# Imagem da fisheye COMPRIMIDA e bag em lz4: com image_raw cru o ensaio
# do E4 na simulação deu 2,3–3,4 GB por missão de ~2 min (2026-09-09).
# Se o tópico /compressed não existir (image_transport sem o plugin),
# cai para o cru e avisa.
IMG="$CAM/fisheye1/image_raw/compressed"
rostopic list 2>/dev/null | grep -q "^$IMG$" || { IMG="$CAM/fisheye1/image_raw"; echo "aviso: sem $CAM/fisheye1/image_raw/compressed — gravando a imagem crua (bags grandes)"; }
TOPICOS_BAG="$IMG $CAM/fisheye1/camera_info $T265ODOM \
/b166er/wall_pose /b166er/mission_status /b166er/tag_pixel /b166er/base_cap \
/joint_states /estimated_joint_states /cmd_vel $ODOM $SCAN /imu/data \
/b166er/tilt /b166er/tilt_critical /b166er/front_clearance /rosout"

# ---------------------------------------------------------------- util
B=$'\e[1m'; R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; Z=$'\e[0m'
NP=0
titulo() { echo; echo "${B}═══ $* ═══${Z}"; echo -e "\n## $(date '+%H:%M') — $*\n" >> "$REG"; }
passo()  { NP=$((NP+1)); echo; echo "${B}[$NP] $*${Z}"; echo "- [$NP] $*" >> "$REG"; }
nota()   { echo "    $*"; echo "    $*" >> "$REG"; }
pausa()  { echo "    ${Y}$*${Z}"; if [ "$AUTO" = 1 ]; then echo "    (auto: seguindo)"; else read -r -p "    ↵ Enter quando estiver feito (ou 'p' para parar): " r; [ "$r" = p ] && { nota "PARADO pelo operador"; exit 1; }; fi; }
pergunta() { # texto var
    local v
    if [ "$AUTO" = 1 ]; then v="(auto)"; else read -r -p "    ? $1: " v; fi
    printf -v "$2" '%s' "$v"; echo "    $1: $v" >> "$REG"; }
checa() { # nome comando…
    local nome="$1"; shift
    if "$@" >/dev/null 2>&1; then echo "    ${G}ok${Z}     $nome"; echo "    ok     $nome" >> "$REG"; return 0
    else echo "    ${R}FALHA${Z}  $nome"; echo "    FALHA  $nome" >> "$REG"; return 1; fi; }
hz() { # tópico mínimo_hz [segundos] → ok se a taxa média ≥ mínimo
    local t="$1" min="$2" seg="${3:-5}" r
    r=$(timeout "$((seg+3))" rostopic hz "$t" -w 50 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
    [ -z "$r" ] && r=0
    if awk -v r="$r" -v m="$min" 'BEGIN{exit !(r>=m)}'; then echo "    ${G}ok${Z}     $t a ${r} Hz (mín $min)"; echo "    ok     $t a ${r} Hz" >> "$REG"; return 0
    else echo "    ${R}FALHA${Z}  $t a ${r} Hz (mín $min)"; echo "    FALHA  $t a ${r} Hz (mín $min)" >> "$REG"; return 1; fi; }
BAG=""
grava_inicio() { # nome
    local f="$DIR/$1_$(date +%H%M%S).bag"
    rosbag record --lz4 -O "$f" $TOPICOS_BAG > "$DIR/rosbag_$1.log" 2>&1 &
    BAG=$!; sleep 2; nota "rosbag gravando → $f"; }
grava_fim() { [ -n "$BAG" ] && { kill -INT "$BAG" 2>/dev/null; wait "$BAG" 2>/dev/null; nota "rosbag fechado"; BAG=""; }; }
trap 'grava_fim' EXIT
missao() { # rótulo args…  → lança chave_mission.launch e espera o resultado; devolve o log
    local rot="$1"; shift
    local log="$DIR/missao_${rot}_$(date +%H%M%S).log"
    if [ "$SIM" = 1 ]; then python3 "$S/reset_sim.py" ${RESET_ARGS:-} > /dev/null 2>&1; sleep 2; fi
    nota "roslaunch b166er_whole_body_control chave_mission.launch $*  → $log"
    # A manobra ALIGN/ADVANCE é logada pelo CONTROLADOR (rosout), não pelo
    # stdout da missão: marca a linha do rosout no início para contar depois.
    ROSOUT=$(ls -t ~/.ros/log/*/rosout.log 2>/dev/null | head -1); ROSOUT_N0=$(wc -l < "${ROSOUT:-/dev/null}")
    roslaunch b166er_whole_body_control chave_mission.launch "$@" > "$log" 2>&1 &
    local pid=$!
    for _ in $(seq 1 240); do grep -qE "resultado: MISSION|inválido" "$log" 2>/dev/null && break; sleep 4; done
    kill -INT "$pid" 2>/dev/null; sleep 3
    grep -E "resultado: MISSION" "$log" | sed 's/.*\[mission\] //' | tee -a "$REG"
    ULTIMO_LOG="$log"; }
criterio() { # texto condição(0=ok)
    if [ "$2" = 0 ]; then echo "    ${G}CRITÉRIO OK${Z}  $1"; echo "    CRITÉRIO OK  $1" >> "$REG"
    else echo "    ${R}CRITÉRIO NÃO ATENDIDO${Z}  $1"; echo "    CRITÉRIO NÃO ATENDIDO  $1" >> "$REG"; fi; }
[ -f "$REG" ] || echo "# Bancada — $(date '+%Y-%m-%d') ($([ "$SIM" = 1 ] && echo simulação || echo hardware))" > "$REG"
echo "${B}b166er · bancada · $ENSAIO $([ "$SIM" = 1 ] && echo '(SIMULAÇÃO)')${Z} — registro em $REG"
rostopic list >/dev/null 2>&1 || { echo "${R}sem roscore/master — suba o stack primeiro${Z}"; echo "  hardware: roslaunch b166er_whole_body_control b166er_wb.launch mode:=hardware"; echo "  simulação: scripts/sim_stack.sh start"; exit 1; }

# ---------------------------------------------------------------- pre
if [ "$ENSAIO" = pre ]; then
    titulo "Item 0 — pré-requisitos"
    if [ "$SIM" = 1 ]; then nota "simulação: portas, ModemManager e firmware não se aplicam"; else
    passo "ModemManager parado (corrompe o rosserial dos Arduinos)"
    if systemctl is-active --quiet ModemManager; then
        echo "    ${R}ATIVO${Z} — rode no SEU terminal (sudo interativo):  sudo systemctl stop ModemManager"
        pausa "depois de parar o serviço"
        checa "ModemManager inativo" bash -c '! systemctl is-active --quiet ModemManager' || exit 1
    else checa "ModemManager inativo" true; fi
    passo "Portas dos Arduinos pelos links do udev (nunca /dev/ttyACM*)"
    for n in 1 2 3; do checa "/dev/arduino_$n" test -e "/dev/arduino_$n"; done
    nota "placa 1 = J1,J2 · placa 2 = J3,J4 · placa 3 = J5,garra;  lançar:  python3 Arduino_N.py /dev/arduino_N"
    passo "Pioneer e IMU nas portas do launch"
    checa "/dev/ttyPioneer" test -e /dev/ttyPioneer; checa "/dev/sparton/ahrs8" test -e /dev/sparton/ahrs8
    passo "T265 enumerada (o lsusb mostra Movidius e isso é normal)"
    if rs-enumerate-devices 2>/dev/null | grep -q "T265"; then checa "rs-enumerate-devices vê a T265" true
        rs-enumerate-devices 2>/dev/null | grep -E "Name|Serial|Firmware" | head -3 | sed 's/^/    /' | tee -a "$REG"
    else checa "rs-enumerate-devices vê a T265" false; fi
    passo "Firmware dos Arduinos"
    nota "Os 3 estão com o firmware de TESTE (ignora set_1..5, GoHome = seletor). Sem o operacional (src/arduino/Joints*/) não há E2 em diante."
    pergunta "firmware OPERACIONAL gravado nas 3 placas? (s/n)" FW
    [ "$FW" = s ] || nota "→ E0 e E1 podem seguir; E2–E5 ficam BLOQUEADOS até regravar (decisão do Marco)."
    fi
    passo "Medidas que definem o arco e o modelo de tombamento"
    pergunta "lâmina pivô→olhal (mm; modelo usa 200)" LAMINA
    pergunta "terminal fixo (descrição/forma)" TERMINAL
    pergunta "massa da base como roda, sem braço (kg; modelo usa 13)" MASSA
    pergunta "altura do CG da base (m; modelo usa 0,15)" CGZ
    nota "→ se lâmina ≠ 200 mm: atualizar chave_seccionadora_task.yaml (raio do arco). Se massa/CG mudarem: base_massa/base_cg_z em b166er_wb.launch (teto por postura)."
    passo "Tag e chave real"
    nota "tag 132 mm, detecta bem até ~1,2 m, ruidosa até 1,8 m; viés fixo +5..8 cm → offset no localizador antes de E5"
    nota "olhal a 805 mm de altura e 130 mm da parede; furo oval 40×30; tag a 170 mm na mesma altura"
    echo; echo "${G}pré-requisitos registrados em $REG${Z}"; exit 0
fi

# ---------------------------------------------------------------- E0
if [ "$ENSAIO" = E0 ]; then
    titulo "E0 — energização, freios e parada"
    nota "Montagem: braço em stow apoiado; base com rodas fora do chão OU travada; parada de emergência ao alcance."
    passo "Stack no ar"
    if [ "$SIM" = 1 ]; then nota "simulação: scripts/sim_stack.sh preflight"; else
        nota "Terminal A:  roslaunch b166er_whole_body_control b166er_wb.launch mode:=hardware"
        nota "Terminais B1..B3 (um por placa):  cd src/movemaster_control/src && python3 Arduino_N.py /dev/arduino_N"
        nota "ATENÇÃO: movemaster_hardware.launch sobe UM rosserial em /dev/ttyArduino — se essa porta não existir o nó morre (inofensivo); não deixe dois rosserial na mesma placa."
        nota "Hokuyo: sobe com o stack (hardware/hokuyo_hardware.launch → $SCAN). Ethernet, IP fixo 192.168.0.10 — cabo ligado e porta cabeada do NUC em 192.168.0.15/16; se o tópico não aparecer, ping 192.168.0.10."
    fi
    pausa "stack e ponte(s) no ar"
    passo "Tópicos vivos (taxa mínima)"
    hz /joint_states 5 || true
    hz /imu/data 5 || true
    hz "$SCAN" 5 || true
    hz "$T265ODOM" 20 || true
    hz "$CAM/fisheye1/image_raw" 10 || true
    hz /b166er/base_cap 5 || true
    if [ "$SIM" = 0 ]; then
        passo "rosserial estável: /status_1..6 por 60 s sem 'Lost sync' nos terminais B"
        for k in 1 2 3; do for st in 1 3 5; do hz "/status_$st" 2 5 || true; done; sleep 15; done
        pausa "confira nos terminais B: nenhum 'Lost sync with device' em 5 min (deixe correr e observe)"
        passo "Freios (SÓ com o firmware de TESTE; com o operacional GoHome é homing — pule)"
        pergunta "firmware de teste nas placas? (s/n)" FWT
        if [ "$FWT" = s ]; then
            nota "J2 (placa 1): código 7 → freio solto 30 s.   J3 (placa 2): código 9 → 5 ciclos de 2 s."
            pausa "mão no botão de emergência; confirme para mandar o código 7 na placa 1"
            rostopic pub -1 /setpoints movemaster_msg/setpoint "{set_1: 0, set_2: 0, set_3: 0, set_4: 0, set_5: 0, set_GRIP: false, emergency_stop: false, GoHome: 7}" >/dev/null 2>&1
            pergunta "freio J2 soltou por ~30 s e travou de novo? (s/n)" FJ2
            pausa "confirme para mandar o código 9 (placa 2, J3: 5 ciclos)"
            rostopic pub -1 /setpoints movemaster_msg/setpoint "{set_1: 0, set_2: 0, set_3: 0, set_4: 0, set_5: 0, set_GRIP: false, emergency_stop: false, GoHome: 9}" >/dev/null 2>&1
            pergunta "freio J3 ciclou 5× (2 s solto / 2 s travado)? (s/n)" FJ3
            nota "o 'Lost sync' DURANTE os ciclos do 9 é esperado (delay sem spinOnce); a placa reseta e trava o freio."
        else nota "freios: pulado (firmware operacional) — soltar/travar pelo procedimento do firmware operacional, à mão."; fi
    fi
    passo "Faixa real do J4 (punho) contra o modelo (±110°, zero = ferramenta alinhada com o antebraço)"
    nota "A missão trabalha a 5° do batente (IK pede até 104° na captura) e o punho real cede 12–22° sob carga. Com o braço apoiado e SEM energia nos motores, leve o punho à mão até cada batente e leia o inclinômetro/transferidor em relação ao antebraço."
    if [ "$SIM" = 1 ]; then nota "simulação: ±110° por construção — pulado"; else
        pergunta "batente POSITIVO do J4 (graus, ferramenta para cima)" J4P
        pergunta "batente NEGATIVO do J4 (graus, ferramenta para baixo)" J4N
        pergunta "com o punho no zero mecânico (alinhado ao antebraço) o modelo diz 0°? (s/n, e o desvio se souber)" J4Z
        nota "→ se a faixa for menor que ±110° ou o zero estiver deslocado: ajustar JOINT_LOWER/JOINT_UPPER do J4 em kinematics.py e o limit do J4 no movemaster.urdf.xacro, e reduzir a postura de captura (chave_seccionadora_task.yaml) para ficar a ≥ 10° do batente real."
    fi
    passo "Trava de inclinação"
    nota "incline o IMU à mão (>26°): /b166er/tilt_critical deve ir a True e o braço congelar; ao nivelar, limpa."
    if [ "$AUTO" = 1 ]; then nota "auto: sem inclinação manual — critério pulado"; else
    pausa "pronto para inclinar? (o script observa 20 s)"
    T=$(timeout 22 rostopic echo /b166er/tilt_critical 2>/dev/null | grep -c "data: True")
    criterio "tilt_critical disparou ao inclinar ($T mensagens True)" $([ "${T:-0}" -gt 0 ] && echo 0 || echo 1)
    fi
    passo "Laser de segurança contra a trena"
    pergunta "distância da parede à FRENTE do chassi pela trena (m)" TRENA
    L=$(timeout 5 rostopic echo -n 1 /b166er/front_clearance 2>/dev/null | grep data | awk '{print $2}')
    nota "front_clearance = ${L:-?} m"
    if [ "$AUTO" = 0 ] && [ -n "$L" ]; then D=$(awk -v a="$L" -v b="$TRENA" 'BEGIN{d=a-b; if(d<0)d=-d; print d}'); criterio "laser × trena diferem ${D} m (≤ 0,03)" $(awk -v d="$D" 'BEGIN{exit !(d<=0.03)}' && echo 0 || echo 1); fi
    echo; echo "${G}E0 registrado. Só há E1 se tudo acima estiver ok.${Z}"; exit 0
fi

# ---------------------------------------------------------------- E1
if [ "$ENSAIO" = E1 ]; then
    titulo "E1 — percepção estática com a T265 (robô parado)"
    nota "Critério: |erro de yaw| ≤ 20° a 2,0 m no anel; viés de distância +5..8 cm reproduzido e corrigido por offset."
    nota "Simulação prevê: centrado 148° (−32°) três vezes; no anel 1–4°."
    passo "Localizador apontado para $CAM (namespace da fisheye real é /t265)"
    hz "$CAM/fisheye1/image_raw" 10 || true
    grava_inicio E1
    for d in 0.75 1.0 1.5 2.0; do
        for at in "centrada (raio < 0,15)" "no anel (raio 0,35–0,5)" "na borda (raio > 0,6)"; do
            passo "Tag a $d m, $at"
            pausa "posicione o robô (trena para a distância, esquadro para o yaw); tag $at no quadro"
            pergunta "distância real pela trena (m)" DR
            pergunta "yaw real da parede pelo esquadro (graus)" YR
            nota "coletando 25 amostras de /b166er/wall_pose e /b166er/tag_pixel…"
            python3 - "$DR" "$YR" <<'PY' 2>/dev/null | tee -a "$REG"
import sys, math, rospy, numpy as np
from geometry_msgs.msg import PoseStamped, Point
from tf.transformations import euler_from_quaternion
rospy.init_node('bancada_e1', anonymous=True)
dr, yr = float(sys.argv[1] or 0) if sys.argv[1] != '(auto)' else 0.0, float(sys.argv[2] or 0) if sys.argv[2] != '(auto)' else 0.0
P, R = [], []
def cb(m):
    q = m.pose.orientation; y = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    P.append((m.pose.position.x, m.pose.position.y, math.degrees(y)))
def cbp(m): R.append(math.hypot(m.x, m.y))
rospy.Subscriber('/b166er/wall_pose', PoseStamped, cb); rospy.Subscriber('/b166er/tag_pixel', Point, cbp)
t0 = rospy.get_time()
while len(P) < 25 and rospy.get_time() - t0 < 30 and not rospy.is_shutdown(): rospy.sleep(0.1)
if not P: print("    SEM wall_pose em 30 s (tag não detectada?)"); sys.exit()
a = np.array(P); dist = np.hypot(a[:,0], a[:,1])
print("    amostras %d | dist %.3f ± %.3f m (trena %.3f → viés %+.3f) | yaw %.1f ± %.1f° (esquadro %.1f → erro %+.1f°) | raio pixel %s"
      % (len(P), dist.mean(), dist.std(), dr, dist.mean()-dr, a[:,2].mean(), a[:,2].std(), yr, a[:,2].mean()-yr, ("%.2f" % np.mean(R)) if R else "?"))
PY
        done
    done
    grava_fim
    echo; echo "${G}E1 registrado. Parar e voltar à simulação se o yaw no anel a 2,0 m passar de 20°.${Z}"; exit 0
fi

# ---------------------------------------------------------------- E2 / E3
if [ "$ENSAIO" = E2 ] || [ "$ENSAIO" = E3 ]; then
    if [ "$ENSAIO" = E2 ]; then titulo "E2 — punho no libera (risco número um)"; NR=1; else titulo "E3 — manipulação completa a partir do standoff"; NR="${N:-3}"; fi
    nota "Critérios: deriva de eixo ≤ 15 mm; cedência do J4 ≤ 22°; soltura = 1 no arco1 (recuo ≥ 40 mm). E3: 3/3 aberturas, nenhuma fase por timeout."
    nota "Simulação prevê: destrava desce 12–14 mm e estagna; libera fecha em 3–10 s; arco1 recuo 47–60 mm, deriva ≤ 1 mm; lâmina 27–33°."
    if [ "$SIM" = 0 ]; then
        passo "Firmware OPERACIONAL nas 3 placas (o de teste ignora os setpoints)"
        pergunta "confirmado? (s/n)" FW; [ "$FW" = s ] || { nota "BLOQUEADO: sem firmware operacional não há movimento comandado."; exit 1; }
        passo "Lâmina e terminal fixo medidos e no chave_seccionadora_task.yaml"
        pergunta "confirmado? (s/n)" _
    fi
    passo "Base estacionada à mão a 0,88 m do olhal (deploy_distance), de frente para a tag, braço em stow"
    nota "É a pose em que a missão chega do APPROACH: o REFINE remede daí e o DEPLOY avança sozinho até os 0,62 m. A 0,62 m com o braço em busca a ferramenta encosta na parede (J4 preso a 22° do alvo; visto na simulação em 2026-09-09)."
    if [ "$SIM" = 1 ]; then RESET_ARGS="--x 0.19 --y 1.99 --yaw 90"; nota "simulação: reset_sim.py $RESET_ARGS (0,88 m do olhal; parede em y=3, olhal a 0,13 da parede)"; fi
    nota "yaw pelo ESQUADRO, dentro de 2°: com 3° de erro a ferramenta escorrega no eixo da chave no arco1 e a guarda de 15 mm aborta (simulação, 2026-09-09)."
    pausa "estacionado; Marco ao lado da parada de emergência"
    for i in $(seq 1 "$NR"); do
        passo "Execução $i/$NR: missão a partir de REFINE, fase a fase (scripts/continua.sh libera cada fase)"
        grava_inicio "${ENSAIO}_run$i"
        nota "acompanhe /b166er/mission_status (e_eixo_mm, e_prof_mm, e_alt_mm, descida_mm, deriva_eixo_mm, soltura) e FILME o punho."
        if [ "$ENSAIO" = E2 ]; then nota "pare depois do arco1 se quiser só o punho (Ctrl-C no roslaunch): destrava → libera → puxa → arco1."; fi
        missao "${ENSAIO}_run$i" estado_inicial:=REFINE pausa_por_fase:=$([ "$AUTO" = 1 ] && echo false || echo true)
        grava_fim
        grep -E "captura: ponta registrada|SOLTURA|deriva de eixo|lâmina medida|MANIPULATE concluída|fase .* (alcançada|sem fechar)|ABORT" "$ULTIMO_LOG" | sed 's/.*\[mission\] /    /' | cut -c1-160 | tee -a "$REG"
        pergunta "cedência do J4 no libera (graus, pelo vídeo/inclinômetro; ≤ 22)" J4
        pergunta "chave abriu? (s/n; contato auxiliar ou operador)" AB
        DER=$(grep -o "deriva de eixo [-+0-9.]* mm" "$ULTIMO_LOG" | tail -1 | grep -o "[-+0-9.]*" | tail -1)
        [ -n "$DER" ] && criterio "deriva de eixo ${DER} mm (≤ 15)" $(awk -v d="$DER" 'BEGIN{if(d<0)d=-d; exit !(d<=15)}' && echo 0 || echo 1)
        grep -q "→ SOLTA" "$ULTIMO_LOG" && criterio "soltura observável no arco1" 0 || criterio "soltura observável no arco1" 1
    done
    echo; echo "${G}$ENSAIO registrado. Parar e redesenhar (puxão só com a base, punho travado) se J4 > 22° ou deriva > 15 mm.${Z}"; exit 0
fi

# ---------------------------------------------------------------- E4
if [ "$ENSAIO" = E4 ]; then
    titulo "E4 — aproximação de 2,0 m sem manipulação"
    nota "Critérios: estacionamento ≤ 30 mm em Y e ≤ 5° em yaw; REFINE ≤ 2°; recuperação da tag < 30 s; 1 ALIGN por execução."
    nota "Teto: 0,15 m/s (nav_linear_vel) e o teto por postura em /b166er/base_cap (travel ≈ 0,30 cheio)."
    for pose in "0.0 1.0 0 padrão" "0.0 1.0 90 de-frente" "0.5 1.0 0 deslocada+0,5" "-0.5 1.0 0 deslocada-0,5"; do
        set -- $pose
        passo "Partida $4: x=$1 y=$2 yaw=$3° (2 m da parede)"
        if [ "$SIM" = 1 ]; then RESET_ARGS="--x $1 --y $2 --yaw $3"; else pausa "posicione o robô à mão nessa partida (trena/esquadro)"; fi
        grava_inicio "E4_$4"
        missao "E4_$4" estado_final:=REFINE
        grava_fim
        grep -E "SEARCH: .*amostras|remedida|REFINE: .*amostras|manobra: |recupera|standoff" "$ULTIMO_LOG" | sed 's/.*\] /    /' | cut -c1-150 | tail -8 | tee -a "$REG"
        NA=$(tail -n +"$ROSOUT_N0" "${ROSOUT:-/dev/null}" 2>/dev/null | grep -c "manobra: ADVANCE -> ALIGN"); criterio "ALIGN por execução = $NA (esperado 1)" $([ "$NA" -le 1 ] && echo 0 || echo 1)
        if [ "$SIM" = 0 ]; then
            pergunta "erro de estacionamento em Y pela trena (mm; ≤ 30)" EY
            pergunta "erro de yaw pelo esquadro (graus; ≤ 5)" EYAW
        else
            python3 - <<'PY' 2>/dev/null | tee -a "$REG"
import rospy, math
from nav_msgs.msg import Odometry
rospy.init_node('bancada_e4', anonymous=True)
m = rospy.wait_for_message('/pioneer3at/odom', Odometry, timeout=5); p = m.pose.pose.position; q = m.pose.pose.orientation
yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
print("    base parou em x=%.3f y=%.3f yaw=%.1f° (sim: fim do APPROACH esperado y≈1,99 (deploy_distance 0,88), yaw≈90°)" % (p.x, p.y, yaw))
PY
        fi
    done
    echo; echo "${G}E4 registrado. Parar se o estacionamento passar de 30 mm em Y.${Z}"; exit 0
fi

# ---------------------------------------------------------------- E5
if [ "$ENSAIO" = E5 ]; then
    titulo "E5 — missão completa"
    NR="${N:-5}"; nota "Critério: ≥ 80 % (4/5) na pose padrão com a falha explicada. ABORT_SAFE recua 0,25 m: parede atrás LIVRE."
    OK=0
    for i in $(seq 1 "$NR"); do
        passo "Execução $i/$NR (pose padrão 0, 1, 0°)"
        if [ "$SIM" = 1 ]; then RESET_ARGS="--x 0.0 --y 1.0 --yaw 0"; else pausa "robô na partida padrão, braço em stow, área atrás livre"; fi
        grava_inicio "E5_run$i"
        missao "E5_run$i"
        grava_fim
        grep -q "MISSION_OK" "$ULTIMO_LOG" && OK=$((OK+1))
        grep -E "lâmina medida|SOLTURA|ABORT —|resultado" "$ULTIMO_LOG" | sed 's/.*\[mission\] /    /' | cut -c1-140 | tail -4 | tee -a "$REG"
        [ "$SIM" = 0 ] && pergunta "chave abriu? (s/n)" _
    done
    criterio "sucessos $OK/$NR (≥ 4/5)" $(awk -v o="$OK" -v n="$NR" 'BEGIN{exit !(o/n>=0.8)}' && echo 0 || echo 1)
    echo; echo "${G}E5 registrado em $REG${Z}"; exit 0
fi
