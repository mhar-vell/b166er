#!/usr/bin/env bash
#
# sim_stack.sh — sobe, derruba e AUDITA o stack de simulação do b166er.
#
# Existe porque em 2026-08-24 uma bateria inteira (8 execuções, ~1 h) foi
# perdida e outra ficou sem valor estatístico por processos sobreviventes:
#
#   · 215 processos órfãos de nós de sessões ANTERIORES ainda vivos na
#     máquina, publicando nos mesmos tópicos;
#   · o state_estimator morto deixando o NOME registrado no master, de
#     modo que `rosnode list` mostrava tudo OK enquanto o nó não existia.
#
# A lição operacional: `rosnode list` lista REGISTROS no master, não
# processos. Um nó que morreu de SIGKILL deixa o registro para trás e o
# stack parece íntegro. A conferência tem que ser por `pgrep`, sempre.
#
# Uso:
#   sim_stack.sh stop          derruba tudo e confirma que sobrou zero
#   sim_stack.sh status        lista PROCESSOS vivos (e registros órfãos)
#   sim_stack.sh assert-clean  sai != 0 se houver qualquer resíduo
#   sim_stack.sh start [args]  exige limpo, sobe o stack + fixture, espera pronto
#   sim_stack.sh restart [args]  stop + start
#   sim_stack.sh preflight     verifica se dá para TESTAR (use antes de bateria)
#
# Todo comando é seguro de repetir.
#
# `start` sobe também a fixture (parede + chave + tag), que vive num
# launch separado (b166er_robot/launch/simulation/spawn_chave_fixture.launch)
# e precisa de um Gazebo já rodando. Esquecer esse passo custou outra
# bateria inteira em 2026-08-24: o stack subiu íntegro, a câmera publicava
# a 15 Hz, o localizador rodava — e as 8 execuções morreram em
# "SEARCH: tag não encontrada" porque não havia tag nenhuma no mundo.
# Processo vivo não é o mesmo que cenário montado; o `preflight` checa os
# dois.

set -uo pipefail

WS="${B166ER_WS:-/home/marco/b166er}"
CONDA_SH="${CONDA_SH:-/home/marco/miniforge3/etc/profile.d/conda.sh}"
CONDA_ENV="${CONDA_ENV:-ros_env}"

# Nós que compõem o stack. Usados só para relatório — o desligamento
# varre por caminho do workspace, para pegar também o que não está aqui.
NODES=(state_estimator fuzzy_wb_controller gazebo_arm_bridge
       tilt_monitor laser_safety apriltag_localizer chave_mission)

# Padrões de processo que pertencem à simulação. O caminho do workspace
# cobre qualquer nó lançado do devel space, inclusive de sessões antigas.
# Visualizadores entram aqui de propósito: eles não são "do stack" mas
# sobrevivem ao stop se não forem listados, e um rviz órfão de uma sessão
# anterior fica publicando/assinando junto com o novo. Aconteceu em
# 2026-08-24 — dois rviz reais ao mesmo tempo, achado pelo Marco.
PATTERNS=("$WS/devel" "gzserver" "gzclient" "rosmaster" "roscore"
          "b166er_wb.launch" "chave_mission.launch"
          "lib/rviz/rviz" "rqt_image_view")

# ROS não é sourceado aqui de propósito: `stop`, `status` e `assert-clean`
# precisam funcionar mesmo com o master morto ou o ambiente quebrado.
source_ros() {
    # setup.zsh vs setup.bash importa: sob zsh, o setup.bash resolve $0
    # errado e falha com "no such file or directory: .../setup.sh".
    # Este script roda em bash (shebang), então setup.bash é o certo.
    #
    # `set +u` aqui não é preguiça: os scripts do conda e do catkin
    # referenciam variáveis não definidas, e sob `set -u` isso aborta o
    # script inteiro sem imprimir uma linha sequer — sintoma observado ao
    # escrever este arquivo (preflight saía com código 1 e zero saída).
    local u_antes; u_antes=$(set +o | grep nounset)
    set +u
    # shellcheck disable=SC1090
    source "$CONDA_SH" && conda activate "$CONDA_ENV" \
        && source "$WS/devel/setup.bash"
    local rc=$?
    eval "$u_antes"
    return $rc
}

# Conta processos que casam com o padrão.
#
# NÃO usar `pgrep -c ... || echo 0`: quando não há match, o pgrep imprime
# "0" E sai com código 1, então o `|| echo 0` acrescenta um SEGUNDO zero e
# a variável vira "0\n0" — que estoura em $(( )) com
# "syntax error in expression (error token is 0)". `wc -l` sempre imprime
# um número e sai com 0.
conta_proc() {
    # Exclui o próprio shell e o pai: a linha de comando de quem INVOCA
    # este script contém o padrão procurado, então o pgrep se casa
    # sozinho e infla toda contagem em +1. Isso fez `status` reportar
    # "2 rviz" com um só rodando (2026-08-24) — falso positivo de
    # duplicata é tão ruim quanto não detectar a real.
    pgrep -f -- "$1" 2>/dev/null | grep -vx "$$" | grep -vx "$PPID" | wc -l
}

pids_vivos() {
    local out=""
    for p in "${PATTERNS[@]}"; do
        out+=$'\n'"$(pgrep -f -- "$p" 2>/dev/null)"
    done
    # Exclui a si mesmo e a filhos diretos deste script.
    printf '%s\n' "$out" | grep -E '^[0-9]+$' | sort -u \
        | grep -vx "$$" | grep -vx "$PPID"
}

cmd_status() {
    echo "── PROCESSOS (fonte de verdade) ─────────────────────────"
    local total=0
    for n in "${NODES[@]}"; do
        local c
        c=$(conta_proc "b166er_whole_body_control/$n.py")
        total=$((total + c))
        printf "  %-22s %s%s\n" "$n" "$c" \
            "$([ "$c" -gt 1 ] && echo '   <<< DUPLICADO')"
    done
    # Contados pelo binário real, não pelo nome solto: `pgrep -f rviz`
    # casa também o wrapper /bin/sh e o roslaunch, inflando o número e
    # fazendo parecer duplicata onde não há.
    for p in "gzserver -u" "gzclient -g" rosmaster "lib/rviz/rviz" rqt_image_view; do
        printf "  %-22s %s\n" "$p" "$(conta_proc "$p")"
    done
    echo "  total de PIDs da simulação: $(pids_vivos | wc -l)"

    echo "── REGISTROS no master (podem estar órfãos) ─────────────"
    if source_ros >/dev/null 2>&1 && timeout 5 rosnode list >/dev/null 2>&1; then
        local orfaos=0
        while read -r n; do
            [ -z "$n" ] && continue
            if ! timeout 3 rosnode ping -c1 "$n" >/dev/null 2>&1; then
                echo "  ÓRFÃO: $n (registrado, não responde)"
                orfaos=$((orfaos + 1))
            fi
        done < <(timeout 5 rosnode list 2>/dev/null)
        [ "$orfaos" -eq 0 ] && echo "  nenhum registro órfão"
    else
        echo "  master fora do ar (normal depois de 'stop')"
    fi
}

cmd_stop() {
    echo "[sim_stack] derrubando..."
    # Ordem: launches primeiro (para não relançarem filhos), depois nós,
    # depois gazebo, master por último.
    for p in "b166er_wb.launch" "chave_mission.launch" "$WS/devel" \
             "gzclient" "gzserver" "rosmaster" "roscore"; do
        pkill -f -- "$p" 2>/dev/null
    done
    sleep 3
    for p in "${PATTERNS[@]}"; do pkill -9 -f -- "$p" 2>/dev/null; done
    sleep 3

    # Teimosos: mata por PID, um a um, e confirma.
    local restantes
    restantes=$(pids_vivos)
    if [ -n "$restantes" ]; then
        echo "[sim_stack] insistindo em: $(echo "$restantes" | tr '\n' ' ')"
        # shellcheck disable=SC2086
        kill -9 $restantes 2>/dev/null
        sleep 3
    fi

    restantes=$(pids_vivos)
    if [ -n "$restantes" ]; then
        echo "[sim_stack] FALHOU — ainda vivos:"
        ps -o pid,args -p $(echo "$restantes" | tr '\n' ',' | sed 's/,$//') 2>/dev/null
        return 1
    fi
    echo "[sim_stack] limpo: zero processos da simulação"
}

cmd_assert_clean() {
    local restantes
    restantes=$(pids_vivos)
    if [ -n "$restantes" ]; then
        echo "[sim_stack] NÃO está limpo — $(echo "$restantes" | wc -l) processo(s):" >&2
        ps -o pid,args -p $(echo "$restantes" | tr '\n' ',' | sed 's/,$//') 2>/dev/null >&2
        return 1
    fi
    echo "[sim_stack] limpo"
}

cmd_start() {
    cmd_assert_clean || {
        echo "[sim_stack] recuse-se a subir sobre resíduo: rode 'stop' antes." >&2
        return 1
    }
    source_ros || { echo "[sim_stack] não consegui sourcear o ROS" >&2; return 1; }

    # PADRÃO É COM VISUALIZAÇÃO. Subir headless economiza CPU mas deixa o
    # Marco cego: sem a janela do Gazebo, sem RViz e sem a imagem da
    # câmera, não dá para ver o robô se aproximar nem conferir se a tag
    # está sendo detectada. Reclamação recorrente ("vc sempre esquece de
    # subir o rviz e a camera"), então o default inverteu. Para rodar
    # headless (bateria longa, CI), passe explicitamente:
    #   sim_stack.sh start mode:=gazebo gui:=false rviz:=false
    local args=("$@")
    [ ${#args[@]} -eq 0 ] && args=(mode:=gazebo gui:=true rviz:=true)
    echo "[sim_stack] subindo: ${args[*]}"
    setsid roslaunch b166er_whole_body_control b166er_wb.launch "${args[@]}" \
        > "${SIM_STACK_LOG:-/tmp/b166er_stack.log}" 2>&1 &
    disown

    # Pronto = os tópicos que a missão realmente consome estão FLUINDO.
    # Esperar por rosnode list aqui repetiria o erro que motivou o script.
    #
    # `rostopic echo -n1`, não `rostopic hz -w 3`: o hz precisa juntar
    # várias amostras E o próprio rostopic leva 1-2 s só para registrar o
    # nó no master, então uma sonda curta reprovava um tópico que estava
    # publicando normalmente. Foi o que aconteceu em 2026-08-24: o start
    # desistiu com TIMEOUT e não spawnou a fixture, enquanto o
    # /b166er/robot_state corria a 20 Hz. O echo -n1 retorna assim que a
    # primeira mensagem chega.
    echo -n "[sim_stack] aguardando fluxo em /b166er/robot_state"
    local pronto=0
    for _ in $(seq 1 40); do
        if timeout 12 rostopic echo -n1 /b166er/robot_state >/dev/null 2>&1; then
            pronto=1; echo " — pronto"; break
        fi
        echo -n "."
    done
    if [ "$pronto" -ne 1 ]; then
        echo " — TIMEOUT"
        echo "[sim_stack] veja ${SIM_STACK_LOG:-/tmp/b166er_stack.log}" >&2
        return 1
    fi

    # A fixture mora num launch separado e não sobe junto com o stack.
    echo "[sim_stack] spawnando a fixture (parede + chave + tag)..."
    timeout 60 roslaunch b166er_robot spawn_chave_fixture.launch \
        >> "${SIM_STACK_LOG:-/tmp/b166er_stack.log}" 2>&1
    sleep 3

    # Visualização da câmera de tarefa. O /b166er/tag_debug_image mostra a
    # tag COM a detecção desenhada, que é o que importa conferir: ver a
    # imagem crua não diz se o pipeline fechou. Só sobe se houver GUI.
    if [[ " ${args[*]} " != *"gui:=false"* ]] && command -v rqt_image_view >/dev/null; then
        if ! pgrep -f rqt_image_view >/dev/null; then
            echo "[sim_stack] abrindo a imagem da câmera (/b166er/tag_debug_image)"
            setsid rqt_image_view /b166er/tag_debug_image \
                >> "${SIM_STACK_LOG:-/tmp/b166er_stack.log}" 2>&1 &
            disown
        fi
    fi

    cmd_status
    cmd_preflight
}

# Verifica se dá para TESTAR — não só se há processos vivos. Cada item
# aqui já custou uma bateria inteira (~1 h) por não ser verificado.
cmd_preflight() {
    source_ros >/dev/null 2>&1
    local falhas=0
    echo "── PREFLIGHT ───────────────────────────────────────────"

    for n in state_estimator fuzzy_wb_controller gazebo_arm_bridge \
             tilt_monitor laser_safety apriltag_localizer; do
        local c
        c=$(conta_proc "b166er_whole_body_control/$n.py")
        if [ "$c" -ne 1 ]; then
            echo "  FALHA  $n: $c processos (esperado 1)"; falhas=$((falhas + 1))
        fi
    done
    [ "$falhas" -eq 0 ] && echo "  ok     um processo por nó"

    # Cenário montado: sem a fixture não há tag, e o SEARCH roda 90 s no vazio.
    if timeout 10 rosservice call /gazebo/get_world_properties 2>/dev/null \
           | grep -q chave_seccionadora_fixture; then
        echo "  ok     fixture presente no mundo"
    else
        echo "  FALHA  fixture AUSENTE do mundo (nada para o SEARCH achar)"
        falhas=$((falhas + 1))
    fi

    # Percepção de fato fechando o laço, não só a câmera publicando.
    if timeout 20 rostopic echo -n1 /b166er/wall_pose >/dev/null 2>&1; then
        echo "  ok     tag detectada (/b166er/wall_pose responde)"
    else
        echo "  AVISO  tag não detectada agora — normal se o robô não estiver"
        echo "         de frente para a parede; o SEARCH gira para procurar."
    fi

    # Estado crítico pendente congela o braço em toda execução seguinte.
    local crit
    crit=$(timeout 10 rostopic echo -n1 /b166er/tilt_critical 2>/dev/null \
           | grep -oE 'True|False' | head -1)
    if [ "$crit" = "False" ]; then
        echo "  ok     tilt_critical limpo"
    else
        echo "  FALHA  tilt_critical='$crit' (robô tombado ou estado preso)"
        falhas=$((falhas + 1))
    fi

    if [ "$falhas" -gt 0 ]; then
        echo "[sim_stack] PREFLIGHT REPROVADO ($falhas) — não gaste uma bateria." >&2
        return 1
    fi
    echo "[sim_stack] preflight aprovado — pode testar"
}

case "${1:-status}" in
    stop)         cmd_stop ;;
    preflight)    cmd_preflight ;;
    status)       cmd_status ;;
    assert-clean) cmd_assert_clean ;;
    start)        shift; cmd_start "$@" ;;
    restart)      shift; cmd_stop && cmd_start "$@" ;;
    *)            echo "uso: $0 {stop|start|restart|status|assert-clean|preflight}" >&2
                  exit 2 ;;
esac
