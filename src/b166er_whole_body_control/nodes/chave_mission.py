#!/usr/bin/env python3
"""
chave_mission — missão completa de operação da chave seccionadora,
orquestrada por máquina de estados (SMACH).

Pedida pelo Marco em 2026-08-13: "realizar um movimento completo de
procura da chave através da apriltag e realizar uma aproximação
adequada, simulando o movimento de abertura da chave e o robot
retornando para o ponto de partida".

Sequência
---------
  STOW_INIT  → braço recolhido (config/arm_postures.yaml: stow_home).
               Nada se move com o braço estendido: a postura estendida
               já tombou o robô contra a parede em 2026-08-12.
  SEARCH     → postura "search" (câmera apontando à frente) e giro no
               próprio eixo até o apriltag_localizer publicar a pose da
               parede em /b166er/wall_pose. É o único estado que
               descobre ONDE está a chave — nada aqui consulta o
               simulador.
  APPROACH   → aproximação em etapas: vai até ~1,30 m (dentro da faixa
               confiável da tag), remede parado, e só então navega até o
               standoff final. Braço recolhido, câmera rastreando a tag.
  REFINE     → já parado e perto, remede a parede pela tag e substitui a
               estimativa grosseira do SEARCH. Sem isso o erro angular
               da detecção distante vira dezenas de cm no alvo.
  DEPLOY     → resolve a IK da ponta para o primeiro alvo e
               pré-posiciona o braço nela (não uma postura fixa: o ramo
               errado trava o controlador num batente de junta).
  MANIPULATE → as 4 fases Cartesianas (engage → release → pos1 → pos2)
               via /b166er/ee_target, conduzidas pelo controlador
               whole-body, exatamente como no task_sequencer.
  RETRACT    → braço de volta ao recolhido, antes de qualquer
               deslocamento.
  RETURN     → base volta à pose de partida (registrada no STOW_INIT).
  DONE

Falhas
------
Cada estado tem timeout próprio e sai por 'failed'; a máquina inteira
termina em ABORT_SAFE, que para a base e recolhe o braço — nunca deixa
o robô parado com o braço estendido, que é a condição de tombamento.

Navegação da base
-----------------
Girar-avançar-girar explícito (mesma filosofia da manobra já existente
no fuzzy_wb_controller): a base é skid-steer e não faz correção
lateral. Aqui o controle é de BASE pura (/cmd_vel direto), não
whole-body — durante deslocamento o braço fica recolhido e parado.

Tópicos
-------
  Subscreve: /b166er/robot_state, /b166er/wall_pose,
             /b166er/arm_posture_reached
  Publica:   /cmd_vel, /b166er/ee_target, /b166er/arm_posture_cmd
"""

import math
import time

import numpy as np
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64, String
from tf.transformations import (quaternion_matrix, euler_from_quaternion,
                                quaternion_from_matrix)

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import (GetJointProperties, GetModelState,
                             SetModelConfiguration, SetModelState)
from b166er_whole_body_control.msg import RobotState
from b166er_whole_body_control.kinematics import (
    JOINT_NAMES, pose_error, T_T265_TOOLTIP, T_BASELINK_ARM, fk_arm,
    ik_tooltip_position, ik_tooltip_com_degrau, ik_tooltip_nivelado)
from b166er_whole_body_control import chave_task

# pre_engage primeiro: aproxima por um ponto afastado da parede e só
# então entra reto no olhal — o controlador Cartesiano não desvia de
# obstáculo sozinho (ver nota no YAML).
# Ordem das fases (ver config/chave_seccionadora_task.yaml).
#
# Reescrita em 2026-08-26 a partir da sequência que o Marco desenhou:
# aproximar ao lado do furo, atravessar o degrau em X, DESCER para
# apoiar o arame, acompanhar o arco e SAIR lateralmente.
#
# A saída lateral é nova e não é detalhe: antes a missão ia direto para
# o recolhimento com a ferramenta ainda enfiada no anel, o que na
# bancada arrastaria a chave de volta.
# 'orienta' entrou em 2026-08-27, por observação do Marco vendo a
# missão rodar: a cinemática estava traçando a trajetória POR BAIXO da
# chave. A primeira fase já ficava 90 mm ao lado do olhal, mas o braço
# CHEGAVA lá partindo do recolhido — subia de baixo para cima, rente ao
# mecanismo, e na execução daquele dia a ferramenta bateu na lâmina
# (19 contatos tool_rod x chave_blade) e o robô tombou.
#
# 'orienta' é o mesmo ponto lateral, porém 150 mm AFASTADO da parede: o
# braço ganha altura e já assume a atitude com o degrau alinhado longe
# da chave, e só então entra. Com isso a sequência fica:
#
#   orienta          sobe e alinha, a 150 mm da parede
#   aproxima_lateral entra em -Y, mantendo o deslocamento lateral
#   atravessa        move em +X, o degrau enfia no furo
#
# ou seja, aproximação LATERAL, nunca de baixo para cima.
#
# 'destrava' e 'libera' entraram em 2026-09-03: o olhal da bancada está
# numa lingueta com mola, presa num laço do contato fixo. É preciso
# DESCER o olhal ~15 mm (solta a aba) e então puxar ~20 mm segurando
# embaixo (a aba passa sob o laço) antes de o arco existir. O Marco:
# "o primeiro movimento ... deve ser um movimento bem acentuado em -Z,
# pois é aí q ocorre o destravamento do gatilho que trava a chave."
PHASE_ORDER = ['orienta', 'aproxima_lateral', 'atravessa', 'captura',
               'destrava', 'libera', 'arco1', 'arco2', 'desengata']

# Fases em que a base NÃO deve participar do movimento: são
# deslocamentos ao longo do eixo do furo (X, lateral à parede), ou de
# afastamento, e a base não anda de lado. Quem faz é o braço.
# 'libera' também: são 3 cm de avanço segurando o olhal embaixo —
# curto demais para as rodas fazerem com precisão, e o braço tem
# folga de sobra ali. A base volta a puxar a partir do arco1.
PHASES_SO_BRACO = {'orienta', 'aproxima_lateral', 'atravessa', 'libera',
                   'desengata'}


def _yaw_of(quat):
    return euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]


def _ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class MissionContext(object):
    """Estado compartilhado + interface ROS, injetado em cada estado SMACH."""

    def __init__(self):
        self.rate_hz       = rospy.get_param('~rate', 20.0)
        # DISTÂNCIA DE ENGATE — 0,62 m, não 0,70.
        #
        # Medido em 2026-09-01 varrendo a pose da base: há um degrau
        # entre 0,62 e 0,66. Abaixo dele o braço consegue posição E
        # alinhamento no waypoint de inserção; acima, cede a direção.
        # Na pose de operação o pior desalinho ao longo das fases era
        # 13,6°; a 0,62 com lateral zero, 1,5°.
        #
        # 0,54 E NÃO 0,62 (2026-09-02). O critério de ontem era o
        # alinhamento da DIREÇÃO da haste, e por ele 0,62 estava ótimo.
        # Faltava medir a ROLAGEM do degrau em torno do próprio eixo —
        # que é o que decide se ele passa pelo aro, e que ninguém
        # imprimia. O degrau ERA uma caixa de 20x17 mm e a abertura livre
        # do oval é 30x40; rolado além de ~20° o canto saía do contorno.
        #
        # DEGRAU 10x10 DESDE 2026-09-02 (movemaster.urdf.xacro). Diagonal
        # de 14,1 mm: cabe no vão de 30 em qualquer rolagem. A restrição
        # que motivou 0,54 contra 0,62 deixou de existir na geometria —
        # o mapa abaixo continua válido como MEDIDA de rolagem, mas o
        # limite de ~20° que o tornava decisivo não vale mais. A escolha
        # da distância de engate fica para ser refeita na revalidação.
        #
        # Mapeado em 2026-09-02, pior rolagem nas fases de inserção:
        #
        #   dist \ lateral   -90mm     0    +60   +120   +170
        #        0,54          3,2*    2,7    7,1    4,2    7,0
        #        0,58         10,6     8,7   14,8   11,6   14,0
        #        0,62         18,0    16,1   22,4   18,9   21,1
        #        0,70         57,3    55,0   55,2   56,5   58,4
        #
        # A 0,62 a previsão era 16,1° com limite 20 — e ao vivo mediu
        # 24,8°: os ~9° de erro de percepção e rastreio consomem a
        # margem inteira.
        #
        # APROXIMAR NÃO FUNCIONOU (2026-09-02). O mapa da rolagem
        # apontava 0,54 (2,7°) e 0,58 (8,7°) como muito melhores que
        # 0,62 (16,1°). As duas foram TENTADAS e as duas COLIDIRAM: a
        # haste prensada contra `chave_plate` e `tag_plate`, com a
        # missão emperrando na fase de encenação. O mapa resolve
        # cinemática inversa e mede alcance e rolagem — ele NÃO tem
        # modelo de colisão, então serve como filtro e não como prova de
        # que a pose é viável.
        #
        # 0,62 fica porque é a última distância que roda de ponta a
        # ponta. A margem de rolagem continua fina (4°) e é problema
        # aberto: reduzi-la exige mexer no waypoint de encenação, que é
        # quem joga a haste contra a parede quando a base se aproxima.
        self.standoff_dist = rospy.get_param('~standoff_distance', 0.62)

        # DUAS DISTÂNCIAS, NÃO UMA (2026-08-25, pedido do Marco).
        #
        # O braço estende num ponto mais afastado da parede e só depois o
        # robô avança para engatar. Estender o braço já colado na parede
        # deixa punho e ferramenta varrendo perto dela durante toda a
        # rampa de postura, e o movimento de puxar o olhal esbarrava.
        # Separar os dois momentos dá folga para a postura se armar.
        self.deploy_dist = rospy.get_param('~deploy_distance', 0.88)
        # Velocidade do avanço/recuo fino, feito só com as rodas.
        self.creep_vel = rospy.get_param('~creep_velocity', 0.06)
        self.creep_timeout = rospy.get_param('~creep_timeout', 25.0)
        # PUXAR COM AS RODAS. A abertura afasta o olhal da parede em
        # 8,5 cm (ver pos2 no YAML). Fazer isso recuando a base, em vez
        # de recolher o braço, mantém a postura do braço estável e usa a
        # tração do chassi para a força — que é o que whole-body deveria
        # significar aqui.
        self.pull_with_base = rospy.get_param('~pull_with_base', True)
        # ABERTURA: por contato (padrão) ou por acoplamento cinemático.
        # O acoplamento continua disponível como regressão — ele separa
        # "a trajetória está certa" de "a força é suficiente", que são
        # perguntas diferentes e falham por motivos diferentes.
        self.blade_coupling = rospy.get_param('~blade_kinematic_coupling', False)
        # Acima disto a chave conta como aberta (o curso da junta é 30°).
        self.blade_open_deg = rospy.get_param('~blade_open_deg', 25.0)
        # Deslocamento lateral do estacionamento, ao longo da parede.
        # Casado com o offset X da primeira fase (aproxima_lateral): o
        # robô para já ao lado do furo, e o braço só avança em linha
        # reta para atravessar o degrau.
        # DESLOCAMENTO LATERAL DE ENGATE — zero.
        #
        # Era -90 mm, para o braço começar a travessia já ao lado do
        # furo e só precisar avançar em linha reta. O raciocínio é
        # plausível e a medição o contradiz: deslocar a base para
        # QUALQUER lado piora o alinhamento, nos dois sentidos e em
        # todas as distâncias varridas. De frente para o olhal é onde o
        # braço tem os dois graus de liberdade que a tarefa pede.
        self.standoff_lateral = rospy.get_param('~standoff_lateral', 0.0)
        self.search_omega  = rospy.get_param('~search_omega', 0.35)
        self.search_timeout   = rospy.get_param('~search_timeout', 90.0)
        self.approach_timeout = rospy.get_param('~approach_timeout', 120.0)
        self.posture_timeout  = rospy.get_param('~posture_timeout', 30.0)
        self.phase_timeout    = rospy.get_param('~phase_timeout', 60.0)
        self.refine_samples   = rospy.get_param('~refine_samples', 10)
        # O SEARCH mede a ~2,8 m, onde a tag subtende pouco e o PnP é
        # mais ruidoso — e onde um "flip" de ambiguidade de pose custa
        # caro, porque a pose da parede define para onde o robô dirige.
        # Na bateria de 2026-08-21 uma execução saiu do SEARCH com a
        # parede 0,58 m deslocada e o yaw 34° errado. Mais amostras (e
        # rejeição de outliers em _sample_wall) atacam justamente isso.
        # O REFINE mede de perto e continua com 10: ali a dispersão é
        # pequena e amostrar demais só custa tempo.
        #
        # VIÉS DE YAW DO SEARCH — ACEITO E DOCUMENTADO (2026-09-03). A
        # estimativa da parede no SEARCH sai com yaw ≈ −9° em quase toda
        # missão. Não é ruído que mais amostras resolvam: a ~2,8 m a tag
        # subtende ~23 px e a pose de um alvo PLANO tem duas soluções
        # quase igualmente boas, ~15° apartadas (ambiguidade do PnP); a
        # mediana escolhe um lado. O que esse yaw governa é só o rumo do
        # APPROACH; o REFINE remede de perto e fecha em ~0,3°, e é essa
        # a estimativa que a manipulação usa. Em 70 missões o APPROACH
        # nunca falhou por isso. Decisão do Marco: "aceita e documenta,
        # não vale a segunda vista" (girar e recoletar de outro ângulo
        # custaria ~10 s e mais uma etapa). Se um dia a primeira vista
        # precisar ser boa em yaw, a segunda vista é o caminho.
        self.search_samples   = rospy.get_param('~search_samples', 25)
        # ── PARADA DO SEARCH PELA POSIÇÃO DA TAG NO QUADRO (2026-09-02) ──
        #
        # O SEARCH parava na PRIMEIRA detecção, com a tag entrando pela
        # borda da fisheye (~71° fora do eixo). Medido na bancada de pose,
        # girando a base a partir dessa parada (tag de 132 mm a ~1,8 m):
        #
        #     fora do eixo   71°    60°    48°    35°    22°
        #     aceitas/5 s     19     27     47     41     22
        #     parede (m)    2,61   2,95   3,07   3,06   2,96   (real 2,98)
        #
        # Na borda o PnP mede 0,4–1,0 m curto; entre 22° e 48° erra 2–9 cm.
        # Com a parede curta, a etapa intermediária do APPROACH parava a
        # 1,6–2,1 m da tag em vez de 1,3 — na zona onde a tag (24 px) é
        # rejeitada por ambiguidade — e a remedida ficava sem amostras
        # (run2, run3, run5, run7 de 2026-09-02).
        #
        # Regra: avistada a tag, a base CONTINUA girando, devagar, até o
        # raio normalizado do tag_pixel (0 = centro, 1 = borda) cair a
        # ~search_raio_max, e só então para e amostra. Raio 0,6 ≈ 52° fora
        # do eixo na equidistante (r = f·θ, f = 285 px, meia-largura 424).
        #
        # Não é o gimbal: nenhuma malha em J1; é a base concluindo o giro
        # que já estava fazendo. Se a tag sumir do quadro durante o ajuste
        # ou o tempo esgotar, para onde estiver e amostra como antes.
        #
        # 0,5 E NÃO 0,6 (run8, 2026-09-02): parado a raio 0,60 (~52°) a tag
        # era detectada em todo quadro e REJEITADA em todo quadro, com a
        # razão de ambiguidade congelada em 1,37 e 1,32 — duas coletas de
        # 25 s com 1 e 0 amostras; a terceira parada, uns graus adiante,
        # rendeu 25 e a parede saiu em 3,069 (real 2,98). A curva da
        # bancada põe a aceitação robusta entre 35° e 48° (41–47 aceitas
        # em 5 s); 0,5 ≈ 43°. Mais para dentro que isso a tag pequena
        # (~22 px a 1,8 m) vira frontal e volta a ser ambígua.
        self.search_raio_max      = rospy.get_param('~search_raio_max', 0.5)
        # Recuo da BASE entre a saída pelo eixo e o recolhimento do braço.
        # Ver _afasta_da_parede: sem ele a rampa do stow leva a ferramenta
        # 100-120 mm para dentro da placa da chave.
        self.saida_recuo_m        = rospy.get_param('~saida_recuo_m', 0.25)
        # ── DITHER NAS COLETAS (2026-09-02) ──
        #
        # Com o robô PERFEITAMENTE parado (depois do fdir1 nas rodas), os
        # quadros da fisheye ficam idênticos e a razão de ambiguidade do
        # PnP congela: a mesma pose é aceita ou rejeitada em 100% dos
        # quadros, conforme o lado de 2,0 em que ela caiu. Medido na
        # bancada de pose com J5 = 0, a 1,5-1,8 m: a 60°, 24° e 4° fora do
        # eixo, 37-74 detecções e ZERO aceitas em 5 s; a 49°, 37° e 12°,
        # 23-41 aceitas. Antes, os 2,6 mm/s de deriva eram um dither
        # involuntário que desfazia isso.
        #
        # E o yaw da parede medido no SEARCH sai com SINAL ALTERNADO de
        # 11 a 17° (−16,7°, +17,5°, −11,1° em poses vizinhas) com a
        # posição boa: ambiguidade de pose planar de uma tag de 23 px —
        # a rotação em torno da vertical é o que os cantos menos
        # restringem. Uma coleta parada pega um lado só; girando a base
        # devagar, as amostras alternam e a média circular cai perto de
        # zero.
        #
        # ±coleta_dither_omega rad/s, invertendo a cada meio período: a
        # ±0,05 por 1,5 s são ±4° de rumo, que a navegação seguinte
        # (APPROACH/DEPLOY) refaz de qualquer jeito. 0 desliga.
        self.coleta_dither_omega   = rospy.get_param('~coleta_dither_omega', 0.05)
        self.coleta_dither_periodo = rospy.get_param('~coleta_dither_periodo', 3.0)
        self.search_omega_fina    = rospy.get_param('~search_omega_fina', 0.15)
        self.search_centra_timeout = rospy.get_param('~search_centra_timeout', 8.0)
        self.search_sample_timeout = rospy.get_param('~search_sample_timeout', 25.0)
        # Nome do modelo da fixture no Gazebo — usado para girar a
        # lâmina da chave (set_blade_angle).
        self.fixture_model    = rospy.get_param('~fixture_model_name',
                                                'chave_seccionadora_fixture')
        # Distância intermediária de aproximação: dentro da faixa
        # confiável da tag (132 mm → ~1,45 m pela regra d/11).
        self.coarse_distance  = rospy.get_param('~coarse_distance', 1.30)
        self.deploy_ik_tol    = rospy.get_param('~deploy_ik_tol', 0.02)
        # IK iterativa da manipulação fina (ver _reach_by_iterative_ik)
        self.ik_max_iters     = rospy.get_param('~ik_max_iters', 5)
        self.ik_settle_time   = rospy.get_param('~ik_settle_time', 1.2)
        self.ik_correction_gain = rospy.get_param('~ik_correction_gain', 0.8)
        self.refine_timeout   = rospy.get_param('~refine_timeout', 15.0)

        # Navegação (girar-avançar-girar)
        self.nav_v         = rospy.get_param('~nav_linear_vel', 0.15)
        self.nav_w         = rospy.get_param('~nav_angular_vel', 0.4)
        self.nav_pos_tol   = rospy.get_param('~nav_pos_tol', 0.06)
        self.nav_yaw_tol   = rospy.get_param('~nav_yaw_tol', 0.09)
        # ── Alinhamento do degrau com o eixo do furo ──
        #
        # Com False, a manipulação volta à IK de posição pura, em que a
        # atitude da ferramenta é a que o ramo da IK calhar de entregar.
        # Existe como chave de comparação, não como modo de operação: na
        # bateria de 2026-08-27 a IK de posição pura entregou o degrau
        # PERPENDICULAR ao furo nas seis fases (89,6° / 88,2° / 88,2° /
        # 85,7° / 84,3° / 79,8°), e a chave não abriu em nenhuma das 8.
        self.degrau_alinhado = rospy.get_param('~degrau_alinhado', True)

        # FORMULAÇÃO DE IK DA MISSÃO INTEIRA, decidida no DEPLOY.
        #
        # Era decidida por FASE, e isso produzia o defeito que o Marco
        # viu na tela em 2026-08-27: o DEPLOY resolvia com uma
        # formulação e a primeira fase com outra, caindo em RAMOS DE
        # SOLUÇÃO diferentes. O destino era quase o mesmo ponto, mas o
        # punho girava 139° e o rolamento 164° para ir de um ramo ao
        # outro — a ferramenta descia, contornava e voltava a subir por
        # dentro do mecanismo.
        #
        # A continuidade de ramo já existia DENTRO de cada resolução
        # (semente = postura atual), mas ramo é escolha global: mudar de
        # formulação no meio invalida a semente. Uma escolha por missão,
        # feita onde o braço é pré-posicionado, mantém o braço no mesmo
        # ramo do começo ao fim.
        self.ik_modo = None

        # Saída pelo eixo do furo antes de recolher (ver _sai_pelo_eixo).
        # raio: a partir de que distância do olhal a ponta é considerada
        # "engatada"; curso: quanto recuar ao longo do eixo.
        self.saida_raio  = rospy.get_param('~saida_raio', 0.25)
        self.saida_curso = rospy.get_param('~saida_curso', 0.12)

        # Âncora da base — contenção da deriva do modelo (ver os métodos
        # ancora_*). Tolerância de 2 mm: acima da amplitude do contato em
        # repouso e bem abaixo do que a deriva acumula numa pausa.
        # Identificador desta execução (relógio de parede do arranque).
        self._run_id = '%d' % int(time.time())
        self.ancora_base  = rospy.get_param('~ancora_base', True)
        self.ancora_tol   = rospy.get_param('~ancora_tol', 0.002)
        self.robot_model  = rospy.get_param('~robot_model_name', 'b166er')
        self._ancora_pose = None
        self._ancora_correcoes = 0
        self._ancora_get = None
        self._ancora_set = None

        # MEDIR DE FRENTE, DESLOCAR DEPOIS.
        #
        # O REFINE acontecia já na pose lateral (deslocada 90 mm ao longo
        # da parede), e ali a tag cai na PERIFERIA do fisheye — a pior
        # região do modelo equidistante. Medido em 2026-08-27, na mesma
        # distância de 0,70 m:
        #
        #   com deslocamento lateral .. tag NÃO detectada sem o gimbal
        #   sem deslocamento .......... viés -2,3 mm, dispersão 0,9 mm
        #
        # E nas 13 execuções pós-correção, medindo na pose lateral, a
        # profundidade ficou com mediana +10 mm e DESVIO DE 9,2 mm — em
        # 2 delas a estimativa já nascia fora do vão de ±15 mm do furo,
        # antes de o braço se mexer. O problema deixou de ser viés (que
        # se compensa) e virou dispersão (que não se compensa).
        #
        # Agora o robô para DE FRENTE para medir, e só então assume o
        # deslocamento lateral por navegação. Paga o erro de odometria de
        # uma manobra curta para ganhar uma ordem de grandeza na medida.
        self.refine_de_frente = rospy.get_param('~refine_de_frente', True)

        # Postura que ANCORA o ramo de solução no DEPLOY (ver a nota lá).
        # Precisa ser estável e conhecida — 'search' serve porque é onde
        # o braço passa antes de qualquer manipulação.
        # ÂNCORA DE RAMO — postura "deploy", não "search".
        #
        # A IK usa esta postura como semente E como critério de
        # continuidade, então é ela que decide em qual FAMÍLIA de
        # configuração o braço cai. A "search" é neutra: J4 = 0, a meio
        # caminho entre as duas famílias, e a escolha saía no acaso.
        #
        # MEDIDO em 2026-09-01, quatro execuções com a mesma geometria:
        #
        #   J4 no DEPLOY   degrau no DEPLOY   degrau na "atravessa"
        #      +73,8°           12,7°               14,1°
        #      +89,0°            4,7°                8,7°
        #      -79,9°            0,2°                0,6°
        #      +82,1°            9,0°               19,6°
        #
        # Só a família J4 NEGATIVO sustenta o alinhamento ao longo das
        # fases; nas outras ele degrada a cada waypoint. E 19,6° numa
        # haste de 200 mm dão 32 mm de desvio no percurso de 90 mm da
        # atravessa, contra 18 mm de semi-eixo do oval — a haste entra
        # torta e bate na borda em vez de enfiar.
        #
        # A postura "deploy" ([0, 0.6, -0.4, -1.2, 0]) tem J4 = -68,8° e
        # já existia, descrita como pré-manipulação. É a família certa,
        # e o padrão de sinais bate com a execução boa.
        self.ancora_ramo = rospy.get_param('~ancora_ramo', 'deploy')

        # ── EXECUÇÃO POR ETAPA ──
        #
        # Pedido do Marco em 2026-08-31: "vamos realizar a simulação por
        # etapa, ao final de cada etapa a gente analisa a trajetória".
        #
        # O motivo é o padrão que se repetiu o dia inteiro: ele enxerga o
        # problema na tela (o choque com a chave, a trajetória por baixo,
        # a ferramenta por trás do olhal) e eu só descubro depois, no
        # log — e nesse intervalo já empilhei mudanças em cima de um
        # diagnóstico errado. Parar em cada fase inverte a ordem.
        #
        # Com isto ligado, a missão congela ao fim de CADA fase (inclusive
        # nas que falham, que são as que mais interessam olhar) e espera
        # /b166er/mission_continue. O braço fica exatamente onde parou.
        self.pausa_por_fase = rospy.get_param('~pausa_por_fase', False)
        self._continuar = False
        rospy.Subscriber('/b166er/mission_continue', Empty,
                         lambda _m: setattr(self, '_continuar', True))

        # Voltar à pose de partida também quando a missão ABORTA. Sem
        # isso o robô fica onde a falha o pegou, encostado na chave, e a
        # execução seguinte começa de lá.
        self.retorna_no_abort = rospy.get_param('~retorna_no_abort', True)

        # Recolher o braço nos trechos de deslocamento do APPROACH.
        #
        # DESLIGADO POR PADRÃO até ser corrigido. A ideia é boa e a
        # medição que a motiva é sólida — a postura alinhada joga a
        # ferramenta 735 mm para fora do eixo J1 contra 388 mm da
        # recolhida, e são 28 kg de braço num robô que já tombou. Mas
        # ligar isto REGREDIU a missão em 2026-08-27: com as sete fases
        # fechando antes, passou a não fechar nenhuma.
        #
        # Causa provável, não confirmada: trocar de postura no meio do
        # APPROACH muda o ramo em que o braço chega à manipulação, e a
        # fase 'orienta' deixa de partir de onde partia. A continuidade
        # de ramo é justamente o que segura a repetibilidade aqui.
        #
        # Para retomar: comparar A/B com este parâmetro, olhando o ramo
        # (J2/J4/J5) com que cada fase começa, não só o erro final.
        self.recolhe_para_andar = rospy.get_param('~recolhe_para_andar', False)

        # Distância mínima (centro da base → obstáculo) pelo laser.
        #
        # SUBIU de 0,55 para 0,64 em 2026-08-27. O número é medido do
        # CENTRO da base, e foi calibrado quando a borda dianteira ficava
        # a 0,257 m do centro — ou seja, valia 0,29 m de folga física.
        # Com a plataforma alongada 92 mm, a borda passou para 0,349 m e
        # a MESMA constante virou 0,20 m de folga, sem ninguém mexer
        # nela. 0,64 = 0,349 + 0,29 devolve a margem original.
        self.min_clearance = rospy.get_param('~min_clearance', 0.64)

        # ── Manipulação por whole-body (ver _reach_by_wholebody) ──
        # DESLIGADO por padrão até ser medido contra o caminho atual.
        #
        # O caminho whole-body foi reativado em 2026-08-24 e faz o que
        # promete (arm_vel_cmd saiu de zero pela primeira vez), mas nas
        # execuções feitas até agora ele NÃO melhorou a missão: o erro do
        # pre_engage ficou em 0,3758 m contra 0,3814 m da IK iterativa —
        # praticamente idêntico, o que aliás indica que a causa daquela
        # falha não está em nenhum dos dois controladores.
        #
        # Ligar um caminho não validado por padrão troca um problema
        # conhecido por um desconhecido. Fica opt-in
        # (use_wholebody:=true) até uma bateria comparar os dois.
        self.use_wholebody     = rospy.get_param('~use_wholebody', False)
        # MODO POR FASE (2026-09-03, "monta o híbrido por fase"). As duas
        # baterias do dia disseram o que cada modo sabe fazer: a IK
        # iterativa fecha tolerância por eixo e alinha o degrau (fases de
        # geometria); o whole-body insiste até o critério ceder e traz a
        # base (fases de contato) — mas estaciona em profundidade fina e,
        # com a base solta, empurrou a lâmina de volta no desengata.
        # Cada fase declara `modo: ik | wb` no YAML. Este parâmetro
        # sobrepõe tudo: 'yaml' (padrão) respeita o YAML; 'ik' e 'wb'
        # forçam um modo só, para reproduzir as baterias puras. O antigo
        # use_wholebody:=true equivale a 'wb'.
        self.modo_fases = rospy.get_param('~modo_fases', 'yaml')
        # Altura da ponta (frame da parede) no instante em que a captura
        # fechou: é o zero do critério observável do destrava. Ver
        # _fase_fechou e a nota em chave_seccionadora_task.yaml.
        self.alt_captura = None
        self.offset_efetivo = None   # offset da fase corrente, já ajustado
        if self.use_wholebody and self.modo_fases == 'yaml':
            self.modo_fases = 'wb'
        if self.modo_fases not in ('yaml', 'ik', 'wb'):
            rospy.logwarn('[mission] modo_fases=%r desconhecido — usando yaml',
                          self.modo_fases)
            self.modo_fases = 'yaml'
        self.wb_phase_timeout  = rospy.get_param('~wb_phase_timeout', 40.0)
        # Amostras consecutivas dentro da tolerância (a 10 Hz) para
        # aceitar a fase. Um instante dentro dela pode ser a ponta
        # passando de raspão no transitório.
        self.wb_settle_samples = rospy.get_param('~wb_settle_samples', 5)

        # Tolerância Cartesiana da manipulação: 20 mm, NÃO os 5 mm que
        # o fuzzy_wb_controller usa por padrão.
        #
        # Razão: não faz sentido exigir do controle uma precisão maior
        # que a da MEDIÇÃO do alvo. A posição do olhal vem da detecção
        # da tag, que mede com ~9 mm de erro (validado contra ground
        # truth em 2026-08-13, três poses). Perseguir 5 mm de erro de
        # EE contra um alvo conhecido a ±9 mm é perseguir ruído — e na
        # prática o sistema estacionava em 12 mm com o ganho Fuzzy já
        # reduzido perto do alvo, gastando o timeout inteiro sem
        # fechar. 20 mm é compatível com o engate: o vão do gancho tem
        # ~20 mm e o anel do olhal 28 mm de raio.
        self.tol_pos    = rospy.get_param('~tol_pos', 0.020)
        self.tol_orient = rospy.get_param('~tol_orient', 0.02)

        self.postures = rospy.get_param('/arm_postures')
        self.phases   = rospy.get_param('/chave_seccionadora_task')

        # Gimbal ("pescoço de galinha", ideia do Marco em 2026-08-13):
        # manter a tag centrada no quadro girando J1 enquanto a base
        # navega. Sem isso a tag sai do campo de visão na aproximação e a
        # estimativa degrada justamente quando mais precisa ser boa.
        # ── GIMBAL DESLIGADO POR PADRÃO ──
        #
        # A malha girava J1 para recentrar a tag no quadro. MEDIDO em
        # 2026-08-31, com a base PARADA e varrendo J1 de -59° a +15°:
        #
        #     J1 = -30,4°  ->  tag em +0,094
        #     J1 = -14,9°  ->  tag em +0,110
        #     J1 =   0,0°  ->  tag em +0,134
        #
        # Ganho de 0,075 por radiano, e com o SINAL INVERTIDO em relação
        # ao que a malha assume. Produto de malha 0,04: para corrigir um
        # erro de 0,10 seriam ~25 passos na direção certa — na direção
        # errada, nunca. Foi o que se viu: o erro preso em +0,06..+0,11
        # (sempre acima do deadband, portanto nunca satisfeito) enquanto
        # J1 percorria 80° até quase o batente, levando a câmera para
        # longe da tag e zerando as amostras do REFINE.
        #
        # A razão é geométrica: a câmera vive no END-EFFECTOR, não numa
        # torre. Girar J1 TRANSLADA o braço em arco — muda muito a
        # posição da câmera e quase nada o apontamento. J1 é o atuador
        # errado para esta tarefa.
        #
        # Sem gimbal, a detecção depende de parar numa pose em que a tag
        # já esteja no quadro. Medido na mesma pose de standoff: tag
        # detectada com 60 px de lado e o localizador publicando 36
        # amostras em 6 s. É o que este teste vai verificar de ponta a
        # ponta.
        self.usa_gimbal     = rospy.get_param('~usa_gimbal', False)
        self.gimbal_gain    = rospy.get_param('~gimbal_gain', 0.6)
        self.gimbal_deadband = rospy.get_param('~gimbal_deadband', 0.06)
        self.gimbal_period  = rospy.get_param('~gimbal_period', 0.4)   # s
        self.tag_pixel      = None
        self.tag_pixel_t    = None
        self._j1_track      = None   # J1 corrente do rastreamento

        self.robot_state  = None
        self.wall_pose    = None
        self.posture_done = False

        self.start_pose    = None   # (x, y, yaw) registrado no STOW_INIT
        self.wall_pos      = None   # np(3,) estimado por visão
        self.wall_R        = None   # np(3,3)
        self.ee_orientation = None  # orientação do EE fixada na manipulação

        self.pub_cmdvel  = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_target  = rospy.Publisher('/b166er/ee_target', PoseStamped,
                                           queue_size=1, latch=True)
        self.pub_posture = rospy.Publisher('/b166er/arm_posture_cmd', JointState,
                                           queue_size=1, latch=True)
        # Arbitragem de /cmd_vel entre a missão (navegação de base) e o
        # fuzzy_wb_controller (manipulação whole-body). Sem isso os dois
        # publicam intercalado a 20 Hz e a base anda aos trancos.
        # HUD: estado da máquina publicado como texto chave=valor.
        #
        # Pedido do Marco em 2026-08-27: "a cada etapa da máquina de
        # estados, coloque a informação na simulação". O motivo é
        # concreto — hoje eu diagnostico lendo números depois do fato e
        # ele diagnostica olhando a tela, e ele acertou três vezes onde
        # eu errei justamente por isso. Publicar torna o estado visível
        # ao vivo para os dois.
        #
        # É só publicação: nenhuma decisão da missão depende disto, para
        # o HUD não poder quebrar a missão.
        self.pub_status = rospy.Publisher('/b166er/mission_status', String,
                                          queue_size=5, latch=True)
        self._status_campos = {}

        self.pub_wb_enable = rospy.Publisher('/b166er/wb_enable', Bool,
                                             queue_size=1, latch=True)
        # Trava de base durante a manipulação: o whole-body não conhece a
        # parede e já dirigiu a base contra ela tentando alcançar o olhal
        # (2026-08-13, robô tombado). Depois do standoff, só o braço serve
        # o alvo.
        self.pub_base_lock = rospy.Publisher('/b166er/base_lock', Bool,
                                             queue_size=1, latch=True)
        # Plano de exclusão da base (substitui a trava total). Ver
        # _apply_keepout no fuzzy_wb_controller.
        self.pub_keepout   = rospy.Publisher('/b166er/base_keepout', PoseStamped,
                                             queue_size=1, latch=True)
        # Alvo publicado como posição da PONTA da ferramenta (não do
        # T265) durante a manipulação — ver fuzzy_wb_controller, 4a.
        self.pub_servo_tip = rospy.Publisher('/b166er/servo_tooltip', Bool,
                                             queue_size=1, latch=True)
        self.pub_markers   = rospy.Publisher('/b166er/mission_markers',
                                             MarkerArray, queue_size=1, latch=True)

        rospy.Subscriber('/b166er/robot_state', RobotState, self._cb_state)
        rospy.Subscriber('/b166er/wall_pose', PoseStamped, self._cb_wall)
        rospy.Subscriber('/b166er/arm_posture_reached', Bool, self._cb_posture)
        rospy.Subscriber('/b166er/tag_pixel', Point, self._cb_tag_pixel)
        # Segurança: tombamento detectado pela IMU aborta a missão.
        # Antes disso o robô tombou várias vezes e a máquina de estados
        # seguia tentando, alheia — a IMU existia e não era usada.
        self.tilt_critical = False
        rospy.Subscriber('/b166er/tilt_critical', Bool, self._cb_tilt)
        # Distância livre medida pelo Hokuyo — usada na navegação para
        # parar antes de encostar, sem depender da estimativa da tag.
        self.front_clearance = None
        rospy.Subscriber('/b166er/front_clearance', Float64,
                         self._cb_clearance)

    # ------------------------------------------------------------------
    def _cb_state(self, msg):
        self.robot_state = msg

    def _cb_wall(self, msg):
        self.wall_pose = msg

    def _cb_posture(self, msg):
        if msg.data:
            self.posture_done = True

    def _cb_clearance(self, msg):
        self.front_clearance = msg.data

    def _cb_tilt(self, msg):
        if msg.data and not self.tilt_critical:
            rospy.logerr('[mission] IMU reportou inclinação crítica — '
                         'abortando a missão')
        self.tilt_critical = msg.data

    def _cb_tag_pixel(self, msg):
        self.tag_pixel   = msg
        self.tag_pixel_t = rospy.Time.now()

    # ------------------------------------------------------------------
    def gimbal_track(self):
        """Ajusta J1 para recentrar a tag no quadro (rastreamento gimbal).

        Chamado periodicamente durante a navegação. Feedback PURO DE
        PIXEL: não depende da estimativa de mundo (que é justamente o
        que está ruim quando o robô está longe), só de onde a tag
        aparece na imagem. Deadband evita ficar caçando ruído.
        """
        if self.tag_pixel is None or self.tag_pixel_t is None:
            return
        if (rospy.Time.now() - self.tag_pixel_t).to_sec() > 1.0:
            return   # detecção velha: tag sumiu, não persegue fantasma
        off_x = self.tag_pixel.x
        if abs(off_x) < self.gimbal_deadband:
            return

        base = list(self.postures['search'])
        if self._j1_track is None:
            self._j1_track = base[0]
        # offset positivo = tag à direita no quadro; o eixo óptico é +X
        # do link e J1 gira em torno de Z, então recentrar pede J1
        # DIMINUINDO. Sinal confirmado ao vivo.
        self._j1_track = float(np.clip(self._j1_track - self.gimbal_gain * off_x,
                                       -2.6, 2.6))
        q = list(base)
        q[0] = self._j1_track
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name     = JOINT_NAMES
        msg.position = q
        self.pub_posture.publish(msg)   # sem esperar: rastreamento é contínuo
        rospy.loginfo_throttle(2.0,
            '[mission] gimbal: tag em x=%+.2f do centro → J1=%.3f rad',
            off_x, self._j1_track)

    # ------------------------------------------------------------------
    # ÂNCORA DA BASE — contenção da deriva do modelo do Pioneer.
    #
    # O QUE É A DERIVA. Na simulação o robô avança sozinho, em linha
    # reta no próprio eixo, a 2,6 mm/s — 16 cm por minuto. Medido em
    # 2026-08-31 e reproduzido em toda condição testada:
    #
    #   stack completo .............. 2,58 mm/s
    #   TODOS os nós derrubados ..... 2,64 mm/s
    #   braço recolhido / estendido . 2,56 / 2,66 mm/s
    #   controladores lig. / parados  2,643 / 2,646 mm/s
    #   /cmd_vel zero reafirmado .... 2,645 mm/s
    #   solver de 50 a 1000 iters ... 6,42 a 6,16 mm/s (rodas livres)
    #
    # É ADITIVA ao comando: mandando 0,05 m/s por 4 s, as rodas
    # traseiras rolaram os 200,3 mm comandados e a base andou 211,4 —
    # a diferença são os 10,6 mm da deriva no mesmo intervalo.
    #
    # DE ONDE VEM. Do modelo de contato das rodas, não do controle: o
    # <fdir1> está comentado nas quatro rodas do Pioneer (com link para
    # o bug upstream do MobileRobots), então o ODE escolhe uma base de
    # atrito arbitrária a cada contato cilindro-plano e o resíduo é
    # retificado em avanço constante. O mesmo arquivo já documenta o
    # sintoma quando ele era dez vezes maior, a ~20-30 mm/s, atacado na
    # época subindo o kd do contato de 100 para 1e4. Isto aqui é o que
    # sobrou.
    #
    # O QUE ISTO NÃO É. Não é comando residual do plugin — essa foi a
    # minha primeira explicação e ela está REFUTADA: a deriva continua
    # com o zero sendo reafirmado a 5 Hz e com o stack inteiro morto.
    # Também não é assentamento (não decai em 80 s), nem inclinação do
    # chão (o rumo acompanha o yaw do robô nas quatro orientações
    # testadas), nem os controladores do braço, nem erro numérico.
    #
    # POR QUE ANCORAR EM VEZ DE CORRIGIR O MODELO. Declarar o <fdir1>
    # é a correção de verdade, mas mexe na tração das quatro rodas que
    # hoje funciona e obriga a revalidar navegação e aproximação. Fica
    # para a bateria de spawn em posições diferentes, que exercita a
    # tração de qualquer jeito. Até lá isto contém o efeito onde ele
    # atrapalha: parado, esperando inspeção ou medindo.
    #
    # SÓ VALE NA SIMULAÇÃO. Teleportar o modelo não existe na bancada;
    # se o serviço não estiver no ar, a âncora se desliga sozinha e a
    # missão segue igual.
    def _ancora_srv(self):
        if self._ancora_get is None:
            rospy.wait_for_service('/gazebo/get_model_state', timeout=2.0)
            rospy.wait_for_service('/gazebo/set_model_state', timeout=2.0)
            self._ancora_get = rospy.ServiceProxy(
                '/gazebo/get_model_state', GetModelState)
            self._ancora_set = rospy.ServiceProxy(
                '/gazebo/set_model_state', SetModelState)
        return self._ancora_get, self._ancora_set

    def ancora_engata(self, motivo=''):
        """Guarda a pose atual do modelo como a pose a manter."""
        self._ancora_pose = None
        self._ancora_correcoes = 0
        if not self.ancora_base:
            return False
        try:
            get, _ = self._ancora_srv()
            r = get(self.robot_model, 'world')
            if not r.success:
                return False
            self._ancora_pose = r.pose
            rospy.loginfo('[mission] âncora engatada%s',
                          (' (%s)' % motivo) if motivo else '')
            return True
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn_throttle(
                30.0, '[mission] âncora indisponível (%s) — sem contenção '
                      'de deriva; é o esperado fora do Gazebo', exc)
            self.ancora_base = False
            return False

    def ancora_mantem(self):
        """Devolve a base à pose ancorada se ela tiver escapado.

        Corrige só acima do limiar para não brigar com a física a cada
        ciclo: dentro da tolerância, deixa quieto.
        """
        if self._ancora_pose is None:
            return
        try:
            get, set_ = self._ancora_srv()
            r = get(self.robot_model, 'world')
            if not r.success:
                return
            a, b = self._ancora_pose.position, r.pose.position
            if math.hypot(b.x - a.x, b.y - a.y) < self.ancora_tol:
                return
            m = ModelState()
            m.model_name = self.robot_model
            m.pose = self._ancora_pose
            m.reference_frame = 'world'
            set_(m)
            self._ancora_correcoes += 1
        except (rospy.ServiceException, rospy.ROSException):
            pass

    def ancora_solta(self):
        """Libera a base. Devolve quantas correções foram precisas."""
        n = self._ancora_correcoes
        if self._ancora_pose is not None and n:
            rospy.loginfo('[mission] âncora solta — %d correções de deriva', n)
        self._ancora_pose = None
        self._ancora_correcoes = 0
        return n

    def blade_angle_now(self):
        """Ângulo atual da lâmina, lido do Gazebo (graus). None se falhar.

        Com a abertura por contato, este é o critério de sucesso da
        tarefa: a chave abriu ou não abriu. Antes o ângulo era imposto
        pela própria missão, então perguntar por ele seria perguntar o
        que ela mesma acabara de mandar.
        """
        try:
            rospy.wait_for_service('/gazebo/get_joint_properties', timeout=2.0)
            srv = rospy.ServiceProxy('/gazebo/get_joint_properties',
                                     GetJointProperties)
            r = srv('chave_blade_joint')
            if r.success and len(r.position):
                return math.degrees(r.position[0])
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn_throttle(10.0,
                '[mission] não consegui ler o ângulo da lâmina (%s)', exc)
        return None

    def lingueta_now(self):
        """Curso atual da lingueta (mm, positivo = olhal para baixo), lido
        do Gazebo. None se falhar.

        Existe desde 2026-09-03, junto com o gatilho no fixture: é a
        medida de que a fase 'destrava' fez o que diz. Chegar ao alvo em
        Z com a ferramenta não prova que o OLHAL desceu — a haste pode
        ter escorregado pelo furo. A junta prova.
        """
        try:
            rospy.wait_for_service('/gazebo/get_joint_properties', timeout=2.0)
            srv = rospy.ServiceProxy('/gazebo/get_joint_properties',
                                     GetJointProperties)
            r = srv('chave_lingueta_joint')
            if r.success and len(r.position):
                return 1000.0 * r.position[0]
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn_throttle(10.0,
                '[mission] não consegui ler a lingueta (%s)', exc)
        return None

    def set_blade_angle(self, deg):
        """Gira a lâmina da chave para o ângulo dado (graus).

        SÓ ATUA COM ~blade_kinematic_coupling (2026-08-25). Com a
        abertura por contato ligada, este método vira observação: a
        lâmina precisa girar porque o gancho puxou. Impor o ângulo aqui
        mascararia exatamente a pergunta que a bancada real vai fazer.

        A chave só ganhou junta revoluta em 2026-08-13 — antes era um
        bloco rígido e não abria quando o robô puxava, como o Marco
        observou. Aqui o ângulo é IMPOSTO cinematicamente, acompanhando
        o waypoint que a ferramenta está executando, em vez de emergir
        do contato gancho-anel.

        Por que não por contato: puxar um anel com um gancho é um
        problema de contato fino (duas superfícies curvas, engate
        geométrico), que o ODE com colisões primitivas não resolve de
        forma confiável — a ferramenta atravessaria ou escorregaria. O
        acoplamento cinemático dá a demonstração correta do movimento
        de abertura, que é o que a missão precisa mostrar. Simular o
        contato de verdade exigiria malhas de colisão e um solver mais
        caro, e não muda nada no que está sendo validado aqui (a
        navegação, a percepção e o alcance do braço).
        """
        if not self.blade_coupling:
            ang = self.blade_angle_now()
            rospy.loginfo('[mission] waypoint de %.0f° — lâmina medida em %s '
                          '(abertura por CONTATO)', deg,
                          '%.1f°' % ang if ang is not None else '?')
            return
        try:
            rospy.wait_for_service('/gazebo/set_model_configuration', timeout=2.0)
            srv = rospy.ServiceProxy('/gazebo/set_model_configuration',
                                     SetModelConfiguration)
            srv(model_name=self.fixture_model,
                urdf_param_name='chave_fixture_description',
                joint_names=['chave_blade_joint'],
                joint_positions=[math.radians(deg)])
            rospy.loginfo('[mission] chave girada para %.0f° (%s)', deg,
                          'aberta' if deg > 25 else
                          'fechada' if deg < 5 else 'parcial')
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn('[mission] não consegui girar a lâmina: %s', e)

    def settle_gimbal(self):
        """Encerra o rastreamento gimbal de forma limpa.

        Reenvia a postura corrente como alvo e espera 'reached', para
        garantir que a ponte de braço não fique com uma rampa pendente
        — enquanto houver, ela ignora /b166er/arm_vel_cmd e o
        controlador Cartesiano não move nada.
        """
        if self._j1_track is None:
            return
        q = list(self.postures['search'])
        q[0] = self._j1_track
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name     = JOINT_NAMES
        msg.position = q
        self.posture_done = False
        self.pub_posture.publish(msg)
        t0 = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.posture_done:
            if (rospy.Time.now() - t0).to_sec() > 5.0:
                rospy.logwarn('[mission] gimbal não assentou em 5s')
                return
            rate.sleep()

    # ------------------------------------------------------------------
    def publish_markers(self, goal=None):
        """Marcadores para o RViz: alvo de navegação, olhal e caminho.

        Antes disso o RViz não mostrava NADA sobre o deslocamento (o
        Marco perguntou por quê): a missão navega com controlador
        próprio escrevendo direto em /cmd_vel, sem move_base e sem
        publicar Path/Marker — não havia o que desenhar.
        """
        arr = MarkerArray()
        now = rospy.Time.now()

        def _mk(mid, mtype, scale, color):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = now
            m.ns = 'chave_mission'
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = color
            m.pose.orientation.w = 1.0
            return m

        if self.wall_pos is not None and self.wall_R is not None:
            olhal = chave_task.olhal_position(self.wall_pos, self.wall_R)
            m = _mk(0, Marker.SPHERE, (0.08, 0.08, 0.08), (1.0, 0.4, 0.0, 0.9))
            m.pose.position.x, m.pose.position.y, m.pose.position.z = olhal
            arr.markers.append(m)

        if goal is not None and self.robot_state is not None:
            gx, gy, _ = goal
            m = _mk(1, Marker.CYLINDER, (0.25, 0.25, 0.02), (0.0, 0.8, 1.0, 0.8))
            m.pose.position.x, m.pose.position.y, m.pose.position.z = gx, gy, 0.01
            arr.markers.append(m)

            x, y, _ = self.base_pose()
            line = _mk(2, Marker.LINE_STRIP, (0.03, 0.0, 0.0), (0.0, 0.8, 1.0, 0.7))
            line.points = [Point(x=x, y=y, z=0.05), Point(x=gx, y=gy, z=0.05)]
            arr.markers.append(line)

        if arr.markers:
            self.pub_markers.publish(arr)

    # ------------------------------------------------------------------
    def wait_for_state(self, timeout=10.0):
        t0 = rospy.Time.now()
        while self.robot_state is None and not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > timeout:
                return False
            rospy.sleep(0.1)
        return self.robot_state is not None

    def base_pose(self):
        """(x, y, yaw) da base, do /b166er/robot_state."""
        p = self.robot_state.base_odom.pose.pose
        return p.position.x, p.position.y, _yaw_of(p.orientation)

    def stop_base(self):
        self.pub_cmdvel.publish(Twist())

    def espera_etapa(self, etapa, ok, detalhe=''):
        """Congela ao fim de uma etapa até receber /b166er/mission_continue.

        Não altera nada do estado do robô: só não retorna. O braço fica
        na postura em que terminou e a base parada, para a inspeção
        visual valer.
        """
        if not self.pausa_por_fase:
            return True
        self._continuar = False
        self._ultimo_aviso = 0.0
        self.stop_base()
        rospy.logwarn('[mission] ⏸  PAUSA após "%s" (%s)%s — aguardando '
                      '/b166er/mission_continue',
                      etapa, 'ok' if ok else 'FALHOU',
                      (' | ' + detalhe) if detalhe else '')
        # NÃO sobrescreve `estado`: o painel usa esse campo para marcar a
        # trilha da máquina, e trocá-lo por "PAUSA" apagava justamente a
        # informação de ONDE a missão parou — que é o que o Marco quer
        # ver na pausa. A pausa é uma condição, não um estado.
        self.status(pausado=1, etapa_pausa=etapa,
                    pausa_ok=(1 if ok else 0), detalhe=detalhe)
        # RELÓGIO DE PAREDE, NÃO TEMPO SIMULADO.
        #
        # Este laço usava rospy.Rate/rospy.Time, que dormem em tempo
        # SIMULADO quando use_sim_time está ligado. Pausar a física do
        # Gazebo — um clique no gzclient, que é a coisa mais natural do
        # mundo para inspecionar a ferramenta parada — congelava este
        # laço para sempre: o /clock não avança, o r.sleep() nunca
        # retorna, o _continuar deixa de ser checado e a missão fica
        # irrecuperável. Nem o mission_continue nem um Ctrl-C limpo
        # surtem efeito.
        #
        # Aconteceu em 2026-08-31, na pausa após a fase "orienta": nó
        # vivo e respondendo a ping, status ainda dizendo pausado=1, e
        # /clock, /joint_states e /b166er/robot_state todos silenciosos.
        # Custou a execução inteira.
        #
        # Aqui não há nada de física para sincronizar — a missão está
        # justamente PARADA, esperando uma pessoa olhar. O relógio de
        # parede é o certo, e de quebra permite pausar o Gazebo para
        # inspecionar sem perder o controle da missão.
        t0 = time.time()
        # ÂNCORA. O zero reafirmado abaixo continua valendo — protege
        # contra o plugin skid_steer, que não tem commandTimeout e
        # mantém a última velocidade recebida para sempre. Mas ele NÃO
        # explicava o robô andando na pausa, e essa foi a minha primeira
        # conclusão, errada: medido em 2026-08-31, a base derivava 2,6
        # mm/s com o zero sendo publicado a 5 Hz, com o braço a 0,00° de
        # variação e depois com o stack INTEIRO derrubado. A deriva é do
        # modelo de contato das rodas (ver ancora_engata). O Marco viu na
        # tela, três vezes, antes de eu aceitar o número.
        self.ancora_engata('pausa em "%s"' % etapa)
        while not rospy.is_shutdown() and not self._continuar:
            self.stop_base()
            self.ancora_mantem()
            agora = time.time()
            if agora - t0 > 15.0 and agora - self._ultimo_aviso > 15.0:
                self._ultimo_aviso = agora
                rospy.logwarn('[mission] ⏸  ainda pausado em "%s" — publique '
                              'em /b166er/mission_continue para seguir', etapa)
            time.sleep(0.2)
        self.ancora_solta()
        rospy.loginfo('[mission] ▶  seguindo após "%s"', etapa)
        self.status(pausado=0, etapa_pausa='', detalhe='')
        return True

    def status(self, **campos):
        """Publica o estado corrente para o HUD (chave=valor).

        Nunca levanta: um erro aqui não pode derrubar a missão.
        """
        try:
            # PUBLICA O ESTADO COMPLETO, não só o que mudou.
            #
            # Antes cada chamada publicava apenas os campos daquele
            # momento e o painel montava o quadro acumulando pedaços.
            # Funciona enquanto o painel está no ar desde o começo — e
            # quebra assim que ele é reiniciado no meio: o assinante novo
            # recebe só a ÚLTIMA mensagem (latch) e fica sem os campos
            # que vieram antes. Foi o que o Marco viu em 2026-08-31, com
            # a trilha de estados inteira e nada marcado nela.
            #
            # Mantendo o dicionário aqui, a mensagem latched é sempre
            # auto-suficiente e qualquer painel que entre no meio vê tudo.
            # CARIMBO DE EXECUÇÃO. O painel funde cada mensagem no
            # estado que já tem, e sobrevive a vários lançamentos de
            # missão — então campos de uma execução MORTA ficavam
            # visíveis junto com os da nova. Em 2026-09-01, com a quinta
            # execução no SEARCH, o painel ainda mostrava fase=atravessa
            # e it=2 da quarta, e o Marco leu aquilo (com razão) como
            # painel travado. O carimbo dá ao painel como perceber que
            # começou outra missão e zerar.
            self._status_campos['run'] = self._run_id
            for k, v in campos.items():
                if v is None or v == '':
                    self._status_campos.pop(k, None)
                elif isinstance(v, float):
                    self._status_campos[k] = '%.4f' % v
                else:
                    self._status_campos[k] = str(v)
            self.pub_status.publish(String(
                data='|'.join('%s=%s' % kv
                              for kv in self._status_campos.items())))
        except Exception:            # noqa: BLE001 — HUD é acessório
            pass

    def send_posture(self, name):
        q = self.postures[name]
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name     = JOINT_NAMES
        msg.position = list(q)
        self.posture_done = False
        self.pub_posture.publish(msg)
        rospy.loginfo('[mission] postura "%s" comandada: %s', name, q)

    def drive(self, v, w):
        t = Twist()
        t.linear.x  = float(np.clip(v, -self.nav_v, self.nav_v))
        t.angular.z = float(np.clip(w, -self.nav_w, self.nav_w))
        self.pub_cmdvel.publish(t)

    def update_wall_from(self, pose_msg):
        """Atualiza a estimativa da parede a partir de um PoseStamped.

        Orientação achatada para yaw puro — ver chave_task.flatten_wall_R
        para o porquê (tilt espúrio do PnP vira erro grande na posição
        calculada do olhal).
        """
        p = pose_msg.pose
        self.wall_pos = np.array([p.position.x, p.position.y, p.position.z])
        R = quaternion_matrix([p.orientation.x, p.orientation.y,
                               p.orientation.z, p.orientation.w])[:3, :3]
        self.wall_R = chave_task.flatten_wall_R(R)

    def take_base(self):
        """Missão assume /cmd_vel (navegação); fuzzy_wb_controller cala."""
        self.pub_wb_enable.publish(Bool(data=False))
        rospy.sleep(0.2)   # deixa o stand-down chegar antes de dirigir

    def publish_keepout(self):
        """Publica o plano que a base não pode cruzar: a face da parede.

        Substitui a trava total de base. A trava evitava a colisão mas
        tirava a base da solução — sobravam 5 juntas para a tarefa e o
        DLS saturava J2/J3 nos batentes, congelando o erro em ~10 cm
        (medido em 2026-08-13). Com a restrição, a base volta a
        participar dos 8 DOF e só perde a liberdade de avançar além do
        limite.

        Convenção esperada pelo controlador: position = ponto no plano,
        eixo X da orientação = normal apontando para o LADO SEGURO.
        """
        n = chave_task.wall_front_normal(self.wall_R)   # aponta para o robô
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.wall_pos

        # Monta uma rotação cujo eixo X seja a normal do lado seguro.
        x = n / max(np.linalg.norm(n), 1e-9)
        z = np.array([0.0, 0.0, 1.0])
        y = np.cross(z, x); y /= max(np.linalg.norm(y), 1e-9)
        z = np.cross(x, y)
        T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2] = x, y, z
        qx, qy, qz, qw = quaternion_from_matrix(T)
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self.pub_keepout.publish(msg)
        rospy.loginfo('[mission] plano de exclusão publicado: parede em %s, '
                      'normal segura %s', self.wall_pos.round(3), x.round(3))

    def release_base(self):
        """Devolve /cmd_vel ao controlador, com a base LIVRE mas restrita.

        Antes isto travava a base por completo. A trava evitava a
        colisão com a parede mas eliminava a redundância: com 5 juntas
        só, o DLS saturava J2/J3 nos batentes e o erro congelava.
        Agora a base participa dos 8 DOF, contida pelo plano de
        exclusão — que é a resposta certa para whole-body.
        """
        self.stop_base()
        self.publish_keepout()
        self.pub_base_lock.publish(Bool(data=False))
        self.pub_servo_tip.publish(Bool(data=True))
        self.pub_wb_enable.publish(Bool(data=True))
        rospy.sleep(0.3)

    def unlock_base(self):
        """Destrava a base e volta a mirar o T265 (navegação)."""
        self.pub_servo_tip.publish(Bool(data=False))
        self.pub_base_lock.publish(Bool(data=False))
        rospy.sleep(0.2)


# ═══════════════════════════════════════════════════════════════════════
# Estados
# ═══════════════════════════════════════════════════════════════════════

class StowInit(smach.State):
    """Recolhe o braço e registra a pose de partida (para o retorno)."""

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['ok', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx_hud = getattr(self, "ctx", None)
        if ctx_hud is not None:
            ctx_hud.status(estado="STOW_INIT")
        ctx = self.ctx
        if not ctx.wait_for_state():
            rospy.logerr('[mission] sem /b166er/robot_state — stack whole-body está rodando?')
            return 'failed'

        ctx.take_base()
        ctx.start_pose = ctx.base_pose()
        rospy.loginfo('[mission] pose de partida registrada: (%.2f, %.2f, %.2f rad)',
                      *ctx.start_pose)

        ctx.send_posture('stow_home')
        return 'ok' if _wait_posture(ctx) else 'failed'


class Search(smach.State):
    """Gira no próprio eixo até a tag ser detectada."""

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['found', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx_hud = getattr(self, "ctx", None)
        if ctx_hud is not None:
            ctx_hud.status(estado="SEARCH")
        ctx = self.ctx
        ctx.send_posture('search')
        if not _wait_posture(ctx):
            return 'failed'

        # Descarta detecções anteriores ao início da busca: a missão
        # precisa achar a tag AGORA, não reaproveitar uma pose estimada
        # antes do braço estar na postura de busca.
        ctx.wall_pose = None

        rospy.loginfo('[mission] SEARCH — girando à procura da tag...')
        rospy.loginfo('[mission] SEARCH: amostra quando a tag estiver a raio '
                      '<= %.2f do quadro (omega fina %.2f rad/s)',
                      ctx.search_raio_max, ctx.search_omega_fina)
        rate = rospy.Rate(ctx.rate_hz)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            if ctx.wall_pose is not None:
                _centra_tag_girando(ctx)
                ctx.stop_base()
                rospy.loginfo('[mission] tag avistada — parando para medir')
                rospy.sleep(1.0)   # deixa a base assentar antes de amostrar
                if not _sample_wall(ctx, ctx.search_samples,
                                    ctx.search_sample_timeout, 'SEARCH'):
                    # Avistou de relance mas não conseguiu medir parada:
                    # segue girando, a tag volta ao quadro.
                    rospy.logwarn('[mission] SEARCH: medição falhou, continuando busca')
                    ctx.wall_pose = None
                    continue
                rospy.loginfo('[mission] parede localizada em (%.3f, %.3f, %.3f)',
                              *ctx.wall_pos)
                ctx.espera_etapa('SEARCH', True, _resumo_geometria(ctx))
                return 'found'

            if (rospy.Time.now() - t0).to_sec() > ctx.search_timeout:
                ctx.stop_base()
                rospy.logerr('[mission] SEARCH: tag não encontrada em %.0fs',
                             ctx.search_timeout)
                return 'failed'

            ctx.drive(0.0, ctx.search_omega)
            rate.sleep()
        return 'failed'


class Approach(smach.State):
    """Aproximação em ETAPAS, remedindo a parede entre elas.

    Uma etapa só não funciona: a estimativa do SEARCH vem de ~3 m, e a
    tag de 132 mm só é confiável até ~1,45 m pela regra tag ≥ d/11 (a
    mesma medida em hardware real na Fase 4). Em 2026-08-13 isso deu
    erro de 0,53 m na pose da parede — o standoff calculado caiu a 9 cm
    da parede e o robô foi direto para cima dela.

    Então: aproxima até uma distância intermediária que caia DENTRO da
    faixa confiável, remede parado, e só então calcula o standoff final.
    Cada etapa recalcula o alvo com a melhor estimativa disponível.
    """

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['arrived', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx_hud = getattr(self, "ctx", None)
        if ctx_hud is not None:
            ctx_hud.status(estado="APPROACH")
        ctx = self.ctx
        # Etapas de distância ao olhal: a intermediária só entra se o
        # robô ainda estiver mais longe que ela.
        # A aproximação termina na distância de DEPLOY, não na de
        # engate: o braço se arma com folga e o robô fecha os últimos
        # centímetros depois, já com a postura montada.
        parada = ctx.deploy_dist
        stages = [d for d in (ctx.coarse_distance,) if d > parada] + [parada]

        for i, dist in enumerate(stages):
            # Na ETAPA FINAL, para de frente (lateral=0) quando o REFINE
            # vai medir ali: é a pose em que a tag fica no centro do
            # quadro. O deslocamento lateral é assumido depois, pelo
            # próprio REFINE, já com a medida boa na mão.
            # DE FRENTE PARA A TAG, não para o olhal.
            #
            # A decisão do Marco foi medir de frente, onde a tag cai no
            # CENTRO do quadro (dispersão 0,9 mm contra 9,2 na periferia).
            # Eu implementei zerando o deslocamento lateral — o que põe a
            # base de frente para o OLHAL, e a tag fica 17 cm ao lado.
            # Resultado medido em 2026-08-31: "REFINE: só 0 amostras da
            # tag", e a missão seguiu com a estimativa ruim da etapa
            # intermediária. Alvo errado, mesmo com a ideia certa.
            #
            # O deslocamento que põe a tag à frente é o dela em relação
            # ao olhal, com sinal invertido: TAG_OLHAL_DX.
            ultima = (i == len(stages) - 1)
            lat = (chave_task.TAG_OLHAL_DX if (ultima and ctx.refine_de_frente)
                   else ctx.standoff_lateral)
            goal = chave_task.standoff_base_pose(ctx.wall_pos, ctx.wall_R, dist,
                                                lateral=lat)
            rospy.loginfo('[mission] APPROACH etapa %d/%d — %.2f m do olhal, '
                          'alvo (%.2f, %.2f, %.2f rad)',
                          i + 1, len(stages), dist, *goal)
            ctx.publish_markers(goal)

            # RECOLHE O BRAÇO PARA ANDAR, ESTENDE PARA OLHAR.
            #
            # A postura de busca alinha J3/J4/J5 na horizontal, que é o
            # que nivela a câmera — e é também o que joga a ferramenta
            # 735 mm para fora do eixo J1, contra 388 mm da postura
            # recolhida (medido em 2026-08-27). São 35 cm de balanço a
            # mais durante o deslocamento, num braço de 28 kg que já
            # tombou o robô antes.
            #
            # O Marco apontou isso olhando a J2, e o instinto estava
            # certo mesmo com o dedo no lugar errado: levar J2 ao limite
            # recolhe só 29 mm; quem recolhe 347 mm é dobrar J4.
            #
            # Como andar e olhar são momentos DIFERENTES — o SEARCH gira
            # parado, o APPROACH dirige —, dá para ter os dois: recolhido
            # nos trechos de deslocamento, alinhado nas medições.
            if ctx.recolhe_para_andar:
                ctx.send_posture('travel')
                _wait_posture(ctx)

            if not _navigate_to(ctx, goal, ctx.approach_timeout, 'APPROACH'):
                return 'failed'

            # Remede parado antes da próxima etapa (a última medição fina
            # fica a cargo do REFINE, já na pose de standoff).
            if i < len(stages) - 1:
                if ctx.recolhe_para_andar:
                    ctx.send_posture('search')
                    _wait_posture(ctx)
                rospy.sleep(1.0)
                if not _sample_wall(ctx, ctx.refine_samples,
                                    ctx.refine_timeout, 'APPROACH/remedida'):
                    rospy.logerr('[mission] APPROACH: perdi a tag na etapa '
                                 'intermediária')
                    return 'failed'
        if ctx.recolhe_para_andar:
            # O REFINE mede parado: volta à postura alinhada, que é a que
            # põe a tag no centro do quadro.
            ctx.send_posture('search')
            _wait_posture(ctx)
        ctx.espera_etapa('APPROACH', True, _resumo_geometria(ctx))
        return 'arrived'


class Refine(smach.State):
    """Remede a parede de perto, já parado na pose de standoff.

    A estimativa do SEARCH vem de longe (~2,5 m), onde o erro angular
    chega a 7,5° — e o olhal fica a 0,86 m da origem de wall_link, então
    esse erro vira dezenas de centímetros no alvo de manipulação. Em
    2026-08-13 isso colocou a fase "engage" atrás da parede e derrubou
    o robô. A ~0,65 m o mesmo pipeline mede com erro de ~18 mm.

    Média de várias amostras para reduzir o ruído por frame; a posição
    é média aritmética simples e o yaw é médio circular.
    """

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['ok', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx_hud = getattr(self, "ctx", None)
        if ctx_hud is not None:
            ctx_hud.status(estado="REFINE")
        ctx = self.ctx
        n_wanted = ctx.refine_samples
        rospy.loginfo('[mission] REFINE — remedindo a parede de perto (%d amostras)',
                      n_wanted)

        antes = ctx.wall_pos.copy()
        if not _sample_wall(ctx, n_wanted, ctx.refine_timeout, 'REFINE'):
            # NÃO é falha: seguimos com a estimativa que já temos.
            #
            # A remedição no standoff era obrigatória quando a tag tinha
            # 132 mm e a medida intermediária (a ~1,3 m) errava ~40 mm.
            # Com a tag de 220 mm essa medida intermediária erra 13 mm —
            # boa o bastante para manipular. E exigir a tag visível no
            # standoff é frágil por geometria: parado de frente para a
            # CHAVE, a tag fica deslocada para o lado e pode sair do
            # campo de visão da câmera de punho. Foram 4 das 5 falhas
            # numa bateria de 8 (2026-08-21), todas por isso.
            rospy.logwarn('[mission] REFINE: tag não visível no standoff — '
                          'seguindo com a estimativa da aproximação '
                          '(erro típico ~13 mm com a tag de 220 mm)')
        else:
            rospy.loginfo('[mission] REFINE: deslocou %.3f m da estimativa anterior',
                          float(np.linalg.norm(ctx.wall_pos - antes)))
        rospy.loginfo('[mission] olhal estimado em (%.3f, %.3f, %.3f)',
                      *chave_task.olhal_position(ctx.wall_pos, ctx.wall_R))

        # NÃO DESLOCA A BASE. Fica onde mediu.
        #
        # A versão anterior media de frente e depois assumia a pose
        # lateral por navegação. A base é skid-steer: para andar 90 mm
        # de lado ela gira, avança meio metro e gira de volta — e o erro
        # angular das três etapas vira erro lateral. MEDIDO em
        # 2026-08-31: alvo x=0,09 e o robô parou em x=-0,028. **118 mm
        # de erro para um deslocamento de 90 mm**, o que pôs o alvo do
        # DEPLOY fora de alcance (resíduo 54 mm) e fez a missão cair na
        # postura fixa de emergência.
        #
        # E o deslocamento nunca foi necessário por ALCANCE. Varredura
        # de IK em 2026-08-31, resíduo por fase com a base de -13 a
        # +17 cm de lado:
        #
        #   lateral   orienta  aprox_lat  atravessa  captura  arco2
        #   -0,090      0,1       0,3        0,1       0,1     0,0
        #   +0,170      0,0       0,2        0,5       0,5     0,3
        #
        # Todas abaixo de 0,5 mm. A justificativa original do
        # deslocamento (dar curso lateral ao braço) era sobre conforto de
        # trajetória, não sobre alcançar — e cobrava uma manobra de base
        # que erra mais do que o furo tolera.
        #
        # PAUSA AQUI a pedido do Marco em 2026-08-31: é o instante em que
        # a base já terminou de se posicionar e o braço AINDA NÃO se
        # mexeu. É a etapa certa para fotografar a aproximação isolada,
        # sem a postura de engate por cima dela.
        ctx.espera_etapa('REFINE', True, _resumo_geometria(ctx))
        return 'ok'


class Deploy(smach.State):
    """Pré-posiciona o braço na SOLUÇÃO DE IK do primeiro alvo.

    Não usa mais uma postura fixa. O braço tem múltiplos ramos de
    solução e uma postura fixa cai na bacia errada com frequência: em
    2026-08-13 a missão convergia suave até 4,5 cm do olhal e travava
    lá, com J3 cravado em −60° (o batente), enquanto a solução do mesmo
    alvo pedia J3 = +31°. Resolver a IK aqui coloca o braço no ramo
    certo, e o controlador Cartesiano só fecha o resíduo.

    Fallback para a postura 'deploy' do YAML se a IK não achar solução
    (alvo fora de alcance a partir do standoff) — aí o MANIPULATE falha
    com diagnóstico claro em vez de o braço sair tentando às cegas.
    """

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['ok', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx_hud = getattr(self, "ctx", None)
        if ctx_hud is not None:
            ctx_hud.status(estado="DEPLOY")
        ctx = self.ctx
        # Mira o primeiro waypoint (pré-engate, afastado da parede) —
        # não o olhal: pré-posicionar direto no olhal levaria o braço a
        # atravessar a parede durante a rampa de postura.
        p_tip = chave_task.phase_target_position(
            ctx.wall_pos, ctx.wall_R, ctx.phases[PHASE_ORDER[0]]['offset_xyz_m'])

        x, y, yaw = ctx.base_pose()
        b = ctx.robot_state.base_odom.pose.pose
        T_wb = quaternion_matrix([b.orientation.x, b.orientation.y,
                                  b.orientation.z, b.orientation.w])
        T_wb[:3, 3] = [b.position.x, b.position.y, b.position.z]

        # RESOLVE PARA A POSE DE MANIPULAÇÃO, NÃO PARA A ATUAL.
        #
        # Logo abaixo o DEPLOY leva a base à pose de engate. Enquanto a
        # IK aqui mirava a pose ATUAL, esse deslocamento invalidava a
        # solução recém-calculada e a primeira fase tinha de refazer o
        # braço inteiro — JÁ PERTO DA CHAVE, que é exatamente o que o
        # pré-posicionamento existe para evitar. A ordem derrotava o
        # próprio objetivo: medido em 2026-08-31, a fase "orienta"
        # gastou 151° de junta para um deslocamento líquido de 4 mm.
        # Prevendo a pose final, caiu para 9-12°.
        #
        # A previsão passou a ser a POSE COMPLETA (2026-09-01), não só um
        # avanço para a frente: a base agora também se desloca
        # lateralmente entre medir e manipular (ver abaixo).
        goal_manip = chave_task.standoff_base_pose(
            ctx.wall_pos, ctx.wall_R, ctx.standoff_dist,
            lateral=ctx.standoff_lateral)
        gx, gy, gyaw = goal_manip
        cg, sg = math.cos(gyaw), math.sin(gyaw)
        T_wb = np.eye(4)
        T_wb[:3, :3] = np.array([[cg, -sg, 0.0], [sg, cg, 0.0],
                                 [0.0, 0.0, 1.0]])
        T_wb[:3, 3] = [gx, gy, b.position.z]
        rospy.loginfo('[mission] DEPLOY: IK resolvida para a POSE DE '
                      'MANIPULAÇÃO (%.2f, %.2f, %.2f rad), não para a atual',
                      gx, gy, gyaw)

        T_arm_world = np.linalg.inv(T_wb @ T_BASELINK_ARM)
        p_local = (T_arm_world @ np.append(p_tip, 1))[:3]

        # Pré-posicionamento COM a direção do degrau, igual às fases.
        # Se o DEPLOY deixasse o braço num ramo com o degrau
        # perpendicular ao furo, a primeira fase teria de girar a
        # ferramenta ~90° no meio da manobra, perto da parede — que é o
        # tipo de rearranjo grande que já derrubou execuções antes.
        q_atual = np.array(ctx.robot_state.q_arm)
        ang_ik = None
        if ctx.degrau_alinhado and ctx.wall_R is not None:
            eixo_arm = T_arm_world[:3, :3] @ (ctx.wall_R
                                              @ np.array([1.0, 0.0, 0.0]))
            up_arm = T_arm_world[:3, :3] @ np.array([0.0, 0.0, 1.0])

            # AQUI a formulação da missão inteira é decidida, comparando
            # as duas no MESMO alvo e com a MESMA métrica (a distância
            # completa — ver a nota em ik_tooltip_nivelado).
            # ÂNCORA DE RAMO: semente FIXA, não a postura corrente.
            #
            # A IK usa q_current como semente E como critério de
            # continuidade. Usar a postura corrente aqui faz o RAMO
            # ESCOLHIDO depender do caminho que o braço percorreu até o
            # DEPLOY — e o caminho muda sempre que a trajetória muda.
            #
            # Aconteceu em 2026-08-27: ao passar a medir de frente e só
            # então assumir a pose lateral, a manobra extra mudou a
            # postura de chegada, e o DEPLOY caiu no ramo espelhado
            # (J5 = -179 em vez de +2,6, com J2, J3 e J4 todos trocados).
            # A pose final da base era IDÊNTICA; só o caminho era outro.
            # O Marco viu na tela que "a cinemática está bem fora do que
            # já fizemos" antes de eu ver no log.
            #
            # Ancorando numa postura conhecida, o ramo passa a ser função
            # só do ALVO — que é o que ele deveria ser. Mudanças de
            # trajetória deixam de mexer nele por efeito colateral.
            q_ancora = np.array(ctx.postures.get(ctx.ancora_ramo, q_atual),
                                dtype=float)
            # SENTIDO FIXO nas duas: o degrau tem que apontar PARA
            # DENTRO do furo. Sem isso a IK aceita o ramo espelhado como
            # solução perfeita e o dedo engata do lado errado (ver a nota
            # em kinematics.ik_tooltip_com_degrau).
            qn, en, angn = ik_tooltip_nivelado(p_local, eixo_arm, up_arm,
                                               q_current=q_ancora,
                                               sentido_fixo=True)
            qe, ee, ange = ik_tooltip_com_degrau(p_local, eixo_arm,
                                                 q_current=q_ancora,
                                                 sentido_fixo=True)
            ctx.ik_modo = 'nivelado' if en <= ee else 'eixo'
            rospy.loginfo('[mission] DEPLOY: IK nivelada %.4f m, só eixo '
                          '%.4f m — a MISSÃO INTEIRA usa "%s"',
                          en, ee, ctx.ik_modo)
            ctx.status(estado='DEPLOY', ik=ctx.ik_modo,
                       ik_nivelado_m=en, ik_eixo_m=ee)
            if ctx.ik_modo == 'nivelado':
                q_ik, err_ik, ang_ik = qn, en, angn
            else:
                q_ik, err_ik, ang_ik = qe, ee, ange
        else:
            q_ik, err_ik = ik_tooltip_position(p_local, q_current=q_atual)
        if err_ik < ctx.deploy_ik_tol:
            rospy.loginfo('[mission] DEPLOY: IK da ponta resolvida (resíduo '
                          '%.4f m, degrau %s) — pré-posicionando em %s graus',
                          err_ik,
                          ('%.1f°' % math.degrees(ang_ik)) if ang_ik is not None
                          else 'n/d',
                          np.degrees(q_ik).round(1))
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name     = JOINT_NAMES
            msg.position = q_ik.tolist()
            ctx.posture_done = False
            ctx.pub_posture.publish(msg)
        else:
            rospy.logwarn('[mission] DEPLOY: IK da ponta não fechou (resíduo '
                          '%.3f m) — alvo provavelmente fora de alcance deste '
                          'standoff. Usando a postura fixa do YAML.', err_ik)
            ctx.send_posture('deploy')

        if not _wait_posture(ctx):
            return 'failed'
        # O DEPLOY continua sendo posicionamento GROSSO por rampa em
        # espaço de juntas; o handoff para o controlador Cartesiano
        # acontece fase a fase, dentro de _reach_by_wholebody.
        #
        # Este bloco dizia "SEM handoff: os dois não podem coexistir",
        # por causa do episódio de 2026-08-13 em que o
        # fuzzy_wb_controller perseguia o último /b166er/ee_target e
        # movia a base enquanto a missão comandava posturas (erro cravado
        # em 3,2 cm por 5 iterações, ponta imóvel). O desligamento
        # resolveu o sintoma e custou caro: medido em 2026-08-24, o
        # controlador ficava em stand-down a missão inteira e o controle
        # whole-body não participava de nada.
        #
        # O alvo estava velho porque a missão nunca publicava um novo —
        # /b166er/ee_target era declarado e nunca usado. Não havia
        # disputa por /cmd_vel: stop_base() emite um único Twist zero.
        # Com alvo fresco publicado ANTES de habilitar, e take_base() no
        # finally de cada fase, os dois convivem.
        #
        # A base fica parada aqui; o plano de exclusão fica publicado
        # para quando o controlador assumir.
        ctx.publish_keepout()
        ctx.take_base()
        ctx.stop_base()

        # O ramo de solução é escolhido AQUI — é a etapa que mais
        # interessa inspecionar antes de qualquer movimento fino.
        ctx.espera_etapa('DEPLOY/pré-posicionamento', True,
                         _resumo_geometria(ctx))

        # SEGUNDO MOMENTO: com o braço já armado, a base vai para a pose
        # de engate. Era um avanço reto; virou navegação, porque a pose
        # de engate deixou de estar na mesma linha da de medição.
        #
        # POR QUE A BASE PRECISA SE DESLOCAR DE LADO. O robô mede DE
        # FRENTE PARA A TAG, que fica 17 cm ao lado do olhal — decisão
        # certa, que tirou a tag da periferia do fisheye e ganhou uma
        # ordem de grandeza na percepção. Só que ele vinha MANIPULANDO
        # da mesma pose, 17 cm de lado.
        #
        # MEDIDO em 2026-09-01, varrendo a pose da base e resolvendo a
        # cadeia inteira de waypoints em cada uma (pior desalinho do
        # degrau ao longo das fases, em graus):
        #
        #   dist \ lateral   -90mm     0    +60   +120   +170
        #        0,62         4,3     1,5    2,4    4,5    6,6
        #        0,70        10,2     4,0    6,0   10,1   13,6  <- operação
        #
        # Os 13,6° previstos batem com os 14,2° medidos ao vivo. Na mesma
        # distância, sair de +170 mm para zero vale 13,6° -> 4,0°; com a
        # distância em 0,62 vai a 1,5°, que sobre os 90 mm de inserção
        # são 2,4 mm de desvio contra 18 mm de meio-eixo do oval.
        #
        # O deslocamento lateral já tinha sido REMOVIDO de propósito
        # (custa erro de odometria, e numa tentativa passou 118 mm do
        # alvo). Volta porque agora é curto, acontece DEPOIS da medida
        # boa, e a âncora segura a base. E a imprecisão da manobra é
        # tolerável: a IK de cada fase é resolvida com a pose REAL da
        # base, então ±6 cm ainda caem na região boa do mapa.
        if not _navigate_to(ctx, goal_manip, ctx.approach_timeout,
                            'DEPLOY/aproxima'):
            return 'failed'
        return 'ok'


class Manipulate(smach.State):
    """As 4 fases Cartesianas da abertura, via controlador whole-body."""

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx_hud = getattr(self, "ctx", None)
        if ctx_hud is not None:
            ctx_hud.status(estado="MANIPULATE")
        ctx = self.ctx
        # Orientação do EE fixada agora e mantida em todas as fases —
        # o documento da tarefa especifica direções de força, não uma
        # orientação de ferramenta (ver task_sequencer.py).
        ctx.ee_orientation = ctx.robot_state.ee_pose.pose.orientation
        R_ee = quaternion_matrix([ctx.ee_orientation.x, ctx.ee_orientation.y,
                                  ctx.ee_orientation.z, ctx.ee_orientation.w])[:3, :3]

        # Começa no offset da PRIMEIRA fase, não em zero: o pre_engage é
        # o waypoint de aproximação (18 cm à frente do olhal), não um
        # puxão. Zerar aqui mandaria a base recuar 18 cm antes de sequer
        # engatar.
        y_anterior = ctx.phases[PHASE_ORDER[0]]['offset_xyz_m'][1]
        for phase in PHASE_ORDER:
            offset = list(ctx.phases[phase]['offset_xyz_m'])
            # ALVO RELATIVO À CAPTURA. Uma fase com curso_min_m fecha por
            # descida medida, então o comando também tem de ser relativo:
            # alt_alvo = alt_captura − curso − margem. Com o alvo fixo do
            # YAML, uma captura que fechasse baixo (a tolerância é 10 mm)
            # deixaria menos de 13 mm para descer e a fase nunca fecharia.
            cfg = ctx.phases[phase]
            if cfg.get('curso_min_m') is not None and ctx.alt_captura is not None:
                margem = float(cfg.get('curso_margem_m', 0.002))
                offset[2] = ctx.alt_captura - float(cfg['curso_min_m']) - margem
                rospy.loginfo('[mission] fase "%s": alvo em altura relativo à '
                              'captura: %+.1f mm (captura %+.1f − curso %.0f − '
                              'margem %.0f)', phase, offset[2] * 1000,
                              ctx.alt_captura * 1000, cfg['curso_min_m'] * 1000,
                              margem * 1000)
            ctx.offset_efetivo = offset

            # PUXAR COM AS RODAS. A componente Y do offset é o quanto o
            # olhal se afasta da parede — exatamente o que recuar o chassi
            # produz. Deixar isso para a base mantém a postura do braço
            # parada e reserva ao braço só a descida (componente Z), que
            # é onde ele tem folga. Antes, o braço fazia as duas coisas e
            # varria perto da parede no meio do movimento.
            dy = offset[1] - y_anterior
            if (ctx.pull_with_base and dy > 0.005
                    and phase not in PHASES_SO_BRACO):
                if not _creep_base(ctx, -dy, 'fase "%s"/puxa' % phase):
                    return 'failed'
            y_anterior = offset[1]

            # O alvo é recalculado DEPOIS do recuo: a ponta já veio junto
            # com a base, então o que sobra para a IK é o resíduo.
            p_tip  = chave_task.phase_target_position(ctx.wall_pos, ctx.wall_R, offset)

            modo = _modo_da_fase(ctx, phase)
            alcancar = (_reach_by_wholebody if modo == 'wb'
                        else _reach_by_iterative_ik)
            rospy.loginfo('[mission] fase "%s": modo %s (%s)', phase,
                          'whole-body/Fuzzy' if modo == 'wb' else 'IK iterativa',
                          'forçado' if ctx.modo_fases != 'yaml' else 'YAML')
            ctx.status(modo=modo)
            ok_fase = alcancar(ctx, p_tip, phase)
            # Pausa TAMBÉM quando a fase falha: é justamente a postura de
            # falha que precisa ser olhada.
            ctx.espera_etapa('fase %s' % phase, ok_fase,
                             _resumo_geometria(ctx))
            if not ok_fase:
                return 'failed'

            # A chave acompanha a ferramenta (ver set_blade_angle).
            ang = ctx.phases[phase].get('blade_angle_deg')
            if ang is not None:
                ctx.set_blade_angle(float(ang))

            # O GATILHO SOLTOU? A fase 'destrava' só vale se o olhal
            # desceu de verdade (lingueta ≥ ~12 mm dos 15 pedidos). Não
            # aborta aqui: o arco seguinte é que vai falhar, e falhar
            # com a lingueta medida no log é o relato correto.
            if phase == 'captura':
                # Zero do critério observável do destrava: onde a ponta
                # parou quando o degrau apoiou no arame.
                ctx.alt_captura = _alt_na_parede(ctx, _tooltip_now(ctx))
                ctx.status(alt_captura_mm=ctx.alt_captura * 1000)
                rospy.loginfo('[mission] captura: altura da ponta registrada '
                              'em %+.1f mm (zero da descida do destrava)',
                              ctx.alt_captura * 1000)
            if phase == 'destrava':
                desc = None
                if ctx.alt_captura is not None:
                    desc = ctx.alt_captura - _alt_na_parede(ctx, _tooltip_now(ctx))
                curso = ctx.lingueta_now()
                rospy.loginfo('[mission] destrava: ponta desceu %s desde a '
                              'captura (critério observável) | lingueta %s '
                              '(só na simulação)',
                              '%.1f mm' % (desc * 1000) if desc is not None else 'n/d',
                              '%.1f mm' % curso if curso is not None else 'n/d')
                if curso is not None and curso < 12.0:
                    rospy.logwarn('[mission] destrava: lingueta em %.1f mm '
                                  '(esperado ≥ 12) — o gatilho pode não '
                                  'ter soltado', curso)
                elif curso is not None:
                    rospy.loginfo('[mission] destrava: lingueta em %.1f mm '
                                  '— gatilho solto', curso)
            rospy.sleep(0.5)

        # A CHAVE ABRIU MESMO? Com o acoplamento cinemático a pergunta
        # era vazia — a missão lia de volta o que ela própria impôs. Com
        # abertura por contato ela é o critério real da tarefa: chegar
        # aos waypoints com 5 mm não vale nada se o gancho escorregou.
        ang = ctx.blade_angle_now()
        if ang is None:
            rospy.logwarn('[mission] MANIPULATE: não consegui medir a lâmina — '
                          'concluindo sem confirmar a abertura')
        elif ang < ctx.blade_open_deg:
            rospy.logerr('[mission] MANIPULATE: lâmina em %.1f° (mínimo %.1f°) '
                         '— a trajetória fechou mas a CHAVE NÃO ABRIU', ang,
                         ctx.blade_open_deg)
            return 'failed'
        else:
            rospy.loginfo('[mission] MANIPULATE concluída — chave ABERTA, '
                          'lâmina medida em %.1f°', ang)
            return 'done'
        rospy.loginfo('[mission] MANIPULATE concluída')
        return 'done'


class Retract(smach.State):
    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['ok', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx_hud = getattr(self, "ctx", None)
        if ctx_hud is not None:
            ctx_hud.status(estado="RETRACT")
        ctx = self.ctx
        # ORDEM IMPORTA: silencia o controlador ANTES de destravar. Ao
        # contrário, existe uma janela em que ele fica com a base livre
        # E ainda perseguindo o último alvo latched — observado ao vivo
        # em 2026-08-13, a base saiu andando ~0,7 m depois do abort.
        ctx.take_base()          # fuzzy cala, missão assume /cmd_vel
        ctx.unlock_base()        # só então destrava
        ctx.stop_base()
        # A fase 'desengata' já afasta lateralmente no caminho normal,
        # mas o RETRACT também pode ser alcançado com ela incompleta.
        _sai_pelo_eixo(ctx, 'RETRACT')
        _afasta_da_parede(ctx, 'RETRACT')
        ctx.send_posture('stow_home')
        return 'ok' if _wait_posture(ctx) else 'failed'


class Return(smach.State):
    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['home', 'failed'])
        self.ctx = ctx

    def execute(self, _):
        ctx_hud = getattr(self, "ctx", None)
        if ctx_hud is not None:
            ctx_hud.status(estado="RETURN")
        ctx = self.ctx
        rospy.loginfo('[mission] RETURN — voltando a (%.2f, %.2f, %.2f rad)',
                      *ctx.start_pose)
        ok = _navigate_to(ctx, ctx.start_pose, ctx.approach_timeout, 'RETURN')
        return 'home' if ok else 'failed'


class AbortSafe(smach.State):
    """Estado terminal de falha: para a base e recolhe o braço.

    Nunca deixa o robô parado com o braço estendido — é a condição que
    tombou o robô em 2026-08-12.
    """

    def __init__(self, ctx):
        smach.State.__init__(self, outcomes=['aborted'])
        self.ctx = ctx

    def execute(self, _):
        ctx_hud = getattr(self, "ctx", None)
        if ctx_hud is not None:
            ctx_hud.status(estado="ABORT_SAFE")
        ctx = self.ctx
        rospy.logwarn('[mission] ABORT — parando base e recolhendo braço')
        ctx.take_base()          # silencia o controlador PRIMEIRO
        ctx.unlock_base()
        ctx.stop_base()

        # SAI DO ANEL ANTES DE RECOLHER. Sem isto o braço vai direto para
        # a postura recolhida e, se o degrau estiver no olhal, arrasta a
        # chave no caminho — foi o que o Marco viu.
        _sai_pelo_eixo(ctx, 'ABORT')
        _afasta_da_parede(ctx, 'ABORT')

        ctx.send_posture('stow_home')
        _wait_posture(ctx)

        # VOLTA PARA A PARTIDA TAMBÉM NO ABORT.
        #
        # O estado RETURN existia, mas só no caminho de sucesso
        # (MANIPULATE -> RETRACT -> RETURN). Como todas as execuções
        # abortavam, o robô ficava parado onde a falha o pegou — em
        # cima da chave — e a execução seguinte começava de lá. O Marco
        # pediu que o robô volte ao ponto de spawn DEPOIS DA MISSÃO, sem
        # qualificar sucesso ou falha, e ele está certo: deixar o robô
        # encostado no alvo é o que já produziu colisões entre execuções.
        #
        # Só depois de recolher o braço: voltar com a ferramenta
        # estendida arrastaria a chave.
        if ctx.retorna_no_abort and ctx.start_pose is not None:
            rospy.logwarn('[mission] ABORT — voltando à pose de partida '
                          '(%.2f, %.2f, %.2f rad)', *ctx.start_pose)
            if not _navigate_to(ctx, ctx.start_pose, ctx.approach_timeout,
                                'ABORT/retorno'):
                rospy.logerr('[mission] ABORT: retorno à partida falhou — '
                             'o robô ficou onde estava')
            ctx.stop_base()
        return 'aborted'


# ═══════════════════════════════════════════════════════════════════════
# Helpers compartilhados entre estados
# ═══════════════════════════════════════════════════════════════════════

# Limiares de rejeição de outlier em _sample_wall. Dimensionados pela
# dispersão real medida: parado e de frente, amostras consecutivas da
# mesma pose variam poucos centímetros e milirradianos; um flip de
# ambiguidade do PnP salta dezenas de centímetros e dezenas de graus.
# A folga entre as duas escalas é grande, então os valores não são
# críticos — precisam só ficar no meio.
_OUTLIER_POS_M   = 0.15
_OUTLIER_YAW_RAD = 0.20


def _resumo_geometria(ctx):
    """Uma linha com o que interessa olhar numa pausa: juntas, ramo,
    distância da ponta ao olhal e ângulo da lâmina."""
    try:
        q = np.degrees(np.array(ctx.robot_state.q_arm)).round(1)
        p = _tooltip_now(ctx)
        olhal = chave_task.olhal_position(ctx.wall_pos, ctx.wall_R)
        d = float(np.linalg.norm(p - olhal)) * 1000
        ang = ctx.blade_angle_now()
        return ('juntas %s | ponta a %.0f mm do olhal | lâmina %s'
                % (q.tolist(), d,
                   ('%.1f°' % ang) if ang is not None else 'n/d'))
    except Exception:            # noqa: BLE001
        return ''


def _tolerancia_da_fase(ctx, phase):
    """Tolerância da fase: por EIXO no frame da parede, ou esférica.

    POR QUE POR EIXO. A tolerância era uma esfera de 20 mm, e o furo do
    olhal não é uma esfera: dá ±15 mm em profundidade, ±20 mm em altura
    e é folgado ao longo do próprio eixo, porque o degrau tem 30 mm de
    comprimento. Uma esfera de 20 mm é FROUXA demais onde a tarefa é
    apertada e apertada demais onde ela é folgada.

    A consequência foi medida em 2026-08-27: as sete fases fecharam
    entre 11 e 19 mm, todas dentro da tolerância, e a ferramenta passou
    o tempo todo FORA do furo — mediana de 24,6 mm em profundidade,
    contra os 15 mm de meio-vão. O critério de sucesso era mais frouxo
    do que a tarefa permite, então "fase alcançada" não significava
    nada para o engate.

    Devolve (tol_xyz ou None, tol_escalar). Sem tol_xyz_m no YAML, a
    fase segue no critério esférico de sempre.
    """
    cfg = ctx.phases.get(phase, {}) if isinstance(ctx.phases, dict) else {}
    tol = cfg.get('tol_xyz_m')
    if tol and len(tol) == 3:
        return np.asarray(tol, dtype=float), None
    return None, ctx.tol_pos


def _modo_da_fase(ctx, phase):
    """'ik' ou 'wb' para a fase: o YAML decide, salvo ~modo_fases forçado.
    Fase sem `modo` no YAML fica em IK iterativa, que é o caminho validado
    desde 26 Ago."""
    if ctx.modo_fases in ('ik', 'wb'):
        return ctx.modo_fases
    cfg = ctx.phases.get(phase, {}) if isinstance(ctx.phases, dict) else {}
    modo = str(cfg.get('modo', 'ik')).strip().lower()
    if modo not in ('ik', 'wb'):
        rospy.logwarn('[mission] fase "%s": modo %r desconhecido no YAML — IK',
                      phase, modo)
        return 'ik'
    return modo


def _alt_na_parede(ctx, p_world):
    """Altura de um ponto do mundo no frame da parede, relativa ao centro
    do olhal estimado (mm positivos = acima)."""
    olhal = chave_task.olhal_position(ctx.wall_pos, ctx.wall_R)
    return float((ctx.wall_R.T @ (np.asarray(p_world, dtype=float) - olhal))[2])


def _descida_desde_captura(ctx, phase, e_parede):
    """Quanto a ponta desceu desde o fecho da captura (m), ou None.
    A altura atual sai do erro por eixo: alt_atual = alt_alvo − e_alt."""
    cfg = ctx.phases.get(phase, {}) if isinstance(ctx.phases, dict) else {}
    if ctx.alt_captura is None or e_parede is None:
        return None
    off = ctx.offset_efetivo if ctx.offset_efetivo is not None else cfg.get('offset_xyz_m', [0, 0, 0])
    alt_alvo = float(off[2])
    alt_atual = alt_alvo - float(e_parede[2])
    return ctx.alt_captura - alt_atual


def _fase_fechou(ctx, phase, err_world, n_err):
    """A fase fechou? Compara no frame da PAREDE quando há tolerância
    por eixo — é lá que os eixos têm significado (X = eixo do furo,
    Y = profundidade, Z = altura).

    CRITÉRIO OBSERVÁVEL (2026-09-03, item do Marco: "criar critério de
    destrava observável (soltura é no libera)"). Uma fase com
    `curso_min_m` no YAML não fecha por chegar a uma altura-alvo, e sim
    por ter DESCIDO pelo menos esse curso desde o instante em que a
    captura fechou — medido pela ponta (T265), sem depender da
    estimativa do olhal. Nas 40 missões anteriores o destrava fechava
    por altura com a lingueta em 3-7 mm e quem soltava o gatilho era a
    libera; com este critério a fase só fecha se a ponta (e o olhal com
    ela) desceu o curso do gatilho. Eixo e profundidade continuam por
    tolerância. Na bancada é a mesma medida; na simulação a lingueta
    confirma."""
    tol_xyz, tol_esf = _tolerancia_da_fase(ctx, phase)
    if tol_xyz is None or ctx.wall_R is None:
        return n_err < tol_esf, None
    e = ctx.wall_R.T @ np.asarray(err_world, dtype=float)
    cfg = ctx.phases.get(phase, {}) if isinstance(ctx.phases, dict) else {}
    curso_min = cfg.get('curso_min_m')
    if curso_min is not None:
        desc = _descida_desde_captura(ctx, phase, e)
        if desc is None:
            rospy.logwarn_throttle(5.0, '[mission] fase "%s": sem altura da '
                                   'captura — critério por altura-alvo', phase)
            return bool(np.all(np.abs(e) < tol_xyz)), e
        ctx.status(descida_mm=desc * 1000, curso_min_mm=float(curso_min) * 1000)
        ok = (abs(e[0]) < tol_xyz[0] and abs(e[1]) < tol_xyz[1]
              and desc >= float(curso_min))
        return bool(ok), e
    return bool(np.all(np.abs(e) < tol_xyz)), e


def _sai_pelo_eixo(ctx, tag):
    """Afasta a ponta ao longo do EIXO DO FURO antes de recolher o braço.

    POR QUE EXISTE. Observação do Marco em 2026-08-27, vendo a missão na
    tela: "para o robô retornar à posição de spawn ele deve realizar uma
    trajetória de simples retirada do degrau do olhal, e novamente faz
    uma trajetória maluca batendo em tudo".

    Ele está certo, e o defeito é assimétrico. No caminho de SUCESSO
    existe a fase 'desengata', que tira o degrau lateralmente. No caminho
    de FALHA não existe nada: o ABORT_SAFE comanda a postura recolhida
    direto, e o braço vai de onde estiver até lá pelo caminho que a
    interpolação de juntas escolher — com o degrau possivelmente dentro
    do anel, arrastando o mecanismo.

    A saída correta é uma só: recuar ao longo do eixo do furo, que é a
    direção em que o degrau entrou. Qualquer outra direção prende.

    Não falha a missão: se não houver estimativa da parede, ou se a IK
    não fechar, apenas avisa e deixa o recolhimento seguir — este é um
    passo de segurança, não um objetivo.
    """
    if ctx.wall_R is None or ctx.robot_state is None:
        return False
    try:
        p_now = _tooltip_now(ctx)
        olhal = chave_task.olhal_position(ctx.wall_pos, ctx.wall_R)
        dist = float(np.linalg.norm(p_now - olhal))
        if dist > ctx.saida_raio:
            # Longe do anel: não há de que se desprender.
            return False

        eixo_mundo = ctx.wall_R @ np.array([1.0, 0.0, 0.0])
        # Sentido: o que AFASTA do olhal a partir de onde a ponta está.
        proj = float((p_now - olhal) @ eixo_mundo)
        sentido = 1.0 if proj >= 0.0 else -1.0
        alvo = p_now + sentido * ctx.saida_curso * eixo_mundo

        rospy.logwarn('[mission] %s: ponta a %.3f m do olhal — saindo '
                      '%.0f mm pelo eixo do furo antes de recolher',
                      tag, dist, ctx.saida_curso * 1000)
        ctx.status(estado=tag, saida_mm=ctx.saida_curso * 1000,
                   dist_olhal_m=dist)
        return _reach_by_iterative_ik(ctx, alvo, 'saida_eixo')
    except Exception as exc:            # noqa: BLE001
        rospy.logwarn('[mission] %s: saída pelo eixo falhou (%s) — '
                      'recolhendo mesmo assim', tag, exc)
        return False


def _raio_tag(ctx, max_idade=0.5):
    """Raio normalizado da tag no quadro (0 = centro, 1 = borda), ou None
    se a última detecção for mais velha que max_idade s."""
    if ctx.tag_pixel is None or ctx.tag_pixel_t is None:
        return None
    if (rospy.Time.now() - ctx.tag_pixel_t).to_sec() > max_idade:
        return None
    return float(np.hypot(ctx.tag_pixel.x, ctx.tag_pixel.y))


def _centra_tag_girando(ctx):
    """Depois da primeira detecção, segue girando devagar até a tag sair da
    borda do quadro (raio <= ~search_raio_max). Ver o comentário em
    search_raio_max: na borda o PnP mede 0,4-1,0 m curto; entre 22 e 48
    graus fora do eixo erra centímetros. Se a tag sumir ou o tempo
    esgotar, devolve com a base onde estiver — a amostragem segue como
    antes."""
    r = _raio_tag(ctx)
    if r is None or r <= ctx.search_raio_max:
        return
    rospy.loginfo('[mission] SEARCH: tag a raio %.2f — girando até <= %.2f',
                  r, ctx.search_raio_max)
    rate = rospy.Rate(ctx.rate_hz)
    t0 = rospy.Time.now()
    perdida_desde = None
    while not rospy.is_shutdown():
        r = _raio_tag(ctx)
        if r is not None and r <= ctx.search_raio_max:
            rospy.loginfo('[mission] SEARCH: tag a raio %.2f — parando', r)
            return
        if r is None:
            perdida_desde = perdida_desde or rospy.Time.now()
            if (rospy.Time.now() - perdida_desde).to_sec() > 1.5:
                rospy.logwarn('[mission] SEARCH: tag saiu do quadro durante '
                              'o ajuste — amostrando onde parou')
                return
        else:
            perdida_desde = None
        if (rospy.Time.now() - t0).to_sec() > ctx.search_centra_timeout:
            rospy.logwarn('[mission] SEARCH: ajuste não convergiu em %.0fs '
                          '(raio %s) — amostrando assim mesmo',
                          ctx.search_centra_timeout, 'n/d' if r is None else '%.2f' % r)
            return
        ctx.drive(0.0, ctx.search_omega_fina)
        rate.sleep()


def _sample_wall(ctx, n_wanted, timeout, tag):
    """Coleta N estimativas da parede com o robô PARADO e devolve a média.

    Amostrar parado importa: em 2026-08-13 o SEARCH aceitava a primeira
    detecção, capturada durante o giro, em ângulo rasante — erro de 46 cm
    na pose da parede, que jogou o standoff quase encostado nela e deixou
    a tag fora do campo de visão para o REFINE. Parado e de frente, o
    mesmo pipeline erra ~2 cm.

    Posição por média aritmética; yaw por média circular; roll/pitch
    descartados (ver chave_task.flatten_wall_R).
    """
    positions, yaws = [], []
    # DOIS RELÓGIOS, DE PROPÓSITO.
    #
    # O orçamento (timeout) corre em tempo SIMULADO: com a física
    # parada nenhuma amostra nova chega, e não faz sentido queimar o
    # prazo esperando por algo que não pode acontecer. Já o sono do
    # laço é de PAREDE, para o nó continuar respondendo e voltar
    # sozinho quando a física voltar.
    #
    # Com rospy.Rate aqui, uma pausa do Gazebo congelava esta coleta
    # inteira — e o timeout também não corria, então nem o caminho de
    # falha salvava. Aconteceu em 2026-09-01 na coleta do REFINE: 8
    # minutos parados, missão viva, /clock sem mensagens. Eu tinha
    # corrigido o laço de pausa entre etapas no mesmo dia e deixado
    # este, que é onde o problema apareceu em seguida.
    t0 = rospy.Time.now()
    t0_parede = time.time()
    avisou_relogio = False
    ctx.wall_pose = None
    # ANCORA DURANTE A COLETA. "Parado" era só uma intenção: o modelo
    # deriva 2,6 mm/s (ver ancora_engata), e uma janela de 10 amostras
    # leva vários segundos — o robô andava ~2 cm ENQUANTO media, com
    # cada amostra tirada de um ponto diferente. A pose da parede é
    # calculada a partir da pose do robô, então isso não vira só ruído:
    # entra como viés na média, na direção do avanço. É a mesma deriva
    # que contaminou as caracterizações de percepção deste mês.
    ctx.ancora_engata('coleta do %s' % tag)
    dither = float(getattr(ctx, 'coleta_dither_omega', 0.0) or 0.0)
    periodo = max(0.5, float(getattr(ctx, 'coleta_dither_periodo', 3.0)))
    if dither > 0:
        rospy.loginfo('[mission] %s: coleta com dither ±%.2f rad/s (período %.1f s)',
                      tag, dither, periodo)
    while len(positions) < n_wanted and not rospy.is_shutdown():
        if (rospy.Time.now() - t0).to_sec() > timeout:
            break
        ctx.ancora_mantem()
        if dither > 0:
            # Ver coleta_dither_omega: a base gira devagar para os quadros
            # não ficarem idênticos (razão de ambiguidade congelada) e
            # para o yaw alternado da tag pequena se cancelar na média.
            fase = ((rospy.Time.now() - t0).to_sec() % periodo) < periodo / 2.0
            ctx.drive(0.0, dither if fase else -dither)
        if ctx.wall_pose is not None:
            p = ctx.wall_pose.pose
            positions.append([p.position.x, p.position.y, p.position.z])
            R = quaternion_matrix([p.orientation.x, p.orientation.y,
                                   p.orientation.z, p.orientation.w])[:3, :3]
            yaws.append(math.atan2(R[1, 0], R[0, 0]))
            ctx.wall_pose = None
        time.sleep(0.1)
        if (not avisou_relogio and time.time() - t0_parede > 30.0
                and (rospy.Time.now() - t0).to_sec() < 5.0):
            avisou_relogio = True
            rospy.logwarn('[mission] %s: 30 s de relógio de parede e o tempo '
                          'SIMULADO não andou — a física do Gazebo está '
                          'pausada. A coleta retoma sozinha quando despausar.',
                          tag)
    if dither > 0:
        ctx.stop_base()
    n_corr = ctx.ancora_solta()
    if n_corr:
        rospy.loginfo('[mission] %s: âncora corrigiu a deriva %d vez(es) '
                      'durante a coleta', tag, n_corr)

    if len(positions) < 3:
        rospy.logerr('[mission] %s: só %d amostras da tag', tag, len(positions))
        return False

    # REJEIÇÃO DE OUTLIERS (2026-08-24). Média pura assume ruído
    # simétrico, e o erro do PnP aqui não é: a ambiguidade de pose da
    # tag plana faz a solução "virar" para um espelho, e uma única
    # amostra virada desloca a média muito mais do que o ruído normal.
    # Na bateria de 2026-08-21 o SEARCH de uma execução devolveu a
    # parede a (0.185, 3.576) yaw=2.549 — 0,58 m e 34° fora, com o
    # mesmo pipeline que nas outras execuções errava 3 cm.
    #
    # Mediana como referência (não a média: a média já está
    # contaminada) e corte por desvio absoluto. O yaw entra no mesmo
    # critério porque é nele que o flip aparece primeiro.
    P = np.array(positions)
    med = np.median(P, axis=0)
    yaw_med = math.atan2(np.median(np.sin(yaws)), np.median(np.cos(yaws)))
    d_pos = np.linalg.norm(P - med, axis=1)
    d_yaw = np.abs(np.arctan2(np.sin(np.array(yaws) - yaw_med),
                              np.cos(np.array(yaws) - yaw_med)))
    keep = (d_pos <= _OUTLIER_POS_M) & (d_yaw <= _OUTLIER_YAW_RAD)
    if keep.sum() >= 3 and keep.sum() < len(positions):
        rospy.logwarn('[mission] %s: %d de %d amostras descartadas como '
                      'outlier (>%.2f m ou >%.2f rad da mediana)',
                      tag, len(positions) - int(keep.sum()), len(positions),
                      _OUTLIER_POS_M, _OUTLIER_YAW_RAD)
        P = P[keep]
        yaws = list(np.array(yaws)[keep])
    elif keep.sum() < 3:
        # Dispersão grande demais para separar sinal de outlier: a
        # medida inteira é suspeita. Falhar aqui é melhor que devolver
        # uma pose errada com cara de confiável — foi assim que o robô
        # já foi mandado contra a parede.
        rospy.logerr('[mission] %s: amostras dispersas demais (só %d de %d '
                     'dentro de %.2f m da mediana) — medida descartada',
                     tag, int(keep.sum()), len(positions), _OUTLIER_POS_M)
        return False

    positions = P.tolist()
    pos_mean = np.mean(P, axis=0)
    yaw_mean = math.atan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws)))
    c, s_ = math.cos(yaw_mean), math.sin(yaw_mean)
    ctx.wall_pos = pos_mean
    ctx.wall_R   = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
    rospy.loginfo('[mission] %s: %d amostras — parede (%.3f, %.3f, %.3f) yaw=%.3f',
                  tag, len(positions), *pos_mean, yaw_mean)
    return True


def _wait_posture(ctx):
    t0 = rospy.Time.now()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if ctx.posture_done:
            return True
        if (rospy.Time.now() - t0).to_sec() > ctx.posture_timeout:
            rospy.logerr('[mission] postura não atingida em %.0fs', ctx.posture_timeout)
            return False
        # Segura a base enquanto o braço percorre a rampa: é aqui que
        # passa a maior parte dos ~3 s de cada iteração de fase, e sem
        # isto a deriva do modelo (2,6 mm/s) move o chão debaixo do
        # alvo. Fora de uma fase a âncora está solta e a chamada é nula.
        ctx.ancora_mantem()
        rate.sleep()
    return False


def _navigate_to(ctx, goal, timeout, tag):
    """Girar-avançar-girar até (x, y, yaw). Base pura, braço parado."""
    gx, gy, gyaw = goal
    rate = rospy.Rate(ctx.rate_hz)
    t0 = rospy.Time.now()
    phase = 'TURN_TO'

    t_gimbal = rospy.Time.now()
    while not rospy.is_shutdown():
        if (rospy.Time.now() - t0).to_sec() > timeout:
            ctx.stop_base()
            rospy.logerr('[mission] %s: timeout de %.0fs em %s', tag, timeout, phase)
            return False

        # Mantém a tag centrada enquanto anda, e alimenta o RViz.
        # ATENÇÃO: o gimbal publica comandos de POSTURA, e a ponte de
        # braço (gazebo_arm_bridge) ignora /b166er/arm_vel_cmd enquanto
        # uma rampa de postura está ativa. Se uma rampa do gimbal ficar
        # em voo quando a navegação terminar, ela sequestra o braço e o
        # controlador Cartesiano da manipulação não consegue mover nada
        # — foi o que travou o "engage" em 2026-08-13, com o
        # controlador comandando velocidade e o braço parado.
        if (rospy.Time.now() - t_gimbal).to_sec() > ctx.gimbal_period:
            if ctx.usa_gimbal:
                ctx.gimbal_track()
            ctx.publish_markers(goal)
            t_gimbal = rospy.Time.now()

        x, y, yaw = ctx.base_pose()
        dx, dy = gx - x, gy - y
        dist    = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)

        if phase == 'TURN_TO':
            err = _ang_diff(bearing, yaw)
            if dist < ctx.nav_pos_tol or abs(err) < ctx.nav_yaw_tol:
                phase = 'DRIVE'
                rospy.loginfo('[mission] %s: alinhado, avançando (%.2f m)', tag, dist)
            else:
                ctx.drive(0.0, math.copysign(ctx.nav_w, err))

        elif phase == 'DRIVE':
            # Freio por MEDIDA: o laser vê o obstáculo real à frente.
            # Antes a única proteção era o alvo calculado a partir da
            # tag — se a estimativa errasse, o robô ia para cima da
            # parede (aconteceu em 2026-08-13, standoff calculado a 9 cm
            # da parede por erro de 53 cm na detecção distante).
            if (ctx.front_clearance is not None
                    and ctx.front_clearance < ctx.min_clearance):
                ctx.stop_base()
                rospy.logwarn('[mission] %s: laser reporta obstáculo a %.2f m '
                              '(mín %.2f) — parando avanço', tag,
                              ctx.front_clearance, ctx.min_clearance)
                phase = 'TURN_FINAL'
            elif dist < ctx.nav_pos_tol:
                ctx.stop_base()
                phase = 'TURN_FINAL'
                rospy.loginfo('[mission] %s: chegou, ajustando heading final', tag)
            else:
                # Correção proporcional de rumo enquanto avança.
                err = _ang_diff(bearing, yaw)
                ctx.drive(ctx.nav_v, 1.2 * err)

        else:  # TURN_FINAL
            err = _ang_diff(gyaw, yaw)
            if abs(err) < ctx.nav_yaw_tol:
                ctx.stop_base()
                # Fecha qualquer rampa de gimbal pendente: reenvia a
                # postura ATUAL do rastreamento como alvo e espera ela
                # assentar, para a ponte de braço sair do modo postura
                # antes de a manipulação começar.
                ctx.settle_gimbal()
                rospy.loginfo('[mission] %s: concluído', tag)
                return True
            ctx.drive(0.0, math.copysign(ctx.nav_w, err))

        rate.sleep()
    return False


def _afasta_da_parede(ctx, tag):
    """Recua a BASE pelas rodas antes de recolher o braço.

    POR QUE EXISTE (2026-09-02). O Marco, vendo a run9: "depois do
    MANIPULATE, a saída/retorno do robot está colidindo com a chave".
    Captura de contatos na run10 mostrou o punho fundido (L5) contra a
    lâmina e a PLACA da chave nos 2 s seguintes ao comando de stow_home,
    e a FK sobre o rastro de juntas explicou: _sai_pelo_eixo recua a
    ponta 120 mm pelo eixo do FURO, que é paralelo à parede — a ponta
    sai do aro mas continua no plano da placa (y = 2,87). A rampa do
    stow interpola J2, J3 e J4 juntos, e nos primeiros 2 s a ponta vai
    para y = 2,97..2,99: 100-120 mm para DENTRO da placa, a 0,85 m de
    altura, antes de subir.

    Recuar a base 0,25 m com o braço parado tira a ferramenta inteira
    da parede sem IK perto da placa; a rampa varre depois com folga.
    Reusa _creep_base (linha reta, sem girar, odometria) pelo mesmo
    motivo que o puxão dos arcos: base parada e braço parado é mais
    estável do que varrer o punho junto da parede. Passo de segurança:
    se falhar (inclinação, timeout), avisa e deixa o recolhimento seguir.
    """
    if ctx.robot_state is None or ctx.saida_recuo_m <= 0:
        return
    if not _creep_base(ctx, -abs(ctx.saida_recuo_m), tag + '/afasta'):
        rospy.logwarn('[mission] %s: recuo da base antes do stow não '
                      'completou — recolhendo assim mesmo', tag)


def _tooltip_now(ctx):
    """Posição atual da PONTA da ferramenta, medida pelo T265."""
    p = ctx.robot_state.ee_pose.pose.position
    o = ctx.robot_state.ee_pose.pose.orientation
    R = quaternion_matrix([o.x, o.y, o.z, o.w])[:3, :3]
    return np.array([p.x, p.y, p.z]) + R @ T_T265_TOOLTIP[:3, 3]


def _creep_base(ctx, distancia, tag):
    """Anda em linha reta pela distância dada (+ avança, − recua).

    Deslocamento fino só com as rodas, sem girar e sem mexer no braço.
    O _navigate_to não serve aqui: ele é girar-avançar-girar, pensado
    para navegação, e girar com a ferramenta engatada no olhal
    arrancaria o gancho.

    Existe por dois motivos, ambos apontados pelo Marco em 2026-08-25:
    fechar os últimos centímetros DEPOIS que o braço já está armado, e
    executar o puxão da abertura recuando o chassi em vez de recolher o
    braço. A base tem tração de sobra e a postura do braço fica parada,
    o que é bem mais estável do que varrer o punho perto da parede.

    Referência é a odometria, não a tag: é deslocamento relativo curto,
    e a odometria da base é boa nessa escala.
    """
    p0 = ctx.robot_state.base_odom.pose.pose.position
    x0, y0 = p0.x, p0.y
    alvo = abs(distancia)
    sentido = 1.0 if distancia >= 0 else -1.0
    rospy.loginfo('[mission] %s: %s %.3f m só com as rodas', tag,
                  'avançando' if sentido > 0 else 'RECUANDO', alvo)

    rate = rospy.Rate(ctx.rate_hz)
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        if ctx.tilt_critical:
            ctx.stop_base()
            rospy.logerr('[mission] %s: abortado por inclinação crítica', tag)
            return False
        if (rospy.Time.now() - t0).to_sec() > ctx.creep_timeout:
            ctx.stop_base()
            rospy.logerr('[mission] %s: timeout andando %.3f m', tag, alvo)
            return False

        p = ctx.robot_state.base_odom.pose.pose.position
        andou = math.hypot(p.x - x0, p.y - y0)
        if andou >= alvo:
            ctx.stop_base()
            rospy.loginfo('[mission] %s: concluído (%.3f m)', tag, andou)
            return True

        # Freio por medida também no avanço: o laser é a única defesa que
        # não depende de estimativa nenhuma.
        if (sentido > 0 and ctx.front_clearance is not None
                and ctx.front_clearance < ctx.min_clearance):
            ctx.stop_base()
            rospy.logwarn('[mission] %s: laser a %.2f m (mín %.2f) — parando '
                          'antes de completar', tag, ctx.front_clearance,
                          ctx.min_clearance)
            return True

        cmd = Twist()
        cmd.linear.x = sentido * ctx.creep_vel
        ctx.pub_cmdvel.publish(cmd)
        rate.sleep()
    ctx.stop_base()
    return False


def _reach_by_wholebody(ctx, p_goal, phase):
    """Fecha a fase com o controlador whole-body de 8 DOF.

    Reativa o caminho que estava morto (2026-08-24): release_base() nunca
    era chamado, /b166er/ee_target nunca era publicado e
    /b166er/servo_tooltip nunca ia a True. Medido numa missão inteira a
    50 Hz: arm_vel_cmd != 0 em 0 de 21396 amostras — o fuzzy_wb_controller
    ficava em stand-down do início ao fim e a manipulação toda acontecia
    por IK em espaço de juntas. Ou seja, o controle whole-body, que é a
    contribuição central, não participava de nada.

    Por que ele tinha sido desligado, e por que dá para religar: o
    comentário de 2026-08-13 dizia que os dois "não podem coexistir",
    porque o controlador seguia perseguindo o último ee_target e movia a
    base enquanto a missão comandava posturas. Mas o alvo estava velho
    justamente porque a missão nunca publicava um novo — e não há disputa
    por /cmd_vel: stop_base() emite um único Twist zero, e nem Manipulate
    nem _reach_by_iterative_ik publicam em laço. O conflito era alvo
    obsoleto, não incompatibilidade.

    Daí a ordem aqui, que não é acidental: publicar o alvo ANTES de
    habilitar, nunca depois. Entre o release_base() e o primeiro
    ee_target o controlador atuaria sobre o alvo da fase anterior.

    A chegada é medida pela MISSÃO, não pelo controlador: ele tem um
    _reached interno com tolerância própria e não publica sinal nenhum.
    Manter o critério aqui evita duas autoridades discordando sobre o
    que é "chegou".
    """
    p_goal = np.asarray(p_goal, dtype=float)

    # O painel só sabia da fase pelo caminho da IK iterativa: neste modo
    # a trilha FASE MANIP. ficava vazia a missão inteira (bateria de
    # 2026-09-03). Carimba a fase aqui também.
    ctx.status(estado='MANIPULATE', fase=phase, it=0)
    alvo = PoseStamped()
    alvo.header.stamp = rospy.Time.now()
    alvo.header.frame_id = 'odom'
    alvo.pose.position.x = float(p_goal[0])
    alvo.pose.position.y = float(p_goal[1])
    alvo.pose.position.z = float(p_goal[2])
    alvo.pose.orientation.w = 1.0   # com servo_tooltip só a posição conta
    ctx.pub_target.publish(alvo)
    rospy.sleep(0.2)

    ctx.release_base()
    rospy.loginfo('[mission] fase "%s": whole-body assumiu (base livre, '
                  'restrita pelo plano de exclusão)', phase)

    ok = False
    try:
        t0 = rospy.Time.now()
        rate = rospy.Rate(10)
        estaveis = 0
        melhor = float('inf')
        cfg_fase = ctx.phases.get(phase, {}) if isinstance(ctx.phases, dict) else {}
        cfg_curso = cfg_fase.get('curso_min_m')
        cfg_estagna = float(cfg_fase.get('curso_estagna_m', 0.010))
        tol_fase = _tolerancia_da_fase(ctx, phase)[0]
        if tol_fase is None:
            tol_fase = np.array([ctx.tol_pos] * 3)
        hist = []
        while not rospy.is_shutdown():
            if ctx.tilt_critical:
                rospy.logerr('[mission] fase "%s": abortada por inclinação '
                             'crítica', phase)
                return False
            if (rospy.Time.now() - t0).to_sec() > ctx.wb_phase_timeout:
                rospy.logerr('[mission] fase "%s": whole-body não fechou em '
                             '%.0fs (melhor %.4f m; régua por eixo da fase)',
                             phase, ctx.wb_phase_timeout, melhor)
                return False

            p_now = _tooltip_now(ctx)
            err   = p_goal - p_now
            n_err = float(np.linalg.norm(err))
            melhor = min(melhor, n_err)

            # A MESMA RÉGUA DA IK ITERATIVA (2026-09-03, pedido do Marco:
            # "vamos realizar os testes com a mesma régua"). Até a bateria
            # das runs 21-25 este modo fechava por esfera de 20 mm e a
            # postura por eixo (4-15 mm): o 'destrava' fechava em ~1 s com
            # a lingueta em 0-7 mm porque o alvo já cabia na esfera. Agora
            # é _fase_fechou nos dois caminhos.
            fechou, e_parede = _fase_fechou(ctx, phase, err, n_err)
            if e_parede is not None:
                ctx.status(estado='MANIPULATE', fase=phase,
                           e_eixo_mm=e_parede[0] * 1000,
                           e_prof_mm=e_parede[1] * 1000,
                           e_alt_mm=e_parede[2] * 1000, erro_m=n_err)
            # ESTAGNAÇÃO = FIM DE CURSO (2026-09-03). A sonda da run62
            # mostrou o que acontece quando a fase de descida continua
            # empurrando depois que o anel bateu no fim de curso: J2/J3
            # a −20 N·m, o J4 saturado em −20 N·m e retro-acionado de 66°
            # (setpoint) para 110° (real) — o punho colapsa, a profundidade
            # foge e a fase estoura o timeout. Numa fase com curso_min_m,
            # se a descida medida parou de crescer (< 0,5 mm em 1,5 s)
            # depois de pelo menos curso_estagna_m, a fase fecha AQUI: o
            # mecanismo chegou onde podia. Na bancada é a mesma leitura
            # (a T265 vê a ponta parar com o comando ainda descendo).
            desc = _descida_desde_captura(ctx, phase, e_parede) if e_parede is not None else None
            if desc is not None and cfg_curso is not None:
                hist.append(((rospy.Time.now() - t0).to_sec(), desc))
                # JANELA: 1,5 s / 0,5 mm. A versão 3 s / 0,3 mm com mínimo de
                # 15 mm (run68) não protegeu o punho: quando o anel bate no
                # fim de curso a ponta CONTINUA descendo — pelo colapso do J4
                # (retro-acionado a 20 N·m), não pelo anel — e a estagnação
                # nunca é vista; o braço tombou. Com a janela curta a fase
                # fecha cedo (anel em 2-9 mm, a libera termina o serviço),
                # mas o punho fica íntegro (esforço do J4 2-3 N·m, 5/5 na
                # bateria 63-67). Enquanto o braço simulado tiver um punho
                # de 20 N·m, é este o compromisso honesto.
                while hist and hist[-1][0] - hist[0][0] > 1.5:
                    hist.pop(0)
                if (len(hist) >= 10 and desc >= cfg_estagna
                        and hist[-1][1] - hist[0][1] < 0.0005
                        and abs(e_parede[0]) < tol_fase[0]
                        and abs(e_parede[1]) < tol_fase[1]):
                    rospy.logwarn('[mission] fase "%s": descida estagnou em %.1f mm '
                                  '(mín %.0f para aceitar; alvo %.0f) — fim de curso do '
                                  'mecanismo, fechando a fase', phase, desc * 1000,
                                  cfg_estagna * 1000, cfg_curso * 1000)
                    ctx.status(descida_mm=desc * 1000, estagnou=1)
                    ok = True
                    return True

            # Exige a tolerância SUSTENTADA: um instante dentro dela pode
            # ser a ponta passando de raspão durante o transitório.
            if fechou:
                estaveis += 1
                if estaveis >= ctx.wb_settle_samples:
                    if e_parede is not None:
                        tol_xyz, _ = _tolerancia_da_fase(ctx, phase)
                        rospy.loginfo('[mission] fase "%s" alcançada por '
                                      'whole-body em %.1fs (%.4f m) — no frame '
                                      'da parede eixo=%+.1f prof=%+.1f alt=%+.1f '
                                      'mm (tolerância %.0f/%.0f/%.0f)', phase,
                                      (rospy.Time.now() - t0).to_sec(), n_err,
                                      *(e_parede * 1000), *(tol_xyz * 1000))
                    else:
                        rospy.loginfo('[mission] fase "%s" alcançada por '
                                      'whole-body em %.1fs (%.4f m)', phase,
                                      (rospy.Time.now() - t0).to_sec(), n_err)
                    ok = True
                    return True
            else:
                estaveis = 0
            if e_parede is not None:
                rospy.loginfo_throttle(2.0,
                    '[mission] fase "%s": whole-body a %.4f m do alvo — eixo=%+.1f '
                    'prof=%+.1f alt=%+.1f mm', phase, n_err, *(e_parede * 1000))
            else:
                rospy.loginfo_throttle(2.0,
                    '[mission] fase "%s": whole-body a %.4f m do alvo', phase, n_err)
            rate.sleep()
        return False
    finally:
        # Devolver o /cmd_vel é obrigatório mesmo no caminho de falha: sem
        # isso o controlador segue perseguindo este alvo na fase seguinte,
        # que é exatamente o bug de 2026-08-13.
        ctx.take_base()
        ctx.stop_base()
        if not ok:
            ctx.publish_keepout()


def _rolagem_degrau(ctx, q):
    """Giro do degrau em torno do próprio eixo, contra a vertical (graus).

    É ESTA a grandeza que decide se o degrau passa pelo aro, e ela nunca
    era medida. O log imprimia o ângulo da DIREÇÃO da haste — que estava
    impecável em 0,1° — enquanto a peça batia na borda por estar rolada.

    O degrau é uma caixa de 20 x 17 mm; a abertura livre do oval é
    30 x 40 mm (já descontado o arame de 6 mm). Rolado, o retângulo
    apresenta uma silhueta maior, e além de ~20° o canto sai do contorno
    da elipse. Medido em 2026-09-02: chegava a 24,8° e travava, com
    `tool_tip_collision_3` contra os segmentos 3 a 7 do anel.
    """
    if ctx.wall_R is None:
        return None
    T = fk_arm(list(q)) @ T_T265_TOOLTIP
    x = -T[:3, 0]
    up = np.array([0.0, 0.0, 1.0])
    v = up - float(up @ x) * x
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return None
    return math.degrees(math.acos(min(1.0, abs(float(T[:3, 2] @ (v / n))))))


def _reach_by_iterative_ik(ctx, p_goal, phase):
    """Alcança p_goal iterando IK + medição, em vez de servo Cartesiano.

    Por que iterar em vez de deixar a malha Cartesiana fechar: depois de
    comandar a solução de IK, o braço FÍSICO não chega nela — os
    controladores de posição têm droop de 1,4 a 3,3° por junta sob
    gravidade (medido em 2026-08-13), o que deixa a ponta 3 a 10 cm
    aquém. A malha Cartesiana enxerga esse resíduo (o T265 mede a pose
    real) mas nem sempre consegue fechá-lo: em algumas configurações a
    Jacobiana não tem autoridade na direção necessária e o DLS empurra
    juntas até os batentes, com o erro estacionando.

    A iteração ataca o droop pelo que ele é — um erro sistemático e
    repetível: mede o quanto faltou e recomanda a IK mirando ALÉM do
    alvo, na mesma medida. Como o droop é função da postura e a postura
    quase não muda entre iterações, duas ou três bastam.

    Cada passo usa a postura atual como semente (continuidade de ramo).
    """
    p_goal = np.asarray(p_goal, dtype=float)
    p_aim  = p_goal.copy()

    # ÂNCORA DA BASE DURANTE A FASE.
    #
    # A base fica parada aqui por intenção — quem se move é o braço. Mas
    # o modelo do Pioneer deriva 2,6 mm/s (ver ancora_engata), e uma
    # fase de 5 iterações leva ~15 s: são ~39 mm de avanço enquanto a IK
    # persegue um alvo FIXO NO MUNDO.
    #
    # MEDIDO em 2026-09-01, na "atravessa": o J1 ficava devendo cada vez
    # mais a cada iteração (-2,1° / -2,5° / -3,2° / -3,7° / -4,5°) e o
    # erro de eixo estagnou em 7,8 mm por três iterações seguidas. Eu
    # tinha lido isso como limite do braço; é chão se movendo debaixo
    # dele — a IK recalcula para uma base que mudou de lugar desde a
    # iteração anterior. O Marco viu na tela ("o robot não está
    # parando... vai pra frente e volta") antes de eu ligar as duas
    # coisas.
    ctx.ancora_engata('fase "%s"' % phase)

    def _resolve_ik(p_local, R_arm_world):
        """IK da fase. Com ~degrau_alinhado, impõe também a DIREÇÃO do
        degrau — sem isso ele chega perpendicular ao furo.

        Medido em 2026-08-27, com a IK de posição pura, o ângulo entre o
        degrau e o eixo do furo nas seis fases: 89,6° / 88,2° / 88,2° /
        85,7° / 84,3° / 79,8°. Não é desalinho, é ortogonal — a
        ferramenta não tinha como atravessar o anel em nenhuma execução,
        e é por isso que a lâmina ficou em 0,0° nas 8 da bateria.

        O eixo do furo é o X LOCAL da parede (o furo olha para o lado);
        `wall_R` já vem achatado para yaw puro.
        """
        q_atual = np.array(ctx.robot_state.q_arm)
        if not ctx.degrau_alinhado or ctx.wall_R is None:
            q, e = ik_tooltip_position(p_local, q_current=q_atual)
            return q, e, None
        eixo_arm = R_arm_world @ (ctx.wall_R @ np.array([1.0, 0.0, 0.0]))

        # NIVELADO PRIMEIRO, EIXO COMO RESERVA.
        #
        # A IK nivelada usa 2 de posição (transversal, pesando pouco a
        # direção do furo) + 3 de atitude, e é ela que impede o degrau de
        # chegar rolado — foi rolado 13,6° que ele raspou o arame de cima
        # do anel em 2026-08-27.
        #
        # Mas a atitude completa nem sempre é alcançável: medido, a fase
        # 'orienta' (150 mm afastada da parede) não fecha com o degrau
        # nivelado, enquanto fecha em 0,3 mm só com o eixo alinhado. Em
        # vez de configurar fase a fase — que envelhece mal e some quando
        # alguém mexe no YAML — a escolha degrada sozinha: se a nivelada
        # não alcança, vale a de eixo, que já é muito melhor que posição
        # pura.
        up_arm = R_arm_world @ np.array([0.0, 0.0, 1.0])

        # SÓ APLICA a formulação que o DEPLOY escolheu. Nenhuma decisão
        # aqui: trocar de formulação entre fases (ou entre iterações da
        # mesma fase) troca o ramo de solução e produz os giros de mais
        # de cem graus que o Marco viu. Se o DEPLOY não rodou — abort
        # precoce, ou modo sem alinhamento —, cai no de eixo, que é o
        # mais conservador dos dois.
        # SEMENTE ÚNICA NAS FASES — busca LOCAL, não global.
        #
        # A IK faz multi-start: além da postura atual, semeia com uma
        # lista de posturas espalhadas pelo espaço de juntas. Isso é o
        # certo no DEPLOY, onde não há histórico e é preciso ACHAR um
        # bom ramo. Durante as fases é nocivo: a busca global encontra
        # uma solução de custo ligeiramente menor na outra família e
        # migra para lá, e o peso de continuidade não segura um salto
        # desse tamanho.
        #
        # MEDIDO em 2026-09-01, com a âncora de ramo já ativa no
        # DEPLOY. O braço percorreu DEPLOY J4=-97,6° → orienta -104,2°
        # → aproxima_lateral -73,4° (tudo na família certa, degrau
        # estável em 7,2/7,3/8,1°) e então, NA atravessa, saltou para
        # J4=+68,4°: 142° de reconfiguração no meio do movimento de
        # inserção. A haste chegou apoiada na barra da lâmina em vez
        # de enfiada no furo — 1085 contatos com chave_blade contra 4
        # com o anel. O Marco viu como 'o manipulador caiu e depois
        # ficou chocando com a chave'.
        #
        # Com semente única a solução só pode ser a contínua com onde
        # o braço já está: sair da família deixa de ser possível.
        seeds = [q_atual]
        if ctx.ik_modo == 'nivelado':
            return ik_tooltip_nivelado(p_local, eixo_arm, up_arm,
                                       q_seeds=seeds, q_current=q_atual,
                                       sentido_fixo=True)
        return ik_tooltip_com_degrau(p_local, eixo_arm, q_seeds=seeds,
                                     q_current=q_atual, sentido_fixo=True)

    for it in range(ctx.ik_max_iters):
        if ctx.tilt_critical:
            ctx.ancora_solta()
            rospy.logerr('[mission] fase "%s": abortada por inclinação crítica',
                         phase)
            return False

        ctx.ancora_mantem()
        b = ctx.robot_state.base_odom.pose.pose
        T_wb = quaternion_matrix([b.orientation.x, b.orientation.y,
                                  b.orientation.z, b.orientation.w])
        T_wb[:3, 3] = [b.position.x, b.position.y, b.position.z]
        T_arm_world = np.linalg.inv(T_wb @ T_BASELINK_ARM)
        p_local = (T_arm_world @ np.append(p_aim, 1))[:3]

        q_ik, err_ik, ang_ik = _resolve_ik(p_local, T_arm_world[:3, :3])

        if err_ik > ctx.deploy_ik_tol:
            # O ponto EXTRAPOLADO saiu do alcance — não o alvo real.
            # Abortar aqui era exagero: em 1 de 5 execuções da bateria
            # de repetibilidade (2026-08-21) a missão morreu no
            # pre_engage porque a mira além caiu 25 mm fora do
            # workspace. Recuar para o alvo verdadeiro é sempre
            # possível (foi alcançável no DEPLOY) e custa só perder a
            # compensação daquela iteração.
            if not np.allclose(p_aim, p_goal):
                rospy.logwarn('[mission] fase "%s" it%d: mira além ficou fora '
                              'de alcance (%.3f m) — recuando para o alvo real',
                              phase, it, err_ik)
                p_aim = p_goal.copy()
                p_local = (T_arm_world @ np.append(p_aim, 1))[:3]
                q_ik, err_ik, ang_ik = _resolve_ik(p_local,
                                                   T_arm_world[:3, :3])
            if err_ik > ctx.deploy_ik_tol:
                rospy.logwarn('[mission] fase "%s" it%d: IK não fechou (%.3f m) '
                              '— alvo fora de alcance deste standoff',
                              phase, it, err_ik)
                return False

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name     = JOINT_NAMES
        msg.position = q_ik.tolist()
        ctx.posture_done = False
        ctx.pub_posture.publish(msg)
        if not _wait_posture(ctx):
            return False
        rospy.sleep(ctx.ik_settle_time)   # deixa o PID assentar antes de medir

        p_now = _tooltip_now(ctx)
        err   = p_goal - p_now
        n_err = float(np.linalg.norm(err))

        # O braço chegou onde a IK mandou? Distingue "IK ruim" de
        # "controlador não seguiu" — sem isso as duas causas produzem o
        # mesmo sintoma (ponta longe do alvo).
        q_real = np.array(ctx.robot_state.q_arm)
        dq = np.degrees(q_real - q_ik)
        # ROTULO CORRETO POR FORMULACAO. As duas IKs devolvem coisas
        # DIFERENTES no terceiro retorno: a de eixo devolve o ângulo
        # entre o degrau e o eixo do furo; a nivelada devolve o erro
        # TOTAL de atitude. Eu imprimia as duas no mesmo campo com o
        # mesmo nome, e li 85° de erro de atitude como se fosse a
        # ferramenta de lado (2026-08-27). Grandezas diferentes, nomes
        # diferentes.
        rotulo = ('atitude' if ctx.ik_modo == 'nivelado' else 'eixo')
        ang_txt = ('%s %.1f°' % (rotulo, math.degrees(ang_ik))
                   if ang_ik is not None else 'n/d')
        rol = _rolagem_degrau(ctx, q_real)
        rol_txt = ('rolagem %.1f°' % rol) if rol is not None else 'rolagem n/d'
        rospy.loginfo('[mission] fase "%s" it%d: ponta (%.3f, %.3f, %.3f) '
                      'erro=%.4f | degrau %s | %s | IK pediu %s | '
                      'faltou %s graus',
                      phase, it, *p_now, n_err, ang_txt, rol_txt,
                      np.degrees(q_ik).round(1), dq.round(1))
        # 'saida_eixo' passa por aqui durante o RETRACT/ABORT: não
        # carimbar MANIPULATE nesse caso (o painel mostrava a saída
        # como se fosse manipulação — Marco, 2026-09-03).
        est = {'estado': 'MANIPULATE'} if phase in PHASE_ORDER else {}
        ctx.status(fase=phase, it=it, erro_m=n_err, **est,
                   tol_m=ctx.tol_pos, ik=ctx.ik_modo or 'n/d',
                   ang_rotulo=rotulo,
                   ang_deg=(math.degrees(ang_ik) if ang_ik is not None else None),
                   q_pedido=np.degrees(q_ik).round(1).tolist(),
                   faltou=dq.round(1).tolist())

        fechou, e_parede = _fase_fechou(ctx, phase, err, n_err)
        if e_parede is not None:
            tol_xyz, _ = _tolerancia_da_fase(ctx, phase)
            rospy.loginfo('[mission] fase "%s" it%d: no frame da parede '
                          'eixo=%+.1f prof=%+.1f alt=%+.1f mm '
                          '(tolerância %.0f/%.0f/%.0f)',
                          phase, it, *(e_parede * 1000), *(tol_xyz * 1000))
            ctx.status(fase=phase, **est,
                       e_eixo_mm=e_parede[0] * 1000,
                       e_prof_mm=e_parede[1] * 1000,
                       e_alt_mm=e_parede[2] * 1000)
        if fechou:
            n_corr = ctx.ancora_solta()
            rospy.loginfo('[mission] fase "%s" alcançada em %d iteração(ões) '
                          '(%.4f m)%s', phase, it + 1, n_err,
                          (' — âncora corrigiu %d vez(es)' % n_corr)
                          if n_corr else '')
            return True

        # Mira além, na medida do que faltou — AMORTECIDA. Corrigir o
        # erro inteiro de uma vez extrapola demais quando o primeiro
        # passo erra muito, e o ponto extrapolado pode sair do
        # workspace. Com 0,8 converge em praticamente o mesmo número de
        # iterações e fica dentro do alcance.
        p_aim = p_aim + ctx.ik_correction_gain * err

    ctx.ancora_solta()
    rospy.logerr('[mission] fase "%s": %d iterações sem fechar (%.4f m, '
                 'tolerância %.3f)', phase, ctx.ik_max_iters, n_err, ctx.tol_pos)
    return False


def _wait_ee_convergence(ctx, T_target, phase):
    rate = rospy.Rate(10)
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        if ctx.tilt_critical:
            rospy.logerr('[mission] fase "%s": abortada por inclinação crítica',
                         phase)
            return False
        st = ctx.robot_state
        p, o = st.ee_pose.pose.position, st.ee_pose.pose.orientation
        T_cur = quaternion_matrix([o.x, o.y, o.z, o.w])
        T_cur[:3, 3] = [p.x, p.y, p.z]
        # Erro medido na PONTA DA FERRAMENTA (o T265 é só o sensor).
        p_tip_cur = T_cur[:3, 3] + T_cur[:3, :3] @ T_T265_TOOLTIP[:3, 3]
        err = pose_error(T_cur, T_target)
        rp = float(np.linalg.norm(T_target[:3, 3] - p_tip_cur))
        ro = float(np.linalg.norm(err[3:]))

        # Só posição: durante a manipulação a base fica travada e o
        # controlador serve apenas posição (5 DOF não fecham pose 6D —
        # ver fuzzy_wb_controller, etapa 4a). Exigir orientação aqui
        # seria esperar por um critério que ninguém está perseguindo.
        if rp < ctx.tol_pos:
            rospy.loginfo('[mission] fase "%s" convergiu (%.4f m; ori %.3f rad, '
                          'não controlada)', phase, rp, ro)
            return True

        ctx.ancora_mantem()

        if (rospy.Time.now() - t0).to_sec() > ctx.phase_timeout:
            rospy.logerr('[mission] fase "%s": timeout (%.4f m, %.4f rad)',
                         phase, rp, ro)
            return False
        rate.sleep()
    return False


# ═══════════════════════════════════════════════════════════════════════
def main():
    rospy.init_node('chave_mission')
    ctx = MissionContext()

    sm = smach.StateMachine(outcomes=['MISSION_OK', 'MISSION_ABORTED'])
    with sm:
        smach.StateMachine.add('STOW_INIT', StowInit(ctx),
                               transitions={'ok': 'SEARCH', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('SEARCH', Search(ctx),
                               transitions={'found': 'APPROACH', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('APPROACH', Approach(ctx),
                               transitions={'arrived': 'REFINE', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('REFINE', Refine(ctx),
                               transitions={'ok': 'DEPLOY', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('DEPLOY', Deploy(ctx),
                               transitions={'ok': 'MANIPULATE', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('MANIPULATE', Manipulate(ctx),
                               transitions={'done': 'RETRACT', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('RETRACT', Retract(ctx),
                               transitions={'ok': 'RETURN', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('RETURN', Return(ctx),
                               transitions={'home': 'MISSION_OK', 'failed': 'ABORT_SAFE'})
        smach.StateMachine.add('ABORT_SAFE', AbortSafe(ctx),
                               transitions={'aborted': 'MISSION_ABORTED'})

    sis = smach_ros.IntrospectionServer('chave_mission', sm, '/CHAVE_MISSION')
    sis.start()
    outcome = sm.execute()
    rospy.loginfo('[mission] resultado: %s', outcome)
    sis.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
