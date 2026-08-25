#!/usr/bin/env python3
"""
gazebo_arm_bridge — integra /b166er/arm_vel_cmd → position controllers do Gazebo.

Equivalente ao arm_vel_integrator.py mas para simulação:
  - Integra ẋ_arm (rad/s) → q_arm (rad) via Euler a 20 Hz
  - Publica Float64 individuais para /Jx_position_controller/command

Feedforward de gravidade ativo desde o primeiro ciclo (usando q_spawn)
para que os controladores nunca vejam erro estático e o braço não caia
ao inicializar. Após o warm-start com /joint_states, usa q_arm real.

Posturas (2026-08-13): este nó é o DONO do estado integrado de juntas —
qualquer publicação externa direta nos Jx_position_controller/command é
sobrescrita a 20 Hz (comportamento confirmado ao vivo). Por isso a
interface de postura mora aqui: /b166er/arm_posture_cmd (JointState com
5 positions) inicia uma rampa em espaço de juntas até a postura pedida,
a ~posture_ramp_vel rad/s — nunca por degrau de setpoint, que com
P=300 (J2) já chutou o chassi no passado (ver b166er_gazebo.launch).
Enquanto uma postura está ativa, /b166er/arm_vel_cmd é ignorado
(movimento de postura é deliberado, o controlador Cartesiano não deve
interferir no meio). Ao concluir, publica True em
/b166er/arm_posture_reached (latched).

Home recolhido: com ~goto_home_on_start=true (default), após o
warm-start o braço rampa sozinho até /arm_postures/stow_home — postura
recolhida definida pelo Marco (J2 pra cima, J3 pra baixo, punho
dobrado) para manter o CG do conjunto compacto; o braço esticado do
antigo q=0 já tombou o robô duas vezes em teste (2026-08-12). O spawn
continua em q=0 (determinístico, sem a race do -J documentada no
b166er_gazebo.launch) — o recolhimento é pós-spawn, suave.
"""

import rospy
import numpy as np
from sensor_msgs.msg import JointState

from b166er_whole_body_control.msg import RobotState
from std_msgs.msg import Float64, Bool, Empty

from b166er_whole_body_control.kinematics import (
    JOINT_NAMES, JOINT_LOWER, JOINT_UPPER, gravity_torque_arm)

_VEL_TIMEOUT = 0.5   # s sem comando → zera velocidade

# Ganhos P dos controladores (arm_controllers.yaml) — usados no feedforward
_P_GAINS = np.array([100.0, 300.0, 200.0, 100.0, 50.0])


class GazeboArmBridge:

    def __init__(self):
        rospy.init_node('gazebo_arm_bridge')

        self._rate_hz = rospy.get_param('~rate', 20.0)
        self._k_ff    = rospy.get_param('~k_gravity_ff', 1.0)

        # Posição de spawn (deve bater com o spawn_model — sem -J, q=0).
        # Usada para calcular o feedforward antes do warm-start real.
        q_spawn_list  = rospy.get_param('~q_spawn', [0.0, 0.0, 0.0, 0.0, 0.0])
        self._q_spawn = np.clip(np.array(q_spawn_list, dtype=float),
                                JOINT_LOWER, JOINT_UPPER)

        self._q_arm      = None          # rad — None até warm-start real
        self._dq_cmd     = np.zeros(5)
        self._t_last_cmd = None

        # ── Posturas (rampa em espaço de juntas) ──
        self._ramp_vel    = rospy.get_param('/arm_postures/ramp_velocity', 0.3)
        self._reached_tol = rospy.get_param('/arm_postures/reached_tolerance', 0.02)
        self._goto_home   = rospy.get_param('~goto_home_on_start', True)
        home_list = rospy.get_param('/arm_postures/stow_home',
                                    [0.0, 1.1, -1.0, -1.8, 0.0])
        self._q_home = np.clip(np.array(home_list, dtype=float),
                               JOINT_LOWER, JOINT_UPPER)
        self._q_posture_target = None    # None = sem postura ativa

        self._pubs = [
            rospy.Publisher(f'/{n}_position_controller/command',
                            Float64, queue_size=1)
            for n in JOINT_NAMES
        ]
        self._pub_posture_reached = rospy.Publisher(
            '/b166er/arm_posture_reached', Bool, queue_size=1, latch=True)
        # Alvo da rampa corrente (latched) — consumido pelo state_estimator
        # como re-seed do IK ao concluir a postura (a postura comandada é
        # informação que o robô real sem encoders também tem: é o setpoint
        # que o firmware acabou de executar, não ground truth de simulador).
        self._pub_posture_target = rospy.Publisher(
            '/b166er/arm_posture_target', JointState, queue_size=1, latch=True)

        rospy.Subscriber('/joint_states', JointState,
                         self._cb_joint_states, queue_size=1)
        rospy.Subscriber('/b166er/arm_vel_cmd', JointState,
                         self._cb_vel_cmd, queue_size=1)
        rospy.Subscriber('/b166er/arm_posture_cmd', JointState,
                         self._cb_posture_cmd, queue_size=1)
        # SEGURANÇA DE TOMBAMENTO: congela o braço onde está. Mexer um
        # braço de 28 kg estendido com o chassi tombando só piora a
        # situação — e a rampa de postura em curso é justamente o tipo
        # de comando que continuaria rodando alheio ao acidente. A
        # reação NÃO é congelar e sim recolher: ver o bloco em _run().
        self._resync_pedido = False
        rospy.Subscriber('/b166er/arm_resync', JointState, self._cb_resync)

        # ── Realimentação da postura MEDIDA ──
        # Fonte: state_estimator (IK da pose da T265). É a única medida de
        # postura que existe no robô real, que não tem encoder — usar
        # /joint_states aqui funcionaria no Gazebo e seria mentira no
        # hardware.
        self._q_med = None
        self._t_q_med = None
        # /b166er/robot_state e NÃO /b166er/estimated_joint_states: só o
        # primeiro carrega ik_converged. A IK do estimador nem sempre
        # fecha — foram 41 não-convergências numa bateria, com resíduos
        # de até 1,03 m e 1,54 rad, ou seja, postura sem sentido nenhum.
        # Consumir isso como medida boa fazia a ponte "corrigir" contra
        # um alvo fantasma e nunca declarar a postura atingida.
        rospy.Subscriber('/b166er/robot_state', RobotState,
                         self._cb_q_medido, queue_size=1)
        # Tolerância na MEDIDA, MUITO mais folgada que a do modelo.
        #
        # Dimensionada pela separação de escalas, não por precisão
        # desejada: o estimador (IK da pose da T265) tem erro próprio de
        # poucos graus — medido 3,2° com o braço parado em stow — e a
        # divergência que precisamos pegar é de dezenas de graus (28° no
        # caso que motivou isto). 0,15 rad ≈ 8,6° fica no meio.
        #
        # Apertar demais é pior que não ter: com 0,05 rad (2,9°) o
        # critério ficou abaixo do ruído do estimador e NENHUMA postura
        # fechava — a missão morria já no primeiro stow_home.
        self._reached_tol_medido = rospy.get_param('~reached_tol_medido', 0.15)
        self._med_timeout = rospy.get_param('~medida_timeout', 1.0)
        # Correção integral contra o erro estacionário dos controladores.
        self._corr_vel = rospy.get_param('~correcao_vel', 0.15)   # rad/s
        self._corr_max = rospy.get_param('~correcao_max', 0.60)   # rad
        # Tempo máximo empurrando a correção antes de aceitar a postura
        # com o resíduo que sobrar. Sem isto o DEPLOY ficava preso até o
        # timeout de 30 s da missão.
        self._corr_timeout = rospy.get_param('~correcao_timeout', 5.0)
        self._correcao = np.zeros(5)
        self._t_no_alvo = None
        self._modelo_no_alvo = False

        self._tilt_critical = False
        # Fração de ramp_velocity usada na retração de emergência.
        self._tilt_retract_scale = rospy.get_param('~tilt_retract_scale', 0.6)
        rospy.Subscriber('/b166er/tilt_critical', Bool, self._cb_tilt_critical)

        rospy.loginfo('[gazebo_arm_bridge] q_spawn=%s  aguardando warm-start...',
                      np.round(self._q_spawn, 3))

    # ------------------------------------------------------------------
    def _q_pub_for(self, q_arm):
        """Posição comandada ao PID: q_arm + feedforward de gravidade."""
        tau_g = gravity_torque_arm(q_arm)
        q_ff  = -self._k_ff * tau_g / _P_GAINS
        return np.clip(q_arm + q_ff, JOINT_LOWER, JOINT_UPPER)

    def _cb_resync(self, _msg):
        """Reancora q_arm nas juntas REAIS. Usar após reset da simulação.

        A ponte integra q_arm em malha aberta e só lê /joint_states uma
        vez, no warm-start — fiel ao robô real, que não tem encoder. Mas
        isso torna o estado interno IMPOSSÍVEL de corrigir depois, e o
        reset do Gazebo teleporta as juntas sem que a ponte saiba: ela
        segue comandando os controladores para a postura antiga e o braço
        SALTA de volta no instante em que a física despausa.

        Observado em 2026-08-24: reset com o braço estendido levava a
        inclinação de 0,007 para 1,114 rad — o robô "nascia caindo". O
        efeito também invalidava baterias inteiras, porque cada execução
        começava com um solavanco de 28 kg.

        Não é atalho de simulação: no hardware o equivalente é o
        procedimento de homing, que também reancora a estimativa a uma
        referência externa.
        """
        if len(_msg.position) == 5:
            # Postura entregue explicitamente. É o caminho que funciona
            # com a FÍSICA PAUSADA: /joint_states não atualiza enquanto o
            # Gazebo está parado, e reancorar só depois de despausar é
            # tarde — os controladores de posição já seguram o setpoint
            # antigo e disparam o braço no primeiro passo de física.
            q = np.array(_msg.position, dtype=float)
            q = (q + np.pi) % (2 * np.pi) - np.pi
            self._q_arm = np.clip(q, JOINT_LOWER, JOINT_UPPER)
            self._q_posture_target = None
            self._dq_cmd = np.zeros(5)
            # Emitir o comando JÁ, ainda pausado, para o setpoint antigo
            # não sobreviver ao unpause.
            q_pub = self._q_pub_for(self._q_arm)
            for i, pub in enumerate(self._pubs):
                pub.publish(Float64(data=float(q_pub[i])))
            rospy.logwarn('[gazebo_arm_bridge] q_arm reancorado por comando '
                          'explícito em %s rad', np.round(self._q_arm, 3))
            return
        self._resync_pedido = True
        rospy.logwarn('[gazebo_arm_bridge] ressincronização pedida — '
                      'reancorando q_arm nas juntas reais')

    def _cb_joint_states(self, msg):
        if self._q_arm is not None and not self._resync_pedido:
            return   # warm-start one-shot
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(n in name_to_pos for n in JOINT_NAMES):
            q_init = np.array([name_to_pos[n] for n in JOINT_NAMES])
            # Desembrulhar é obrigatório: J4 é contínua no Gazebo e
            # acumula voltas (medido: 423° e 257,7° para a mesma postura
            # física). Sem isto o clip nos limites destrói a leitura.
            q_init = (q_init + np.pi) % (2 * np.pi) - np.pi
            self._q_arm = np.clip(q_init, JOINT_LOWER, JOINT_UPPER)
            if self._resync_pedido:
                self._resync_pedido = False
                self._q_posture_target = None   # a rampa antiga não vale mais
                self._dq_cmd = np.zeros(5)
                rospy.logwarn('[gazebo_arm_bridge] q_arm reancorado em %s rad',
                              np.round(self._q_arm, 3))
            rospy.loginfo('[gazebo_arm_bridge] warm-start q=%s rad',
                          np.round(self._q_arm, 3))
            if self._goto_home:
                self._start_posture(self._q_home, 'home recolhido (boot)')

    def _cb_vel_cmd(self, msg):
        if len(msg.velocity) == 5:
            self._dq_cmd     = np.array(msg.velocity, dtype=float)
            self._t_last_cmd = rospy.Time.now()

    def _cb_tilt_critical(self, msg):
        if msg.data and not self._tilt_critical:
            rospy.logerr('[gazebo_arm_bridge] INCLINAÇÃO CRÍTICA — abortando '
                         'a tarefa e RECOLHENDO o braço para stow_home')
            self._q_posture_target = None   # cancela rampa da tarefa
            self._dq_cmd = np.zeros(5)
        elif not msg.data and self._tilt_critical:
            rospy.logwarn('[gazebo_arm_bridge] inclinação normalizada — '
                          'operação liberada')
        self._tilt_critical = msg.data

    def _cb_q_medido(self, msg):
        # Estimativa que não convergiu é pior que estimativa nenhuma:
        # ela tem cara de medida e leva o controle a agir na direção
        # errada. Descartar deixa _q_medido() devolver None, e o critério
        # cai para o modelo — degradar é melhor que agir sobre lixo.
        if not msg.ik_converged:
            return
        if len(msg.q_arm) == 5:
            self._q_med = np.array(msg.q_arm, dtype=float)
            self._t_q_med = rospy.Time.now()

    def _q_medido(self):
        """Postura medida, ou None se ausente/velha demais.

        Envelhecer importa: agir sobre uma medida parada é pior que não
        agir — foi assim que a ponte declarou chegada com 28° de erro.
        """
        if self._q_med is None or self._t_q_med is None:
            return None
        if (rospy.Time.now() - self._t_q_med).to_sec() > self._med_timeout:
            return None
        return self._q_med

    def _erro_por_junta(self):
        """Desvio por junta entre a postura ALVO e a MEDIDA, em rad."""
        q = self._q_medido()
        if q is None or self._q_posture_target is None:
            return None
        d = self._q_posture_target - q
        return (d + np.pi) % (2 * np.pi) - np.pi   # J4 é contínua no Gazebo

    def _erro_medido(self):
        """Maior desvio entre a postura ALVO e a MEDIDA, em rad."""
        d = self._erro_por_junta()
        return None if d is None else float(np.max(np.abs(d)))

    def _cb_posture_cmd(self, msg):
        if self._tilt_critical:
            rospy.logwarn_throttle(5.0, '[gazebo_arm_bridge] comando de postura '
                                        'ignorado: inclinação crítica')
            return
        if len(msg.position) == 5:
            q_target = np.clip(np.array(msg.position, dtype=float),
                               JOINT_LOWER, JOINT_UPPER)
            self._start_posture(q_target, 'comando externo')

    def _start_posture(self, q_target, why):
        self._q_posture_target = q_target
        self._correcao = np.zeros(5)   # cada postura recomeça do zero
        self._modelo_no_alvo = False
        self._t_no_alvo = None
        self._pub_posture_reached.publish(Bool(data=False))
        tgt = JointState()
        tgt.header.stamp = rospy.Time.now()
        tgt.name     = JOINT_NAMES
        tgt.position = q_target.tolist()
        self._pub_posture_target.publish(tgt)
        rospy.loginfo('[gazebo_arm_bridge] rampa de postura (%s): %s rad',
                      why, np.round(q_target, 3))

    # ------------------------------------------------------------------
    def spin(self):
        rate = rospy.Rate(self._rate_hz)
        dt   = 1.0 / self._rate_hz

        # Feedforward pré-calculado para q_spawn: evita queda do braço
        # antes do warm-start real. Com este comando, o equilíbrio do
        # PID coincide com q_spawn → nenhuma força de reação na base.
        q_pub_spawn = self._q_pub_for(self._q_spawn)

        while not rospy.is_shutdown():
            if self._q_arm is None:
                # Publica feedforward de spawn para segurar o braço
                # enquanto aguarda /joint_states.
                for i, pub in enumerate(self._pubs):
                    pub.publish(Float64(data=float(q_pub_spawn[i])))
                rate.sleep()
                continue

            if self._tilt_critical:
                # RECOLHER, não congelar (corrigido em 2026-08-24).
                #
                # A primeira versão congelava o braço (dq = 0). Errado
                # por física e por consequência:
                #
                # Física — quem derruba o robô é o momento do braço de
                # 28 kg estendido. Congelar mantém exatamente o braço
                # de alavanca que causou o tombamento; recolher para
                # junto do corpo reduz o momento e é sempre a direção
                # segura, mesmo com o chassi já em movimento.
                #
                # Consequência — o congelamento criou um DEADLOCK real,
                # observado numa bateria inteira (8/8 abortadas em
                # 2026-08-24): robô tomba com o braço estendido → tilt
                # crítico congela o braço → o reset põe o chassi em pé,
                # mas o braço continua estendido → tomba de novo em
                # menos de 2 s → nunca cumpre o tempo de nivelamento
                # para limpar o estado → braço segue congelado. O
                # sistema não conseguia se recuperar sozinho, e a
                # postura que impedia a recuperação era justamente a
                # que a trava protegia.
                #
                # Velocidade reduzida: é manobra de segurança com o
                # chassi instável, não movimento de tarefa.
                err = self._q_home - self._q_arm
                if np.max(np.abs(err)) < self._reached_tol:
                    dq = np.zeros(5)
                else:
                    v = self._ramp_vel * self._tilt_retract_scale
                    dq = np.clip(err / dt, -v, v)
                    rospy.logwarn_throttle(
                        2.0, '[gazebo_arm_bridge] recolhendo por inclinação '
                             'crítica (falta %.2f rad)', np.max(np.abs(err)))
            elif self._q_posture_target is not None:
                # Rampa de postura: move q_arm em direção ao alvo a
                # ~ramp_velocity, ignorando arm_vel_cmd (movimento
                # deliberado em espaço de juntas, ver docstring).
                #
                # "ATINGIDA" É JULGADO PELA MEDIDA, NÃO PELO MODELO
                # (2026-08-24). Antes o critério era só
                # |alvo − q_arm| < tol, com q_arm sendo a integração
                # interna desta ponte. Como ela é malha aberta, o modelo
                # podia estar no alvo com o braço REAL longe — e, por
                # achar que chegou, a ponte parava de acionar e o erro
                # virava permanente.
                #
                # Medido na missão: a ponte anunciou
                # "postura atingida: [-0.059, -0.572, 0.199, 1.89, 0.0]"
                # enquanto a missão media faltar [0.1, -9.6, -17.9,
                # -28.1, 0.0] graus. A ponta ficava cravada e as 5
                # iterações do pre_engage davam erro idêntico
                # (0,3565 m), porque cada nova postura comandada era
                # comparada com o modelo, que já dizia estar no alvo.
                #
                # A medida vem do state_estimator, não de /joint_states:
                # é a fonte que EXISTE no hardware real (IK da pose da
                # T265), onde não há encoder. Quando ela falta, cai no
                # critério antigo — degradar é melhor que travar.
                err = self._q_posture_target - self._q_arm
                err_med = self._erro_medido()

                # A MEDIDA CORRIGE, MAS NÃO BLOQUEIA (revisto em 2026-08-25).
                #
                # A versão anterior exigia a postura medida dentro da
                # tolerância para declarar chegada. Instrumentando por
                # junta, o erro persistente é do J2 — o ombro, a junta
                # mais carregada — entre 9° e 12°, e ele não cede nem com
                # a correção saturada. É erro estacionário do PID sob
                # carga, não divergência: o braço faz isso o tempo todo.
                #
                # Exigir que ele suma impõe um requisito que a missão foi
                # projetada para não precisar — o _reach_by_iterative_ik
                # mede onde a PONTA foi parar e re-mira, justamente para
                # absorver esse resíduo. Bloquear ali travou o DEPLOY e o
                # recolhimento e derrubou a bateria de 5/8 para 3/8.
                #
                # Agora a medida serve para (a) dar tempo à correção
                # atuar e (b) deixar o resíduo registrado. Passado esse
                # tempo, a postura fecha com o resíduo no log.
                # O "modelo chegou" é LATCHED, e isso não é detalhe.
                #
                # A correção move q_arm ALÉM do alvo de propósito, para
                # vencer o erro estacionário. Isso faz |alvo − q_arm|
                # crescer de novo, e sem o latch o código caía de volta no
                # ramo da rampa, que desfazia a correção — os dois ramos
                # se anulavam e o modelo nunca convergia. Sintoma:
                # "postura não atingida em 30s" em quase toda execução
                # (bateria de 2026-08-25, 2/8). Uma vez que o modelo
                # encostou no alvo, ele não volta a "não ter chegado".
                if not self._modelo_no_alvo and np.max(np.abs(err)) < self._reached_tol:
                    self._modelo_no_alvo = True
                    self._t_no_alvo = rospy.Time.now()

                tempo_corrigindo = (
                    (rospy.Time.now() - self._t_no_alvo).to_sec()
                    if self._t_no_alvo is not None else 0.0)

                chegou = (self._modelo_no_alvo
                          and (err_med is None
                               or err_med < self._reached_tol_medido
                               or tempo_corrigindo > self._corr_timeout))

                if chegou:
                    self._q_arm = self._q_posture_target.copy()
                    self._q_posture_target = None
                    self._correcao = np.zeros(5)
                    self._t_no_alvo = None
                    self._modelo_no_alvo = False
                    if err_med is not None and err_med >= self._reached_tol_medido:
                        d = self._erro_por_junta()
                        rospy.logwarn('[gazebo_arm_bridge] postura fechada com '
                                      'resíduo de %.3f rad (%s) — erro '
                                      'estacionário sob carga, a missão '
                                      'compensa medindo a ponta', err_med,
                                      np.round(np.degrees(d), 1)
                                      if d is not None else '?')
                    self._pub_posture_reached.publish(Bool(data=True))
                    rospy.loginfo('[gazebo_arm_bridge] postura atingida: %s rad '
                                  '(erro medido %s)', np.round(self._q_arm, 3),
                                  '—' if err_med is None else '%.3f rad' % err_med)
                    dq = np.zeros(5)
                elif self._modelo_no_alvo and err_med is not None:
                    # Modelo chegou, braço real não. Empurra o comando ALÉM
                    # do alvo, devagar e com teto: é ação integral contra o
                    # erro estacionário dos controladores de posição sob
                    # carga. Sem teto isso viraria fuga descontrolada se a
                    # medida estiver ruim.
                    alvo_med = self._q_posture_target - self._q_medido()
                    passo = np.clip(alvo_med, -self._corr_vel * dt,
                                    self._corr_vel * dt)
                    novo = np.clip(self._correcao + passo,
                                   -self._corr_max, self._corr_max)
                    saturou = np.allclose(novo, self._correcao, atol=1e-6)
                    self._correcao = novo
                    dq = passo / dt
                    if saturou:
                        self._q_posture_target = None
                        self._correcao = np.zeros(5)
                        self._pub_posture_reached.publish(Bool(data=False))
                        rospy.logerr('[gazebo_arm_bridge] postura NÃO atingida: '
                                     'correção saturou em %.2f rad com erro '
                                     'medido de %.3f rad. O braço real não '
                                     'segue o comando.', self._corr_max, err_med)
                        dq = np.zeros(5)
                    else:
                        d = self._erro_por_junta()
                        pior = int(np.argmax(np.abs(d)))
                        rospy.logwarn_throttle(
                            2.0, '[gazebo_arm_bridge] modelo no alvo mas braço '
                                 'real a %.3f rad (pior: %s) — corrigindo '
                                 '(%.2f rad de %.2f) | por junta %s',
                            err_med, JOINT_NAMES[pior],
                            np.max(np.abs(self._correcao)), self._corr_max,
                            np.round(np.degrees(d), 1))
                else:
                    dq = np.clip(err / dt, -self._ramp_vel, self._ramp_vel)
            elif self._t_last_cmd is not None:
                age = (rospy.Time.now() - self._t_last_cmd).to_sec()
                dq = self._dq_cmd if age < _VEL_TIMEOUT else np.zeros(5)
            else:
                dq = np.zeros(5)

            self._q_arm = np.clip(self._q_arm + dq * dt,
                                  JOINT_LOWER, JOINT_UPPER)

            q_pub = self._q_pub_for(self._q_arm)
            for i, pub in enumerate(self._pubs):
                pub.publish(Float64(data=float(q_pub[i])))

            # Instrumentação (2026-08-13): rastrear se a ponte está de
            # fato integrando arm_vel_cmd. Sintoma investigado: o
            # controlador comandava 0,1 rad/s por 60 s e o braço não
            # saía do lugar — precisa distinguir "não recebeu",
            # "recebeu mas está em modo postura" e "integrou mas o PID
            # não seguiu".
            age = ((rospy.Time.now() - self._t_last_cmd).to_sec()
                   if self._t_last_cmd else -1.0)
            rospy.loginfo_throttle(2.0,
                '[bridge] postura=%s | dq_cmd=%s idade=%.2fs | dq_aplicado=%s | '
                'q_arm=%s',
                'ATIVA' if self._q_posture_target is not None else 'nao',
                np.round(self._dq_cmd, 3), age, np.round(dq, 3),
                np.round(self._q_arm, 3))

            rate.sleep()


if __name__ == '__main__':
    try:
        GazeboArmBridge().spin()
    except rospy.ROSInterruptException:
        pass
