#!/usr/bin/env python3
"""mission_hud — painel ao vivo da missão da chave, no terminal.

POR QUE EXISTE. Pedido do Marco em 2026-08-27: "a cada etapa da máquina
de estados, coloque a informação na simulação". O motivo é concreto e
está no diário do dia: eu diagnostico lendo log depois do fato, ele
diagnostica olhando a tela — e naquele dia ele acertou três vezes onde
eu errei (a ferramenta colidindo com a chave, o robô andando depois do
fim da missão, a trajetória passando por baixo do mecanismo).

Este nó NÃO comanda nada e a missão não depende dele.

REESCRITO em 2026-08-31, depois de o Marco relatar que "a posição de
agora não está marcando no painel". Dois defeitos:

  · na PAUSA a missão publicava estado='PAUSA', que não é um dos estados
    da trilha — nada ficava marcado, e o estado real se perdia. Pausa
    virou uma CONDIÇÃO (campo `pausado`), não um estado;
  · o quadro não fechava: eu media a largura das linhas COM os códigos
    de cor dentro, então cada cor comia caracteres da borda.

REVISTO em 2026-09-03, a pedido do Marco: "o painel precisa ser
revisado, adequado quanto as fases, e precisa ser sempre resetado
quando começar uma nova missão". Três coisas:

  · a trilha de fases ganhou 'destrava' e 'libera' (o gatilho da chave)
    e com nove nomes não cabia mais em uma linha: a borda direita
    quebrava. Agora a trilha QUEBRA EM LINHAS dentro do quadro;
  · o zeramento por execução nova existia, mas trocava o deque de
    eventos por uma lista sem limite — a partir da segunda missão o
    quadro só crescia. E nada dizia quando a missão tinha acabado: o
    último estado ficava lá como se fosse o atual. Agora o cabeçalho
    mostra há quanto tempo não há mensagem, e a trilha apaga;
  · GATILHO: curso da lingueta e ângulo da lâmina, lidos do Gazebo —
    só quando o serviço existe (na bancada não existe, e o campo some).

Uso:  sim_stack.sh watch     (ou rosrun b166er_whole_body_control mission_hud.py)
"""

import collections
import math
import os
import re
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

ESTADOS = ['STOW_INIT', 'SEARCH', 'APPROACH', 'REFINE', 'DEPLOY',
           'MANIPULATE', 'RETRACT', 'RETURN']
# Fases cartesianas do MANIPULATE (PHASE_ORDER em chave_mission.py).
FASES = ['orienta', 'aproxima_lateral', 'atravessa', 'captura',
         'destrava', 'libera', 'arco1', 'arco2', 'desengata']
# NOMES EM INGLÊS NO PAINEL, como os estados (Marco, 2026-09-03: "se
# estamos usando as palavras em ingles para os estados porque não usar
# também para as fases do manipulate?"). Só a exibição: o YAML, o
# PHASE_ORDER e os logs continuam com os nomes em português — renomear
# tudo é outra mudança, maior, se ele quiser.
FASES_EN = {'orienta': 'ORIENT', 'aproxima_lateral': 'SIDE_APPROACH',
            'atravessa': 'INSERT', 'captura': 'CAPTURE',
            'destrava': 'UNLATCH', 'libera': 'RELEASE',
            'arco1': 'ARC1', 'arco2': 'ARC2', 'desengata': 'DISENGAGE',
            'saida_eixo': 'EXIT_AXIS'}
# Abreviações da trilha: só a fase ATUAL sai por extenso.
FASES_ABREV = {'orienta': 'ORIENT', 'aproxima_lateral': 'SIDE_APP',
               'atravessa': 'INSRT', 'captura': 'CAPTR', 'destrava': 'UNLATCH',
               'libera': 'RELEASE', 'arco1': 'ARC1', 'arco2': 'ARC2',
               'desengata': 'DISENG'}

V, R, A, C, N, F = ('\033[32m', '\033[31m', '\033[33m', '\033[90m',
                    '\033[1m', '\033[0m')
_ANSI = re.compile(r'\033\[[0-9;]*m')
LARG = 92


def vis(t):
    """Largura VISÍVEL: sem os códigos de cor. Medir com eles dentro era
    o que impedia o quadro de fechar."""
    return len(_ANSI.sub('', t))


class Hud(object):

    def __init__(self):
        rospy.init_node('mission_hud', anonymous=True, disable_signals=True)
        self.st = {}
        self.clearance = self.close = self.tilt = None
        self.wall = self.q = None
        self.eventos = collections.deque(maxlen=6)
        self.t0 = time.time()
        self.t_status = None          # última mensagem da missão
        self.tty = sys.stdout.isatty()
        self._ult = None
        self.gatilho = None           # (lingueta_mm, lamina_deg) ou None
        self._gj = None
        try:
            rospy.wait_for_service('/gazebo/get_joint_properties', timeout=1.0)
            from gazebo_msgs.srv import GetJointProperties
            self._gj = rospy.ServiceProxy('/gazebo/get_joint_properties',
                                          GetJointProperties)
        except (rospy.ROSException, ImportError):
            self._gj = None

        rospy.Subscriber('/b166er/mission_status', String, self._cb_status)
        rospy.Subscriber('/b166er/front_clearance', Float64,
                         lambda m: setattr(self, 'clearance', m.data))
        rospy.Subscriber('/b166er/obstacle_close', Bool,
                         lambda m: setattr(self, 'close', m.data))
        rospy.Subscriber('/b166er/tilt_critical', Bool,
                         lambda m: setattr(self, 'tilt', m.data))
        rospy.Subscriber('/b166er/wall_pose', PoseStamped, self._cb_wall)
        rospy.Subscriber('/joint_states', JointState, self._cb_js)

    def _cb_status(self, msg):
        novo = {}
        for parte in msg.data.split('|'):
            if '=' in parte:
                k, v = parte.split('=', 1)
                novo[k] = v
        # EXECUÇÃO NOVA ZERA O QUADRO.
        #
        # Sem isto o painel fundia tudo para sempre: sobrevivendo a
        # vários lançamentos, mostrava campos de missões mortas ao lado
        # dos da viva (fase=atravessa da execução anterior enquanto a
        # atual estava no SEARCH), porque a missão nova nunca publica
        # aqueles nomes e nada os apagava. Parecia painel congelado.
        if novo.get('run') and novo['run'] != self.st.get('run'):
            self.st = {}
            self.eventos.clear()        # era `= []`: virava lista sem limite
            self.t0 = time.time()
        self.t_status = time.time()
        ant_e, ant_f = self.st.get('estado'), self.st.get('fase')
        self.st.update(novo)
        if novo.get('estado') and novo['estado'] != ant_e:
            self.eventos.append((time.time() - self.t0,
                                 'estado → %s' % novo['estado']))
        if novo.get('fase') and novo['fase'] != ant_f:
            self.eventos.append((time.time() - self.t0,
                                 'fase → %s' % novo['fase']))
        if novo.get('pausado') == '1':
            self.eventos.append((time.time() - self.t0,
                                 'PAUSA em %s' % novo.get('etapa_pausa', '?')))

    def _cb_wall(self, m):
        p = m.pose.position
        self.wall = (p.x, p.y, p.z)

    def _cb_js(self, m):
        d = dict(zip(m.name, m.position))
        nn = ['J1', 'J2', 'J3', 'J4', 'J5']
        if all(x in d for x in nn):
            self.q = [math.degrees(d[x]) for x in nn]

    # ── desenho ──────────────────────────────────────────────────────
    def _trilha(self, atual, seq, apagada=False, abrev=None, nomes=None):
        """✓ já passou · ▶ atual · · pendente. Devolve LINHAS que cabem
        no quadro, todas cinza se a missão já acabou.

        Com `abrev`, os nomes que NÃO são o atual saem abreviados e só
        o atual aparece por extenso — ideia do Marco (2026-09-03) para a
        trilha de fases caber em uma linha depois que ganhou nove nomes.
        Um nome fora da sequência (saida_eixo, na saída) é acrescentado
        no fim, para não sumir."""
        if atual in seq:
            i_at = seq.index(atual)
        else:
            i_at = -1
        nomes = nomes or {}
        def rot(i, nome):
            if abrev and i != i_at:
                return abrev.get(nome, nome)
            return nomes.get(nome, nome)
        itens = []
        for i, nome in enumerate(seq):
            if apagada:
                itens.append(C + ('✓' if 0 <= i <= i_at else '·')
                             + rot(i, nome) + F)
            elif i_at >= 0 and i < i_at:
                itens.append(V + '✓' + rot(i, nome) + F)
            elif i == i_at:
                itens.append(N + A + '▶' + rot(i, nome) + F)
            else:
                itens.append(C + '·' + rot(i, nome) + F)
        if i_at < 0 and atual and atual != '—':
            itens.append((C if apagada else N + A) + '▶'
                         + nomes.get(atual, atual) + F)
        larg = LARG - 12
        linhas, atual_l = [], ''
        for it in itens:
            cand = (atual_l + ' ' + it) if atual_l else it
            if vis(cand) > larg and atual_l:
                linhas.append(atual_l)
                atual_l = it
            else:
                atual_l = cand
        if atual_l:
            linhas.append(atual_l)
        return linhas

    def _campo_trilha(self, rot, linhas):
        for i, l in enumerate(linhas):
            self._campo(rot if i == 0 else '', l)

    def _lin(self, txt=''):
        pad = max(0, LARG - vis(txt))
        print('│ ' + txt + ' ' * pad + ' │')

    def _sep(self, ch='─'):
        print('├' + ch * (LARG + 2) + '┤')

    def _campo(self, rot, txt):
        self._lin('%-11s %s' % (rot, txt))

    def desenha(self):
        st = self.st
        if self.tty:
            os.system('clear')
        else:
            ch = (st.get('estado'), st.get('fase'), st.get('it'),
                  st.get('pausado'))
            if ch == self._ult:
                return
            self._ult = ch

        pausado = st.get('pausado') == '1'
        # MISSÃO ENCERRADA OU AUSENTE: sem mensagem há mais de 20 s o
        # quadro deixa de parecer "ao vivo" — o último estado não é o
        # atual, é o último. Zera de verdade quando a próxima missão
        # publicar o carimbo 'run' novo.
        silencio = (time.time() - self.t_status) if self.t_status else None
        encerrada = silencio is None or silencio > 20.0
        print('┌' + '─' * (LARG + 2) + '┐')
        cab = N + 'b166er · missão da chave seccionadora' + F
        if pausado and not encerrada:
            cab += '   ' + N + A + '⏸ PAUSADA' + F
        if silencio is None:
            cab += '   ' + C + 'sem missão (nenhuma mensagem ainda)' + F
        elif encerrada:
            cab += '   ' + C + ('sem missão há %ds — abaixo, a ÚLTIMA'
                                % silencio) + F
        self._lin(cab)
        self._sep()

        self._campo_trilha('ESTADO', self._trilha(st.get('estado', '—'),
                                                  ESTADOS, encerrada))
        # 'FASE MANIP.': a trilha é só do MANIPULATE, e o rótulo diz isso
        # (sugestão do Marco, 2026-09-03). Onze caracteres, o máximo do
        # campo.
        self._campo_trilha('FASE MANIP.', self._trilha(st.get('fase', '—'),
                                                       FASES, encerrada,
                                                       abrev=FASES_ABREV,
                                                       nomes=FASES_EN))
        if self.gatilho is not None:
            l_mm, b_deg = self.gatilho
            cor_l = V if l_mm >= 12.0 else (A if l_mm > 1.0 else C)
            cor_b = V if b_deg >= 25.0 else (A if b_deg > 1.0 else C)
            self._campo('GATILHO', 'lingueta %s%5.1f mm%s   lâmina %s%5.1f°%s'
                        '   %s(Gazebo)%s'
                        % (cor_l, l_mm, F, cor_b, b_deg, F, C, F))

        if pausado:
            self._sep()
            ok = st.get('pausa_ok') == '1'
            self._lin('%s⏸  parada após: %s%s   [%s]'
                      % (N + A, st.get('etapa_pausa', '?'), F,
                         (V + 'ok' + F) if ok else (R + 'FALHOU' + F)))
            if st.get('detalhe'):
                self._lin('   ' + C + st['detalhe'] + F)
            self._lin('   ' + C + 'liberar:  scripts/continua.sh' + F)

        self._sep()
        if st.get('erro_m') is not None:
            e = float(st['erro_m']) * 1000
            tol = float(st.get('tol_m', 0.02)) * 1000
            cor = V if e < tol else A
            self._campo('ERRO', '%s%6.1f mm%s  (tol %.0f mm · iteração %s)'
                        % (cor, e, F, tol, st.get('it', '?')))
        if st.get('e_prof_mm') is not None:
            def cor_eixo(v, lim):
                return (V if abs(float(v)) < lim else R)
            self._campo('POR EIXO',
                        'eixo %s%+6.1f%s   prof %s%+6.1f%s   alt %s%+6.1f%s  mm'
                        % (C, float(st.get('e_eixo_mm', 0)), F,
                           cor_eixo(st['e_prof_mm'], 6),
                           float(st['e_prof_mm']), F,
                           cor_eixo(st.get('e_alt_mm', 0), 10),
                           float(st.get('e_alt_mm', 0)), F))
        if st.get('ang_deg') is not None:
            self._campo('DEGRAU', '%s %.1f°   [IK: %s]'
                        % (st.get('ang_rotulo', 'ang'),
                           float(st['ang_deg']), st.get('ik', 'n/d')))
        if st.get('q_pedido'):
            self._campo('IK PEDIU', st['q_pedido'])
        if st.get('faltou'):
            self._campo('FALTOU', st['faltou'] + '  graus')
        if self.q:
            self._campo('JUNTAS', '[' + ' '.join('%+7.1f' % v for v in self.q)
                        + ' ]')

        self._sep()
        if self.wall:
            self._campo('PAREDE', '(%.3f, %.3f, %.3f)' % self.wall)
        if self.clearance is not None:
            cor = R if self.close else V
            self._campo('LASER', '%s%.2f m%s do centro%s'
                        % (cor, self.clearance, F,
                           '   ⚠ OBSTÁCULO' if self.close else ''))
        if self.tilt:
            self._campo('INCLINAÇÃO', R + N + 'CRÍTICA' + F)

        self._sep()
        for t, ev in self.eventos:
            self._lin('%s%7.1fs  %s%s' % (C, t, ev, F))
        print('└' + '─' * (LARG + 2) + '┘')

    def _le_gatilho(self):
        if self._gj is None:
            return
        try:
            l = self._gj('chave_lingueta_joint')
            b = self._gj('chave_blade_joint')
            if l.success and b.success and l.position and b.position:
                # + 0.0 mata o "-0.0" que a mola deixa no repouso
                self.gatilho = (round(1000.0 * l.position[0], 1) + 0.0,
                                round(math.degrees(b.position[0]), 1) + 0.0)
            else:
                self.gatilho = None
        except rospy.ServiceException:
            self.gatilho = None

    def spin(self):
        rate = rospy.Rate(4.0)
        n = 0
        while not rospy.is_shutdown():
            try:
                if n % 2 == 0:
                    self._le_gatilho()      # 2 Hz basta, e o serviço é leve
                n += 1
                self.desenha()
            except Exception as exc:            # noqa: BLE001
                print('HUD: %s' % exc)
            rate.sleep()


if __name__ == '__main__':
    try:
        Hud().spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
