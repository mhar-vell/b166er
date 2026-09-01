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
FASES = ['orienta', 'aproxima_lateral', 'atravessa', 'captura',
         'arco1', 'arco2', 'desengata']

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
        self.lamina = self.wall = self.q = None
        self.eventos = collections.deque(maxlen=5)
        self.t0 = time.time()
        self.tty = sys.stdout.isatty()
        self._ult = None

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
            self.eventos = []
            self.t0 = time.time()
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
    def _trilha(self, atual, seq):
        """✓ já passou · ▶ atual · · pendente."""
        if atual in seq:
            i_at = seq.index(atual)
        else:
            i_at = -1
        out = []
        for i, nome in enumerate(seq):
            if i_at >= 0 and i < i_at:
                out.append(V + '✓' + nome + F)
            elif i == i_at:
                out.append(N + A + '▶' + nome + F)
            else:
                out.append(C + '·' + nome + F)
        return ' '.join(out)

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
        print('┌' + '─' * (LARG + 2) + '┐')
        cab = N + 'b166er · missão da chave seccionadora' + F
        if pausado:
            cab += '   ' + N + A + '⏸ PAUSADA' + F
        self._lin(cab)
        self._sep()

        self._campo('ESTADO', self._trilha(st.get('estado', '—'), ESTADOS))
        self._campo('FASE', self._trilha(st.get('fase', '—'), FASES))

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

    def spin(self):
        rate = rospy.Rate(4.0)
        while not rospy.is_shutdown():
            try:
                self.desenha()
            except Exception as exc:            # noqa: BLE001
                print('HUD: %s' % exc)
            rate.sleep()


if __name__ == '__main__':
    try:
        Hud().spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
