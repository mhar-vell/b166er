#!/usr/bin/env python3
"""mission_hud — painel ao vivo da missão da chave, no terminal.

POR QUE EXISTE. Pedido do Marco em 2026-08-27: "a cada etapa da máquina
de estados, coloque a informação na simulação". O motivo é concreto e
está registrado no diário do dia: eu diagnostico lendo log depois do
fato, ele diagnostica olhando a tela — e naquele dia ele acertou três
vezes onde eu errei (a ferramenta colidindo com a chave, o robô andando
depois do fim da missão, a trajetória passando por baixo do mecanismo).
Um painel ao vivo põe os dois olhando a mesma coisa.

O que ele mostra vem de tópicos que já existiam, mais o
/b166er/mission_status que a missão publica. Este nó NÃO comanda nada e
a missão não depende dele: se o HUD morrer, a missão segue.

Uso:
    rosrun b166er_whole_body_control mission_hud.py
ou, com o stack no ar:
    sim_stack.sh watch
"""

import collections
import math
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from b166er_whole_body_control.msg import RobotState

# Sequência da máquina de estados, para desenhar o progresso.
ESTADOS = ['STOW_INIT', 'SEARCH', 'APPROACH', 'REFINE', 'DEPLOY',
           'MANIPULATE', 'RETRACT', 'RETURN']
FASES = ['orienta', 'aproxima_lateral', 'atravessa', 'captura',
         'arco1', 'arco2', 'desengata']

VERDE, VERM, AMAR, CINZA, NEGRITO, FIM = (
    '\033[32m', '\033[31m', '\033[33m', '\033[90m', '\033[1m', '\033[0m')


class Hud(object):

    def __init__(self):
        rospy.init_node('mission_hud', anonymous=True, disable_signals=True)
        self.st = {}
        self.clearance = None
        self.close = None
        self.tilt = None
        self.lamina = None
        self.wall = None
        self.q = None
        self.eventos = collections.deque(maxlen=6)
        self.t0 = time.time()
        self.tty = sys.stdout.isatty()
        self._ult_chave = None

        rospy.Subscriber('/b166er/mission_status', String, self._cb_status)
        rospy.Subscriber('/b166er/front_clearance', Float64,
                         lambda m: setattr(self, 'clearance', m.data))
        rospy.Subscriber('/b166er/obstacle_close', Bool,
                         lambda m: setattr(self, 'close', m.data))
        rospy.Subscriber('/b166er/tilt_critical', Bool,
                         lambda m: setattr(self, 'tilt', m.data))
        rospy.Subscriber('/b166er/wall_pose', PoseStamped, self._cb_wall)
        rospy.Subscriber('/joint_states', JointState, self._cb_js)

    # ── entrada ──────────────────────────────────────────────────────
    def _cb_status(self, msg):
        novo = {}
        for parte in msg.data.split('|'):
            if '=' in parte:
                k, v = parte.split('=', 1)
                novo[k] = v
        antes = self.st.get('estado')
        self.st.update(novo)
        marca = novo.get('estado')
        fase = novo.get('fase')
        if marca and marca != antes:
            self.eventos.append((time.time() - self.t0,
                                 'estado -> %s' % marca))
        elif fase and fase != self.st.get('_ult_fase'):
            self.eventos.append((time.time() - self.t0, 'fase -> %s' % fase))
        if fase:
            self.st['_ult_fase'] = fase

    def _cb_wall(self, m):
        p = m.pose.position
        self.wall = (p.x, p.y, p.z)

    def _cb_js(self, m):
        d = dict(zip(m.name, m.position))
        nomes = ['J1', 'J2', 'J3', 'J4', 'J5']
        if all(n in d for n in nomes):
            self.q = [math.degrees(d[n]) for n in nomes]
        if 'chave_blade_joint' in d:
            self.lamina = math.degrees(d['chave_blade_joint'])

    # ── desenho ──────────────────────────────────────────────────────
    def _trilha(self, atual, seq):
        saida = []
        vist = atual in seq
        passou = True
        for nome in seq:
            if nome == atual:
                saida.append(NEGRITO + VERDE + nome + FIM)
                passou = False
            elif passou and vist:
                saida.append(CINZA + nome + FIM)
            else:
                saida.append(CINZA + nome + FIM)
        return ' › '.join(saida)

    def desenha(self):
        st = self.st
        # Painel de tela cheia só faz sentido numa TTY. Redirecionado
        # para arquivo ou pipe, limpar a tela produz lixo de escape e
        # apaga o histórico — então lá ele vira log incremental, e só
        # imprime quando algo muda.
        if self.tty:
            os.system('clear')
        else:
            chave = (st.get('estado'), st.get('fase'), st.get('it'))
            if chave == self._ult_chave:
                return
            self._ult_chave = chave
        larg = 78
        print('┌' + '─' * larg + '┐')
        print('│ ' + NEGRITO + 'b166er · missão da chave seccionadora' + FIM
              + ' ' * (larg - 38) + '│')
        print('├' + '─' * larg + '┤')

        est = st.get('estado', '—')
        print('│ ESTADO   ' + self._trilha(est, ESTADOS))
        if est == 'MANIPULATE' or st.get('fase'):
            print('│ FASE     ' + self._trilha(st.get('fase', '—'), FASES))
        print('├' + '─' * larg + '┤')

        def linha(rot, txt):
            print('│ %-9s %s' % (rot, txt))

        if st.get('erro_m') is not None:
            e = float(st['erro_m']) * 1000
            tol = float(st.get('tol_m', 0.02)) * 1000
            cor = VERDE if e < tol else AMAR
            linha('ERRO', '%s%6.1f mm%s   (tolerância %.0f mm, iteração %s)'
                  % (cor, e, FIM, tol, st.get('it', '?')))
        if st.get('ang_deg') is not None:
            linha('DEGRAU', '%s %.1f°   [IK: %s]'
                  % (st.get('ang_rotulo', 'ang'), float(st['ang_deg']),
                     st.get('ik', 'n/d')))
        if st.get('q_pedido'):
            linha('IK PEDIU', st['q_pedido'])
        if st.get('faltou'):
            linha('FALTOU', st['faltou'] + '  graus')
        if self.q:
            linha('JUNTAS', '[' + ' '.join('%6.1f' % v for v in self.q) + ' ]')

        print('├' + '─' * larg + '┤')
        if self.wall:
            linha('PAREDE', '(%.3f, %.3f, %.3f)' % self.wall)
        if self.clearance is not None:
            cor = VERM if self.close else VERDE
            linha('LASER', '%s%.2f m%s do centro da base%s'
                  % (cor, self.clearance, FIM,
                     '   OBSTÁCULO PERTO' if self.close else ''))
        if self.lamina is not None:
            cor = VERDE if self.lamina >= 25.0 else CINZA
            linha('LÂMINA', '%s%.1f°%s   (missão exige 25°)'
                  % (cor, self.lamina, FIM))
        if self.tilt:
            linha('INCLINAÇÃO', VERM + 'CRÍTICA' + FIM)

        print('├' + '─' * larg + '┤')
        for t, ev in self.eventos:
            print('│ %s%7.1fs  %s%s' % (CINZA, t, ev, FIM))
        print('└' + '─' * larg + '┘')

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
