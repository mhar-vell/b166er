#!/usr/bin/env python3
"""
plot_shaking.py — plota q(t) e amplitude de oscilação por junta a partir de um rosbag.

Uso:
  python3 scripts/plot_shaking.py <path/ao/arquivo.bag>

Grava o bag durante o teste:
  rosbag record -O shaking_test.bag -d 30 /joint_states /b166er/robot_state
"""

import sys
import numpy as np
import matplotlib.pyplot as plt

JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5']


def load_joint_states(bag_path):
    try:
        import rosbag
    except ImportError:
        print('[ERRO] rosbag não encontrado. Ative o ambiente ROS antes de rodar.')
        sys.exit(1)

    times = []
    positions = {n: [] for n in JOINT_NAMES}
    t0 = None

    with rosbag.Bag(bag_path, 'r') as bag:
        for _, msg, t in bag.read_messages(topics=['/joint_states']):
            if t0 is None:
                t0 = t.to_sec()
            name_to_pos = dict(zip(msg.name, msg.position))
            if all(n in name_to_pos for n in JOINT_NAMES):
                times.append(t.to_sec() - t0)
                for n in JOINT_NAMES:
                    positions[n].append(name_to_pos[n])

    return np.array(times), {n: np.array(positions[n]) for n in JOINT_NAMES}


def analyse(times, positions):
    print(f'\n{"Junta":>6}  {"Média (°)":>10}  {"Std (°)":>8}  {"Pico-pico (°)":>14}')
    print('-' * 46)
    for n in JOINT_NAMES:
        q = np.degrees(positions[n])
        print(f'{n:>6}  {np.mean(q):>10.2f}  {np.std(q):>8.3f}  {np.ptp(q):>14.3f}')
    print()


def plot(times, positions, out_path):
    fig, axes = plt.subplots(5, 1, figsize=(13, 10), sharex=True)
    for n, ax in zip(JOINT_NAMES, axes):
        q_deg = np.degrees(positions[n])
        ax.plot(times, q_deg, linewidth=0.8)
        ax.axhline(np.mean(q_deg), color='r', linestyle='--', linewidth=0.6, alpha=0.7)
        ax.set_ylabel(f'{n} (°)')
        ax.grid(True, alpha=0.3)
        ax.set_title(
            f'{n}  média={np.mean(q_deg):.2f}°  std={np.std(q_deg):.3f}°  '
            f'pico-pico={np.ptp(q_deg):.3f}°',
            fontsize=9
        )
    axes[-1].set_xlabel('Tempo (s)')
    fig.suptitle('Oscilação das juntas — q(t)', fontsize=12)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f'Gráfico salvo em: {out_path}')
    plt.show()


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    bag_path = sys.argv[1]
    print(f'Lendo {bag_path} ...')
    times, positions = load_joint_states(bag_path)

    if len(times) == 0:
        print('[ERRO] Nenhuma mensagem encontrada em /joint_states')
        sys.exit(1)

    print(f'{len(times)} amostras, duração {times[-1]:.1f} s')
    analyse(times, positions)

    out_path = bag_path.replace('.bag', '_shaking.png')
    plot(times, positions, out_path)


if __name__ == '__main__':
    main()
