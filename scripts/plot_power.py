#!/usr/bin/env python3
"""
Plots power use from recorded bags so runs can be compared.

Reads /power_monitor (5 Hz). Excavation, depositing and navigation are shaded when a single bag
is plotted, and the energy used in each is printed.

Usage: plot_power.py BAG [BAG ...] [--out FILE] [--show]
Re-runs itself with the workspace sourced if lunabot_msgs is not found.
"""

import argparse
import os
import sys
from pathlib import Path

import matplotlib
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Action status topics and the phase they mark
PHASES = {
    '/auto_excavation_action/_action/status': ('Excavating', '#8d6e63'),
    '/auto_depositing_action/_action/status': ('Depositing', '#7e57c2'),
    '/navigate_to_pose/_action/status': ('Navigating', '#42a5f5'),
}
EXECUTING = 2


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    # Empty storage id lets rosbag2 detect mcap or sqlite3
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if '/power_monitor' not in types:
        return np.empty((0, 3)), []
    wanted = ['/power_monitor'] + [t for t in PHASES if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))

    msg_types = {}
    for topic in wanted:
        try:
            msg_types[topic] = get_message(types[topic])
        except (AttributeError, ModuleNotFoundError, ValueError):
            print(f'{path.name}: cannot load {types[topic]}, source the workspace')

    t0 = None
    samples = []  # (time, voltage, power)
    status = {topic: {} for topic in PHASES}  # goal id -> [start, end]
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t0 = stamp if t0 is None else t0
        t = (stamp - t0) / 1e9
        if topic not in msg_types:
            continue
        msg = deserialize_message(data, msg_types[topic])
        if topic == '/power_monitor':
            samples.append((t, msg.voltage, msg.power))
        else:
            for st in msg.status_list:
                goal = bytes(st.goal_info.goal_id.uuid)
                span = status[topic].setdefault(goal, [None, None])
                if st.status == EXECUTING and span[0] is None:
                    span[0] = t
                elif st.status > EXECUTING and span[0] is not None and span[1] is None:
                    span[1] = t

    # A goal still running at the end of the bag runs to the last sample
    phases = []
    for topic, goals in status.items():
        name, color = PHASES[topic]
        for start, end in goals.values():
            if start is not None:
                phases.append((name, color, start, end if end is not None else t))
    return np.array(samples), sorted(phases, key=lambda p: p[2])


def cumulative_wh(t, p):
    """Energy used since the first sample at each sample time (trapezoid rule)."""
    return np.concatenate([[0.0], np.cumsum(np.diff(t) * (p[1:] + p[:-1]) / 2) / 3600.0])


def ensure_workspace():
    """Re-run under the workspace setup if lunabot_msgs (the PowerMonitor type) is not sourced."""
    try:
        import lunabot_msgs.msg  # noqa: F401
        return
    except ImportError:
        pass
    # <ws>/src/lunabot_ros/scripts/plot_power.py -> <ws>/install/setup.bash
    setup = Path(__file__).resolve().parents[3] / 'install' / 'setup.bash'
    if os.environ.get('PLOT_POWER_SOURCED') or not setup.is_file():
        sys.exit('lunabot_msgs not found, source the workspace: source <ws>/install/setup.bash')
    os.environ['PLOT_POWER_SOURCED'] = '1'
    os.execvp('bash', ['bash', '-c', f'source {setup} && exec python3 "$0" "$@"',
                       str(Path(__file__).resolve()), *sys.argv[1:]])


def main():
    ensure_workspace()
    parser = argparse.ArgumentParser(description='Plot power use from recorded bags')
    parser.add_argument('bags', nargs='+', type=Path)
    parser.add_argument('--out', type=Path, help='Image file, defaults to power.png in the first bag')
    parser.add_argument('--show', action='store_true', help='Open a window as well')
    args = parser.parse_args()

    if not args.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, (ax_p, ax_e) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    plotted = 0
    for bag in args.bags:
        if not bag.exists():
            print(f'{bag}: not found')
            continue
        try:
            samples, phases = read_bag(bag)
        except RuntimeError as e:
            print(f'{bag}: cannot open as a bag ({e})')
            continue
        if len(samples) == 0:
            print(f'{bag.name}: no /power_monitor data')
            continue
        t, volts, p = samples[:, 0], samples[:, 1], samples[:, 2]
        energy = cumulative_wh(t, p)
        total = energy[-1]
        print(f'\n{bag.name} ({len(samples)} samples)')
        print(f'  {t[-1] - t[0]:.0f} s, average {p.mean():.0f} W, peak {p.max():.0f} W, '
              f'{total:.2f} Wh, {volts.min():.2f}-{volts.max():.2f} V')
        for name, _, start, end in phases:
            used = np.interp(end, t, energy) - np.interp(start, t, energy)
            print(f'  {name:11s} {start:6.0f}-{end:6.0f} s  {used:.3f} Wh')

        plotted += 1
        line, = ax_p.plot(t, p, lw=1, label=bag.name)
        ax_e.plot(t, energy, lw=1.5, color=line.get_color(), label=f'{bag.name} ({total:.2f} Wh)')

        # Shading only reads well for one run
        if len(args.bags) == 1:
            seen = set()
            for name, color, start, end in phases:
                for ax in (ax_p, ax_e):
                    ax.axvspan(start, end, color=color, alpha=0.2,
                               label=name if (name not in seen and ax is ax_p) else None)
                seen.add(name)

    if not plotted:
        return

    ax_p.set_ylabel('Power (W)')
    ax_e.set_ylabel('Energy used (Wh)')
    ax_e.set_xlabel('Time since bag start (s)')
    for ax in (ax_p, ax_e):
        ax.grid(alpha=0.3)
        ax.legend(loc='upper left', fontsize=8)
    fig.tight_layout()

    out = args.out or args.bags[0] / 'power.png'
    fig.savefig(out, dpi=110)
    print(f'\nSaved {out}')
    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
