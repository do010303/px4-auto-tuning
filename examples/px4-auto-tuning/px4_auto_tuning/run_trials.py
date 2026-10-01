#!/usr/bin/env python3
"""Repeat one flight until N valid runs are collected, and tabulate the metrics.

Starts SITL itself. A run is invalid when the flight aborted, PX4 logged an
error, or the simulator stalled; SITL is then restarted and the run repeated.

    python3 -m px4_auto_tuning.run_trials --name baseline_figure8 -n 10 -- --kind figure8 --laps 2

Everything after `--` is passed to trajectory_node. Each row of
results/<name>.csv is one flight; the spread across rows is the measurement
noise that any gain comparison has to beat.
"""

import argparse
import csv
import glob
import os
import subprocess
import sys

from . import metrics
from .sim import Sim

# A sensor gap longer than this means the host stalled during the flight
MAX_SIM_GAP_MS = 40.0


def latest_ulog(log_root):
    logs = glob.glob(os.path.join(log_root, '*', '*.ulg'))

    if not logs:
        raise FileNotFoundError(f'no ulog under {log_root}')

    return max(logs, key=os.path.getmtime)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--name', required=True)
    parser.add_argument('-n', type=int, default=10)
    parser.add_argument('--log-root', default='run/rootfs/log')
    parser.add_argument('--out', default='results')
    parser.add_argument('--attach', action='store_true',
                        help='use the SITL that is already running (started by hand) instead of starting one')
    parser.add_argument('--world', default='default')
    parser.add_argument('--gui', action='store_true', help='show the Gazebo GUI')
    parser.add_argument('node_args', nargs='*')
    args = parser.parse_args()

    raw = os.path.join(args.out, 'raw', args.name)
    os.makedirs(raw, exist_ok=True)
    table = os.path.join(args.out, f'{args.name}.csv')
    rows = []
    sim = Sim(world=args.world, headless=not args.gui, attach=args.attach)
    sim.start()
    log_pos = sim.log_errors()[1]
    attempt = 0

    while sum(r['valid'] for r in rows) < args.n and attempt < 3 * args.n:
        events = os.path.join(raw, f'events_{attempt:03d}.csv')
        code = subprocess.call([sys.executable, '-m', 'px4_auto_tuning.trajectory_node',
                                '--events', events, *args.node_args])
        errors, log_pos = sim.log_errors(log_pos)
        row = {'trial': attempt, 'exit_code': code, 'valid': False, 'px4_errors': len(errors)}

        if code == 0:
            ulog = latest_ulog(args.log_root)
            row['ulog'] = os.path.relpath(ulog, args.log_root)
            row.update(metrics.compute(ulog, *metrics.window_from_events(metrics.read_events(events))))
            row['valid'] = row['sim_max_gap_ms'] < MAX_SIM_GAP_MS and not errors

        if row['valid']:
            print(f"trial {attempt}: true_rmse_xy={row['true_rmse_xy']:.3f} "
                  f"tracking_rmse_xy={row['tracking_rmse_xy']:.3f}", flush=True)

        else:
            # Never reuse a vehicle that may have crashed or a stalled simulator
            print(f'trial {attempt}: INVALID (exit {code}, {len(errors)} PX4 errors, '
                  f"gap {row.get('sim_max_gap_ms', float('nan')):.0f} ms) -> restarting SITL", flush=True)
            sim.start()
            log_pos = sim.log_errors()[1]

        rows.append(row)
        attempt += 1
        fields = sorted({k for r in rows for k in r}, key=lambda k: (k not in ('trial', 'exit_code', 'valid', 'ulog'), k))

        with open(table, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

    sim.stop()


if __name__ == '__main__':
    main()
