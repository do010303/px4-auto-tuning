#!/usr/bin/env python3
"""Sweep one parameter while the vehicle keeps flying laps.

The vehicle takes off once and flies the reference continuously. At lap
boundaries the next value is written over MAVLink; nothing is restarted. Each
value gets `--settle-laps` to let the transient die out, then `--measure-laps`
that are scored. SITL is only restarted when a value makes the flight diverge,
which is recorded as a failure of that value.

    python3 -m px4_auto_tuning.sweep_runner --param MPC_XY_P --values 0.4:1.8:0.2 --repeats 3

Values are visited in random order so that slow drifts (battery model, GPS
bias) do not line up with the swept value. A region of good values is only
meaningful with several repeats per point: one pass or fail proves little.
"""

import argparse
import csv
import os
import random
import subprocess
import sys
import time

import numpy as np

from . import metrics
from .param_client import ParamClient
from .run_trials import MAX_SIM_GAP_MS, latest_ulog
from .sim import Sim


def parse_values(text):
    if ':' in text:
        start, stop, step = (float(v) for v in text.split(':'))
        return [round(v, 6) for v in np.arange(start, stop + step / 2, step)]

    return [float(v) for v in text.split(',')]


def read_laps(events_path):
    """Lap start times, the hover window and the terminal event of a flight so far."""
    laps, hover, end = {}, [None, None], None

    if not os.path.exists(events_path):
        return laps, hover, end

    for t, name, detail in metrics.read_events(events_path):
        if name == 'LAP':
            laps[int(detail)] = t

        elif name == 'SETTLE':
            hover[0] = t

        elif name == 'TRACK':
            hover[1] = t

        elif name in ('SETTLE_END', 'ABORT', 'DONE'):
            end = end or (t, name)

    return laps, hover, end


def fly(args, queue, raw, flight, sim):
    """One continuous flight working through `queue`. Returns the rows it produced."""
    laps_per_value = args.settle_laps + args.measure_laps
    # One warm-up lap at the default value, plus one closing lap to end the last window
    total_laps = 2 + len(queue) * laps_per_value
    events = os.path.join(raw, f'events_{flight:03d}.csv')

    if os.path.exists(events):
        os.remove(events)

    node = subprocess.Popen([sys.executable, '-m', 'px4_auto_tuning.trajectory_node', '--events', events,
                             '--laps', str(total_laps), '--max-error', str(args.max_error), *args.node_args])
    params = ParamClient()
    default = params.get(args.param)
    plan = []   # (value, first lap of this value)
    errors_before = sim.log_errors()[1]

    while node.poll() is None:
        laps, _, _ = read_laps(events)
        current = max(laps) if laps else -1
        slot = (current - 1) // laps_per_value if current >= 1 else -1

        if 0 <= slot < len(queue) and slot == len(plan):
            reported = params.set(args.param, queue[slot])
            plan.append((queue[slot], current))
            print(f'lap {current}: {args.param} = {reported:g}', flush=True)

        time.sleep(0.2)

    # Leave the vehicle with sane gains whatever happened
    try:
        params.set(args.param, default)

    except TimeoutError:
        pass

    laps, hover, end = read_laps(events)
    errors, _ = sim.log_errors(errors_before)
    ulog = metrics.load(latest_ulog(args.log_root)) if hover[1] else None
    t_end = end[0] if end else None
    rows = []

    for value, first_lap in plan:
        start = laps.get(first_lap + args.settle_laps)
        stop = laps.get(first_lap + laps_per_value, t_end if node.returncode == 0 else None)
        row = {'param': args.param, 'value': value, 'flight': flight, 'lap': first_lap,
               'default': default, 'failed': False, 'valid': True}

        if start is None or stop is None:
            # The flight ended inside this value's window
            row['failed'] = True
            row['reason'] = end[1] if end else 'node died'

        else:
            row.update(metrics.compute(ulog, hover[0], start, stop, t_hover_end=hover[1]))
            row['valid'] = row['sim_max_gap_ms'] < MAX_SIM_GAP_MS

        rows.append(row)

    # A divergence caused by a simulator stall says nothing about the value
    if errors and rows and rows[-1]['failed']:
        rows[-1]['valid'] = False
        rows[-1]['reason'] = 'PX4 error: ' + errors[0][-60:]

    return rows, node.returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--param', required=True)
    parser.add_argument('--values', required=True, help='start:stop:step or v1,v2,...')
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--settle-laps', type=int, default=1)
    parser.add_argument('--measure-laps', type=int, default=2)
    parser.add_argument('--max-error', type=float, default=6.0, help='divergence threshold [m]')
    parser.add_argument('--name', help='result name (default: sweep_<param>)')
    parser.add_argument('--attach', action='store_true',
                        help='use the SITL that is already running (started by hand) instead of starting one')
    parser.add_argument('--world', default='default')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--log-root', default='run/rootfs/log')
    parser.add_argument('--out', default='results')
    parser.add_argument('node_args', nargs='*', help='extra trajectory_node arguments after --')
    args = parser.parse_args()

    name = args.name or f'sweep_{args.param}'
    raw = os.path.join(args.out, 'raw', name)
    os.makedirs(raw, exist_ok=True)
    table = os.path.join(args.out, f'{name}.csv')

    queue = parse_values(args.values) * args.repeats
    random.Random(args.seed).shuffle(queue)

    sim = Sim(world=args.world, headless=not args.gui, attach=args.attach)
    rows, flight, retries = [], 0, {}

    while queue:
        sim.start()
        new_rows, code = fly(args, queue, raw, flight, sim)
        flight += 1

        for row in new_rows:
            if row['valid']:
                queue.remove(row['value'])
                rows.append(row)
                state = f"FAILED ({row['reason']})" if row['failed'] else f"true_rmse_xy={row['true_rmse_xy']:.3f}"
                print(f"{args.param}={row['value']:g}: {state}", flush=True)

            elif retries.setdefault(row['value'], 0) < 3:
                retries[row['value']] += 1
                print(f"{args.param}={row['value']:g}: invalid ({row.get('reason', 'simulator stall')}), will repeat",
                      flush=True)

            else:
                queue.remove(row['value'])
                rows.append(row)

        fields = sorted({k for r in rows for k in r},
                        key=lambda k: (k not in ('param', 'value', 'failed', 'valid', 'reason'), k))

        with open(table, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

        if not new_rows and code != 0:
            raise SystemExit('flight failed before any value was tried, see run/px4.log')

    sim.stop()


if __name__ == '__main__':
    main()
