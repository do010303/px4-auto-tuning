#!/usr/bin/env python3
"""Sweep one or more parameters while the vehicle keeps flying laps.

The vehicle takes off once and flies the reference continuously. At lap
boundaries the next value is written over MAVLink; nothing is restarted. Each
value gets `--settle-laps` to let the transient die out, then `--measure-laps`
that are scored. SITL is only restarted when a value makes the flight diverge,
which is recorded as a failure of that value.

    python3 -m px4_auto_tuning.sweep_runner --param MPC_XY_P --values 0.4:1.8:0.2 --repeats 3

Several parameters form a grid (every combination is flown):

    ... --param MPC_XY_P,MPC_XY_VEL_P_ACC --values "1,2,3;1.8,3.5,5"

`--set NAME=VALUE` fixes other parameters for the whole sweep, e.g. a GPS
noise level or a second gain.

Values are visited in random order so that slow drifts (battery model, GPS
bias) do not line up with the swept value. A region of good values is only
meaningful with several repeats per point: one pass or fail proves little.
"""

import argparse
import csv
import itertools
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


def label(point):
    """CSV/console form of a grid point: a number for one parameter, 'a|b' for several."""
    return point[0] if len(point) == 1 else '|'.join(f'{v:g}' for v in point)


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

    node = subprocess.Popen([sys.executable, '-m', 'px4_auto_tuning.trajectory_node', '--events', events, '--speed', str(args.speed),
                             '--laps', str(total_laps), '--max-error', str(args.max_error), *args.node_args])

    try:
        return _fly(args, queue, events, flight, sim, node, laps_per_value)

    finally:
        # Never leave a node behind that would arm the vehicle on the next SITL
        if node.poll() is None:
            node.terminate()


def _fly(args, queue, events, flight, sim, node, laps_per_value):
    params = ParamClient()
    defaults = [params.get(name) for name in args.params]
    plan = []   # (value, first lap of this value)
    fixed_set = not args.fixed
    errors_before = sim.log_errors()[1]

    while node.poll() is None:
        laps, _, _ = read_laps(events)
        current = max(laps) if laps else -1
        slot = (current - 1) // laps_per_value if current >= 1 else -1

        if not fixed_set and current >= 0:
            # Fixed parameters (GPS profile, ...) go in once the vehicle is
            # tracking. Set on the ground, a noisy GPS fails EKF2's strict
            # pre-flight checks and the vehicle never gets a position; in the
            # air only loose checks apply. The warm-up lap absorbs the change.
            for name, value in args.fixed.items():
                params.set(name, value)

            fixed_set = True
            print(f'lap {current}: fixed parameters set', flush=True)

        if 0 <= slot < len(queue) and slot == len(plan):
            for name, value in zip(args.params, queue[slot]):
                params.set(name, value)

            plan.append((queue[slot], current))
            print(f'lap {current}: {args.param} = {label(queue[slot])}', flush=True)

        time.sleep(0.2)

    # Leave the vehicle with sane gains whatever happened
    try:
        for name, value in zip(args.params, defaults):
            params.set(name, value)

    except TimeoutError:
        pass

    laps, hover, end = read_laps(events)
    errors, _ = sim.log_errors(errors_before)
    ulog_path = latest_ulog(args.log_root) if hover[1] else None
    ulog = metrics.load(ulog_path) if ulog_path else None
    t_end = end[0] if end else None
    rows = []

    for value, first_lap in plan:
        start = laps.get(first_lap + args.settle_laps)
        stop = laps.get(first_lap + laps_per_value, t_end if node.returncode == 0 else None)
        row = {'param': args.param, 'value': label(value), 'flight': flight, 'lap': first_lap,
               'default': label(defaults), 'failed': False, 'valid': True, 'point': value,
               'ulog': os.path.relpath(ulog_path, args.log_root) if ulog_path else ''}
        row.update(zip(args.params, value))
        row.update(args.fixed)

        if start is None or stop is None:
            # The flight ended inside this value's window
            row['failed'] = True
            row['reason'] = end[1] if end else 'node died'

            # Only blame the value if the simulator ran cleanly up to the end
            if ulog is not None and end and first_lap in laps:
                row['sim_max_gap_ms'] = metrics.sim_gap_ms(ulog, laps[first_lap], end[0])
                row['valid'] = row['sim_max_gap_ms'] < MAX_SIM_GAP_MS

        else:
            row.update(metrics.compute(ulog, hover[0], start, stop, t_hover_end=hover[1]))
            row['valid'] = row['sim_max_gap_ms'] < MAX_SIM_GAP_MS

        rows.append(row)

    # A divergence caused by a simulator stall says nothing about the value
    if errors and rows and rows[-1]['failed']:
        rows[-1]['valid'] = False
        rows[-1]['reason'] = 'PX4 error: ' + errors[0][-60:]

    elif rows and rows[-1]['failed'] and not rows[-1]['valid']:
        rows[-1]['reason'] = 'simulator stall'

    return rows, node.returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--param', required=True, help='parameter name, or several separated by commas')
    parser.add_argument('--values', required=True,
                        help='start:stop:step or v1,v2,...; one such list per parameter, separated by ";"')
    parser.add_argument('--set', action='append', default=[], metavar='NAME=VALUE',
                        help='parameter held fixed during the whole sweep (repeatable)')
    parser.add_argument('--profile', help='GPS/environment profile from ./profiles, e.g. forest')
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--settle-laps', type=int, default=1)
    parser.add_argument('--measure-laps', type=int, default=2)
    parser.add_argument('--max-error', type=float, default=6.0, help='divergence threshold [m]')
    parser.add_argument('--name', help='result name (default: sweep_<param>)')
    parser.add_argument('--attach', action='store_true',
                        help='use the SITL that is already running (started by hand) instead of starting one')
    parser.add_argument('--world', default='default')
    parser.add_argument('--model', default='x500', help='x500 or a variant from ./models')
    parser.add_argument('--speed', type=float, default=1.0, help='simulation speed factor')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--resume', action='store_true', help='continue an interrupted sweep of the same name')
    parser.add_argument('--log-root', default='run/rootfs/log')
    parser.add_argument('--out', default='results')
    parser.add_argument('node_args', nargs='*', help='extra trajectory_node arguments after --')
    args = parser.parse_args()

    args.params = args.param.split(',')
    args.fixed = {}

    if args.profile:
        # GPS/environment profile: a file of "NAME VALUE" lines in ./profiles
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

        for line in open(os.path.join(here, 'profiles', f'{args.profile}.params')):
            if line.strip() and not line.startswith('#'):
                key, value = line.split()
                args.fixed[key] = float(value)

    args.fixed.update({k: float(v) for k, v in (item.split('=') for item in args.set)})
    axes = [parse_values(text) for text in args.values.split(';')]

    if len(axes) != len(args.params):
        parser.error('--values needs one list per parameter')

    name = args.name or 'sweep_' + '_x_'.join(args.params)
    raw = os.path.join(args.out, 'raw', name)
    os.makedirs(raw, exist_ok=True)
    table = os.path.join(args.out, f'{name}.csv')

    queue = list(itertools.product(*axes)) * args.repeats
    random.Random(args.seed).shuffle(queue)

    sim = Sim(world=args.world, headless=not args.gui, attach=args.attach, speed=args.speed, model=args.model)
    rows, flight, retries, barren = [], 0, {}, 0

    if args.resume and os.path.exists(table) and os.path.getsize(table):
        # Keep what an interrupted sweep already measured
        with open(table) as f:
            rows = list(csv.DictReader(f))

        for row in rows:
            point = tuple(float(row[name]) for name in args.params)

            if point in queue:
                queue.remove(point)

        flight = 1 + max((int(r['flight']) for r in rows), default=-1)
        print(f'resuming: {len(rows)} results kept, {len(queue)} to go', flush=True)

    while queue:
        sim.start()
        new_rows, code = fly(args, queue, raw, flight, sim)
        flight += 1

        for row in new_rows:
            point = row.pop('point')
            shown = f"{args.param}={row['value']}"

            if row['valid']:
                queue.remove(point)
                rows.append(row)
                state = f"FAILED ({row['reason']})" if row['failed'] else f"true_rmse_xy={row['true_rmse_xy']:.3f}"
                print(f'{shown}: {state}', flush=True)

            elif retries.setdefault(point, 0) < 3:
                retries[point] += 1
                print(f"{shown}: invalid ({row.get('reason', 'simulator stall')}), will repeat", flush=True)

            else:
                queue.remove(point)
                rows.append(row)

        fields = sorted({k for r in rows for k in r},
                        key=lambda k: (k not in ('param', 'value', 'failed', 'valid', 'reason'), k))

        with open(table, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

        # A flight that dies during take-off or warm-up tried no value; that
        # is the simulator, so try again, but not forever
        barren = barren + 1 if not new_rows else 0

        if barren >= 5:
            raise SystemExit('5 flights in a row failed before any value was tried, see run/px4.log')

    sim.stop()


if __name__ == '__main__':
    main()
