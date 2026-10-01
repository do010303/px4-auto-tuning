#!/usr/bin/env python3
"""Tracking metrics from a PX4 ulog, cut to a window given in PX4 time.

Three errors are kept apart, because gains only act on the first one while
the mission only cares about the last one:

    tracking    setpoint - estimate   what the controller sees and corrects
    estimation  estimate - truth      what GPS/EKF contribute
    true        setpoint - truth      where the vehicle really is vs. the plan

The Gazebo ground truth is expressed in the world frame and the estimate in
the EKF local frame; their origins differ by a constant. That constant is
taken from a hover window before the trial, so "estimation" measures how the
estimate moves relative to truth *during* the trial.

    python3 -m px4_auto_tuning.metrics log.ulg --events events.csv
"""

import argparse
import csv
import json

import numpy as np
from pyulog import ULog
from scipy.spatial import cKDTree

SETPOINT = 'trajectory_setpoint'
ESTIMATE = 'vehicle_local_position'
TRUTH = 'vehicle_local_position_groundtruth'


def _dataset(ulog, name):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == 0:
            return d.data

    return None


def _xyz(data, fields):
    return np.column_stack([data[f] for f in fields])


def _interp(t_new, t, values):
    return np.column_stack([np.interp(t_new, t, values[:, i]) for i in range(values.shape[1])])


def _rms(v):
    return float(np.sqrt(np.mean(np.square(v)))) if len(v) else float('nan')


def _lag(t, reference, t_other, other, span=0.6, step=0.01):
    """Time shift [s] of `other` that best matches `reference` (positive: other is late)."""
    shifts = np.arange(-span, span + step, step)
    cost = [_rms(np.linalg.norm((reference - _interp(t + s * 1e6, t_other, other))[:, :2], axis=1))
            for s in shifts]
    return float(shifts[int(np.argmin(cost))])


def _error_stats(prefix, err):
    xy = np.linalg.norm(err[:, :2], axis=1)
    z = err[:, 2]
    return {
        f'{prefix}_rmse_xy': _rms(xy),
        f'{prefix}_max_xy': float(xy.max()),
        f'{prefix}_rmse_z': _rms(z),
        f'{prefix}_max_z': float(np.abs(z).max()),
    }


def read_events(path):
    with open(path) as f:
        return [(int(r['px4_boot_time_us']), r['event'], r['detail']) for r in csv.DictReader(f)]


def window_from_events(events, start='TRACK', end='SETTLE_END', hover='SETTLE'):
    """(hover_start, start, end) in microseconds from a trajectory_node events file."""
    times = {name: t for t, name, _ in events if name != 'LAP'}
    return times[hover], times[start], times[end]


def load(ulog_path):
    return ULog(ulog_path, [SETPOINT, ESTIMATE, TRUTH, 'vehicle_angular_velocity', 'actuator_motors'])


def compute(ulog, t_hover, t_start, t_end, t_hover_end=None):
    """Metrics for the window [t_start, t_end] of a ulog (path or `load()` result).

    The frames are aligned on the hover window [t_hover, t_hover_end], which
    defaults to ending where the metrics window starts.
    """
    if isinstance(ulog, str):
        ulog = load(ulog)

    if t_hover_end is None:
        t_hover_end = t_start

    sp = _dataset(ulog, SETPOINT)
    est = _dataset(ulog, ESTIMATE)
    truth = _dataset(ulog, TRUTH)

    if sp is None or est is None or truth is None:
        raise RuntimeError('ulog is missing setpoint, estimate or ground truth topics')

    t_est = est['timestamp']
    p_est = _xyz(est, ('x', 'y', 'z'))
    v_est = _xyz(est, ('vx', 'vy', 'vz'))
    t_truth = truth['timestamp']
    p_truth = _xyz(truth, ('x', 'y', 'z'))

    # Constant offset between the EKF local frame and the Gazebo world frame
    hover = (t_truth >= t_hover) & (t_truth <= t_hover_end)
    offset = np.mean(_interp(t_truth[hover], t_est, p_est) - p_truth[hover], axis=0)
    p_truth = p_truth + offset

    # Everything is compared on the setpoint time base
    sel = (sp['timestamp'] >= t_start) & (sp['timestamp'] <= t_end)
    t = sp['timestamp'][sel]
    p_sp = _xyz(sp, ('position[0]', 'position[1]', 'position[2]'))[sel]
    v_sp = _xyz(sp, ('velocity[0]', 'velocity[1]', 'velocity[2]'))[sel]

    p_est_i = _interp(t, t_est, p_est)
    p_truth_i = _interp(t, t_truth, p_truth)

    out = {'duration_s': float((t[-1] - t[0]) * 1e-6), 'samples': int(len(t))}
    out.update(_error_stats('tracking', p_sp - p_est_i))
    out.update(_error_stats('estimation', p_est_i - p_truth_i))
    out.update(_error_stats('true', p_sp - p_truth_i))

    # How much of each error is a pure time delay
    out['tracking_lag_s'] = _lag(t, p_sp, t_est, p_est)
    out['estimation_lag_s'] = _lag(t, p_truth_i, t_est, p_est)

    # Cross-track: distance from the real position to the nearest point of the
    # reference path, i.e. the true error with the time lag taken out
    dist, _ = cKDTree(p_sp[:, :2]).query(p_truth_i[:, :2])
    out['true_crosstrack_rmse'] = _rms(dist)
    out['true_crosstrack_max'] = float(dist.max())

    if np.all(np.isfinite(v_sp)):
        out['tracking_vel_rmse_xy'] = _rms(np.linalg.norm((v_sp - _interp(t, t_est, v_est))[:, :2], axis=1))

    # Oscillation: angular rate with its 0.5 s moving average removed
    rates = _dataset(ulog, 'vehicle_angular_velocity')

    if rates is not None:
        sel = (rates['timestamp'] >= t_start) & (rates['timestamp'] <= t_end)
        w = _xyz(rates, ('xyz[0]', 'xyz[1]', 'xyz[2]'))[sel]
        dt = np.median(np.diff(rates['timestamp'][sel])) * 1e-6
        n = max(int(0.5 / dt), 1)
        kernel = np.ones(n) / n
        hf = w - np.column_stack([np.convolve(w[:, i], kernel, mode='same') for i in range(3)])
        out['rate_hf_rms'] = _rms(np.linalg.norm(hf[n:-n], axis=1))

        # Simulation health: Gazebo does not wait for PX4, so when the host
        # stalls, sensor samples go missing and the flight is not comparable
        out['sim_max_gap_ms'] = float(np.max(np.diff(rates['timestamp_sample'][sel])) * 1e-3)

    motors = _dataset(ulog, 'actuator_motors')

    if motors is not None:
        sel = (motors['timestamp'] >= t_start) & (motors['timestamp'] <= t_end)
        u = _xyz(motors, [f'control[{i}]' for i in range(4)])[sel]
        out['motor_saturation'] = float(np.mean((u.max(axis=1) > 0.98) | (u.min(axis=1) < 0.02)))
        out['motor_mean'] = float(u.mean())

    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('ulog')
    parser.add_argument('--events', required=True, help='events CSV written by trajectory_node')
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args()

    result = compute(args.ulog, *window_from_events(read_events(args.events)))

    if args.json:
        print(json.dumps(result, indent=2))

    else:
        for key, value in result.items():
            print(f'{key:24s} {value:10.4f}' if isinstance(value, float) else f'{key:24s} {value:>10}')


if __name__ == '__main__':
    main()
