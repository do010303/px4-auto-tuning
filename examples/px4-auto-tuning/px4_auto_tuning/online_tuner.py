#!/usr/bin/env python3
"""Tune the horizontal position/velocity gains while the vehicle is flying.

There is no PX4 autotune for the position loop (mc_autotune_attitude_control
only identifies the rate loop). This is a supervisor that sits outside the
firmware: it flies the reference continuously, scores every lap from what
PX4 publishes over ROS, and changes gains over MAVLink.

Method: coordinate search with a safety net.

  * score of a lap = tracking RMSE + weight * oscillation
  * try one gain at a time, one multiplicative step up; keep it only if the
    score improves by a clear margin and oscillation stays under a limit
  * while a candidate is flying, a watchdog reverts to the last good gains
    at once if oscillation or error exceeds a hard limit, without waiting
    for the lap to end
  * when no gain can be raised any more, halve the step once and retry
  * finally back off from the best point found by `--margin`, because the
    sweeps showed the best score sits right next to the unstable region

    python3 -m px4_auto_tuning.online_tuner            # against a running SITL
    python3 -m px4_auto_tuning.online_tuner --sitl --speed 4
"""

import argparse
import csv
import json
import math
import os
import queue
import threading

import numpy as np
import rclpy

from px4_msgs.msg import VehicleAttitude

from .param_client import ParamClient
from .trajectory_node import TrajectoryNode
from .trajectory_node import parse_args as trajectory_args

GAINS = ['MPC_XY_P', 'MPC_XY_VEL_P_ACC', 'MPC_XY_VEL_I_ACC']


class LapScore:
    """Accumulates tracking error and attitude oscillation over a window."""

    def __init__(self):
        self.err2 = 0.0
        self.err_max = 0.0
        self.n = 0
        self.osc2 = 0.0
        self.osc_n = 0

    def add_error(self, err):
        self.err2 += err * err
        self.err_max = max(self.err_max, err)
        self.n += 1

    def add_osc(self, hf_rate):
        self.osc2 += hf_rate * hf_rate
        self.osc_n += 1

    @property
    def rmse(self):
        return math.sqrt(self.err2 / self.n) if self.n else float('nan')

    @property
    def osc(self):
        return math.sqrt(self.osc2 / self.osc_n) if self.osc_n else 0.0


class OnlineTuner(TrajectoryNode):

    def __init__(self, args):
        base = trajectory_args(['--kind', args.kind, '--laps', str(args.max_laps + 4), '--events', args.events,
                                '--speed', str(args.speed), '--max-error', '8'])
        super().__init__(base)
        self.tune = args
        self._wanted['/fmu/out/vehicle_attitude'] = (VehicleAttitude, self._on_att)
        self.params = ParamClient()
        self.gains = {name: float(self.params.get(name)) for name in args.gains}
        self.start_gains = dict(self.gains)
        self.best = dict(self.gains)
        self.best_score = None
        self.baseline = None

        # attitude oscillation: tilt rate with its slow part removed
        self._att_prev = None
        self._rate_slow = np.zeros(2)
        self._osc_recent = []

        self.score = LapScore()
        self.phase = 'baseline'          # baseline -> search -> validate -> done
        self.window_lap = None           # set once the ramp-up lap is over
        self.window_start = 0.0          # lap position where the current window started
        self.settling = True
        self.recovering = False          # waiting for calm after a watchdog revert
        self.step = args.step
        self.queue = list(args.gains)
        self.exhausted = set()
        self.candidate = None            # (gain name, value)
        self.halved = False
        self.history = []
        self.validation = []

        # Parameter writes wait for an acknowledgement. Doing that in the
        # control loop would starve the setpoint stream and drop PX4 out of
        # offboard, so a worker thread does the writing.
        self.applying = False
        self._jobs = queue.Queue()
        threading.Thread(target=self._writer, daemon=True).start()

    # -- measurements ------------------------------------------------------

    def _on_att(self, msg):
        w, x, y, z = msg.q
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        t = msg.timestamp * 1e-6
        tilt = np.array([roll, pitch])

        if self._att_prev is not None:
            dt = t - self._att_prev[0]

            if 1e-4 < dt < 0.2:
                rate = (tilt - self._att_prev[1]) / dt
                alpha = dt / (0.5 + dt)
                self._rate_slow += alpha * (rate - self._rate_slow)
                hf = float(np.linalg.norm(rate - self._rate_slow))

                if self.state == 'TRACK' and not self.settling:
                    self.score.add_osc(hf)

                self._osc_recent.append((t, hf))
                self._osc_recent = [(ts, v) for ts, v in self._osc_recent if t - ts < 1.0]

        self._att_prev = (t, tilt)

    def _osc_now(self):
        values = [v for _, v in self._osc_recent]
        return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0

    def _cost(self, score):
        return score.rmse + self.tune.osc_weight * score.osc

    # -- gain handling -----------------------------------------------------

    def _writer(self):
        while True:
            gains = self._jobs.get()

            try:
                for name, value in gains.items():
                    self.params.set(name, value)

            except TimeoutError as error:
                self.get_logger().error(f'parameter write failed: {error}')

            if self._jobs.empty():
                self.applying = False

    def _apply(self, gains):
        self.gains = dict(gains)
        self.applying = True
        self._jobs.put(dict(gains))

    def _log(self, action, score=None, note=''):
        row = {'px4_time_s': round(self._now(), 2), 'action': action, 'note': note, **self.gains}

        if score is not None:
            row.update(rmse=round(score.rmse, 4), osc=round(score.osc, 4), cost=round(self._cost(score), 4),
                       err_max=round(score.err_max, 3))

        self.history.append(row)
        self.get_logger().info(json.dumps(row))

    def _next_candidate(self):
        """Pick the next gain to raise; False when the search is over."""
        while True:
            remaining = [g for g in self.queue if g not in self.exhausted]

            if remaining:
                name = remaining[0]
                self.queue.remove(name)
                self.queue.append(name)
                value = self.best[name] * (1.0 + self.step) if self.best[name] > 0 else self.step
                self.candidate = (name, value)
                trial = dict(self.best)
                trial[name] = value
                self._apply(trial)
                return True

            if self.halved:
                return False

            # every gain was rejected at this step size: look closer once
            self.halved = True
            self.step *= 0.5
            self.exhausted.clear()

    def _reject(self, score, note):
        name, _ = self.candidate
        self._log('reject', score, f'{name}: {note}')
        self.exhausted.add(name)
        self._apply(self.best)

    # -- hook into the flight ----------------------------------------------

    def _publish(self, pos, vel=None, acc=None):
        super()._publish(pos, vel, acc)

        if self.state != 'TRACK' or self.phase == 'done':
            return

        t = self._elapsed()
        lap_pos = self.ramp.phase(t) / self.args.period
        lap = int(lap_pos)
        err = float(np.linalg.norm(np.asarray(pos[:2]) - np.array([self.lpos.x, self.lpos.y])))

        if self.window_lap is None:
            # the first lap contains the speed ramp: skip it
            if lap >= 1:
                self.window_lap = lap
                self.window_start = lap_pos
                self.settling = False
                self.score = LapScore()

            return

        if self.applying:
            # nothing is measured until PX4 has confirmed the new gains
            self._restart_window(lap_pos, settle=True)
            return

        if self.recovering:
            # Back on the last good gains after the watchdog fired: let the
            # shaking die out before the next candidate, or it gets the blame
            calm = self._osc_now() < 0.3 * self.tune.osc_abort

            if calm and lap_pos - self.window_start >= 0.25:
                self.recovering = False
                self._restart_window(lap_pos, settle=True)

                if not self._next_candidate():
                    self._finish_search()

            return

        # Watchdog: do not wait for the lap to end when a candidate goes wrong
        if self.phase == 'search' and self.baseline is not None:
            too_shaky = self._osc_now() > self.tune.osc_abort
            too_far = err > max(3.0 * self.baseline.err_max, 1.0)

            if too_shaky or too_far:
                self._reject(self.score, 'watchdog: ' + ('oscillation' if too_shaky else 'error'))
                self._restart_window(lap_pos, settle=True)
                self.recovering = True
                return

        if self.settling:
            if lap_pos - self.window_start >= self.tune.settle_laps:
                self.settling = False
                self.window_start = lap_pos
                self.score = LapScore()

            return

        self.score.add_error(err)

        if lap_pos - self.window_start < 1.0:
            return

        # one full lap measured
        score = self.score

        if self.phase == 'baseline':
            self.baseline = score
            self.best_score = self._cost(score)
            self._log('baseline', score)
            self.phase = 'search'

            if not self._next_candidate():
                self._finish_search()

        elif self.phase == 'search':
            self._judge(score)

            if lap >= self.tune.max_laps or not self._next_candidate():
                self._finish_search()

        elif self.phase == 'validate':
            self.validation.append(score)
            self._log('validate', score)

            if len(self.validation) >= 2:
                self.phase = 'done'
                self._enter('SETTLE_END')
                return

        self._restart_window(lap_pos, settle=self.phase != 'baseline')

    def _judge(self, score):
        """Decide about the candidate that just flew one full lap."""
        name, _ = self.candidate
        cost = self._cost(score)

        if score.osc > self.tune.osc_limit:
            self._reject(score, 'oscillation above limit')

        elif cost < self.best_score * (1.0 - self.tune.min_gain):
            self.best = dict(self.gains)
            self.best_score = cost
            self._log('accept', score, name)

        else:
            self._reject(score, 'no clear improvement')

    def _restart_window(self, lap_pos, settle):
        self.window_start = lap_pos
        self.settling = settle
        self.score = LapScore()

    def _finish_search(self):
        # Step back from the best point towards where we started
        final = {name: self.start_gains[name] + self.tune.margin * (self.best[name] - self.start_gains[name])
                 for name in self.best}
        self._apply(final)
        self._log('final', note=f'backed off to {self.tune.margin:.0%} of the way to the best point')
        self.phase = 'validate'

    def report(self):
        result = {
            'start': self.start_gains,
            'best_found': self.best,
            'final': self.gains,
            'baseline_rmse': round(self.baseline.rmse, 4) if self.baseline else None,
            'baseline_osc': round(self.baseline.osc, 4) if self.baseline else None,
            'final_rmse': round(float(np.mean([s.rmse for s in self.validation])), 4) if self.validation else None,
            'final_osc': round(float(np.mean([s.osc for s in self.validation])), 4) if self.validation else None,
            'candidates_tried': sum(r['action'] in ('accept', 'reject', 'trial') for r in self.history),
            'flight_time_s': round(self.history[-1]['px4_time_s'] - self.history[0]['px4_time_s'], 1)
            if len(self.history) > 1 else None,
        }
        fields = list(dict.fromkeys(k for r in self.history for k in r))
        os.makedirs(os.path.dirname(self.tune.out) or '.', exist_ok=True)

        with open(self.tune.out, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.history)

        return result


def make_parser(doc=__doc__):
    parser = argparse.ArgumentParser(description=doc, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--gains', default='MPC_XY_P,MPC_XY_VEL_P_ACC', help=f'comma separated, from {GAINS}')
    parser.add_argument('--kind', default='figure8')
    parser.add_argument('--step', type=float, default=0.3, help='relative step per trial')
    parser.add_argument('--min-gain', type=float, default=0.03, help='relative score improvement needed to accept')
    parser.add_argument('--osc-weight', type=float, default=0.5, help='metres of error one rad/s of oscillation is worth')
    parser.add_argument('--osc-limit', type=float, default=0.15, help='lap oscillation above this rejects a candidate [rad/s]')
    parser.add_argument('--osc-abort', type=float, default=0.5, help='1 s oscillation above this reverts at once [rad/s]')
    parser.add_argument('--settle-laps', type=float, default=0.5)
    parser.add_argument('--margin', type=float, default=0.75, help='how far to go from the start gains to the best found')
    parser.add_argument('--max-laps', type=int, default=60)
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--sitl', action='store_true', help='start (and stop) SITL here')
    parser.add_argument('--model', default='x500')
    parser.add_argument('--set', action='append', default=[], metavar='NAME=VALUE', help='fixed parameter, e.g. GPS noise')
    parser.add_argument('--events', default='tuner_events.csv')
    parser.add_argument('--out', default='results/online_tuner.csv')
    return parser


def run(args, tuner_class):
    args.gains = args.gains.split(',')

    sim = None

    if args.sitl:
        from .sim import Sim
        sim = Sim(speed=args.speed, model=args.model)
        sim.start()

    rclpy.init()
    node = tuner_class(args)

    for item in args.set:
        name, value = item.split('=')
        node.params.set(name, float(value))

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    result = node.report()
    result['exit_code'] = node.exit_code
    node.events_file.close()
    node.destroy_node()
    rclpy.shutdown()
    print('RESULT ' + json.dumps(result), flush=True)

    if sim is not None:
        sim.stop()


def main():
    run(make_parser().parse_args(), OnlineTuner)


if __name__ == '__main__':
    main()
