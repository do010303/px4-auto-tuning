#!/usr/bin/env python3
"""Run PX4's own attitude/rate autotune in a hover and report what it found.

Takes off in offboard position hold, sends VEHICLE_CMD_DO_AUTOTUNE_ENABLE and
keeps hovering while mc_autotune_attitude_control excites roll, pitch and yaw
and identifies the rate loop. With MC_AT_APPLY=0 (set here) the gains are only
computed and logged, not applied.

    python3 -m px4_auto_tuning.px4_autotune --sitl
"""

import argparse
import glob
import json
import os

import numpy as np
import rclpy
from pyulog import ULog

from px4_msgs.msg import VehicleCommand

from .param_client import ParamClient
from .trajectory_node import TrajectoryNode
from .trajectory_node import parse_args as trajectory_args

# autotune_attitude_control_status.state (the *_amp entries are fixed-wing only)
STATES = ['idle', 'init', 'roll_amp', 'roll', 'roll_pause', 'pitch_amp', 'pitch', 'pitch_pause', 'yaw_amp', 'yaw',
          'yaw_pause', 'verification', 'apply', 'test', 'complete', 'fail', 'wait_for_disarm']
RATE_PARAMS = ['MC_ROLLRATE_P', 'MC_ROLLRATE_I', 'MC_ROLLRATE_D', 'MC_PITCHRATE_P', 'MC_PITCHRATE_I',
               'MC_PITCHRATE_D', 'MC_YAWRATE_P', 'MC_YAWRATE_I', 'MC_YAWRATE_D', 'MC_ROLL_P', 'MC_PITCH_P', 'MC_YAW_P']


class AutotuneFlight(TrajectoryNode):

    def __init__(self, args):
        super().__init__(trajectory_args(['--kind', 'hover', '--altitude', '10', '--events', args.events,
                                          '--settle', '6']))
        self.duration = args.duration
        self.sent = 0

    def _step(self):
        if self.state != 'SETTLE':
            return super()._step()

        hover = self.origin + np.array([0.0, 0.0, -self.args.altitude])
        self._publish(hover)
        t = self._elapsed()

        # The command is not acknowledged over this link: send it a few times
        if t > self.args.settle and self.sent < 3:
            self._command(VehicleCommand.VEHICLE_CMD_DO_AUTOTUNE_ENABLE, 1.0)
            self.sent += 1

            if self.sent == 1:
                self._event('AUTOTUNE_START')

        if t > self.args.settle + self.duration:
            self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._enter('LAND')


def read_result(log_root):
    path = max(glob.glob(os.path.join(log_root, '*', '*.ulg')), key=os.path.getmtime)
    ulog = ULog(path, ['autotune_attitude_control_status'])

    if not ulog.data_list:
        return {'ulog': path, 'error': 'autotune_attitude_control_status not logged'}

    d = ulog.data_list[0].data
    state = d['state'].astype(int)
    changes = np.flatnonzero(np.diff(state)) + 1
    sequence = [(round(float(d['timestamp'][i] - d['timestamp'][0]) * 1e-6, 1), STATES[state[i]]) for i in changes]
    out = {'ulog': path, 'sequence': sequence, 'final_state': STATES[state[-1]]}

    # Gains the identification had converged to at the end of each axis
    for axis in ('roll', 'pitch', 'yaw'):
        idx = np.flatnonzero(state == STATES.index(axis))

        if len(idx):
            i = idx[-1]
            out[axis] = {'rate_k': round(float(d['kc'][i]), 4), 'rate_i': round(float(d['ki'][i]), 4),
                         'rate_d': round(float(d['kd'][i]), 5), 'att_p': round(float(d['att_p'][i]), 3),
                         'seconds': round(float(d['timestamp'][i] - d['timestamp'][idx[0]]) * 1e-6, 1)}

    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--duration', type=float, default=75.0, help='seconds to hover after starting autotune')
    parser.add_argument('--sitl', action='store_true', help='start (and stop) SITL here')
    parser.add_argument('--model', default='x500')
    parser.add_argument('--events', default='autotune_events.csv')
    parser.add_argument('--log-root', default='run/rootfs/log')
    parser.add_argument('--out', default='results/px4_autotune.json')
    args = parser.parse_args()

    sim = None

    if args.sitl:
        from .sim import Sim
        sim = Sim(model=args.model)
        sim.start()

    params = ParamClient()
    params.set('MC_AT_EN', 1)
    params.set('MC_AT_APPLY', 0)
    current = {name: round(float(params.get(name)), 5) for name in RATE_PARAMS}

    rclpy.init()
    node = AutotuneFlight(args)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    node.events_file.close()
    node.destroy_node()
    rclpy.shutdown()

    result = read_result(args.log_root)
    result['model'] = args.model
    result['current_params'] = current
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)

    with open(args.out, 'w') as f:
        json.dump(result, f, indent=2)

    print(json.dumps(result, indent=2))

    if sim is not None:
        sim.stop()


if __name__ == '__main__':
    main()
