#!/usr/bin/env python3
"""What does each PX4 controller layer do? Take it away and watch.

The vehicle climbs and hovers under normal position control. Then the
offboard level is switched and ROS sends the most naive setpoint that level
accepts ("zero velocity", "level attitude at hover thrust", "equal motors"),
with no feedback loop of its own, for a few seconds. Whatever drift builds up
is what the bypassed PX4 layers had been taking care of. Position control is
then handed back to see whether the vehicle can be recovered.

Optionally the setpoint stream is cut afterwards to see which failsafe
reacts when the companion computer goes silent.

`--rate` lowers how often ROS sends during the test (50 Hz by default), to
find the slowest stream PX4 still accepts at each level. With `--closed-loop`
(velocity level only) ROS closes the position loop itself with a plain P
controller, so the same sweep shows how slow an outer loop in ROS may run
before the vehicle drifts or oscillates.

    python3 -m px4_auto_tuning.layer_test --level attitude        # against a running SITL
    python3 -m px4_auto_tuning.layer_test --level all             # starts SITL per level, writes results/layer_test.csv
"""

import argparse
import csv
import json
import math
import os
import subprocess
import sys

import numpy as np
import rclpy

from px4_msgs.msg import (
    ActuatorMotors,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleAttitudeSetpoint,
    VehicleCommand,
    VehicleRatesSetpoint,
    VehicleStatus,
    VehicleThrustSetpoint,
    VehicleTorqueSetpoint,
)

from .trajectory_node import NAN3, TrajectoryNode
from .trajectory_node import parse_args as trajectory_args

# Outermost to innermost; each level bypasses one more PX4 layer than the one before
LEVELS = ['position', 'velocity', 'acceleration', 'attitude', 'body_rate', 'thrust_and_torque', 'direct_actuator']

BYPASSED = {
    'position': 'nothing',
    'velocity': 'position P',
    'acceleration': 'position P, velocity PID',
    'attitude': 'all of position control',
    'body_rate': 'position + attitude control',
    'thrust_and_torque': 'position + attitude + rate control',
    'direct_actuator': 'all controllers and the control allocator',
}


class LayerTest(TrajectoryNode):

    def __init__(self, args):
        base = trajectory_args(['--kind', 'hover', '--altitude', str(args.altitude), '--events', args.events,
                                '--speed', str(args.speed), '--settle', '6'])
        super().__init__(base)
        self.test = args
        self.att = None
        self._wanted['/fmu/out/vehicle_attitude'] = (VehicleAttitude, self._on_att)

        # Versioned input topics carry a `_vN` suffix as well; resolved once the agent is up
        self._inputs = {
            '/fmu/in/vehicle_attitude_setpoint': VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_rates_setpoint': VehicleRatesSetpoint,
            '/fmu/in/vehicle_thrust_setpoint': VehicleThrustSetpoint,
            '/fmu/in/vehicle_torque_setpoint': VehicleTorqueSetpoint,
            '/fmu/in/actuator_motors': ActuatorMotors,
        }
        self._pubs = {}
        self.result = {'level': args.level, 'bypassed': BYPASSED[args.level], 'hold_s': args.hold,
                       'px4_stamp': bool(args.px4_stamp),
                       'rate_hz': args.rate, 'closed_loop': bool(args.closed_loop)}
        self.send_every = max(1, round(self.RATE_HZ / args.rate))
        self.test_tick = 0
        self.samples = []
        self.nav_states = []

    # -- plumbing ----------------------------------------------------------

    def _on_att(self, msg):
        self.att = msg

    def _on_status(self, msg):
        if not self.nav_states or self.nav_states[-1][1] != msg.nav_state:
            self.nav_states.append((self._now(), msg.nav_state, msg.failsafe))

        super()._on_status(msg)

    def _resolve_inputs(self):
        available = [name for name, _ in self.get_topic_names_and_types()]

        for base, msg_type in self._inputs.items():
            if base in self._pubs:
                continue

            for name in available:
                suffix = name[len(base):]

                if name.startswith(base) and (suffix == '' or suffix.startswith('_v')):
                    self._pubs[base] = self.create_publisher(msg_type, name, self._qos)
                    break

    def _tilt(self) -> float:
        if self.att is None:
            return 0.0

        _, x, y, _ = self.att.q
        # angle between body z and world z
        return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))

    def _send(self, level, hover):
        # PX4 judges how old a setpoint is from this timestamp. The ROS clock is
        # Unix time; with UXRCE_DDS_SYNCT=0 that is far ahead of PX4's boot time
        # and the setpoint never looks stale, so --px4-stamp uses PX4's own clock.
        now = self._px4_time() if self.test.px4_stamp else self.get_clock().now().nanoseconds // 1000
        thrust = float(self.test.hover_thrust)

        mode = OffboardControlMode()
        mode.timestamp = now
        setattr(mode, level, True)
        self.pub_mode.publish(mode)

        if level in ('position', 'velocity', 'acceleration'):
            sp = TrajectorySetpoint()
            sp.timestamp = now
            sp.position = [float(v) for v in hover] if level == 'position' else NAN3
            sp.velocity = self._velocity_command(hover) if level == 'velocity' else NAN3
            sp.acceleration = [0.0, 0.0, 0.0] if level == 'acceleration' else NAN3
            sp.yaw = self.yaw0
            self.pub_sp.publish(sp)

        elif level == 'attitude':
            sp = VehicleAttitudeSetpoint()
            sp.timestamp = now
            sp.q_d = [math.cos(self.yaw0 / 2.0), 0.0, 0.0, math.sin(self.yaw0 / 2.0)]
            sp.thrust_body = [0.0, 0.0, -thrust]
            self._pubs['/fmu/in/vehicle_attitude_setpoint'].publish(sp)

        elif level == 'body_rate':
            sp = VehicleRatesSetpoint()
            sp.timestamp = now
            sp.thrust_body = [0.0, 0.0, -thrust]
            self._pubs['/fmu/in/vehicle_rates_setpoint'].publish(sp)

        elif level == 'thrust_and_torque':
            force = VehicleThrustSetpoint()
            force.timestamp = now
            force.xyz = [0.0, 0.0, -thrust]
            self._pubs['/fmu/in/vehicle_thrust_setpoint'].publish(force)
            torque = VehicleTorqueSetpoint()
            torque.timestamp = now
            torque.xyz = [0.0, 0.0, 0.0]
            self._pubs['/fmu/in/vehicle_torque_setpoint'].publish(torque)

        elif level == 'direct_actuator':
            sp = ActuatorMotors()
            sp.timestamp = now
            sp.control = [thrust] * 4 + [float('nan')] * 8
            self._pubs['/fmu/in/actuator_motors'].publish(sp)

    def _velocity_command(self, hover):
        if not self.test.closed_loop:
            return [0.0, 0.0, 0.0]

        # The position loop PX4 no longer runs, done here: P only, limited to 3 m/s
        error = np.asarray(hover) - np.array([self.lpos.x, self.lpos.y, self.lpos.z])
        return [float(v) for v in np.clip(self.test.kp * error, -3.0, 3.0)]

    def _sample(self, hover):
        p = np.array([self.lpos.x, self.lpos.y, self.lpos.z])
        self.samples.append((self._elapsed(), float(np.linalg.norm(p[:2] - hover[:2])), float(hover[2] - p[2]),
                             self._tilt(), float(math.hypot(self.lpos.vx, self.lpos.vy))))

    def _summarise(self, prefix):
        s = np.array(self.samples)
        self.samples = []

        if not len(s):
            return

        self.result.update({
            f'{prefix}_duration_s': round(float(s[-1, 0]), 2),
            f'{prefix}_drift_xy_max_m': round(float(s[:, 1].max()), 2),
            f'{prefix}_drift_xy_end_m': round(float(s[-1, 1]), 2),
            f'{prefix}_climb_max_m': round(float(s[:, 2].max()), 2),
            f'{prefix}_sink_max_m': round(float(-s[:, 2].min()), 2),
            f'{prefix}_tilt_max_deg': round(float(s[:, 3].max()), 1),
            f'{prefix}_speed_end_m_s': round(float(s[-1, 4]), 2),
        })

    # -- state machine -----------------------------------------------------

    def _step(self):
        if self.state not in ('TEST', 'RECOVER', 'SILENT'):
            if self.state == 'SETTLE':
                self._resolve_inputs()

                if self._elapsed() > self.args.settle and len(self._pubs) == len(self._inputs):
                    self.yaw0 = float(self.lpos.heading)
                    self.hover = self.origin + np.array([0.0, 0.0, -self.args.altitude])
                    self.nav_states = self.nav_states[-1:]
                    self._enter('TEST')

                else:
                    self._publish(self.origin + np.array([0.0, 0.0, -self.args.altitude]))

                return

            return super()._step()

        hover = self.hover
        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

        if self.state == 'TEST':
            if self.test_tick % self.send_every == 0:
                self._send(self.test.level, hover)

            self.test_tick += 1
            self._sample(hover)
            _, drift, up, tilt, _ = self.samples[-1]
            unsafe = tilt > 70.0 or drift > 20.0 or self.args.altitude + up < 1.0 or up > 25.0
            # A stream that is too slow makes PX4 drop out of offboard by itself
            dropped = self._elapsed() > 0.5 and self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD

            if self._elapsed() > self.test.hold or unsafe or dropped or not armed:
                self.result['test_cut_short'] = bool(unsafe or not armed)
                self.result['left_offboard'] = bool(dropped)
                self.result['left_offboard_after_s'] = round(self._elapsed(), 2) if dropped else None
                self.result['test_nav_states'] = ' > '.join(str(n) for _, n, _ in self.nav_states)
                self._summarise('test')

                if dropped:
                    self.result['recovered'] = None
                    self._end(armed)

                else:
                    self._enter('RECOVER')

        elif self.state == 'RECOVER':
            self._send('position', hover)
            self._sample(hover)

            if self._elapsed() > self.test.recover or not armed:
                s = np.array(self.samples)
                back = bool(armed and s[-1, 1] < 0.5 and abs(s[-1, 2]) < 0.5 and s[-1, 3] < 10.0)
                self.result['recovered'] = back
                self._summarise('recover')

                if self.test.silence > 0 and back:
                    self.nav_states = self.nav_states[-1:]
                    self._enter('SILENT')

                else:
                    self._end(armed)

        elif self.state == 'SILENT':
            # Nothing is published: the companion computer has gone quiet
            self._sample(hover)

            if self._elapsed() > self.test.silence or not armed:
                self.result['silent_nav_states'] = ' > '.join(str(n) for _, n, _ in self.nav_states)
                self.result['silent_reaction_s'] = round(self.nav_states[1][0] - self.state_t0, 2) \
                    if len(self.nav_states) > 1 else None
                self._summarise('silent')
                self._end(armed)

    def _end(self, armed):
        if armed:
            self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._enter('LAND')

        else:
            self.result['disarmed_in_flight'] = True
            self._finish()


def run_one(args):
    rclpy.init()
    node = LayerTest(args)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    node.result['exit_code'] = node.exit_code
    node.events_file.close()
    node.destroy_node()
    rclpy.shutdown()
    print('RESULT ' + json.dumps(node.result), flush=True)
    return node.result


def run_all(args):
    from .sim import Sim

    sim = Sim(world=args.world, headless=not args.gui, speed=args.speed, model=args.model)
    rows = []
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)

    runs = [(level, rate, run) for level in args.levels.split(',') for rate in args.rates for run in range(args.repeats)]

    for level, rate, run in runs:
        sim.start()
        cmd = [sys.executable, '-m', 'px4_auto_tuning.layer_test', '--level', level, '--hold', str(args.hold),
               '--recover', str(args.recover), '--silence', str(args.silence), '--speed', str(args.speed),
               '--hover-thrust', str(args.hover_thrust), '--altitude', str(args.altitude),
               '--rate', str(rate), '--kp', str(args.kp), *(['--closed-loop'] if args.closed_loop else []),
               *(['--px4-stamp'] if args.px4_stamp else []),
               '--events', os.path.join(os.path.dirname(args.out) or '.', 'raw', f'layer_{level}.csv')]
        os.makedirs(os.path.dirname(cmd[-1]), exist_ok=True)

        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=300).stdout

        except subprocess.TimeoutExpired:
            out = ''

        line = [ln for ln in out.splitlines() if ln.startswith('RESULT ')]
        row = json.loads(line[-1][7:]) if line else {'level': level, 'rate_hz': rate, 'exit_code': 'no result'}
        row['run'] = run
        row['px4_errors'] = len(sim.log_errors()[0])
        rows.append(row)
        print(row, flush=True)

        fields = list(dict.fromkeys(k for r in rows for k in r))

        with open(args.out, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

    sim.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--level', choices=LEVELS + ['all'], required=True)
    parser.add_argument('--levels', default=','.join(LEVELS), help="with --level all: the levels to run")
    parser.add_argument('--rate', default='50', help='setpoint rate during the test [Hz]; with --level all a comma list')
    parser.add_argument('--repeats', type=int, default=1, help='with --level all: runs per level and rate')
    parser.add_argument('--closed-loop', action='store_true',
                        help='velocity level: ROS runs a P position controller instead of sending zero velocity')
    parser.add_argument('--kp', type=float, default=0.95, help='gain of that controller [1/s]')
    parser.add_argument('--px4-stamp', action='store_true', help='stamp setpoints with PX4 time instead of the ROS clock')
    parser.add_argument('--hold', type=float, default=4.0, help='seconds flown at the tested level')
    parser.add_argument('--recover', type=float, default=10.0, help='seconds of position control afterwards')
    parser.add_argument('--silence', type=float, default=0.0, help='then stop publishing for this long (0: skip)')
    parser.add_argument('--hover-thrust', type=float, default=0.73, help='normalised thrust that holds the x500 in hover')
    parser.add_argument('--altitude', type=float, default=10.0)
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--model', default='x500')
    parser.add_argument('--world', default='default', help="e.g. 'windy' for a steady disturbance")
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--events', default='layer_events.csv')
    parser.add_argument('--out', default='results/layer_test.csv')
    args = parser.parse_args()

    if args.level == 'all':
        args.rates = [float(v) for v in args.rate.split(',')]
        run_all(args)

    else:
        args.rate = float(args.rate)
        result = run_one(args)
        raise SystemExit(0 if result.get('exit_code') == 0 else 1)


if __name__ == '__main__':
    main()
