#!/usr/bin/env python3
"""Fly a sine or figure-8 reference in PX4 offboard mode (ROS 2 / uXRCE-DDS).

Sequence: stream setpoints -> OFFBOARD + arm -> climb -> settle -> N laps ->
settle -> land. Every phase change is written to an events CSV stamped with
PX4 time, so a ulog can be cut into laps afterwards.

    python3 -m px4_auto_tuning.trajectory_node --kind figure8 --laps 3
"""

import argparse
import csv
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TimesyncStatus,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

from .trajectories import SpeedRamp, Trajectory, sample

NAN3 = [float('nan')] * 3


class TrajectoryNode(Node):

    RATE_HZ = 50.0

    def __init__(self, args):
        super().__init__('trajectory_node')
        self.args = args
        self.traj = Trajectory(kind=args.kind, period=args.period, size_x=args.size_x,
                               size_y=args.size_y, altitude=args.altitude, waves=args.waves)
        self.ramp = SpeedRamp(args.laps, args.period, args.ramp)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        # Versioned messages get a `_vN` topic suffix; resolve it at runtime
        self._qos = qos
        self._subs = {}
        self._wanted = {
            '/fmu/out/vehicle_local_position': (VehicleLocalPosition, self._on_lpos),
            '/fmu/out/vehicle_status': (VehicleStatus, self._on_status),
            '/fmu/out/vehicle_land_detected': (VehicleLandDetected, self._on_land),
            '/fmu/out/timesync_status': (TimesyncStatus, self._on_timesync),
        }

        self.lpos = None
        self.status = None
        self.time_offset = None   # PX4 boot time minus the (Unix) time stamped on DDS messages
        self.landed = True

        self.state = 'WAIT'
        self.state_t0 = 0.0
        self.tick = 0
        self.origin = np.zeros(3)
        self.done = False

        self.events_file = open(args.events, 'w', newline='')
        self.events = csv.writer(self.events_file)
        self.events.writerow(['px4_boot_time_us', 'event', 'detail'])

        self.timer = self.create_timer(1.0 / self.RATE_HZ, self._step)

    # -- subscriptions -----------------------------------------------------

    def _resolve_topics(self):
        available = [name for name, _ in self.get_topic_names_and_types()]

        for base, (msg_type, callback) in self._wanted.items():
            if base in self._subs:
                continue

            for name in available:
                suffix = name[len(base):]

                if name.startswith(base) and (suffix == '' or suffix.startswith('_v')):
                    self._subs[base] = self.create_subscription(msg_type, name, callback, self._qos)
                    self.get_logger().info(f'subscribed to {name}')
                    break

    def _on_lpos(self, msg):
        self.lpos = msg

    def _on_status(self, msg):
        self.status = msg

    def _on_timesync(self, msg):
        self.time_offset = int(msg.estimated_offset)

    def _on_land(self, msg):
        self.landed = msg.landed

    # -- helpers -----------------------------------------------------------

    def _px4_time(self) -> int:
        # uXRCE-DDS rewrites timestamps to the agent's clock; undo that so the
        # events line up with the ulog, which is in PX4 boot time
        if self.lpos is None or self.time_offset is None:
            return 0

        return int(self.lpos.timestamp) + self.time_offset

    def _event(self, name: str, detail: str = ''):
        self.events.writerow([self._px4_time(), name, detail])
        self.events_file.flush()
        self.get_logger().info(f'{name} {detail}')

    def _now(self) -> float:
        # PX4 time, so the reference stays in step with the simulation when it
        # runs faster or slower than real time
        return self.lpos.timestamp * 1e-6 if self.lpos is not None else time.monotonic()

    def _enter(self, state: str):
        self.state = state
        self.state_t0 = self._now()
        self._event(state)

    def _elapsed(self) -> float:
        return self._now() - self.state_t0

    def _command(self, command: int, p1: float = 0.0, p2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)

    def _publish(self, pos, vel=None, acc=None):
        now = self.get_clock().now().nanoseconds // 1000

        mode = OffboardControlMode()
        mode.timestamp = now
        mode.position = True
        self.pub_mode.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = now
        sp.position = [float(v) for v in pos]
        sp.velocity = [float(v) for v in vel] if vel is not None else NAN3
        sp.acceleration = [float(v) for v in acc] if acc is not None else NAN3
        sp.yaw = float(self.args.yaw)
        self.pub_sp.publish(sp)

    # -- state machine -----------------------------------------------------

    def _step(self):
        self.tick += 1

        if self.state == 'WAIT':
            self._resolve_topics()

            if self.lpos is not None and self.status is not None and self.time_offset is not None \
               and self.lpos.xy_valid:
                # Path coordinates are relative to where the vehicle sits now
                self.origin = np.array([self.lpos.x, self.lpos.y, self.lpos.z])
                self._enter('ARM')

            return

        hover = self.origin + np.array([0.0, 0.0, -self.args.altitude])

        if self.state == 'ARM':
            self._publish(hover)

            # PX4 needs a setpoint stream before it accepts OFFBOARD
            if self._elapsed() > 1.0 and self.tick % 25 == 0:
                armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                offboard = self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

                if not offboard:
                    self._command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

                elif not armed:
                    self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

                else:
                    self._enter('CLIMB')

            if self._elapsed() > 60.0:
                self._abort('could not arm / enter offboard')

        elif self.state == 'CLIMB':
            self._publish(hover)

            if abs(self.lpos.z - hover[2]) < 0.3 and abs(self.lpos.vz) < 0.2:
                self._enter('SETTLE')

            if self._elapsed() > 60.0:
                self._abort('climb timeout')

        elif self.state == 'SETTLE':
            self._publish(hover)

            if self._elapsed() > self.args.settle:
                self.lap = -1
                self._enter('TRACK')

        elif self.state == 'TRACK':
            t = self._elapsed()
            pos, vel, acc = sample(self.traj, self.ramp, t)
            pos[:2] += self.origin[:2]
            pos[2] = hover[2]

            if self.args.no_ff:
                self._publish(pos)

            else:
                self._publish(pos, vel, acc)

            if np.linalg.norm(pos - np.array([self.lpos.x, self.lpos.y, self.lpos.z])) > self.args.max_error:
                self._abort('diverged from the reference')

            lap = int(self.ramp.phase(t) // self.args.period)

            if lap != self.lap and lap < self.args.laps:
                self.lap = lap
                self._event('LAP', str(lap))

            if t >= self.ramp.duration:
                self._enter('SETTLE_END')

        elif self.state == 'SETTLE_END':
            self._publish(hover)

            if self._elapsed() > self.args.settle:
                if self.args.no_land:
                    self._finish()

                else:
                    self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self._enter('LAND')

        elif self.state == 'LAND':
            if self._elapsed() > 2.0 and self.tick % 50 == 0 and \
               self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

            if self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self._finish()

            if self._elapsed() > 90.0:
                self._abort('land timeout')

        # Safety net during any airborne phase
        if self.state in ('CLIMB', 'SETTLE', 'TRACK', 'SETTLE_END') and \
           self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD and self._elapsed() > 1.0:
            self._abort(f'left offboard (nav_state={self.status.nav_state})')

    def _finish(self):
        self._enter('DONE')
        self.done = True

    def _abort(self, reason: str):
        self._event('ABORT', reason)
        self.exit_code = 1
        self.done = True

    exit_code = 0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--kind', choices=['figure8', 'sine', 'hover'], default='figure8')
    parser.add_argument('--laps', type=int, default=3)
    parser.add_argument('--period', type=float, help='seconds per lap')
    parser.add_argument('--size-x', type=float, help='half extent north [m]')
    parser.add_argument('--size-y', type=float, help='half extent east [m]')
    parser.add_argument('--waves', type=int, default=2, help='sine only: sine periods along the leg')
    parser.add_argument('--altitude', type=float, default=5.0)
    parser.add_argument('--yaw', type=float, default=0.0, help='constant heading [rad]')
    parser.add_argument('--ramp', type=float, default=4.0, help='speed ramp duration [s]')
    parser.add_argument('--settle', type=float, default=5.0, help='hover before/after the laps [s]')
    parser.add_argument('--max-error', type=float, default=10.0, help='abort when this far off the reference [m]')
    parser.add_argument('--no-ff', action='store_true', help='send position only (no velocity/acceleration feedforward)')
    parser.add_argument('--no-land', action='store_true', help='keep hovering in offboard when done')
    parser.add_argument('--events', default='events.csv', help='phase/lap marker file (PX4 time)')
    args = parser.parse_args(argv)

    # Peaks: figure8 about 3.6 m/s and 1.8 m/s^2, sine about 2.6 m/s and 2.5 m/s^2
    defaults = {'figure8': (20.0, 8.0, 4.0), 'sine': (30.0, 8.0, 1.5), 'hover': (20.0, 0.0, 0.0)}[args.kind]

    for name, value in zip(('period', 'size_x', 'size_y'), defaults):
        if getattr(args, name) is None:
            setattr(args, name, value)

    return args


def main(argv=None):
    args = parse_args(argv)
    rclpy.init()
    node = TrajectoryNode(args)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.exit_code = 130

    code = node.exit_code
    node.events_file.close()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
