#!/usr/bin/env python3
"""Draw the setpoint and the flown path into the Gazebo GUI while flying.

  * blue line: the position setpoints ROS sends to PX4 (/fmu/in/trajectory_setpoint)
  * coloured line: where the vehicle really went (Gazebo ground truth), coloured
    by attitude shaking (default) or by distance to the current setpoint:
    green = calm / on track, yellow, red = shaking / far off

Every new arming starts a fresh drawing. Runs until stopped; ./run_sitl.sh
starts it by itself when the Gazebo GUI is shown (NO_VIZ=1 to disable).

    python3 -m px4_auto_tuning.gz_viz                       # against a running SITL
    python3 -m px4_auto_tuning.gz_viz --color error --keep  # colour by tracking error, keep old flights
    python3 -m px4_auto_tuning.gz_viz --width 0             # 1 px lines instead of ribbons
    python3 -m px4_auto_tuning.gz_viz --clear               # remove the drawings and exit

Needs the MarkerManager GUI plugin, which is in Gazebo's default GUI layout.
"""

import argparse
import collections
import math
import multiprocessing
import os
import queue
import threading

# Gazebo's generated Python messages predate the protobuf release installed in
# ~/.local; the pure-Python implementation still accepts them
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')
# Same interface as the simulator started by run_sitl.sh
os.environ.setdefault('GZ_IP', '127.0.0.1')

import numpy as np
import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.marker_v_pb2 import Marker_V
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition, VehicleStatus

NS = 'px4_auto_tuning'
CHUNK = 120            # points per line marker; finished chunks are never sent again
LEVELS = 6             # colour steps: Gazebo draws a line marker in one colour only
MIN_STEP = 0.05        # m between drawn points

SCALES = {             # value at which the colour turns fully red, and the unit shown
    'vibration': (0.3, 'rad/s'),
    'error': (1.0, 'm'),
}


def send_markers(batches):
    """Body of the sender process: passes marker batches on to the Gazebo GUI.

    It is a process of its own because a request made by the Python binding
    keeps the interpreter locked while it waits; a sensor message arriving in
    that moment blocks Gazebo's receive thread, the answer is never read and
    every request runs into its timeout. This process subscribes to nothing.
    """
    node = GzNode()
    parent = os.getppid()

    while os.getppid() == parent:
        try:
            data = batches.get(timeout=1.0)

        except queue.Empty:
            continue

        if data is None:
            break

        batch = Marker_V()
        batch.ParseFromString(data)
        node.request('/marker_array', batch, Marker_V, Boolean, 1000)


def ned_to_enu(v):
    return np.array([v[1], v[0], -v[2]])


def ribbon(points, width, upright=False):
    """Corner points of a band along `points`, as a triangle strip.

    The band lies flat, or stands on its edge with `upright`. The strip runs to
    the end and back again, so it has a face on both sides.
    """
    xyz = np.array(points, dtype=float)
    side = np.zeros((len(xyz), 3))

    if upright:
        side[:, 2] = 1.0

    else:
        last = np.array([0.0, 1.0])

        for i, d in enumerate(np.gradient(xyz[:, :2], axis=0)):
            norm = np.linalg.norm(d)

            # Climbing straight up has no heading: keep the previous direction
            if norm > 1e-3:
                last = np.array([-d[1], d[0]]) / norm

            side[i, :2] = last

    pairs = [(p + 0.5 * width * n, p - 0.5 * width * n) for p, n in zip(xyz, side)]
    return [corner for pair in pairs + pairs[::-1] for corner in pair]


def ramp_color(fraction):
    """Green -> yellow -> red for 0 -> 0.5 -> 1, in LEVELS steps."""
    f = round(min(max(fraction, 0.0), 1.0) * (LEVELS - 1)) / (LEVELS - 1)
    return (min(1.0, 2.0 * f), min(1.0, 2.0 * (1.0 - f)), 0.0)


class Viz(Node):

    def __init__(self, args):
        super().__init__('gz_viz')
        self.args = args
        self.gz = GzNode()
        self.lock = threading.Lock()
        self.flight = 0
        self.armed = False

        self.truth = None                       # latest Gazebo position of the vehicle (ENU)
        self.offset = None                      # Gazebo ENU minus PX4 local position (as ENU)
        self.setpoint = None                    # latest setpoint, Gazebo ENU
        self.rates = collections.deque()        # (time, |high-frequency tilt rate|)
        self.rate_slow = np.zeros(2)
        self.rate_t = None

        self.batch = Marker_V()                 # markers changed in this drawing step
        context = multiprocessing.get_context('spawn')
        self.batches = context.Queue()
        self.sender = context.Process(target=send_markers, args=(self.batches,), daemon=True)
        self.sender.start()
        self.path = []                          # flown points of the current chunk: (xyz, rgb)
        self.path_chunk = 0
        self.sp_path = []
        self.sp_chunk = 0
        self.dirty = False
        self.stats = []

        self.gz.subscribe(Pose_V, f'/world/{args.world}/dynamic_pose/info', self._on_poses)
        self.gz.subscribe(IMU, f'/world/{args.world}/model/{args.model}/link/base_link/sensor/imu_sensor/imu', self._on_imu)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        sp_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
        self._wanted = {
            '/fmu/out/vehicle_local_position': (VehicleLocalPosition, self._on_lpos, qos),
            '/fmu/out/vehicle_status': (VehicleStatus, self._on_status, qos),
            '/fmu/in/trajectory_setpoint': (TrajectorySetpoint, self._on_setpoint, sp_qos),
        }
        self._subscribed = set()
        self.lpos = None
        self.create_timer(1.0, self._resolve)
        self.create_timer(1.0 / args.rate, self._draw)

    # -- inputs ------------------------------------------------------------

    def _resolve(self):
        """PX4 topics carry a `_vN` suffix that depends on the message version."""
        for name, _ in self.get_topic_names_and_types():
            for base, (msg_type, callback, qos) in self._wanted.items():
                suffix = name[len(base):]

                if base not in self._subscribed and name.startswith(base) and (suffix == '' or suffix.startswith('_v')):
                    self.create_subscription(msg_type, name, callback, qos)
                    self._subscribed.add(base)

    def _on_poses(self, msg):
        for pose in msg.pose:
            if pose.name == self.args.model:
                with self.lock:
                    self.truth = np.array([pose.position.x, pose.position.y, pose.position.z])
                return

    def _on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        rate = np.array([msg.angular_velocity.x, msg.angular_velocity.y])

        with self.lock:
            if self.rate_t is not None and 0.0 < t - self.rate_t < 0.2:
                dt = t - self.rate_t
                # Shaking = tilt rate minus its slow part (same measure as online_tuner)
                self.rate_slow += dt / (0.5 + dt) * (rate - self.rate_slow)
                self.rates.append((t, float(np.linalg.norm(rate - self.rate_slow))))

                while self.rates and t - self.rates[0][0] > 0.5:
                    self.rates.popleft()

            self.rate_t = t

    def _on_lpos(self, msg):
        self.lpos = msg

        with self.lock:
            # The local frame starts where the EKF did; tie it to Gazebo while on the ground
            if not self.armed and self.truth is not None:
                self.offset = self.truth - ned_to_enu([msg.x, msg.y, msg.z])

    def _on_status(self, msg):
        armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

        if armed and not self.armed:
            self._new_flight()

        elif self.armed and not armed:
            self._report()

        self.armed = armed

    def _on_setpoint(self, msg):
        if self.offset is None or any(math.isnan(v) for v in msg.position):
            return

        with self.lock:
            self.setpoint = self.offset + ned_to_enu(msg.position)

    # -- drawing -----------------------------------------------------------

    def _shaking(self):
        values = [v for _, v in self.rates]
        return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0

    def _new_flight(self):
        with self.lock:
            if not self.args.keep:
                self.clear()

            self.flight += 1
            self.path, self.sp_path = [], []
            self.path_chunk = self.sp_chunk = 0
            self.stats = []

        self.get_logger().info(f'flight {self.flight}: drawing')

    def _report(self):
        if self.stats:
            s = np.array(self.stats)
            unit = SCALES[self.args.color][1]
            self.get_logger().info(f'flight {self.flight} landed: {self.args.color} mean {s.mean():.3f}, '
                                   f'max {s.max():.3f} {unit}')

    def _draw(self):
        with self.lock:
            if not self.armed or self.truth is None:
                return

            position = self.truth.copy()
            shaking = self._shaking()
            setpoint = None if self.setpoint is None else self.setpoint.copy()

        value = shaking if self.args.color == 'vibration' else (
            float(np.linalg.norm(position - setpoint)) if setpoint is not None else 0.0)
        self.stats.append(value)
        color = ramp_color(value / SCALES[self.args.color][0])

        if not self.path or np.linalg.norm(position - self.path[-1][0]) >= MIN_STEP:
            if self.path and (color != self.path[-1][1] or len(self.path) >= CHUNK):
                # A marker has one colour: start a new one where this one ended,
                # so the line stays continuous
                self.path_chunk += 1
                self.path = [(self.path[-1][0], color)]

            self.path.append((position, color))
            self._send_line(f'path_{self.path_chunk}', self.path, self.args.width)

        if setpoint is not None and (not self.sp_path or np.linalg.norm(setpoint - self.sp_path[-1][0]) >= MIN_STEP):
            self.sp_path.append((setpoint, (0.2, 0.4, 1.0)))
            # Narrower than the flown path, plus a fin standing on it: the setpoint
            # stays visible whether the vehicle flies just above or just below it
            self._send_line(f'sp_{self.sp_chunk}', self.sp_path, 0.4 * self.args.width)

            if self.args.width > 0.0:
                self._send_line(f'fin_{self.sp_chunk}', self.sp_path, self.args.width, upright=True)

            if len(self.sp_path) >= CHUNK:
                self.sp_chunk += 1
                self.sp_path = [self.sp_path[-1]]

        self._flush()

    def _flush(self):
        """Hand all changed markers to the sender process as one request."""
        if len(self.batch.marker):
            self.batches.put(self.batch.SerializeToString())
            self.batch = Marker_V()

    def close(self):
        """Let the sender finish what is queued."""
        self.batches.put(None)
        self.sender.join(timeout=5.0)

    def _send_line(self, name, points, width=0.0, upright=False):
        if len(points) < 2:
            return

        corners = [xyz for xyz, _ in points]

        if width > 0.0:
            corners = ribbon(corners, width, upright)

        marker = self.batch.marker.add()
        marker.ns = NS
        kind, chunk = name.split('_')
        marker.id = (self.flight * 100000 + ('path', 'sp', 'fin').index(kind) * 30000 + int(chunk) % 30000) % (2 ** 31)
        marker.action = Marker.ADD_MODIFY
        marker.type = Marker.TRIANGLE_STRIP if width > 0.0 else Marker.LINE_STRIP
        marker.visibility = Marker.GUI

        for xyz in corners:
            p = marker.point.add()
            p.x, p.y, p.z = (float(v) for v in xyz)

        rgb = points[-1][1]

        # Emissive only: the colour then does not depend on which way a face looks at the sun
        for c, gain in ((marker.material.ambient, 0.0), (marker.material.diffuse, 0.0), (marker.material.emissive, 1.0)):
            c.r, c.g, c.b, c.a = gain * rgb[0], gain * rgb[1], gain * rgb[2], 1.0

    def clear(self):
        self.batch = Marker_V()
        marker = self.batch.marker.add()
        marker.ns = NS
        marker.action = Marker.DELETE_ALL
        self._flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--world', default='default')
    parser.add_argument('--model', default='x500_0', help='model instance name in Gazebo')
    parser.add_argument('--color', choices=sorted(SCALES), default='vibration')
    parser.add_argument('--rate', type=float, default=10.0, help='drawing updates per second')
    parser.add_argument('--width', type=float, default=0.08,
                        help='width of the flown path [m]; 0 draws 1 px lines')
    parser.add_argument('--keep', action='store_true', help='keep the drawings of earlier flights')
    parser.add_argument('--clear', action='store_true', help='remove all drawings and exit')
    args = parser.parse_args()

    rclpy.init()
    node = Viz(args)

    if args.clear:
        node.clear()
        node.close()
        node.destroy_node()
        rclpy.shutdown()
        return

    scale, unit = SCALES[args.color]
    node.get_logger().info(f'colour by {args.color}: green 0, yellow {scale / 2:g}, red {scale:g} {unit} and above')

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.close()
    node.destroy_node()

    try:
        rclpy.shutdown()

    except Exception:
        pass


if __name__ == '__main__':
    main()
