#!/usr/bin/env python3
"""How late and how irregular is the uXRCE-DDS link between PX4 and ROS?

Run against a SITL at real-time speed (./run_sitl.sh); the vehicle stays on the
ground. Two measurements:

  * PX4 -> ROS: arrival times of /fmu/out/vehicle_attitude against the PX4
    timestamps inside the messages. The clocks have an unknown offset, so the
    result is the delay *above the fastest message seen* (jitter), plus the
    rate that actually arrives.
  * ROS -> PX4 -> ROS: a harmless vehicle_command (disarm, on the ground) is sent and the time until
    its vehicle_command_ack comes back is taken with one clock. This round
    trip includes the commander loop that handles the command.

    python3 -m px4_auto_tuning.dds_latency --out results/dds_latency.json
"""

import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import VehicleAttitude, VehicleCommand, VehicleCommandAck

# "Disarm" sent to a vehicle that is disarmed on the ground: the commander
# answers with an ack and nothing changes
PING_COMMAND = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM


class Probe(Node):

    def __init__(self):
        super().__init__('dds_latency')
        self.qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST, depth=10)
        self.arrivals = []      # (wall time, PX4 timestamp) of attitude messages
        self.ack_time = None
        self.subscribed = set()
        self.pub_cmd = None

    def resolve(self):
        """Versioned topics carry a `_vN` suffix; subscribe once the agent has created them."""
        wanted = {'/fmu/out/vehicle_attitude': (VehicleAttitude, self._on_att),
                  '/fmu/out/vehicle_command_ack': (VehicleCommandAck, self._on_ack)}

        for name, _ in self.get_topic_names_and_types():
            for base, (msg_type, callback) in wanted.items():
                suffix = name[len(base):]

                if name.startswith(base) and (suffix == '' or suffix.startswith('_v')) and base not in self.subscribed:
                    self.create_subscription(msg_type, name, callback, self.qos)
                    self.subscribed.add(base)

            base = '/fmu/in/vehicle_command'
            suffix = name[len(base):]

            if name.startswith(base) and (suffix == '' or suffix.startswith('_v')) and self.pub_cmd is None:
                self.pub_cmd = self.create_publisher(VehicleCommand, name, self.qos)

        return len(self.subscribed) == len(wanted) and self.pub_cmd is not None

    def _on_att(self, msg):
        self.arrivals.append((time.monotonic(), msg.timestamp * 1e-6))

    def _on_ack(self, msg):
        if msg.command == PING_COMMAND and self.ack_time is None:
            self.ack_time = time.monotonic()

    def ping(self, timeout=1.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = PING_COMMAND
        msg.param1 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.ack_time = None
        sent = time.monotonic()
        self.pub_cmd.publish(msg)

        while self.ack_time is None and time.monotonic() - sent < timeout:
            rclpy.spin_once(self, timeout_sec=0.001)

        return None if self.ack_time is None else self.ack_time - sent


def percentiles(values_ms):
    p50, p95, p99 = np.percentile(values_ms, [50, 95, 99])
    return {'p50_ms': round(float(p50), 2), 'p95_ms': round(float(p95), 2), 'p99_ms': round(float(p99), 2),
            'max_ms': round(float(np.max(values_ms)), 2)}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--listen', type=float, default=10.0, help='seconds of attitude messages to collect')
    parser.add_argument('--pings', type=int, default=100)
    parser.add_argument('--out', default='results/dds_latency.json')
    args = parser.parse_args()

    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + 30.0

    while not node.resolve():
        rclpy.spin_once(node, timeout_sec=0.2)

        if time.monotonic() > deadline:
            raise SystemExit('PX4 topics not found: is SITL and the agent running?')

    node.arrivals.clear()
    end = time.monotonic() + args.listen

    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)

    arrivals = np.array(node.arrivals)
    result = {}

    if len(arrivals) > 10:
        wall, stamp = arrivals[:, 0], arrivals[:, 1]
        delay = (wall - stamp) * 1e3
        result['px4_to_ros'] = {
            'messages': len(arrivals),
            'rate_hz': round(float((len(arrivals) - 1) / (wall[-1] - wall[0])), 1),
            'published_rate_hz': round(float((len(arrivals) - 1) / (stamp[-1] - stamp[0])), 1),
            'delay_above_fastest': percentiles(delay - delay.min()),
            'arrival_interval': percentiles(np.diff(wall) * 1e3),
        }

    trips = [node.ping() for _ in range(args.pings)]
    good = [t * 1e3 for t in trips if t is not None]
    result['ros_to_px4_to_ros'] = {'sent': args.pings, 'answered': len(good), **(percentiles(good) if good else {})}

    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(result, indent=2))

    with open(args.out, 'w') as f:
        json.dump(result, f, indent=2)


if __name__ == '__main__':
    main()
