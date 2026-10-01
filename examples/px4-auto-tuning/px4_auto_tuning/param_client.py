"""Read and write PX4 parameters over MAVLink while the vehicle is flying.

uXRCE-DDS has no parameter service, so this is the channel used to change
gains in flight. A PARAM_SET makes PX4 publish the uORB `parameter_update`
topic; every controller re-reads its gains on the next cycle, nothing restarts.
"""

import struct
import time

from pymavlink import mavutil

# PX4 SITL "onboard" MAVLink instance sends to this UDP port (px4-rc.mavlink)
DEFAULT_URL = 'udpin:0.0.0.0:14540'

_INT32 = mavutil.mavlink.MAV_PARAM_TYPE_INT32
_REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32


def _decode(msg):
    # PX4 sends integer parameters as raw bytes inside the float field
    if msg.param_type == _INT32:
        return struct.unpack('<i', struct.pack('<f', msg.param_value))[0]

    return float(msg.param_value)


class ParamClient:

    def __init__(self, url: str = DEFAULT_URL, timeout: float = 10.0):
        self.mav = mavutil.mavlink_connection(url, source_system=250, source_component=191)

        if self.mav.wait_heartbeat(timeout=timeout) is None:
            raise TimeoutError(f'no MAVLink heartbeat on {url}')

        self._types = {}

    def _wait_value(self, name: str, timeout: float):
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            msg = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.2)

            if msg is not None and msg.param_id.rstrip('\x00') == name:
                self._types[name] = msg.param_type
                return _decode(msg)

        return None

    def get(self, name: str, timeout: float = 2.0, retries: int = 3):
        for _ in range(retries):
            self.mav.mav.param_request_read_send(
                self.mav.target_system, self.mav.target_component, name.encode(), -1)
            value = self._wait_value(name, timeout)

            if value is not None:
                return value

        raise TimeoutError(f'no PARAM_VALUE for {name}')

    def set(self, name: str, value, timeout: float = 2.0, retries: int = 3):
        """Set a parameter and return the value PX4 reports back."""
        if name not in self._types:
            self.get(name)

        ptype = self._types[name]

        if ptype == _INT32:
            wire = struct.unpack('<f', struct.pack('<i', int(value)))[0]

        else:
            wire = float(value)

        for _ in range(retries):
            self.mav.mav.param_set_send(
                self.mav.target_system, self.mav.target_component, name.encode(), wire, ptype)
            reported = self._wait_value(name, timeout)

            if reported is not None:
                return reported

        raise TimeoutError(f'no PARAM_VALUE after setting {name}')

    def set_many(self, values: dict) -> dict:
        return {name: self.set(name, value) for name, value in values.items()}


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Get/set PX4 parameters over MAVLink')
    parser.add_argument('--url', default=DEFAULT_URL)
    parser.add_argument('name')
    parser.add_argument('value', nargs='?', type=float)
    args = parser.parse_args()

    client = ParamClient(args.url)

    if args.value is None:
        print(f'{args.name} = {client.get(args.name)}')

    else:
        before = client.get(args.name)
        after = client.set(args.name, args.value)
        print(f'{args.name}: {before} -> {after}')


if __name__ == '__main__':
    main()
