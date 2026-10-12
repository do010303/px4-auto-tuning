#!/usr/bin/env python3
"""Run PX4's attitude/rate autotune the way QGroundControl does, without QGroundControl.

Against a running SITL (./run_sitl.sh, real-time speed). The steps and the
MAVLink messages are those of the QGroundControl procedure: take off, hover in
Hold mode, send MAV_CMD_DO_AUTOTUNE_ENABLE and keep resending it to read the
progress, land, and let PX4 write the new gains when it disarms
(MC_AT_APPLY = 1, the default).

    python3 -m px4_auto_tuning.qgc_autotune --out results/qgc_autotune.json
    python3 -m px4_auto_tuning.qgc_autotune --set MC_AT_SYSID_AMP=1.7
"""

import argparse
import json
import os
import subprocess
import time

from pymavlink import mavutil

from .param_client import ParamClient

PX4_DIR = os.environ.get('PX4_DIR', os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

GAINS = ['MC_ROLLRATE_K', 'MC_ROLLRATE_P', 'MC_ROLLRATE_I', 'MC_ROLLRATE_D', 'MC_ROLL_P',
         'MC_PITCHRATE_K', 'MC_PITCHRATE_P', 'MC_PITCHRATE_I', 'MC_PITCHRATE_D', 'MC_PITCH_P',
         'MC_YAWRATE_K', 'MC_YAWRATE_P', 'MC_YAWRATE_I', 'MC_YAWRATE_D', 'MC_YAW_P']
RESULTS = {mavutil.mavlink.MAV_RESULT_ACCEPTED: 'accepted', mavutil.mavlink.MAV_RESULT_IN_PROGRESS: 'in progress',
           mavutil.mavlink.MAV_RESULT_FAILED: 'failed', mavutil.mavlink.MAV_RESULT_DENIED: 'denied',
           mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED: 'rejected', mavutil.mavlink.MAV_RESULT_UNSUPPORTED: 'unsupported'}


def commander(*words):
    subprocess.run([f'{PX4_DIR}/build/px4_sitl_default/bin/px4-commander', *words], capture_output=True, timeout=20)


def wait_for(mav, kind, test, timeout):
    """Next message of `kind` for which test(msg) holds, or None after the timeout."""
    end = time.monotonic() + timeout

    while time.monotonic() < end:
        msg = mav.recv_match(type=kind, blocking=True, timeout=0.5)

        if msg is not None and test(msg):
            return msg

    return None


def poll_autotune(mav):
    """Send the autotune command once and return (result, progress) from its ack."""
    mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_DO_AUTOTUNE_ENABLE, 0,
                              1, 0, 0, 0, 0, 0, 0)
    ack = wait_for(mav, 'COMMAND_ACK', lambda m: m.command == mavutil.mavlink.MAV_CMD_DO_AUTOTUNE_ENABLE, 2.0)
    return (None, None) if ack is None else (RESULTS.get(ack.result, str(ack.result)), ack.progress)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--altitude', type=float, default=10.0, help='hover height for the tuning [m]')
    parser.add_argument('--set', action='append', default=[], metavar='NAME=VALUE', help='parameter set before take-off')
    parser.add_argument('--out', default='results/qgc_autotune.json')
    args = parser.parse_args()

    client = ParamClient()
    mav = client.mav
    extra = {name: float(value) for name, value in (item.split('=') for item in args.set)}

    for name, value in {'MIS_TAKEOFF_ALT': args.altitude, **extra}.items():
        client.set(name, value)

    before = {name: client.get(name) for name in GAINS}
    result = {'set': extra, 'apply': client.get('MC_AT_APPLY'), 'amplitude': client.get('MC_AT_SYSID_AMP'),
              'before': before, 'progress': []}

    armed = None

    # Arming is refused until the estimator is ready ("Ready for takeoff" in QGroundControl)
    for _ in range(20):
        commander('takeoff')
        armed = wait_for(mav, 'HEARTBEAT', lambda m: m.get_srcComponent() == 1 and
                         m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 3.0)

        if armed is not None:
            break

    if armed is None:
        raise SystemExit('PX4 does not arm: see the messages of the pxh> console')

    climbed = wait_for(mav, 'GLOBAL_POSITION_INT', lambda m: m.relative_alt > 1000.0 * (args.altitude - 0.5), 60.0)

    if climbed is None:
        raise SystemExit('the vehicle did not reach the hover height')

    time.sleep(8.0)     # let the hover settle, as a pilot would
    start = time.monotonic()
    last = None

    # QGroundControl resends the command about once a second and shows the progress of the ack
    while time.monotonic() - start < 90.0:
        state = poll_autotune(mav)

        if state != last:
            result['progress'].append([round(time.monotonic() - start, 1), *state])
            print(f'{time.monotonic() - start:5.1f} s  {state[0]}  {state[1]} %', flush=True)
            last = state

        # 95 %: gains found, waiting for the disarm. Anything final: done or failed
        if state[0] in ('accepted', 'failed', 'denied', 'unsupported') or state[1] == 95:
            break

        time.sleep(1.0)

    result['tuning'] = last[0] if last else None
    result['tuning_time_s'] = round(time.monotonic() - start, 1)
    commander('land')
    disarmed = wait_for(mav, 'HEARTBEAT', lambda m: m.get_srcComponent() == 1 and
                        not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 90.0)
    result['landed'] = disarmed is not None
    time.sleep(3.0)     # MC_AT_APPLY = 1 writes the gains at this point
    after = {name: client.get(name) for name in GAINS}
    result['after'] = after
    result['changed'] = {name: [round(before[name], 5), round(after[name], 5)] for name in GAINS
                         if abs(after[name] - before[name]) > 1e-6}
    print(json.dumps({key: result[key] for key in ('tuning', 'tuning_time_s', 'landed', 'changed')}, indent=2))

    with open(args.out, 'w') as f:
        json.dump(result, f, indent=2)


if __name__ == '__main__':
    main()
