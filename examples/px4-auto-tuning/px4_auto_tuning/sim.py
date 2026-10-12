"""Start, stop and restart the SITL stack (PX4 + Gazebo + DDS agent)."""

import os
import signal
import subprocess
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PX4_LOG = os.path.join(HERE, 'run', 'px4.log')


class Sim:

    def __init__(self, world='default', headless=True, env=None, attach=False, speed=1.0, model='x500'):
        # attach: the user runs ./run_sitl.sh in their own terminal; never
        # start, stop or restart it from here
        self.attach = attach
        self.started = False
        self.env = dict(os.environ, PX4_GZ_WORLD=world, PX4_DAEMON='1', **(env or {}))

        if headless:
            self.env['HEADLESS'] = '1'

        self.env['PX4_SIM_MODEL'] = f'gz_{model}'

        if speed != 1.0:
            self.env['PX4_SIM_SPEED_FACTOR'] = str(speed)

        self.proc = None

    def start(self, timeout=240.0):
        if self.attach:
            if self.started:
                raise SystemExit('the flight failed and --attach cannot restart SITL: '
                                 'restart ./run_sitl.sh by hand, then run this again')

            self.started = True
            return

        # Restarting kills every SITL on the machine, including one the user
        # opened by hand with ./run_sitl.sh (that PX4 runs without -d)
        manual = subprocess.run(['pgrep', '-f', r'^\S*/px4_sitl_default/bin/px4 /'],
                                capture_output=True, text=True).stdout.split()

        if manual:
            raise SystemExit(f'a SITL started by hand is running (pid {" ".join(manual)}): '
                             'add --attach to use it, or close it first')

        self.stop()

        # Stale log would look "ready"; stored parameters would carry gains
        # from the previous run into this one
        rootfs = os.path.join(HERE, 'run', 'rootfs')
        stale = [PX4_LOG] + [os.path.join(rootfs, f) for f in ('parameters.bson', 'parameters_backup.bson')]

        for path in stale:
            if os.path.exists(path):
                os.remove(path)

        self.proc = subprocess.Popen([os.path.join(HERE, 'run_sitl.sh')], env=self.env,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                     start_new_session=True)
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError('run_sitl.sh exited during startup, see run/px4.log')

            # "home set" is the last thing PX4 prints once the EKF has a position
            if os.path.exists(PX4_LOG) and b'home set' in open(PX4_LOG, 'rb').read():
                time.sleep(3.0)
                return

            time.sleep(1.0)

        raise TimeoutError('SITL did not become ready')

    def stop(self):
        if self.attach:
            return

        if self.proc is not None and self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGTERM)

            try:
                self.proc.wait(timeout=10)

            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)

        self.proc = None

        # Leftovers of this or an earlier run. Patterns are anchored so that
        # they cannot match an unrelated command line that mentions them.
        for pattern in (r'^\S*/px4_sitl_default/bin/px4 -d', r'^gz sim .* -s ', r'MicroXRCEAgent udp4 -p 8888$'):
            subprocess.call(['pkill', '-f', pattern])

        time.sleep(2.0)

    def log_errors(self, since=0):
        """PX4 error lines printed after byte offset `since`; returns (lines, new offset)."""
        # In attach mode PX4 prints to the user's terminal, not to the log
        if self.attach or not os.path.exists(PX4_LOG):
            return [], 0

        with open(PX4_LOG, 'rb') as f:
            f.seek(since)
            data = f.read()

        lines = [line.decode(errors='replace').strip() for line in data.splitlines() if b'ERROR' in line]
        return lines, since + len(data)
