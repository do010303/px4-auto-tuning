"""Closed, periodic reference trajectories in the NED local frame.

Every trajectory is a function of a *phase time* ``s`` (seconds along the
nominal path). The caller is free to warp ``s`` (e.g. to ramp the speed up
from hover), so velocity and acceleration are obtained numerically from the
warped path rather than from closed-form derivatives.
"""

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Trajectory:
    kind: str = 'figure8'
    period: float = 20.0      # s per lap at full speed
    size_x: float = 8.0       # m, half-extent along north
    size_y: float = 4.0       # m, half-extent along east
    altitude: float = 5.0     # m above the local origin (positive up)
    waves: int = 3            # sine only: number of sine periods along the leg

    def position(self, s: float) -> np.ndarray:
        """NED position at phase time ``s``. Starts at the origin for s=0."""
        w = 2.0 * math.pi / self.period

        if self.kind == 'figure8':
            # Lissajous 1:2 — crosses the origin twice per lap
            x = self.size_x * math.sin(w * s)
            y = self.size_y * math.sin(2.0 * w * s)

        elif self.kind == 'sine':
            # Out-and-back leg of length 2*size_x, weaving in east. The leg
            # coordinate is a raised cosine so the turn-around is smooth and
            # the return leg retraces the outbound one.
            x = self.size_x * (1.0 - math.cos(w * s))
            y = self.size_y * math.sin(self.waves * math.pi * x / self.size_x)

        elif self.kind == 'hover':
            x, y = 0.0, 0.0

        else:
            raise ValueError(f'unknown trajectory kind: {self.kind}')

        return np.array([x, y, -self.altitude])


class SpeedRamp:
    """Maps wall time to phase time with a smooth start and stop.

    Phase rate goes 0 -> 1 over ``ramp`` seconds (smoothstep), stays at 1, and
    goes back to 0 so that the vehicle starts and ends at rest on the path.
    """

    def __init__(self, laps: int, period: float, ramp: float = 4.0):
        self.ramp = ramp
        # A smoothstep ramp covers ramp/2 seconds of phase; two of them cover
        # `ramp`. The cruise part makes up the rest of the requested laps.
        self.cruise = max(laps * period - ramp, 0.0)
        self.duration = 2.0 * ramp + self.cruise

    @staticmethod
    def _smooth_int(u: float) -> float:
        # integral of smoothstep 3u^2 - 2u^3 from 0 to u
        u = min(max(u, 0.0), 1.0)
        return u ** 3 - 0.5 * u ** 4

    def phase(self, t: float) -> float:
        t = min(max(t, 0.0), self.duration)
        r = self.ramp

        if t < r:
            return r * self._smooth_int(t / r)

        if t < r + self.cruise:
            return 0.5 * r + (t - r)

        u = (t - r - self.cruise) / r
        return 0.5 * r + self.cruise + r * (u - self._smooth_int(u))


def sample(traj: Trajectory, ramp: SpeedRamp, t: float, h: float = 5e-3):
    """Position, velocity and acceleration at wall time ``t`` (central differences)."""
    p0 = traj.position(ramp.phase(t - h))
    p1 = traj.position(ramp.phase(t))
    p2 = traj.position(ramp.phase(t + h))
    vel = (p2 - p0) / (2.0 * h)
    acc = (p2 - 2.0 * p1 + p0) / (h * h)
    return p1, vel, acc
