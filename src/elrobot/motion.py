"""Joint-space trajectories with a trapezoidal speed profile."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Trajectory:
    """A path through joint waypoints, traversed with one trapezoidal speed profile over its
    whole length: a third of the time accelerating, a third cruising, a third decelerating.

    The duration is the shortest that keeps the cruise speed (1.5 L/T) and the acceleration
    (4.5 L/T^2) within the limits. The servos follow a moving setpoint with a lag that grows with
    speed, and the arm is declared blocked when that lag stays large, so what matters is how fast
    the profile ever goes: for the same move time this peaks 20 % lower than a minimum-jerk
    profile does.
    """

    waypoints: np.ndarray  # (N, dof), N >= 2
    duration: float
    _cum: np.ndarray  # (N,) cumulative path length (max-norm in joint space), _cum[-1] == length

    @classmethod
    def through(cls, waypoints, max_speed: float, max_accel: float, min_duration: float = 0.25) -> "Trajectory":
        W = np.asarray(waypoints, dtype=float)
        if W.ndim != 2 or len(W) < 2:
            raise ValueError("need at least two waypoints")
        seg = np.max(np.abs(np.diff(W, axis=0)), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        length = float(cum[-1])
        duration = max(min_duration, 1.5 * length / max_speed, math.sqrt(4.5 * length / max_accel))
        return cls(waypoints=W, duration=duration, _cum=cum)

    @property
    def length(self) -> float:
        return float(self._cum[-1])

    @property
    def start(self) -> np.ndarray:
        return self.waypoints[0]

    @property
    def end(self) -> np.ndarray:
        return self.waypoints[-1]

    def sample(self, t: float) -> np.ndarray:
        if self.length <= 1e-9:
            return self.waypoints[-1].copy()
        T, r = self.duration, self.duration / 3
        t = min(T, max(0.0, t))
        v = self.length / (T - r)  # cruise speed
        if t < r:
            s = 0.5 * v / r * t * t
        elif t <= T - r:
            s = v * (t - 0.5 * r)
        else:
            s = self.length - 0.5 * v / r * (T - t) ** 2
        i = int(np.searchsorted(self._cum, s, side="right")) - 1
        i = min(max(i, 0), len(self.waypoints) - 2)
        span = self._cum[i + 1] - self._cum[i]
        f = 0.0 if span <= 1e-12 else (s - self._cum[i]) / span
        return self.waypoints[i] + f * (self.waypoints[i + 1] - self.waypoints[i])


def interpolate_joint(q0, q1, max_step: float = 0.05) -> np.ndarray:
    """Straight line in joint space, sampled finely enough for clearance checks."""
    q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
    n = max(2, int(math.ceil(float(np.max(np.abs(q1 - q0))) / max_step)) + 1)
    return np.linspace(q0, q1, n)
