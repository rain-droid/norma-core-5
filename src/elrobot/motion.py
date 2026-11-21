"""Joint-space trajectories with smooth (minimum-jerk) time scaling."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def min_jerk(tau: float) -> float:
    """Position fraction for normalised time tau in [0, 1]: zero velocity and acceleration at both ends."""
    tau = min(1.0, max(0.0, tau))
    return tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau)


# Peak velocity and acceleration of the min-jerk profile for unit distance in unit time.
_PEAK_VEL = 1.875
_PEAK_ACC = 5.7735


@dataclass
class Trajectory:
    """A path through joint waypoints, traversed with one min-jerk profile over its whole length."""

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
        duration = max(
            min_duration,
            _PEAK_VEL * length / max_speed,
            math.sqrt(_PEAK_ACC * length / max_accel),
        )
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
        s = min_jerk(t / self.duration) * self.length
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
