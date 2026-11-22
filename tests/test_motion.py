import math

import numpy as np
import pytest

from elrobot.motion import Trajectory


@pytest.mark.parametrize("length", [0.02, 0.3, 2.5])  # stretched to the minimum time, acceleration-bound, speed-bound
def test_trapezoid_reaches_the_end_within_its_limits(length):
    v_max, a_max = 1.0, 4.0
    traj = Trajectory.through([[0.0, 0.0], [length, -length / 2]], v_max, a_max)
    assert traj.duration == pytest.approx(max(0.25, 1.5 * length / v_max, math.sqrt(4.5 * length / a_max)))
    t = np.linspace(0.0, traj.duration, 3001)
    s = np.array([traj.sample(x)[0] for x in t])
    assert s[0] == 0.0 and abs(s[-1] - length) < 1e-12
    assert np.allclose(traj.sample(traj.duration + 1.0), [length, -length / 2])
    vel = np.gradient(s, t)
    assert vel.min() > -1e-9, "never moves backwards"
    assert vel.max() <= v_max * 1.001
    assert np.abs(np.gradient(vel, t)).max() <= a_max * 1.02
    cruising = (t > traj.duration * 0.35) & (t < traj.duration * 0.65)
    assert np.ptp(vel[cruising]) < 1e-6 * max(1.0, vel.max()), "constant speed through the middle third"
