import math
import time
from pathlib import Path

import numpy as np

from elrobot.kinematics import Kinematics, approach_from_pitch, jaw_from_yaw

URDF = Path(__file__).resolve().parents[1] / "assets/elrobot/elrobot_follower.urdf"
kin = Kinematics(URDF)


def test_zero_pose_matches_mesh_measurements():
    T = kin.fk(np.zeros(7))
    # gripper base origin measured from the URDF at (0.003, 0.2596, 0.2189); TCP is 91.5 mm further out
    assert np.allclose(T[:3, 3], [0.0047, 0.351, 0.218], atol=0.004), T[:3, 3]
    # at zero the gripper points forward (+y) and the jaws close along x
    assert T[:3, 2] @ np.array([0, 1, 0]) > 0.999
    assert abs(T[:3, 0] @ np.array([1, 0, 0])) > 0.999
    assert abs(np.linalg.det(T[:3, :3]) - 1) < 1e-9


def test_straight_line_holds_position_and_direction_both_ways():
    """A descent and the retreat back up it. Solving each waypoint on its own used to refuse the
    way back ("would have to flip configuration") after planning the way down."""
    a = approach_from_pitch([-0.06, 0.20], 90)
    low = np.array([-0.06, 0.20, 0.032])
    high = low - a * 0.07
    start = kin.ik(high, approach=a, jaw=jaw_from_yaw(a, 0), q_seed=np.radians([0, -21, -29, 36, 0, 73.5, 0]))
    assert start.ok

    def follow(q0, origin, goal):
        path, r = kin.ik_line(q0, goal, approach=a)
        assert r.ok, r.describe()
        assert np.array_equal(path[0], q0) and kin.within_limits(path[-1])
        assert np.max(np.abs(np.diff(path, axis=0))) < 0.2, "no jumps between waypoints"
        along = (goal - origin) / np.linalg.norm(goal - origin)
        for q in path:
            T = kin.fk(q)
            d = T[:3, 3] - origin
            assert np.linalg.norm(d - (d @ along) * along) < 0.002 and T[:3, 2] @ a > math.cos(math.radians(4))
        assert np.linalg.norm(kin.fk(path[-1])[:3, 3] - goal) < 0.002
        return path[-1]

    follow(follow(start.q, high, low), low, high)


def test_straight_line_out_of_reach_stops_and_says_where():
    start = kin.ik([0.0, 0.25, 0.10], approach=approach_from_pitch([0.0, 0.25], 60), q_seed=np.zeros(7))
    assert start.ok
    path, r = kin.ik_line(start.q, [0.0, 0.60, 0.10])
    assert not r.ok and "stuck" in r.reason
    assert 1 <= len(path) < 45  # 35 cm at 8 mm a step would be 45 waypoints
    assert np.linalg.norm(kin.fk(path[-1])[:3, 3] - [0.0, 0.25, 0.10]) < 0.30


def test_ik_recovers_reachable_poses():
    rng = np.random.default_rng(2)
    ok = 0
    n = 40
    for _ in range(n):
        q_true = rng.uniform(kin.lower + 0.2, kin.upper - 0.2)
        T = kin.fk(q_true)
        r = kin.ik(T[:3, 3], approach=T[:3, 2], jaw=T[:3, 0], q_seed=np.zeros(7))
        if r.ok:
            ok += 1
            Ts = kin.fk(r.q)
            assert np.linalg.norm(Ts[:3, 3] - T[:3, 3]) < 0.003
            assert kin.within_limits(r.q)
    assert ok >= n * 0.9, f"only {ok}/{n} solved"


def test_top_down_grasp_region():
    """Top-down grasps near the table work in the area close in front of the base."""
    solved = []
    for x in np.linspace(-0.12, 0.12, 5):
        for y in np.linspace(0.10, 0.24, 5):
            a = approach_from_pitch([x, y], 90)
            solved.append(kin.ik([x, y, 0.03], approach=a, q_seed=np.zeros(7)).ok)
    assert sum(solved) >= len(solved) * 0.9, f"{sum(solved)}/{len(solved)}"


def test_leaning_approach_extends_reach():
    """Further out the gripper has to lean forward; 45 degrees reaches well past 30 cm."""
    for y in (0.26, 0.30, 0.34):
        a = approach_from_pitch([0.0, y], 45)
        assert kin.ik([0.0, y, 0.03], approach=a, q_seed=np.zeros(7)).ok, y


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    T = kin.fk(np.zeros(7))
    print("TCP at zero:", T[:3, 3], "approach", T[:3, 2], "jaw", T[:3, 0])
    print("joint origins at zero:\n", kin.geometry(np.zeros(7)).joints)
    print("limits deg:", np.degrees(kin.lower).round(1), np.degrees(kin.upper).round(1))
    # reach map for top-down grasps at z = 3 cm, and timing
    t0 = time.perf_counter(); n = 0
    print("\ntop-down reach (rows y far->near, cols x -0.3..0.3), o = ok, . = fail")
    for y in np.linspace(0.42, 0.06, 13):
        row = ""
        for x in np.linspace(-0.3, 0.3, 13):
            a = approach_from_pitch([x, y], 90)
            r = kin.ik([x, y, 0.03], approach=a, q_seed=np.zeros(7), random_seeds=4); n += 1
            row += "o" if r.ok else "."
        print(f"y={y:5.2f} {row}")
    dt = time.perf_counter() - t0
    print(f"{n} IK solves in {dt:.2f}s = {dt / n * 1000:.1f} ms each")
    print("\n45deg-pitch reach at z = 3 cm")
    for y in np.linspace(0.46, 0.06, 11):
        row = ""
        for x in np.linspace(-0.3, 0.3, 13):
            a = approach_from_pitch([x, y], 45)
            r = kin.ik([x, y, 0.03], approach=a, q_seed=np.zeros(7), random_seeds=4)
            row += "o" if r.ok else "."
        print(f"y={y:5.2f} {row}")
