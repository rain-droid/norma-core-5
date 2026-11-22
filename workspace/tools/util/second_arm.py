"""The second arm on the table: an SO-101 with a parallel gripper, driven by its own daemon.

That daemon is the `autopilot` branch of this repository (its SO-101 profile: URDF, servo
calibration, IK), run next to this one against the same Station, on another port and the other
servo bus. It speaks in its own base frame (origin under its base, x to its right, y forward, z
up) and offers its own tools over HTTP: status, home, rest, move_to, gripper, grasp. Where it
stands in this arm's base frame is in robot_config/second_arm.json, fitted by
tools/calibrate_second_arm.py with the depth camera.
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np

from util.arm import RobotError

URL = os.environ.get("ELROBOT_SECOND_ARM", "http://127.0.0.1:8770")


def _request(path: str, body: dict | None = None, timeout: float = 120.0) -> dict:
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(URL + path, data=data, headers={"Content-Type": "application/json"} if data else {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())
    except OSError as e:
        raise RobotError(f"the second arm's daemon is not reachable at {URL} ({e}): start it (docs/SECOND_ARM.md)") from None


def call(tool: str, **args) -> dict:
    """Run one of the second arm's own tools. Raises with its words if it refuses or fails."""
    res = _request(f"/api/tools/{tool}", {"args": {k: v for k, v in args.items() if v is not None}, "source": "agent"})
    if not res.get("ok"):
        raise RobotError(f"second arm, {tool}: {res.get('error') or res.get('summary') or 'failed'}")
    res.pop("images", None)
    return res


def state() -> dict:
    return _request("/api/state", timeout=10.0)


def tool_point() -> np.ndarray:
    """Where the second arm's tool point measurably is, in its own base frame."""
    return np.array(state()["ee"]["pos"], dtype=float)


def load(root: Path) -> np.ndarray:
    """T_base_second: a point in the second arm's frame, in this arm's base frame."""
    path = root / "robot_config" / "second_arm.json"
    if not path.exists():
        raise RobotError("where the second arm stands is not known (robot_config/second_arm.json is missing): run tools/calibrate_second_arm.py")
    return np.array(json.loads(path.read_text())["T_base_second"])


def fit(points_second: np.ndarray, points_base: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The rigid transform that carries the first points onto the second (least squares), and each point's miss in metres."""
    a, b = np.asarray(points_second, dtype=float), np.asarray(points_base, dtype=float)
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    U, _, Vt = np.linalg.svd((a - ca).T @ (b - cb))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, cb - R @ ca
    return T, np.linalg.norm(a @ R.T + T[:3, 3] - b, axis=1)


READY = np.radians([0.0, -45.0, 45.0, 75.0, 0.0])  # its daemon's ready pose (home)


def joints_now() -> np.ndarray:
    """The second arm's five joint angles. Raises if its daemon has no arm: unplugged, or not powered."""
    joints = state().get("joints") or []
    if len(joints) < 5:
        raise RobotError("the second arm's daemon is running but sees no arm: is the SO-101 plugged in and powered?")
    return np.array([j["q"] for j in joints[:5]], dtype=float)


def move_checked(root: Path, way, this_q, speed: float = 0.5, grippers_may_meet: bool = False) -> None:
    """Send the second arm through joint angles (one set, or several in turn), after walking the
    whole way in the shared space against this arm standing at `this_q`. Raises instead of moving
    if any part of it would come within 2 cm of this arm."""
    from util import shared_space

    way = np.atleast_2d(np.asarray(way, dtype=float))
    space = shared_space.space(root)
    whole = [joints_now(), *way]
    show(space, whole)
    why = space.path_clear("second", whole, this_q, grippers_may_meet=grippers_may_meet)
    if why:
        advice = "Go over it or another way; if it has been moved since, scan again (tools/scan_obstacles.py)." if "the thing" in why else "Move this arm away first (home or rest), or go another way."
        raise RobotError(f"second arm not moving: {why}. {advice}")  # the way refused stays on show in the 3D view
    try:
        for q in way:
            call("move_joints", joints={str(i + 1): round(float(np.degrees(v)), 1) for i, v in enumerate(q)}, speed=speed)
    finally:
        show(space, [])


def show(space, whole: list) -> None:
    """Draw the way the second arm's tool point is about to take in the web UI's 3D view (nothing to draw: clear it). Display only."""
    from util.arm import _request as to_this_daemon

    points = []
    for a, b in zip(whole, whole[1:]):
        for t in np.linspace(0.0, 1.0, max(2, int(np.abs(np.asarray(b) - np.asarray(a)).max() / 0.08) + 1)):
            points.append([round(float(v), 4) for v in space.second_tool(np.asarray(a) + (np.asarray(b) - np.asarray(a)) * t)])
    try:
        to_this_daemon("/api/event", {"kind": "plan", "call_id": "second-arm", "path": points, "target": None}, timeout=2.0)
    except RobotError:
        pass


def move_to_checked(root: Path, pos, this_q, pitch_deg: float = 90.0, yaw_deg: float = 0.0, speed: float = 0.5, grippers_may_meet: bool = False) -> np.ndarray:
    """Put the second arm's tool point at `pos`, given in THIS arm's base frame: joint angles from
    the shared space's inverse kinematics, the way there checked against this arm. Returns the angles."""
    from util import shared_space

    q = shared_space.space(root).second_ik(pos, pitch_deg, yaw_deg, seed=joints_now())
    move_checked(root, q, this_q, speed, grippers_may_meet)
    return q


def home_checked(root: Path, this_q, speed: float = 0.5) -> None:
    """Its ready pose, the way there checked against this arm."""
    move_checked(root, READY, this_q, speed, grippers_may_meet=True)


def reach(root: Path, pos, this_q, yaw_deg: float = 0.0, speed: float = 0.5, grippers_may_meet: bool = False) -> float:
    """Put the second arm's tool point at `pos` (this arm's base frame) pointing down as steeply as it can
    there: straight down low and close, leaning further out or higher up. Returns the pitch used, degrees."""
    from util import shared_space

    space = shared_space.space(root)
    pos = np.asarray(pos, dtype=float)

    def steepest(place, seed):
        last = None
        for pitch in (90.0, 75.0, 60.0, 45.0):
            try:
                return space.second_ik(place, pitch, yaw_deg, seed=seed), pitch
            except RobotError as e:
                last = e
        raise last

    q, pitch = steepest(pos, joints_now())
    try:
        move_checked(root, q, this_q, speed, grippers_may_meet)
        return pitch
    except RobotError as e:
        if "the thing" not in str(e) or space.obstacles is None:
            raise
    # Something on the table is in the way of going straight there: up where it stands, across above
    # the tallest thing on the line, then down. The same rule this arm's carry follows.
    here = space.second_tool(joints_now())
    high = space.obstacles["table_z"] + space.travel_height(here[:2], pos[:2]) + 0.10
    way, seed = [], joints_now()
    for place in ([here[0], here[1], max(high, here[2])], [pos[0], pos[1], max(high, pos[2])]):
        try:
            seed, _ = steepest(place, seed)
            way.append(seed)
        except RobotError:
            pass  # it does not reach that high there: the check decides about what is left
    move_checked(root, [*way, q], this_q, speed, grippers_may_meet)
    return pitch


def tool_in_base(root: Path) -> np.ndarray:
    """Where the second arm's tool point is, by its own joints, in this arm's base frame."""
    from util import shared_space

    return shared_space.space(root).second_tool(joints_now())
