"""Safe mode: the gripper only ever points straight down, and only travels in straight lines.

Switched on in robot_config/motion.json (`"safe_mode": true`; the environment variable
ELROBOT_SAFE_MODE=0 or 1 overrides it). It changes three things, all in this workspace:

* Grasps and placements come straight down, never leaning (util/planning.py `approach_for`).
  The jaws are turned to suit the object; nothing else about the gripper's direction varies.
* Every move between two places is three straight lines: straight up to the carry height,
  straight across at that height, straight down (`route`, applied to every single-target joint
  command in util/arm.py). No arcs: the gripper cannot swing through anything on the way.
* What cannot be done that way is refused, with the reason, before anything moves. On this arm
  pointing straight down works about 16 cm either side of the base and 12 to 24 cm in front of
  it, at up to about 7 cm above the table: an object further out is "too far" and stays where
  it is until a person moves it closer.

Two moves are not straight lines, and both happen in the air above the table: getting up from
the resting seat and sitting down on it (tools/home.py, tools/rest.py), and reaching the
straight-down ready pose from a pose that is not straight down.

The carried object's foot travels at `carry_height` minus the grasp height above the table,
about 4 cm with the defaults: it clears a cube, not a tall object. The price of pointing
straight down is that the arm cannot reach higher than that.
"""

import json
import math
import os
from pathlib import Path

import numpy as np

from util.arm import RobotError

CONFIG = Path(__file__).resolve().parents[2] / "robot_config" / "motion.json"
CARRY_HEIGHT = 0.065  # metres above the table that the tool point travels at, unless motion.json says otherwise
DOWN = np.array([0.0, 0.0, -1.0])
UPRIGHT_DEG = 15.0  # a gripper within this of straight down counts as pointing down
READY_XY = (0.0, 0.16)  # where the straight-down ready pose holds the tool point
AREA = "about 16 cm either side of the base and 12 to 24 cm in front of it"
ALWAYS_ALLOWED = ("getting up", "sitting down")  # labels of the two moves that are not lines


def _config() -> dict:
    try:
        data = json.loads(CONFIG.read_text())
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def enabled() -> bool:
    env = os.environ.get("ELROBOT_SAFE_MODE", "").strip().lower()
    if env:
        return env not in ("0", "false", "no", "off")
    return bool(_config().get("safe_mode", False))


def carry_height() -> float:
    value = _config().get("carry_height", CARRY_HEIGHT)
    return float(value) if isinstance(value, (int, float)) and 0.03 <= value <= 0.12 else CARRY_HEIGHT


def tilt_deg(T: np.ndarray) -> float:
    """How far from straight down the gripper points, degrees."""
    return math.degrees(math.acos(float(np.clip(T[:3, 2] @ DOWN, -1.0, 1.0))))


def ready(plan) -> np.ndarray:
    """Joint angles of the straight-down ready pose: over the near middle of the table, at carry height."""
    from util.motion import READY_DEG

    pos = [READY_XY[0], READY_XY[1], plan.table_z + carry_height()]
    r = plan.solve(pos, pitch=90, yaw=0.0, q_seed=np.radians(READY_DEG))
    if not r.ok:
        raise RobotError(f"safe mode: the straight-down ready pose cannot be reached ({r.describe()}); lower carry_height in robot_config/motion.json")
    return r.q


def route(plan, q_now, q_goal, allow_low: bool = False) -> np.ndarray:
    """Joint waypoints from `q_now` to `q_goal` as straight lines of the tool point: up (or down)
    to the travel height, across, down. Both ends must point straight down. Raises RobotError
    when there is no such way."""
    T0, T1 = plan.fk(q_now), plan.fk(q_goal)
    start, goal = T0[:3, 3], T1[:3, 3]
    travel = max(plan.table_z + carry_height(), float(goal[2]))
    corners = [np.array([start[0], start[1], travel]), np.array([goal[0], goal[1], travel]), goal]
    if np.linalg.norm(goal[:2] - start[:2]) < 0.003:
        corners = [goal]  # straight up or down: one line
    path = [np.asarray(q_now, dtype=float)]
    at = start
    for corner in corners:
        if np.linalg.norm(corner - at) < 0.002:
            continue
        try:
            path.extend(plan.line(path[-1], corner, approach=DOWN, allow_low=allow_low or corner[2] < plan.table_z + 0.03)[1:])
        except RobotError as e:
            raise RobotError(f"safe mode: there is no straight-line, straight-down way to x={goal[0]:+.3f} y={goal[1]:+.3f} z={goal[2]:.3f} ({e}). "
                             f"Pointing straight down the arm works {AREA}; move the object closer, or switch safe mode off in "
                             "robot_config/motion.json if a person is supervising.") from None  # fmt: skip
        at = corner
    path.append(np.asarray(q_goal, dtype=float))  # the jaws' final turn about the vertical, on the spot
    return np.array(path[1:])


def linear(arm, path, label: str, allow_low: bool):
    """What util/arm.py sends in safe mode in place of `path`.

    A list of waypoints is a line someone already planned and goes through as it is. A single
    target is replaced by the three-line route to it. A target that does not point straight
    down is refused; so is a start that is low over the table and not pointing down, where a
    joint-space swing could sweep the table.
    """
    p = np.asarray(path, dtype=float)
    if p.ndim == 2 and len(p) > 1:
        return path
    if label in ALWAYS_ALLOWED:
        return path
    q_goal = p.reshape(-1)
    plan = arm.planner()
    q_now = arm.q()
    T0, T1 = plan.fk(q_now), plan.fk(q_goal)
    if tilt_deg(T1) > UPRIGHT_DEG:
        raise RobotError(f"safe mode: the gripper only points straight down, and this move would tilt it {tilt_deg(T1):.0f} degrees. "
                         "Use a straight-down pose (pitch_deg 90), or switch safe mode off in robot_config/motion.json if a person is supervising.")  # fmt: skip
    if tilt_deg(T0) > UPRIGHT_DEG:
        low = min(T0[2, 3], T1[2, 3]) - plan.table_z
        if low < 0.04:
            raise RobotError("safe mode: the gripper is low over the table and not pointing straight down, so no straight line leads from here. "
                             "Lift it clear first (adjust_tcp with dz), then home.")  # fmt: skip
        return path  # in the air above the table: the one swing allowed, into the straight-down posture
    return route(plan, q_now, q_goal, allow_low)
