"""Small things the motion tools share: named poses, readiness checks, pose reports."""

import json
import math
from pathlib import Path

import numpy as np

from util.arm import RobotError

ROOT = Path(__file__).resolve().parents[2]

# Joint angles in degrees. READY holds the gripper over the table in front of the base, pointing
# down and forward: a safe pose to start and finish in, and the seed for most IK.
READY_DEG = [0.0, -21.0, -29.0, 36.0, 0.0, 73.5, 0.0]


def seat(arm) -> tuple[np.ndarray, np.ndarray]:
    """The folded resting pose and the pose just above it (shoulder raised 22 degrees), radians.
    The arm sits down onto the first through the second, and gets up the same way."""
    lo, hi = arm.limits()
    sit = np.clip(json.loads((ROOT / "robot_config" / "sit_pose.json").read_text())["q"], lo, hi)  # saved a hair past a limit
    above = sit.copy()
    above[1] += math.radians(22)
    return sit, above


def require_ready(ctx, awake: bool = False) -> dict:
    """The measured state, or a RobotError saying why the arm cannot be used. With `awake` the
    servos must also be holding: a tool that plans from the current pose is no use on a limp arm."""
    state = ctx.arm.state()
    why = ctx.arm.why_not_ready(state)
    if why:
        raise RobotError(why)
    if awake and not state["torque"]:
        raise RobotError("the arm is resting with its motors off: run tools/home.py first")
    return state


def tcp_report(ee: dict, gripper: dict) -> dict:
    """Where the tool point (between the fingertips) measurably is: from a state or a joint-command result."""
    return {
        "xyz_m": ee["pos"],
        "pitch_deg": round(math.degrees(math.asin(max(-1.0, min(1.0, -ee["approach"][2])))), 1),
        "gripper_open": gripper["open"],
        "holding": gripper["holding"],
    }


def carry(ctx, q_to, pos_to, speed: float = 0.8, label: str = "carrying") -> dict:
    """Go to a pose above a place (joint angles `q_to`, tool point `pos_to`), over what stands on the way.

    With a recent map of the things on the table (tools/scan_obstacles.py) the tallest thing along
    the straight line from here to there is looked up, and the move becomes: straight up where the
    arm stands, to where the gripper and what hangs from it clear that thing; across at that height;
    then to the pose. Without a map, or with nothing on the way, it is the one direct move it always
    was. Whatever it ends up sending, the check in util/arm.py walks it and lets it through or refuses.
    """
    arm, plan = ctx.arm, ctx.plan
    fit = ctx.root / "robot_config" / "second_arm.json"
    tall = 0.0
    state = arm.state()
    here = np.array(state["ee"]["pos"], dtype=float)
    if fit.exists() and state.get("mode") == "real":  # the map is of the real table
        from util import shared_space  # here, not at the top: it builds on this arm's kinematics

        tall = shared_space.space(ctx.root).travel_height(here[:2], np.asarray(pos_to, dtype=float)[:2])
    if tall <= 0.0:
        return arm.joints(q_to, speed=speed, label=label)
    high = plan.table_z + tall + 0.085  # for the tool point: the gripper's own 36 mm, what hangs below it, and 2 cm to spare
    # Up high this arm cannot point straight down, so the gripper may lean there: the steepest pose that
    # reaches, above where it stands and above where it is going. Between those it swings round at that height.
    way, seed = [], arm.q()
    for x, y, needed in ((here[0], here[1], here[2] < high), (pos_to[0], pos_to[1], pos_to[2] < high)):
        if needed:
            solution, _ = plan.reach([x, y, high], q_seed=seed)
            if solution.ok:
                way.append(solution.q)
                seed = solution.q
    return arm.joints(np.array([*way, np.asarray(q_to, dtype=float)]), speed=speed, label=label + (" over what stands on the way" if way else ""))
