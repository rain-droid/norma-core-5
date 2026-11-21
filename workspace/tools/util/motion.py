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
