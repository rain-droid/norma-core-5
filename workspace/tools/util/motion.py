"""Geometry shared by the motion tools: named poses, pose reports, grasp approaches."""

import math

import numpy as np

from elrobot.kinematics import approach_from_pitch, jaw_from_yaw
from elrobot.robot import RobotError

# Joint angles in degrees. READY holds the gripper over the table in front of the base, pointing
# down and forward: a safe pose to start and finish in, and the seed for most IK.
READY_DEG = [0.0, -21.0, -29.0, 36.0, 0.0, 73.5, 0.0]


def require_ready(ctx) -> None:
    why = ctx.robot.why_not_ready()
    if why:
        raise RobotError(why)
    # A servo that tripped its overload protection only gives 20 % torque: the joint sags and
    # the arm no longer goes where it is sent. Moving on from there is how things get hit.
    faulted = [i for i, m in sorted(ctx.robot.link.motors.items()) if m.status]
    if faulted:
        raise RobotError(
            f"servo fault on joint(s) {faulted} (status bit 32 = overload). Do not move. A person must hold the arm, "
            "then torque is cycled off and on to clear it (POST /api/torque enable false, then true)."
        )


def tcp_report(ctx) -> dict:
    """Where the tool point (between the fingertips) measurably is right now."""
    T = ctx.robot.tcp()
    return {
        "xyz_m": [round(float(v), 4) for v in T[:3, 3]],
        "pitch_deg": round(math.degrees(math.asin(float(np.clip(-T[2, 2], -1, 1)))), 1),
        "gripper_open": round(float(ctx.robot.gripper_open()), 3),
        "holding": bool(ctx.robot.holding),
    }


def approach_for(ctx, target: np.ndarray, grasp_z: float, yaw: float | None, hover: float = 0.07):
    """Choose how to come down on a table point.

    Tries the most vertical approach first and leans forward in 10 degree steps until IK reaches
    both the grasp point (`grasp_z` above the table) and a hover point backed off `hover` metres
    along the approach. Straight-down only works within about 27 cm of the base.

    Returns (approach unit vector, pitch in degrees, hover joint angles, hover xyz, grasp xyz).
    """
    r = ctx.robot
    grasp = np.array([target[0], target[1], ctx.cfg.table_z + grasp_z])
    last = None
    blocked: list[str] = []  # what the clearance check said about each pose IK reached but may not use

    def clear(q, allow_low: bool) -> bool:
        why = r.check_config(q, allow_low=allow_low)
        if why:
            blocked.append(why)
        return why is None

    for pitch in (90, 80, 70, 60, 50, 40, 30):
        a = approach_from_pitch(grasp[:2], pitch)
        jaw = jaw_from_yaw(a, yaw if yaw is not None else 0.0)
        hover_pos = grasp - a * hover
        rg = r.kin.ik(grasp, approach=a, jaw=jaw, q_seed=np.radians(READY_DEG), limit_margin=ctx.cfg.limit_margin,
                      accept=lambda q: clear(q, True))  # fmt: skip
        if not rg.ok:
            last = rg
            continue
        rh = r.kin.ik(hover_pos, approach=a, jaw=jaw, q_seed=rg.q, limit_margin=ctx.cfg.limit_margin,
                      accept=lambda q: clear(q, False))  # fmt: skip
        if rh.ok:
            return a, float(pitch), rh.q, hover_pos, grasp
        last = rh
    d = math.hypot(target[0], target[1] - 0.04)
    if d > 0.36:
        why = f"it is {d * 100:.0f} cm from the base (reach is about 38 cm on the table)"
    elif d < 0.09:
        why = "it is too close to the base"
    elif blocked and last is not None and "rejected" in last.reason:
        # reachable, but not allowed: say why, most often a fingertip of tilted, turned jaws
        # dipping below the table, which a higher grasp or yaw 0 avoids
        why = f"every pose that gets there was refused, most recently: {blocked[-1]}. Try a larger grasp height, or yaw 0"
    else:
        why = last.describe() if last else "no approach found"
    raise RobotError(f"can't reach a grasp at x={target[0]:+.3f} y={target[1]:+.3f}: {why}")
