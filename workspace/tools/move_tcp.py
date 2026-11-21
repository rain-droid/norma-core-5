"""Move the tool point (the spot between the fingertips) to x, y, z.

Metres in the base frame: origin on the table under the base, x to the robot's right, y forward
away from the base, z up. On the table the arm reaches a half-disc of about 0.38 m in front of
the base; straight-down poses only exist within about 0.27 m.

`pitch` is how far below horizontal the gripper points: 90 straight down, 0 level. Left out,
the most vertical approach that reaches is used (tries 90, 75, 60, 45, 30, 15). `yaw` rotates
the jaws about the pointing direction, degrees (0 = jaws close left-right).

How it gets there: inverse kinematics (util/planning.py) turns the pose into joint angles, and
one joint command goes there, so the tool sweeps an arc, not a line. `straight: true` instead
plans a waypoint every 8 mm along the straight segment, keeping the gripper's direction, and
sends them as one command; use it for approach and retreat near objects. Poses that put the
fingertips within 8 mm of the table are refused unless `allow_low`.

Returns where the tool measurably ended up and how far that is from the target.
"""

import numpy as np

from util.kinematics import approach_from_pitch
from util.arm import RobotError
from util.motion import require_ready, tcp_report


def move_tcp(ctx, x: float, y: float, z: float, pitch: float | None = None, yaw: float | None = None,
             straight: bool = False, speed: float = 0.8, allow_low: bool = False) -> dict:  # fmt: skip
    require_ready(ctx, awake=True)
    plan, pos, start = ctx.plan, np.array([x, y, z], dtype=float), ctx.arm.q()
    if straight:
        approach = approach_from_pitch(pos[:2], pitch) if pitch is not None else None
        target, used = plan.line(start, pos, approach=approach, allow_low=allow_low), pitch
    else:
        if pitch is None:
            solution, used = plan.reach(pos, yaw=yaw, q_seed=start, allow_low=allow_low)
        else:
            solution, used = plan.solve(pos, pitch=pitch, yaw=yaw, q_seed=start, allow_low=allow_low), pitch
        if not solution.ok:
            raise RobotError(plan.unreachable(pos, solution))
        target = solution.q
    res = ctx.arm.joints(target, speed=speed, allow_low=allow_low, label="moving straight" if straight else "moving")
    report = tcp_report(res["tcp"], res["gripper"])
    return {
        "target_xyz_m": [round(float(v), 4) for v in pos],
        "tcp": report,
        "position_error_mm": round(float(np.linalg.norm(np.array(report["xyz_m"]) - pos)) * 1000, 1),
        "pitch_deg_requested": used,
        "path": "straight line" if straight else "joint-space arc",
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(move_tcp)
