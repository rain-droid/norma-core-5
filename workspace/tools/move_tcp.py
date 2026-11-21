"""Move the tool point (the spot between the fingertips) to x, y, z.

Metres in the base frame: origin on the table under the base, x to the robot's right, y forward
away from the base, z up. On the table the arm reaches a half-disc of about 0.38 m in front of
the base; straight-down poses only exist within about 0.27 m.

`pitch` is how far below horizontal the gripper points: 90 straight down, 0 level. Left out,
the most vertical approach that reaches is used (tries 90, 75, 60, 45, 30, 15). `yaw` rotates
the jaws about the pointing direction, degrees (0 = jaws close left-right).

How it gets there: inverse kinematics picks joint angles for the pose, then one joint-space
move goes there, the same motion joint_command makes. The tool therefore sweeps an arc, not a
line. `straight: true` instead solves IK every 8 mm along the straight segment and follows
those waypoints, holding the gripper's direction; use it for approach and retreat near objects.
Poses that would put the fingertips within 8 mm of the table are refused unless `allow_low`.

Returns where the tool measurably ended up and how far that is from the target.
"""

import numpy as np

from elrobot.kinematics import approach_from_pitch
from elrobot.robot import RobotError
from util.motion import require_ready, tcp_report


async def move_tcp(ctx, x: float, y: float, z: float, pitch: float | None = None, yaw: float | None = None,
                   straight: bool = False, speed: float = 0.8, allow_low: bool = False) -> dict:  # fmt: skip
    robot = ctx.robot
    require_ready(ctx)
    pos = np.array([x, y, z], dtype=float)
    start = robot._plan_start()
    if straight:
        approach = approach_from_pitch(pos[:2], pitch) if pitch is not None else None
        path = robot.plan_linear(start, pos, approach=approach, allow_low=allow_low)
        await robot.move_path(path, speed=speed, target={"pos": pos.tolist(), "approach": robot.kin.fk(path[-1])[:3, 2].tolist()})
        used = pitch
    else:
        if pitch is None:
            solution, used = robot.solve_reach(pos, yaw=yaw, allow_low=allow_low)
        else:
            solution, used = robot.solve(pos, pitch=pitch, yaw=yaw, allow_low=allow_low), pitch
        if not solution.ok:
            raise RobotError(robot._unreachable(pos, solution))
        await robot.move_joints(solution.q, speed=speed, allow_low=allow_low)
    report = tcp_report(ctx)
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
