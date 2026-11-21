"""Nudge the tool point by a small offset in a straight line, keeping the gripper's direction.

`frame: "base"` (default): dx right, dy forward away from the base, dz up, in metres.
`frame: "camera"`: the offset is given the way the wrist image shows it: dx towards the right
edge of the image, dy towards the bottom edge, dz forward along the view (towards what the
gripper points at). Use it to centre something seen in the wrist camera, or to advance/retreat
along the approach. Each offset is limited to 0.15 m; use move_tcp for bigger moves.

IK is solved every 8 mm along the segment; if any step is unreachable or would hit the table
or the arm, nothing moves and the error says how far along it failed.
"""

import numpy as np

from elrobot.robot import RobotError
from util.motion import require_ready, tcp_report


async def adjust_tcp(ctx, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, frame: str = "base",
                     speed: float = 0.5, allow_low: bool = False) -> dict:  # fmt: skip
    robot = ctx.robot
    require_ready(ctx)
    delta = np.array([dx, dy, dz], dtype=float)
    if np.max(np.abs(delta)) > 0.15:
        raise RobotError("offsets are limited to 0.15 m per call; use move_tcp for big moves")
    start = robot._plan_start()
    T = robot.kin.fk(start)
    if frame == "camera":
        delta = T[:3, :3] @ ctx.skills.cam.R_tool_cam @ delta
    elif frame != "base":
        raise RobotError("frame must be 'base' or 'camera'")
    before = T[:3, 3].copy()
    path = robot.plan_linear(start, before + delta, allow_low=allow_low)
    await robot.move_path(path, speed=speed)
    report = tcp_report(ctx)
    return {
        "requested_base_offset_m": [round(float(v), 4) for v in delta],
        "measured_base_offset_m": [round(float(v), 4) for v in np.array(report["xyz_m"]) - before],
        "tcp": report,
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(adjust_tcp)
