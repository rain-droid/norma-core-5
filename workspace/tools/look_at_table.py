"""Raise the wrist camera over the workspace and capture what it sees.

Moves the tool point to (x, y, height) pointing `pitch` degrees below horizontal: by default
10 cm up, 14 cm in front of the base, looking almost straight down. From there the camera is
20 cm above the table and sees roughly y 0.18 to 0.38 m and x within 0.28 m; anything nearer
the base is hidden behind the jaws. Few poses up here are reachable: change x to look left or
right, and expect an IK error if you raise it much. Then freezes a wrist capture exactly as
capture_camera does and returns its `image_id`.

The normal first step of a task, and the way to re-look after acting: a held object or the
gripper itself hides what is under it, so look from here, not from where you were working.
"""

import numpy as np

from elrobot.robot import RobotError
from util.motion import READY_DEG, require_ready


async def look_at_table(ctx, x: float = 0.0, y: float = 0.14, height: float = 0.10, pitch: float = 80.0, speed: float = 0.8) -> dict:
    robot = ctx.robot
    require_ready(ctx)
    solution = robot.solve([x, y, height], pitch=pitch, yaw=0, q_seed=np.radians(READY_DEG))
    if not solution.ok:
        raise RobotError(f"can't put the camera at x={x:+.3f} y={y:+.3f} height={height:.3f} pitch={pitch:.0f}: {solution.describe()}")
    await robot.move_joints(solution.q, speed=speed, label="looking around")
    return await ctx.call("capture_camera", camera="wrist")


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(look_at_table)
