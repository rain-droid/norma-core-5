"""Bring the wrist camera over a spot on the table for a close look, and capture what it sees.

Moves the tool point to (x, y) at `height` metres above the table, pointing `pitch` degrees
below horizontal: by default 9 cm up, 14 cm in front of the base, looking almost straight down.
From there the camera is about 20 cm above the table and sees roughly y 0.18 to 0.38 m and x
within 0.28 m; nearer the base is hidden behind the jaws. Few poses up here are reachable:
change x to look left or right, and expect an IK error if you raise it much. Then freezes a
wrist capture exactly as capture_camera does and returns its `image_id`.

For finding and locating things use the top camera (capture_camera, find_objects): it sees the
whole mat and is the calibrated one. This is for looking closely.
"""

import numpy as np

from capture_camera import capture_camera
from util.arm import RobotError
from util.motion import READY_DEG, require_ready


def look_at_table(ctx, x: float = 0.0, y: float = 0.14, height: float = 0.09, pitch: float = 80.0, speed: float = 0.8) -> dict:
    require_ready(ctx, awake=True)
    solution = ctx.plan.solve([x, y, ctx.plan.table_z + height], pitch=pitch, yaw=0, q_seed=np.radians(READY_DEG))
    if not solution.ok:
        raise RobotError(f"can't put the camera at x={x:+.3f} y={y:+.3f} height={height:.3f} pitch={pitch:.0f}: {solution.describe()}")
    ctx.arm.joints(solution.q, speed=speed, label="looking closer")
    return ctx.call(capture_camera, camera="wrist")


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(look_at_table)
