"""Open the jaws to `fraction` of full travel: 1 = fully open (53 mm), 0 = closed.

Drives the gripper servo to that position and waits for it to stop. Opening past half clears
the `holding` flag. To grip something use close_gripper, which detects whether the jaws stalled
on an object. Returns the opening actually reached.
"""

from elrobot.kinematics import GRIPPER_MAX_WIDTH
from util.motion import require_ready, tcp_report


async def open_gripper(ctx, fraction: float = 1.0) -> dict:
    require_ready(ctx)
    reached = await ctx.robot.set_gripper(fraction)
    return {"requested_open": fraction, "reached_open": round(float(reached), 3), "width_mm": round(float(reached) * GRIPPER_MAX_WIDTH * 1000, 1), "tcp": tcp_report(ctx)}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(open_gripper)
