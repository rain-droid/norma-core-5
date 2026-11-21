"""Go to the ready pose: gripper over the table in front of the base, pointing down and forward.

One joint-space move to fixed joint angles (util/motion.py READY_DEG). A safe place to start,
to finish, and to return to when a pose has become awkward. Turns torque on if it was off.
"""

import numpy as np

from util.motion import READY_DEG, require_ready, tcp_report


async def home(ctx, speed: float = 0.8) -> dict:
    require_ready(ctx)
    await ctx.robot.move_joints(np.radians(READY_DEG), speed=speed, label="going home")
    return {"pose": "ready", "tcp": tcp_report(ctx)}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(home)
