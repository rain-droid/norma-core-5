"""Go to the ready pose: gripper over the table in front of the base, pointing down and forward.

One joint-space move to fixed joint angles (util/motion.py READY_DEG). A safe place to start,
to finish, and to return to when a pose has become awkward. Turns torque on if it was off.
A joint-space move sweeps the gripper along an arc, so if the tool point is within 6 cm of the
table it is first lifted straight up to 9 cm: otherwise it can knock over what it just put down.
"""

import numpy as np

from elrobot.robot import RobotError
from util.motion import READY_DEG, require_ready, tcp_report


async def home(ctx, speed: float = 0.5) -> dict:
    robot = ctx.robot
    require_ready(ctx)
    tcp = robot.tcp()[:3, 3]
    if robot.torque_on and tcp[2] < ctx.cfg.table_z + 0.06:
        try:
            up = robot.plan_linear(robot._plan_start(), [tcp[0], tcp[1], ctx.cfg.table_z + 0.09], allow_low=True)
            await robot.move_path(up, speed=min(speed, 0.5), label="lifting clear")
        except RobotError:
            pass  # no straight lift exists from here; the joint move below is all that is left
    await robot.move_joints(np.radians(READY_DEG), speed=speed, label="going home")
    return {"pose": "ready", "tcp": tcp_report(ctx)}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(home)
