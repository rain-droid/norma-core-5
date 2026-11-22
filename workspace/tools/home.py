"""Go to the ready pose: gripper over the table in front of the base, pointing down and forward.

One joint command to fixed joint angles (util/motion.py READY_DEG). A safe place to start, to
finish, and to return to when a pose has become awkward. Switches the motors on if they were off.
In safe mode (util/safe.py) the ready pose points straight down instead, low over the near
middle of the table, and from there on the arm only travels in straight lines.

Two cases get a lead-in first, because a joint-space move sweeps the gripper along an arc:
from the resting seat the arm first rises to the pose just above it, the way it sat down; and
a tool point within 6 cm of the table is first lifted straight up to 9 cm, so the gripper does
not knock over what it just put down.

Unfolding from rest is the heaviest move the arm makes: above speed 0.5 the robot side has
refused it as blocked.
"""

import numpy as np

from util import safe
from util.arm import RobotError
from util.motion import READY_DEG, require_ready, seat, tcp_report


def home(ctx, speed: float = 0.5) -> dict:
    state = require_ready(ctx)
    q = np.array([j["q"] for j in state["joints"][:7]])
    if not state["torque"]:
        sit, above = seat(ctx.arm)
        if np.max(np.abs(q - sit)) < 0.6:  # resting on the seat, give or take how it was left
            ctx.arm.joints(above, speed=min(speed, 0.5), check=False, label="getting up")
    elif state["ee"]["pos"][2] < ctx.arm.meta()["table_z"] + 0.06:
        x, y, _ = state["ee"]["pos"]
        try:
            up = ctx.plan.line(q, [x, y, ctx.plan.table_z + 0.09], allow_low=True)
            ctx.arm.joints(up, speed=min(speed, 0.5), allow_low=True, label="lifting clear")
        except RobotError:
            pass  # no straight lift exists from here; the joint move below is all that is left
    target = safe.ready(ctx.plan) if safe.enabled() else np.radians(READY_DEG)  # safe mode: pointing straight down, at carry height
    res = ctx.arm.joints(target, speed=speed, label="going home")
    return {"pose": "ready", "tcp": tcp_report(res["tcp"], res["gripper"])}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(home)
