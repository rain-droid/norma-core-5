"""Let a person show the hand-over: where this arm holds the thing while the second arm's jaws are round it.

Before: this arm holds the thing by its top (see hand_over). Then:
1. This arm holds the thing out and the second arm brings its open jaws to where hand_over
   would send them, and stays rigid.
2. This arm becomes hand-guidable for `seconds` (motors on, it follows a push). The person moves
   its gripper until the thing sits well between the second arm's jaws, and lets go. Three
   seconds after it was last moved, or when the time is up, both arms' poses are read: this
   arm's joints and tool point, the second arm's tool point in its own frame.
3. They are written to robot_config/hand_over_taught.json, and the hand-over is finished from
   that pose: the second arm closes, this arm lets go and goes home.

From then on hand_over goes to the pose shown and sends the second arm's jaws to where they were
when shown, instead of computing both from the depth camera and the fit. The fit is still used,
for its turn only, to carry over any difference in where this arm ends up. Show it again when
either arm has been moved; delete the file to go back to the computed hand-over.

Say before starting what the person is to do and that letting go ends it. `moved` tells whether
the arm was moved at all: if not, the pose recorded is simply the computed one.
"""

import json
import time

import numpy as np

from hand_over import TAUGHT, bring_jaws, hold_out, jaws_place, take
from util import second_arm
from util.arm import RobotError
from util.motion import require_ready


def teach_hand_over(ctx, colour: str = "blue", seconds: float = 30.0, jaws: str = "level", speed: float = 0.5) -> dict:
    state = require_ready(ctx, awake=True)
    if not state["gripper"]["holding"]:
        raise RobotError("this arm is not holding anything: pick the thing up first, by its top")
    T = second_arm.load(ctx.root)
    hold_out(ctx, T, None, speed)
    time.sleep(0.5)
    grip, _ = jaws_place(ctx, T, None, colour, 0.025, 0.009)
    try:
        bring_jaws(grip, jaws, speed)
    except RobotError as e:
        second_arm.call("home")
        raise RobotError(f"the second arm could not get to the thing ({e}); nothing was recorded") from None
    led = ctx.arm.guide(seconds)  # the person moves this arm until the thing sits between the other's jaws
    time.sleep(1.0)
    here = ctx.arm.state()
    shown = {
        "this_arm_q": [round(float(v), 5) for v in ctx.arm.q()],
        "this_arm_tcp": [round(float(v), 4) for v in here["ee"]["pos"]],
        "second_arm_tool": [round(float(v), 4) for v in second_arm.tool_point()],
        "colour": colour,
        "shown": time.strftime("%Y-%m-%d %H:%M"),
    }
    path = ctx.root / "robot_config" / TAUGHT
    path.write_text(json.dumps(shown, indent=1) + "\n")
    return {"moved": bool(led.get("moved")), "ended": led.get("ended"), "recorded": shown, "saved": str(path.relative_to(ctx.root)), **take(ctx, T, colour)}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(teach_hand_over)
