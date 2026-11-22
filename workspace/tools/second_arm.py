"""Run one of the second arm's tools. The second arm is the SO-101 with the orange parallel gripper.

    second_arm(tool="status")
    second_arm(tool="home")                      its ready pose
    second_arm(tool="rest")                      folded, motors off
    second_arm(tool="gripper", args={"open": 1.0})
    second_arm(tool="grasp")                     close on what is between the jaws; says whether it holds
    second_arm(tool="move_to", args={"x": 0.15, "y": 0.25, "z": 0.06, "pitch": 90})
    second_arm(tool="move_joints", args={"joints": {"1": -40, "2": -30}})     degrees

Positions for move_to are in THIS arm's base frame, the one every other tool here uses
(`frame="own"` for its own: origin under its base, x to its right, y forward). `pitch` is degrees
below level (90 straight down, the default), `yaw` turns the jaws. It has five joints and
reaches about 30 cm; level poses only exist 24 to 30 cm from its base.

home, move_to and move_joints are planned in the shared space of both arms
(util/shared_space.py) and walked against this arm where it stands before anything moves: a way
that would bring any part of it within 2 cm of this arm is refused, with the parts named. Then
move this arm away (home, rest) or choose another place. `near_this_arm=true` lets the two
grippers meet, for passing something across. Only one arm moves at a time. Its other tools
(pick, place, touch, gestures) are its daemon's own and are NOT checked against this arm.
To pass something from this arm to it in the air use hand_over.
"""

import numpy as np

from util import second_arm as arm2
from util import shared_space
from util.arm import RobotError


def second_arm(ctx, tool: str, args: dict | None = None, frame: str = "base", near_this_arm: bool = False, speed: float = 0.5) -> dict:
    args = dict(args or {})
    if frame not in ("own", "base"):
        raise RobotError("frame is 'base' (this arm's base frame) or 'own' (the second arm's)")
    this_q = ctx.arm.q()
    if tool == "home":
        arm2.call("status")  # its daemon must be there before anything is planned
        if not arm2.state()["torque"]:
            arm2.call("home")  # from rest its own daemon knows the way up; it starts folded, far from this arm
        else:
            arm2.home_checked(ctx.root, this_q, speed)
    elif tool == "move_to":
        space = shared_space.space(ctx.root)
        p = np.array([args["x"], args["y"], args.get("z", 0.08)], dtype=float)
        if frame == "own":
            p = space.T[:3, :3] @ p + space.T[:3, 3]
        arm2.move_to_checked(ctx.root, p, this_q, float(args.get("pitch", 90.0)), float(args.get("yaw", 0.0)), speed, near_this_arm)
    elif tool == "move_joints":
        q = arm2.joints_now()
        for k, deg in (args.get("joints") or {}).items():
            q[int(k) - 1] = np.radians(float(deg)) + (q[int(k) - 1] if args.get("relative") else 0.0)
        arm2.move_checked(ctx.root, q, this_q, speed, near_this_arm)
    else:
        res = arm2.call(tool, **args)
        return {"summary": res.get("summary"), "data": res.get("data")}
    space = shared_space.space(ctx.root)
    q = arm2.joints_now()
    gap, where = space.between(this_q, q)
    return {
        "tool_point": [round(float(v), 3) for v in space.second_tool(q)],
        "joints_deg": [round(float(v), 1) for v in np.degrees(q)],
        "gap_to_this_arm_mm": round(gap * 1000),
        "nearest": where,
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(second_arm)
