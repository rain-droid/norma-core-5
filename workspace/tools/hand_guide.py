"""Let a person lead the arm by hand, and keep what they showed as a demonstration to replay.

The arm keeps holding itself up. Where a hand pushes a joint, the arm follows and stays when
let go, so a person can take the gripper to a place instead of describing it. The call returns
when the arm has been moved and then left alone for three seconds, after `seconds` at the
latest, or on stop. Tell the person before calling: "lead the arm now, let go when it is there".

`save_as` names a demonstration and adds this stretch to it (`state/demos/NAME.json`);
`then_gripper` (0 closed .. 1 open) sets the jaws once the arm has been left, and is recorded
as the next step. That is how a pick and place is taught in four calls:

    hand_guide {"save_as": "to-the-tray", "then_gripper": 1.0, "new": true}   (open, start of the demo)
    hand_guide {"save_as": "to-the-tray", "then_gripper": 0.0}   (person leads it onto the object; jaws close)
    hand_guide {"save_as": "to-the-tray", "then_gripper": 1.0}   (person leads it to the tray; jaws open)
    replay_demo {"name": "to-the-tray"}                          (the arm does it alone)

Start from `home`, so a replay can begin where the teaching began. `new: true` starts the
named demonstration afresh instead of adding to it.

Returns `moved`, why it `ended` (`left alone`, `time`, `stopped`), `waypoints`, where the tool
point was taken `from` and `to`, and `saved`. A demonstration is joint angles: it repeats the
motion, not the intention. If the object is somewhere else next time, it will miss.

The arm resists a little before it follows and the table still stops it. It does not follow
into joint limits. Leading works on the arm joints only; the jaws are set with `then_gripper`.
"""

import json
import re
import time


from util.arm import RobotError
from util.motion import require_ready, tcp_report


def demo_file(ctx, name: str):
    slug = re.sub(r"[^a-z0-9]+", "-", name.lower()).strip("-")
    if not slug:
        raise RobotError("a demonstration needs a name with letters or digits in it")
    return ctx.root / "state" / "demos" / f"{slug}.json"


def hand_guide(ctx, seconds: float = 30.0, save_as: str | None = None, then_gripper: float | None = None, new: bool = False) -> dict:
    state = require_ready(ctx, awake=True)
    file = demo_file(ctx, save_as) if save_as else None
    before = tcp_report(state["ee"], state["gripper"])
    led = ctx.arm.guide(seconds)
    steps = []
    if led["moved"]:
        steps.append({"path": led["path"]})
    if then_gripper is not None:
        grip = ctx.arm.joints(gripper=float(then_gripper), label="closing" if then_gripper <= 0 else "opening")["gripper"]
        steps.append({"gripper": float(then_gripper), "holding": bool(grip.get("holding"))})
    after = ctx.arm.state()
    out = {"moved": led["moved"], "ended": led["ended"], "waypoints": len(led["path"]), "from": before,
           "to": tcp_report(after["ee"], after["gripper"])}  # fmt: skip
    if file is not None:
        demo = {"name": save_as, "start": led["path"][0], "steps": []}
        if file.exists() and not new:
            demo = json.loads(file.read_text())
        demo["steps"] += steps
        demo["recorded"] = time.strftime("%Y-%m-%d %H:%M:%S")
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text(json.dumps(demo))
        out["saved"] = str(file.relative_to(ctx.root))
        out["steps_in_demo"] = len(demo["steps"])
    if not led["moved"]:
        out["note"] = "the arm was not moved: nobody pushed it, or not hard enough to pass the deadband"
    return out


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(hand_guide)
