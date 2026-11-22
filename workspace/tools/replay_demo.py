"""Repeat a demonstration that was taught by hand with hand_guide: one-shot teaching.

The arm first goes to where the teaching started, then travels each stretch it was led along
and sets the jaws where they were set, in order. `speed` scales the motion (0.5 by default: a
hand-led path has corners a fast arm would cut). `reverse: true` runs the stretches backwards
with the jaw steps left out, to return along the same way.

Returns `steps` (what was done), `holding` and the final tool point. With no `name` it lists
the demonstrations there are.

It repeats joint angles, not an intention: check with a camera that the things it handled are
where they were when it was taught (describe_objects), and that it did what was meant
afterwards. A jaw step that held something when taught and holds nothing now is reported as
`lost: true`, and the replay stops there.
"""

import json

import numpy as np

from hand_guide import demo_file
from util.arm import RobotError
from util.motion import require_ready, tcp_report


def replay_demo(ctx, name: str | None = None, speed: float = 0.5, reverse: bool = False) -> dict:
    folder = ctx.root / "state" / "demos"
    if not name:
        return {"demos": sorted(p.stem for p in folder.glob("*.json"))}
    file = demo_file(ctx, name)
    if not file.exists():
        raise RobotError(f"no demonstration called {name!r}; there are: {sorted(p.stem for p in folder.glob('*.json')) or 'none yet'}")
    demo = json.loads(file.read_text())
    steps = demo["steps"]
    if not any("path" in s for s in steps):
        raise RobotError(f"the demonstration {name!r} has no motion in it")
    require_ready(ctx, awake=True)
    done = []
    if reverse:
        paths = [s["path"][::-1] for s in steps if "path" in s][::-1]
        ctx.arm.joints(np.array(paths[0][0]), speed=speed, allow_low=True, label="to the end of the demo")
        for path in paths:
            ctx.arm.joints(np.array(path), speed=speed, allow_low=True, label=f"replaying {name} backwards")
            done.append({"path": len(path)})
    else:
        ctx.arm.joints(np.array(demo["start"]), speed=speed, allow_low=True, label="to the start of the demo")
        for step in steps:
            if "path" in step:
                ctx.arm.joints(np.array(step["path"]), speed=speed, allow_low=True, label=f"replaying {name}")
                done.append({"path": len(step["path"])})
            else:
                grip = ctx.arm.joints(gripper=step["gripper"], label="closing" if step["gripper"] <= 0 else "opening")["gripper"]
                done.append({"gripper": step["gripper"], "holding": bool(grip.get("holding"))})
                if step.get("holding") and not grip.get("holding"):
                    done[-1]["lost"] = True
                    state = ctx.arm.state()
                    return {"name": name, "completed": False, "steps": done, "holding": False, "tcp": tcp_report(state["ee"], state["gripper"]),
                            "note": "the jaws closed on nothing where the demonstration held something: the object is not where it was when this was taught"}  # fmt: skip
    state = ctx.arm.state()
    return {"name": name, "completed": True, "steps": done, "holding": bool(state["gripper"]["holding"]), "tcp": tcp_report(state["ee"], state["gripper"])}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(replay_demo)
