"""Play back poses a person showed with both arms (record_poses): this arm and the second arm.

    replay_poses(name="hand_over")                      every step after the first
    replay_poses(name="hand_over", from_step=2, to_step=5)

For each step, in the order shown: a gripper that was closed or opened by hand since the step
before is closed (until it holds) or opened first, where the arm then stood; then each arm that
moved goes to its pose, by the way the person took it there (the joint angles read in between,
thinned out), one arm at a time. When both moved in a step, this arm goes first, unless it has
just let go of something: then the second arm leaves first, with the thing.

It repeats joint angles, not intentions: a thing to be picked up must lie where it lay when the
poses were shown (`this_tool` of the gripping step in the recording says where), and nothing
checks for the other arm or for anything in the way. Returns what each gripper reported when it
closed. Both arms are woken (home) before the first step.
"""

import json
import math
import time

import numpy as np

from close_gripper import close_gripper
from home import home
from util import second_arm
from util.arm import RobotError


def _way(track: list, arm: str, t_from: float, t_to: float, q_end, coarse: float) -> list:
    """The joint angles read while the person moved that arm between two steps, thinned to one every `coarse` radians, ending at the step's own."""
    out, last = [], None
    for r in track:
        if t_from < r["t"] <= t_to:
            q = np.array(r[arm]["q"])
            if last is None or np.abs(q - last).max() > coarse:
                out.append(q)
                last = q
    if out and np.abs(out[-1] - np.array(q_end)).max() < coarse / 2:
        out.pop()
    return [*out, np.array(q_end)]


def replay_poses(ctx, name: str = "hand_over", from_step: int = 2, to_step: int | None = None, speed: float = 0.5) -> dict:
    path = ctx.root / "state" / "two_arm" / f"{name}.json"
    if not path.exists():
        raise RobotError(f"no poses recorded under that name ({path.relative_to(ctx.root)}): record_poses first")
    rec = json.loads(path.read_text())
    steps, track = rec["steps"], rec["track"]
    to_step = min(to_step or len(steps), len(steps))
    if not 2 <= from_step <= to_step:
        raise RobotError(f"from_step is 2 to {len(steps)} (step 1 is where the arms were when the recording began)")
    t0 = track[0]["t"]
    if not ctx.arm.state()["torque"]:
        ctx.call(home)
    if not second_arm.state()["torque"]:
        second_arm.call("home")
    done = []
    for n in range(from_step, to_step + 1):
        before, step = steps[n - 2], steps[n - 1]
        note = {"step": n}
        # grippers first, where the arms stand
        released = False
        change = step["this"]["gripper"] - before["this"]["gripper"]
        if change < -0.15:
            note["this_gripper"] = {k: v for k, v in ctx.call(close_gripper).items() if k in ("holding", "width_mm")}
        elif change > 0.15:
            ctx.arm.joints(gripper=min(1.0, step["this"]["gripper"]), label="letting go")
            released = True
        change = step["second"]["gripper"] - before["second"]["gripper"]
        if change < -0.15:
            note["second_gripper"] = second_arm.call("grasp").get("data")
        elif change > 0.15:
            second_arm.call("gripper", open=min(1.0, step["second"]["gripper"]))
        time.sleep(0.5 if len(note) > 1 or released else 0.0)
        # then the arms that moved
        order = [a for a in (("second", "this") if released else ("this", "second")) if a in step["moved"]]
        for arm in order:
            if np.abs(np.array(step[arm]["q"]) - np.array(before[arm]["q"])).max() < 0.03:
                continue  # only its gripper moved
            first = n == from_step  # where the arm is now is not where the recording had it: go straight there
            if arm == "this":
                way = [np.array(step[arm]["q"])] if first else _way(track, arm, t0 + before["at_s"], t0 + step["at_s"], step[arm]["q"], 0.12)
                ctx.arm.joints(np.array(way), speed=speed, label=f"shown pose {n}")
            else:
                way = [np.array(step[arm]["q"])] if first else _way(track, arm, t0 + before["at_s"], t0 + step["at_s"], step[arm]["q"], 0.3)
                for q in way:
                    second_arm.call("move_joints", joints={str(i + 1): round(math.degrees(float(v)), 1) for i, v in enumerate(q)}, speed=speed)
        done.append(note)
    return {"steps_done": done, "this_gripper": ctx.arm.state()["gripper"], "second_gripper": second_arm.state()["gripper"]}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(replay_poses)
