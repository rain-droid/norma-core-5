"""Record poses a person shows with both arms: this arm and the second arm (the SO-101).

Both arms should be limp (rest, or torque off) so they can be moved by hand. While this runs,
both arms' joints and gripper openings are read ten times a second. Each time everything has
been still for `still_s` seconds in a pose that differs from the last one kept, that pose of both
arms is kept as the next step. It ends when nothing has moved for `done_s` seconds after at
least one step, or after `max_seconds`.

Written to state/two_arm/NAME.json: `steps`, each with both arms' joint angles (radians, in
each arm's own joint order), gripper openings (0 to 1) and tool points (each in its own base
frame), and which arm moved to get there; and `track`, everything that was read, for looking at
how the person got from one pose to the next. replay_poses plays the steps back.
"""

import json
import time

import numpy as np

from util import second_arm
from util.arm import RobotError


def _read(ctx) -> dict:
    a, b = ctx.arm.state(), second_arm.state()
    return {
        "t": round(time.time(), 2),
        "this": {"q": [round(j["q"], 4) for j in a["joints"][:7]], "gripper": round(a["gripper"]["open"], 3), "tool": [round(v, 4) for v in a["ee"]["pos"]]},
        "second": {"q": [round(j["q"], 4) for j in b["joints"][:5]], "gripper": round(b["gripper"]["open"], 3), "tool": [round(v, 4) for v in b["ee"]["pos"]]},
    }


def _vector(r: dict) -> np.ndarray:
    return np.array([*r["this"]["q"], r["this"]["gripper"] * 0.5, *r["second"]["q"], r["second"]["gripper"] * 0.5])


def record_poses(ctx, name: str = "hand_over", max_seconds: float = 300.0, still_s: float = 1.5, done_s: float = 20.0) -> dict:
    if ctx.arm.meta()["mode"] == "sim":
        raise RobotError("the simulated table has no second arm")
    track, steps = [], []
    t0 = last_move = time.time()
    kept = None
    while time.time() - t0 < max_seconds:
        r = _read(ctx)
        track.append(r)
        v = _vector(r)
        recent = [_vector(x) for x in track if x["t"] > r["t"] - still_s]
        if len(recent) >= 5 and np.abs(np.array(recent) - v).max() > 0.012:
            last_move = time.time()
        still = time.time() - last_move
        if still >= still_s and track[0]["t"] <= r["t"] - still_s and (kept is None or np.abs(v - kept).max() > 0.06):
            moved = [] if kept is None else [arm for arm, part in (("this", slice(0, 8)), ("second", slice(8, 14))) if np.abs((v - kept)[part]).max() > 0.06]
            steps.append({**{k: r[k] for k in ("this", "second")}, "moved": moved, "at_s": round(r["t"] - t0, 1)})
            kept = v
            ctx.emit("log", level="info", text=f"pose {len(steps)} recorded")
        if len(steps) > 1 and still >= done_s:
            break
        time.sleep(0.1)
    path = ctx.root / "state" / "two_arm" / f"{name}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"name": name, "recorded": time.strftime("%Y-%m-%d %H:%M"), "steps": steps, "track": track}) + "\n")
    return {
        "steps": [{"n": i + 1, "at_s": s["at_s"], "moved": s["moved"], "this_tool": s["this"]["tool"], "this_gripper": s["this"]["gripper"],
                   "second_tool": s["second"]["tool"], "second_gripper": s["second"]["gripper"]} for i, s in enumerate(steps)],
        "seconds": round(time.time() - t0),
        "saved": str(path.relative_to(ctx.root)),
    }  # fmt: skip


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(record_poses)
