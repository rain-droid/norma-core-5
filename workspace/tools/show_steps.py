"""Show the people watching what you are doing and what comes next.

The web UI has a "Steps" panel: the task, its steps in order, which one is being worked on.
It shows what this tool was last given, so call it when you start a task of more than one
step, again each time you move on to the next step, and once more when you are finished.

`steps` is the whole list every time, in plain words a bystander understands ("find the blue
cube", "pick it up", "set it on the pink box"): at most 12, a line each. `current` is the
number of steps already finished, so it is also the index of the step being worked on:
0 at the start, `len(steps)` when all are done. `title` is the task in a few words.
`failed: true` marks the current step as the one that went wrong. An empty `steps` clears
the panel.

Display only: it moves nothing and remembers nothing, and the robot does not read it. Change
the list freely when the plan changes. A person can switch the panel off; that is theirs to
decide and no reason to stop calling this.
"""

from util.arm import RobotError, _request


def show_steps(ctx, steps: list | None = None, current: int = 0, title: str = "", failed: bool = False) -> dict:
    steps = [] if steps is None else steps
    if not isinstance(steps, list) or not all(isinstance(s, str) and s.strip() for s in steps):
        raise RobotError("steps is a list of short sentences, one per step")
    if len(steps) > 12:
        raise RobotError(f"{len(steps)} steps is more than the panel shows: give at most 12, and merge the small ones")
    if isinstance(current, bool) or not isinstance(current, int) or not 0 <= current <= len(steps):
        raise RobotError(f"current is the number of steps already finished, 0 to {len(steps)}")
    shown = [s.strip()[:120] for s in steps]
    _request("/api/event", {"kind": "steps", "title": str(title).strip()[:120], "steps": shown, "current": current,
                            "failed": bool(failed) and current < len(shown), "source": ctx.arm.source}, timeout=5.0)  # fmt: skip
    if not shown:
        return {"shown": False, "note": "the steps panel is cleared"}
    return {"shown": True, "steps": len(shown), "working_on": shown[current] if current < len(shown) else None, "all_done": current == len(shown)}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(show_steps)
