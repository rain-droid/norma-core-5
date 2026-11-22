"""Write a new skill from a task you have just worked out, so the next session starts from it.

A skill is `skills/NAME/SKILL.md`: a description that says when it applies, and the procedure in
your own words: the steps that worked, what to check after each, what went wrong on the way and
what fixed it. Write one when you finish a task no skill covered, or when you found a better way
than a skill describes (`replace: true`).

`name` is lower-case words joined by hyphens ("stack-by-colour"). `description` is one sentence
that lets a later agent decide whether this is the skill for its task. `steps` is the procedure
as Markdown; number the steps, name the tools, and end with what must be true before leaving the
skill. Say what you observed on this arm, not what should work in general.

`calls` (default 12) appends "How it was done the first time": the last that many tool calls
from tool_call_history, each with its arguments and what came back, failures included. It is
evidence for the steps, and a later agent can see exactly what was run. `calls: 0` leaves it out.

The skill is available at once: activate_skill lists it, and agents that read skills/ find it.
Does not move the arm.
"""

import json
import re
import time

from util.arm import RobotError

KEEP = ("holding", "width_mm", "seen_in_jaws", "aligned_before_grasp", "placed_at", "released_something", "pose", "position_error_mm",
        "object", "on", "move_seen", "certain", "played", "verified_by_camera", "drawn", "planned", "strokes")  # fmt: skip


def _trace(root, calls: int) -> list[str]:
    """The last `calls` tool calls, oldest first, one line each."""
    folders = sorted(p for p in (root / "tool_call_history").iterdir() if p.name[:6].isdigit())
    lines = []
    for folder in folders[-(calls + 1) : -1]:  # not this call itself
        try:
            call = json.loads((folder / "input.json").read_text())
            result = json.loads((folder / "result.json").read_text())
        except (OSError, ValueError):
            continue
        if call["tool"] in ("create_skill", "activate_skill", "deactivate_skill"):
            continue
        args = json.dumps(call.get("arguments") or {}, separators=(",", ":"))
        if "error" in result:
            outcome = "FAILED: " + str(result["error"])[:160]
        else:
            shown = {k: result[k] for k in KEEP if k in result}
            if "objects" in result and isinstance(result["objects"], list):
                shown["objects"] = len(result["objects"])
            outcome = json.dumps(shown, separators=(",", ":")) if shown else "done"
        lines.append(f"{len(lines) + 1}. `{call['tool']}` `{args[:200]}` -> {outcome}")
    return lines


def create_skill(ctx, name: str, description: str, steps: str, calls: int = 12, replace: bool = False) -> dict:
    if not re.fullmatch(r"[a-z0-9]+(-[a-z0-9]+)*", name) or len(name) > 48:
        raise RobotError("name is lower-case words joined by hyphens, such as stack-by-colour")
    description = " ".join(description.split())
    if len(description) < 12 or len(description) > 300:
        raise RobotError("description is one sentence, at most 300 characters, saying when the skill applies")
    if len(steps.strip()) < 40:
        raise RobotError("steps is the procedure itself: numbered steps naming the tools, what to check, what went wrong")
    folder = ctx.root / "skills" / name
    path = folder / "SKILL.md"
    if path.exists() and not replace:
        raise RobotError(f"there is already a skill '{name}': read it (activate_skill), and pass replace true if yours is better")
    body = steps.strip() + "\n"
    trace = _trace(ctx.root, calls) if calls > 0 else []
    if trace:
        body += f"\n## How it was done the first time ({time.strftime('%Y-%m-%d')})\n\nThe tool calls behind this skill, in order, with what came back:\n\n" + "\n".join(trace) + "\n"
    folder.mkdir(parents=True, exist_ok=True)
    path.write_text(f"---\nname: {name}\ndescription: {description}\n---\n\n{body}")
    return {"skill": name, "file": str(path.relative_to(ctx.root)), "calls_recorded": len(trace), "replaced": bool(replace and path.exists()),
            "next": f"it is listed by activate_skill; try it on the same task from a fresh start to see that the steps are enough"}  # fmt: skip


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(create_skill)
