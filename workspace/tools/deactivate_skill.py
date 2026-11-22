"""End the skill in use, when its task is finished or abandoned.

Do this once the skill's closing condition holds, or when you give the task up, so that what is
recorded as in use is true. Says which skill it was.
"""

from activate_skill import NOTE


def deactivate_skill(ctx) -> dict:
    note = ctx.root / NOTE
    was = note.read_text().strip() if note.exists() else None
    note.unlink(missing_ok=True)
    if was:
        ctx.emit("log", level="info", text=f"skill ended: {was}")
    return {"active_skill": None, "was": was}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(deactivate_skill)
