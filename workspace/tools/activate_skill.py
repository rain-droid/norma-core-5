"""Take up a skill: get its instructions, and note it as the one in use.

Skills are task procedures written from experience with this arm, one folder each under
skills/ with a SKILL.md. Activate the one that fits before starting a task and follow it;
each ends with what must be true before you leave it. One skill is in use at a time:
activating another replaces it, and deactivate_skill ends it. Call this with no name to see
what there is. You may also just read the files, and write new ones.
"""

from util.arm import RobotError

NOTE = ".runtime/active_skill"  # which skill is in use, for whoever looks


def skills(root) -> dict[str, dict]:
    found = {}
    for path in sorted((root / "skills").glob("*/SKILL.md")):
        head, _, body = path.read_text().removeprefix("---\n").partition("\n---\n")
        fields = dict(line.split(": ", 1) for line in head.splitlines() if ": " in line)
        found[path.parent.name] = {"description": fields.get("description", ""), "instructions": body.strip(), "file": str(path.relative_to(root))}
    return found


def activate_skill(ctx, name: str = "") -> dict:
    known = skills(ctx.root)
    if name not in known:
        listing = "; ".join(f"{k}: {v['description']}" for k, v in known.items())
        raise RobotError(f"{'no skill named ' + repr(name) if name else 'give the name of a skill'}. There are: {listing}")
    note = ctx.root / NOTE
    previous = note.read_text().strip() if note.exists() else None
    note.parent.mkdir(exist_ok=True)
    note.write_text(name + "\n")
    ctx.emit("log", level="info", text=f"skill in use: {name}")
    return {"active_skill": name, "replaced": previous if previous != name else None, "file": known[name]["file"], "instructions": known[name]["instructions"]}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(activate_skill)
