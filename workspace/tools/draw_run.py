"""Draw the planned strokes on the sheet with the pen in the gripper.

Needs a drawing (draw_plan) and a gripped pen (draw_pen). The drawing is scaled to fit the
sheet inside `margin` and centred. Every stroke is planned before any ink goes down, so a
drawing that does not fit the arm's reach is refused with the pen still in the air.

Each stroke is one joint command: lower the pen, follow the line with a waypoint every 4 mm,
lift `pen_up` metres. The gripper keeps the direction it gripped the pen in. `press` is how far
below the paper the tip is aimed, metres: the arm's own give turns that into pressure. If the
lines are faint or broken raise it a millimetre at a time; if the pen digs in or the arm
reports a blocked joint, lower it (it may be negative). `speed` is the arm's speed scale while
drawing; slower lines are straighter.

`dry_run: true` plans and reports without moving: do it first. `first_stroke` starts part way
through; after a failure the error says which stroke to resume from. Strokes are numbered as
in draw_plan's preview.

Returns the drawing's size on the paper. It does not know what the paper looks like: look
with capture_camera afterwards and compare with the preview.
"""

import numpy as np

from util import strokes as S
from util.arm import RobotError
from util.motion import require_ready

STEP = 0.004  # metres between waypoints along a line


def plan_strokes(ctx, st: dict, margin: float, pen_up: float, press: float) -> tuple[list[np.ndarray], float]:
    """Joint paths, one per stroke: from above its start, down, along it, and up again."""
    plan = ctx.plan
    a, reach = np.array(st["pen"]["approach"]), st["pen"]["tip_to_tool"]
    lines, width = S.on_sheet(st["drawing"], st["sheet"]["corners"], margin)

    def tool(xy, height):  # the tool point that puts the pen tip `height` above the paper at xy
        return np.array([xy[0], xy[1], plan.table_z + height]) - a * reach

    q = None
    paths = []
    for i, line in enumerate(lines):
        pts = S.dense(line, STEP)
        start = plan.solve(tool(pts[0], pen_up), approach=a, yaw=0.0, q_seed=q if q is not None else ctx.arm.q(), allow_low=True)
        if not start.ok:
            raise RobotError(f"stroke {i} starts out of reach: {plan.unreachable(tool(pts[0], pen_up), start)}")
        path = [start.q]
        try:
            for target in [tool(pts[0], -press), *[tool(p, -press) for p in pts[1:]], tool(pts[-1], pen_up)]:
                path.extend(plan.line(path[-1], target, approach=a, allow_low=True)[1:])
        except RobotError as e:
            raise RobotError(f"stroke {i} cannot be drawn: {e}") from None
        q = path[-1]
        paths.append(np.array(path))
    return paths, width


def draw_run(ctx, margin: float = 0.015, pen_up: float = 0.015, press: float = 0.001, speed: float = 0.3, first_stroke: int = 0,
             dry_run: bool = False) -> dict:  # fmt: skip
    state = require_ready(ctx, awake=True)
    st = S.load()
    if "drawing" not in st:
        raise RobotError("nothing to draw: run tools/draw_plan.py first")
    if "pen" not in st:
        raise RobotError("no pen in hand: run tools/draw_pen.py (corners, then grip) first")
    if not dry_run and not state["gripper"]["holding"]:
        raise RobotError("the gripper is not holding anything: the pen has gone. Run tools/draw_pen.py again")
    paths, width = plan_strokes(ctx, st, margin, pen_up, press)
    info = {
        "strokes": len(paths),
        "drawing_mm": [round(width * 1000), round(width * st["drawing"]["height"] * 1000)],
        "waypoints": int(sum(len(p) for p in paths)),
        "gripper_pitch_deg": st["sheet"]["pitch_deg"],
    }
    if dry_run:
        return {"planned": True, **info}
    for i, path in enumerate(paths):
        if i < first_stroke:
            continue
        try:
            ctx.arm.joints(path[0], speed=0.6, allow_low=True, label=f"to stroke {i}")
            # no slower second try on paper: a blocked stroke is the pen pressing too hard, and the
            # second try starts at the nearest waypoint, which on a closed shape can be its end
            ctx.arm.joints(path[1:], speed=speed, allow_low=True, label=f"stroke {i}", retry_blocked=False)
        except RobotError as e:
            raise RobotError(f"stopped during stroke {i} of {len(paths)}: {e}. Lift the pen clear (adjust_tcp, dz 0.03), "
                             f"see what happened, then continue with first_stroke {i}.") from None  # fmt: skip
        st["strokes_done"] = i + 1
        S.save(st)
    return {"drawn": True, **info}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(draw_run)
