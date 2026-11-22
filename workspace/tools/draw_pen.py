"""Say where the sheet of paper is, and take hold of the pen so its tip is known.

Two calls, with a person's hand in between:

 1. `corners`: the sheet's four corners in the base frame, metres, as [x, y]: bottom-left,
    bottom-right, top-right, top-left of the drawing as it should read (get them from a top
    capture with pixel_to_table). The arm chooses the most upright gripper direction that
    reaches the whole sheet, goes to the middle of it with the tool point `seat_height` above
    the paper, and opens its jaws. Then ask the person to stand the pen between the jaws,
    parallel to the fingers, cap off, **tip resting on the paper**, and to hold it there.
 2. `grip: true`: the jaws close on the pen. Because the tip was resting on the paper, the tip
    is now known exactly: it is on the paper, straight along the gripper's direction from the
    tool point. No measuring, and no pressing the pen down to find the paper.

`holding: false` after the second call means the jaws closed on nothing: open them again
(open_gripper), have the pen put back and repeat `grip: true`.

The gripper keeps that one direction for the whole drawing, so the pen never tilts. If the pen
slips in the jaws, or the paper is moved, do both calls again. `release: true` opens the jaws
to give the pen back.
"""

import numpy as np

from util import strokes as S
from util.arm import RobotError
from util.kinematics import approach_from_pitch
from util.motion import READY_DEG, require_ready


def sheet_approach(ctx, corners, tip_to_tool: float):
    """The steepest gripper direction with which the tool point can be over every corner and the
    middle of the sheet. Returns (unit approach vector, pitch in degrees)."""
    plan = ctx.plan
    pts = [np.asarray(c, dtype=float) for c in corners]
    centre = np.mean(pts, axis=0)
    why = ""
    for pitch in (90, 80, 70, 60, 50, 45):
        a = approach_from_pitch(centre, pitch)
        q = np.radians(READY_DEG)
        try:
            for p in (centre, *pts):
                target = np.array([p[0], p[1], plan.table_z]) - a * tip_to_tool
                r = plan.solve(target, approach=a, yaw=0.0, q_seed=q, allow_low=True)
                if not r.ok:
                    raise RobotError(plan.unreachable(target, r))
                q = r.q
            return a, float(pitch)
        except RobotError as e:
            why = str(e)
    raise RobotError(f"the arm cannot keep a pen on the whole sheet: {why}. Move the sheet closer to the base (its far edge within about 30 cm) or use a smaller one.")


def draw_pen(ctx, corners: list | None = None, grip: bool = False, release: bool = False, seat_height: float = 0.05) -> dict:
    require_ready(ctx, awake=True)
    arm, plan, st = ctx.arm, ctx.plan, S.load()
    if release:
        arm.joints(gripper=1.0, label="letting go of the pen")
        st.pop("pen", None)
        S.save(st)
        return {"holding": False}
    if grip:
        if "sheet" not in st:
            raise RobotError("first call draw_pen with the sheet's corners")
        got = arm.joints(gripper=0.0, label="gripping the pen")
        if not got["gripper"]["holding"]:
            return {"holding": False, "note": "the jaws closed on nothing: open_gripper, have the pen put back between the jaws with its tip on the paper, and call draw_pen with grip true again"}
        tcp, a = np.array(got["tcp"]["pos"]), np.array(got["tcp"]["approach"])
        if a[2] > -0.5:
            raise RobotError("the gripper is not pointing down at the paper; call draw_pen with the sheet's corners again")
        tip_to_tool = float((tcp[2] - plan.table_z) / -a[2])  # along the gripper's direction, from the paper up to the tool point
        st["pen"] = {"tip_to_tool": round(tip_to_tool, 4), "approach": [round(float(v), 5) for v in a]}
        S.save(st)
        return {"holding": True, "pen_tip_beyond_tool_point_mm": round(tip_to_tool * 1000, 1), "width_mm": got["gripper"]["width_mm"],
                "next": "draw_run with dry_run true, then draw_run"}  # fmt: skip
    if corners is None or len(corners) != 4:
        raise RobotError("corners is four [x, y] positions: bottom-left, bottom-right, top-right, top-left of the sheet")
    c = np.array(corners, dtype=float)
    w, h = float(np.linalg.norm(c[1] - c[0])), float(np.linalg.norm(c[3] - c[0]))
    if min(w, h) < 0.04:
        raise RobotError(f"those corners make a sheet of {w * 1000:.0f} x {h * 1000:.0f} mm; check their order and positions")
    a, pitch = sheet_approach(ctx, c, seat_height / 0.7)
    centre = c.mean(axis=0)
    seat = np.array([centre[0], centre[1], plan.table_z]) - a * (seat_height / -a[2])
    r = plan.solve(seat, approach=a, yaw=0.0, q_seed=np.radians(READY_DEG))
    if not r.ok:
        raise RobotError(plan.unreachable(seat, r))
    st.update({"sheet": {"corners": c.round(4).tolist(), "approach": [round(float(v), 5) for v in a], "pitch_deg": pitch}})
    st.pop("pen", None)
    S.save(st)
    arm.joints(r.q, gripper=1.0, speed=0.6, label="presenting the jaws for the pen")
    return {
        "sheet_mm": [round(w * 1000), round(h * 1000)],
        "gripper_pitch_deg": pitch,
        "jaws": "open over the middle of the sheet",
        "next": "ask the person to stand the pen between the jaws, parallel to the fingers, tip resting on the paper, and hold it; then call draw_pen with grip true",
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(draw_pen)
