"""Work out the best way to grasp an object, considering what stands around it; optionally do it.

pick_at takes one grasp as given. This compares many and says why it prefers one:

* jaw directions: across each side of the object's footprint, and across its diagonals;
* where along the object: its middle, and for a long object a quarter of its length either way;
* how high: about half the object's height, and 6 mm higher.

A candidate is dropped when the jaws cannot span it (they open 53 mm), when a finger would come
down on a neighbouring object, or when the arm cannot get there, descend in a straight line and
lift. The rest are ranked by a cost, lower is better, whose parts are reported for each:

    clearance  fingers closer than 12 mm to a neighbour
    span       how much of the jaws' opening the object takes up
    lean       how far from straight down the gripper must point (far objects need a lean)
    twist      turned jaws on a leaning gripper, which dip one fingertip towards the table
    offset     gripping away from the object's middle, where it can swing
    limits     a joint within 0.15 rad of its limit at the grasp
    lift       no straight vertical lift from the grasp, only a retreat along the approach

Say which object by `object` (an id from find_objects, e.g. 2, or a tracked name such as
"red-1"; the newest listing is used), or describe it: `x`, `y` (metres), `footprint_mm` as
[across the jaws, the other way] for the jaw rotation `yaw_deg`, and `height_mm`. Everything
else in the listing is taken as an obstacle; `obstacles` adds more, each
{"x", "y", "yaw_deg", "footprint_mm", "height_mm"}. A neighbour lower than the fingertips is
not in the way.

Does not move the arm unless `pick: true`: then it grasps with the best candidate, opening the
jaws only as far as that grasp needs, and if the jaws close on nothing or the robot refuses,
goes on to the next candidates (`attempts`). Returns `candidates` (best first, each with the
pick_at arguments that would perform it), `rejected` with reasons, and `grasp_image`: the top
view with each candidate's jaw line drawn and numbered. Positions are only as good as
find_objects: about a centimetre.
"""

import json
import math
import time

import cv2
import numpy as np

from util import capture, tracker, vision
from util.arm import RobotError
from util.kinematics import jaw_from_yaw
from util.motion import require_ready

JAW_MAX = 0.053  # metres between the pads, fully open
PAD = (0.010, 0.022)  # a finger seen from above: thickness along the jaw axis, width across it
TIP_BELOW_TOOL = 0.012  # the fingertips reach this far beyond the tool point


def _rect(centre, axis, half_along: float, half_across: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)[:2]
    a = a / np.linalg.norm(a)
    n = np.array([-a[1], a[0]])
    c = np.asarray(centre, dtype=float)[:2]
    return np.array([c + sa * half_along * a + sn * half_across * n for sa, sn in ((1, 1), (-1, 1), (-1, -1), (1, -1))])


def _gap(p: np.ndarray, q: np.ndarray) -> float:
    """Distance between two convex polygons; negative when they overlap (by the smallest overlap)."""
    deepest = np.inf
    for poly in (p, q):
        for i in range(len(poly)):
            e = poly[(i + 1) % len(poly)] - poly[i]
            n = np.array([e[1], -e[0]]) / (np.linalg.norm(e) + 1e-12)
            a, b = p @ n, q @ n
            sep = max(a.min() - b.max(), b.min() - a.max())
            if sep > 0:  # a separating axis: they do not overlap, so measure corner to edge
                def to_edges(pts, other):
                    best = np.inf
                    for x in pts:
                        for j in range(len(other)):
                            s, t = other[j], other[(j + 1) % len(other)]
                            u = np.clip((x - s) @ (t - s) / ((t - s) @ (t - s) + 1e-12), 0, 1)
                            best = min(best, float(np.linalg.norm(x - (s + u * (t - s)))))
                    return best
                return min(to_edges(p, q), to_edges(q, p))
            deepest = min(deepest, -sep)
    return -float(deepest)


def _jaw_xy(yaw_deg: float) -> np.ndarray:
    """Table direction the jaws close along, for a jaw rotation as pick_at takes it."""
    return jaw_from_yaw([0.0, 0.0, -1.0], yaw_deg)[:2]


def _shape(o: dict) -> dict:
    """An object record as a rectangle on the table: centre, the axis its `yaw_deg` closes along, half sizes."""
    sides = [s / 1000 for s in (o.get("footprint_mm") or [o.get("width_mm") or 25])]
    across = (o["width_mm"] / 1000) if o.get("width_mm") else sides[0]
    other = next((s for s in sides if abs(s - across) > 1e-6), across)
    return {"c": np.array([o["x"], o["y"]], dtype=float), "axis": _jaw_xy(o.get("yaw_deg") or 0.0), "along": across, "across": other,
            "height": (o["height_mm"] / 1000) if o.get("height_mm") else None, "id": o.get("id")}  # fmt: skip


def _listing(ctx) -> list[dict]:
    live = tracker.read()
    if live:
        return [t for t in live["tracks"] if t.get("state") == "visible"]
    f = ctx.root / "perception" / "exteroception" / "objects.json"
    return json.loads(f.read_text())["objects"] if f.exists() else []


def candidates(ctx, target: dict, others: list[dict]) -> tuple[list[dict], list[dict]]:
    plan = ctx.plan
    lo, hi = ctx.arm.limits()
    height = target["height"] or 0.025
    n_axis = np.array([-target["axis"][1], target["axis"][0]])
    base_yaw = math.degrees(math.atan2(-target["axis"][1], target["axis"][0]))
    long_side = max(target["along"], target["across"])
    options = [(0.0, target["along"], 0.0), (90.0, target["across"], 0.0)]
    if abs(target["along"] - target["across"]) < 0.25 * long_side:  # near enough square: the diagonals are grasps too
        options += [(45.0, math.hypot(target["along"], target["across"]), 0.0), (-45.0, math.hypot(target["along"], target["across"]), 0.0)]
    if long_side > 0.05:  # a long object: also a quarter of its length either side of the middle
        turn, span, slide = (90.0, target["across"], target["axis"]) if target["along"] >= target["across"] else (0.0, target["along"], n_axis)
        options += [(turn, span, s * 0.25 * long_side) for s in (1, -1)]
        long_dir = slide
    good, bad = [], []
    for turn, span, offset in options:
        yaw = (base_yaw + turn + 90) % 180 - 90
        centre = target["c"] + (offset * np.asarray(long_dir)[:2] if offset else 0.0)
        for grasp_h in sorted({round(float(np.clip(0.5 * height, 0.022, 0.030)), 3), round(float(np.clip(0.5 * height, 0.022, 0.030)) + 0.006, 3)}):
            c = {"x": round(float(centre[0]), 3), "y": round(float(centre[1]), 3), "yaw_deg": round(yaw), "grasp_height": grasp_h, "closes_on_mm": round(span * 1000)}
            if span > JAW_MAX - 0.004:
                bad.append({**c, "why": f"{span * 1000:.0f} mm is wider than the jaws open"})
                break
            opening = min(JAW_MAX, span + 0.014)
            jaw = _jaw_xy(yaw)
            fingers = [_rect(centre + s * (opening / 2 + PAD[0] / 2 - (opening - span) / 4) * jaw, jaw, PAD[0] / 2 + (opening - span) / 4, PAD[1] / 2) for s in (1, -1)]
            tip = grasp_h - TIP_BELOW_TOOL
            clear, nearest = np.inf, None
            for o in others:
                if o["height"] is not None and o["height"] < tip - 0.003:
                    continue  # lower than the fingertips come: not in the way
                g = min(_gap(f, _rect(o["c"], o["axis"], o["along"] / 2, o["across"] / 2)) for f in fingers)
                if g < clear:
                    clear, nearest = g, o["id"]
            if clear < 0.002:
                bad.append({**c, "why": f"a finger would come down on object {nearest} ({clear * 1000:.0f} mm)"})
                continue
            try:
                approach, pitch, q_hover, _, grasp = plan.approach_for(np.array([centre[0], centre[1], plan.table_z]), grasp_h, yaw)
                down = plan.line(q_hover, grasp, approach=approach, allow_low=True)
            except RobotError as e:
                bad.append({**c, "why": str(e)})
                continue
            try:
                lift = plan.line(down[-1], grasp + [0.0, 0.0, 0.07], approach=approach, allow_low=True)
            except RobotError:
                lift = None
            radial = centre / (np.linalg.norm(centre) + 1e-9)
            cost = {
                "clearance": round(max(0.0, 0.012 - clear) / 0.012 * 3.0, 2) if np.isfinite(clear) else 0.0,
                "span": round(span / JAW_MAX, 2),
                "lean": round((90 - pitch) / 90 * 1.5, 2),
                "twist": round(math.cos(math.radians(pitch)) * abs(float(jaw @ radial)) * 2.0, 2),
                "offset": round(abs(offset) / max(long_side, 1e-6) * 2.0, 2),
                "limits": round(max(0.0, 0.15 - float(np.min(np.minimum(down[-1] - lo, hi - down[-1])))) / 0.15, 2),
                "lift": 0.0 if lift is not None else 0.5,
            }
            good.append({**c, "open_to": round(opening / JAW_MAX, 2), "approach_pitch_deg": pitch, "finger_clearance_mm": round(clear * 1000) if np.isfinite(clear) else None,
                         "cost": round(sum(cost.values()), 2), "cost_parts": cost, "_plan": (q_hover, down, lift), "_jaw": jaw, "_c": centre})  # fmt: skip
    good.sort(key=lambda c: c["cost"])
    return good, bad


def _picture(ctx, cands: list[dict], target: dict, others: list[dict]) -> str | None:
    try:
        meta = ctx.arm.meta()
        cam = capture.TopCamera(ctx.root, meta["mode"], meta["table_z"])
        img = cv2.resize(ctx.frame("top", settle=0.1), None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
    except RobotError:
        return None

    def px(p):
        return tuple(int(round(2 * v)) for v in cam.to_pixel([p[0], p[1], cam.table_z]))

    for o, colour in [(target, (0, 255, 0)), *((o, (0, 160, 255)) for o in others)]:
        cv2.polylines(img, [np.int32([px(p) for p in _rect(o["c"], o["axis"], o["along"] / 2, o["across"] / 2)])], True, colour, 1, cv2.LINE_AA)
    for rank, c in reversed(list(enumerate(cands[:5], 1))):
        colour = (0, 255, 255) if rank == 1 else (200, 200, 200)
        a, b = c["_c"] + c["_jaw"] * JAW_MAX / 2, c["_c"] - c["_jaw"] * JAW_MAX / 2
        cv2.line(img, px(a), px(b), colour, 2 if rank == 1 else 1, cv2.LINE_AA)
        vision._label(img, str(rank), px(a), scale=0.5, color=colour)
    return ctx.save_image("grasps.jpg", img, "grasp candidates: the line is where the jaws close, 1 is the best; green the object, orange its neighbours")


def _grasp(ctx, c: dict) -> dict:
    """Perform one planned candidate: as pick_at, but opening only as far as needed and lifting straight up when that is possible."""
    arm = ctx.arm
    q_hover, down, lift = c["_plan"]
    arm.joints(gripper=c["open_to"], label="opening")
    arm.joints(q_hover, speed=0.9, label="reaching")
    arm.joints(down, speed=0.5, allow_low=True, label="descending")
    grip = arm.joints(gripper=0.0, label="grasping")["gripper"]
    if not grip["holding"]:
        arm.joints(gripper=c["open_to"], label="opening")
        arm.joints(down[::-1], speed=0.7, allow_low=True, label="backing off")
        raise RobotError("closed on nothing")
    arm.joints(lift if lift is not None else down[::-1], speed=0.6, allow_low=True, label="lifting")
    time.sleep(0.25)
    if not arm.state()["gripper"]["holding"]:
        raise RobotError("it slipped out while lifting")
    return {"holding": True, "width_mm": grip["width_mm"]}


def plan_grasp(ctx, object: str | int | None = None, x: float | None = None, y: float | None = None, footprint_mm: list | None = None,
               yaw_deg: float = 0.0, height_mm: float | None = None, obstacles: list | None = None, pick: bool = False, attempts: int = 3) -> dict:  # fmt: skip
    state = require_ready(ctx, awake=pick)
    listed = _listing(ctx)
    if object is not None:
        match = [o for o in listed if str(o.get("id")) == str(object)]
        if not match:
            raise RobotError(f"no object '{object}' in the newest listing ({', '.join(str(o.get('id')) for o in listed) or 'empty'}): run find_objects or track_objects first")
        target = _shape(match[0])
        others = [_shape(o) for o in listed if o is not match[0]]
    elif x is not None and y is not None:
        fp = footprint_mm or [25, 25]
        target = _shape({"x": x, "y": y, "yaw_deg": yaw_deg, "width_mm": fp[0], "footprint_mm": fp, "height_mm": height_mm, "id": "target"})
        others = [_shape(o) for o in listed if math.hypot(o["x"] - x, o["y"] - y) > 0.012]
    else:
        raise RobotError("say which object: `object` (an id from find_objects or track_objects), or `x` and `y` with `footprint_mm`")
    others += [_shape({"id": f"obstacle-{i}", **o}) for i, o in enumerate(obstacles or [])]
    if height_mm is not None:
        target["height"] = height_mm / 1000

    good, bad = candidates(ctx, target, others)
    public = [{k: v for k, v in c.items() if not k.startswith("_")} for c in good]
    for c in public:
        c["pick_at"] = {"x": c["x"], "y": c["y"], "yaw_deg": c["yaw_deg"], "grasp_height": c["grasp_height"]}
    out = {"candidates": public[:6], "rejected": bad[:8], "neighbours_considered": [o["id"] for o in others]}
    image = _picture(ctx, good, target, others)
    if image:
        out["grasp_image"] = image
    if not good:
        out["note"] = "no grasp survives: see `rejected`. Move a neighbour out of the way first, or push the object clear."
        return out
    if pick:
        if state["gripper"]["holding"]:
            raise RobotError("already holding something; place it first")
        tried = []
        for c, shown in list(zip(good, public))[:attempts]:
            try:
                out.update(_grasp(ctx, c), used=shown)
                break
            except RobotError as e:
                tried.append({"candidate": shown["pick_at"], "outcome": str(e)})
        out["tried"] = tried
        if "holding" not in out:
            out["holding"] = False
            out["note"] = "none of the grasps tried came up holding: look again (find_objects) before another attempt"
    return out


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(plan_grasp)
