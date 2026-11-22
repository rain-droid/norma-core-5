"""Go and look at an object from the side with the wrist camera, to settle what the top camera cannot.

From above, two objects on each other are one outline, and so are two lying against each
other. The tracker concludes a pile from what it has watched (`on_source: "inferred"` in
track_objects); this tool goes and checks. Use it before picking from or stacking on a pile
that is only inferred, and for any outline that looks like more than one thing.

`object` is a tracked name (the upper or the lower one of a pile, it is the same place), or
give `x`, `y` for a spot. The arm brings the wrist camera about 20 cm from the spot, looking
down at it at a slant, from the robot's side and, where it can reach, from about 40 degrees to
the left and to the right of that, or further round (`views`, at most 3; `from_deg` says from
where each was taken). Each view is a wrist image (`image`) and the same with what was found
marked (`marked`): open them. Both are turned upright; they are for looking at, not for
pixel_to_table. Then it returns to the ready pose so the top camera sees the table again
(`back: false` leaves it at the last view).

What it reads from each view: the outline nearest the spot is split by colour, and for two
colours it says whether one is above the other in the picture or beside it. From all views:

  "pile"          the colours are one above the other in every view (`certain` only with two
                  views or more: in a single view an object standing behind another looks the same)
  "side_by_side"  some view shows them next to each other: two objects touching, no pile
  "one_colour"    one colour only. If the tracker had two differently coloured objects here,
                  the upper one is not there. If they have the same colour this cannot tell:
                  open the images and judge the height yourself.
  "not_seen"      no outline at the spot in any view (black, or the colour of the mat)

`colours` lists them from the top down, and `told_tracker` is what was passed on. Only colours that every view shows at the spot count:
something standing behind it gets into one view and not the others (`also_in_view`). With `object`, a sure verdict is passed to the tracker, about this object and the one it has
on or beside it, or else one that went missing and whose colour shows here: a certain pile
becomes `on` with `on_source: "seen"` (the upper colour is the upper object); side by side,
the one without an outline of its own is listed as `beside` the other and no longer `on` it;
one colour where the tracker had two, the upper one is `missing`.

Limits: beyond about 30 cm from the base few viewpoints are reachable, often only one. Layers
of the same colour are not separated. Positions through the wrist camera are 1 to 2 cm off on
the real arm: this counts and compares, it does not measure. The arm passes over the table on
its way; the robot side checks its own clearance, not the objects'.
"""

import json
import math
import os
import time

import cv2
import numpy as np

from pick_verified import _objects
from util import capture, tracker, vision
from util.arm import RobotError
from util.motion import READY_DEG, require_ready

AZIMUTHS = (0.0, 40.0, -40.0, 80.0, -80.0, 120.0, -120.0)  # degrees round the spot from the robot's side, in the order they are tried
SLANTS = (35.0, 25.0, 50.0)  # degrees the camera looks down at the spot, tried in this order
DISTANCES = (0.18, 0.23)  # metres from the camera to the spot
KEEP_OFF = 0.06  # metres the tool point stays away from the spot
AT = (0.5, 0.36)  # where in the wrist picture the spot should appear (the jaws fill the bottom)
PART = 0.12  # share of an outline a colour needs to count as a part of it
HUES = (("red", 0, 8), ("orange", 8, 20), ("yellow", 20, 33), ("green", 33, 85), ("blue", 85, 130), ("purple", 130, 160), ("pink", 160, 172), ("red", 172, 181))


def viewpoint(plan, cam: vision.CameraModel, spot: np.ndarray, azimuth: float, seed) -> dict | None:
    """Joint angles that put the wrist camera where it sees `spot` at a slant from `azimuth`
    degrees round it, or None. Worked out from the camera, not the tool point: the camera is
    placed on the line of sight and turned so that the spot appears at AT in the picture, and
    the tool pose follows from how the camera is mounted (beside the tool axis, tilted against
    it). The picture may be upright or upside down, whichever the wrist can do."""
    out = spot[:2] / np.linalg.norm(spot[:2])
    c, s = math.cos(math.radians(azimuth)), math.sin(math.radians(azimuth))
    along = np.array([c * out[0] - s * out[1], s * out[0] + c * out[1]])
    ray = cam.ray_cam(AT[0] * cam.width, AT[1] * cam.height)  # where in the camera's own frame the spot should be
    tilt = cv2.Rodrigues(np.cross(ray, [0.0, 0.0, 1.0]) / max(np.linalg.norm(np.cross(ray, [0.0, 0.0, 1.0])), 1e-9) * math.acos(np.clip(ray[2], -1, 1)))[0]  # turns that ray onto the optical axis
    mount = cam.T_tool_cam()
    for slant in SLANTS:
        e = math.radians(slant)
        sight = np.array([along[0] * math.cos(e), along[1] * math.cos(e), -math.sin(e)])  # from the camera to the spot
        right = np.cross([0.0, 0.0, -1.0], sight)
        right /= np.linalg.norm(right)
        level = np.column_stack([right, np.cross(sight, right), sight])  # a camera looking along the line of sight, picture upright
        for dist in DISTANCES:
            eye = spot - sight * dist
            for turned in (np.eye(3), np.diag([-1.0, -1.0, 1.0])):
                R = level @ turned @ tilt @ mount[:3, :3].T  # the tool's axes in the base frame
                tcp = eye - R @ mount[:3, 3]
                if np.linalg.norm(tcp - spot) < KEEP_OFF or tcp[2] < spot[2]:
                    continue  # the fingers would be in what it is looking at
                # the jaws are the same turned half round, so IK takes either way; the camera is not
                r = plan.kin.ik(tcp, approach=R[:, 2], jaw=R[:, 0], q_seed=seed, limit_margin=plan.margin,
                                accept=lambda q: plan.blocked(q) is None and plan.fk(q)[:3, 0] @ R[:, 0] > 0)  # fmt: skip
                time.sleep(0.005)  # IK is heavy: let whatever else runs in this process breathe
                if r.ok:
                    return {"q": r.q, "slant_deg": slant, "distance": dist}
    return None


def colour_parts(img: np.ndarray, mask: np.ndarray) -> list[dict]:
    """The colours an outline is made of, each with where it is in the picture, top one first.
    Shaded sides keep their hue; if most of the outline has colour, the colourless rest
    (shadow, glare) is left out, and if little of it has, all of that is one part."""
    h, s, v = cv2.split(cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV))
    inside = mask > 0
    plain = inside & (s < 45)
    names: dict[str, np.ndarray] = {}
    for name, lo, hi in HUES:
        names[name] = names.get(name, np.zeros_like(inside)) | (inside & ~plain & (h >= lo) & (h < hi))
    if plain.sum() > 0.6 * inside.sum():
        level = float(np.median(v[plain]))
        names["white" if level > 170 else "black" if level < 70 else "grey"] = plain
    parts = []
    for name, where in names.items():
        where = cv2.morphologyEx(where.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)) > 0
        if where.sum() >= PART * inside.sum():
            ys, xs = np.nonzero(where)
            parts.append({"color": name, "share": round(float(where.sum() / inside.sum()), 2), "u": float(xs.mean()), "v": float(ys.mean()), "mask": where})
    return sorted(parts, key=lambda p: p["v"])


def read_view(img: np.ndarray, expected, up) -> dict:
    """What one wrist view shows at the spot: its colours, uppermost first, and how two of them
    lie to each other. `up` is the direction in the picture that is up in the room: the camera
    is turned with the wrist, and in some poses the picture is upside down."""
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    blob = vision.nearest_blob(_objects(img), *expected, max_dist=0.3 * img.shape[1])
    marked = img.copy()
    cv2.drawMarker(marked, (int(expected[0]), int(expected[1])), (0, 255, 255), cv2.MARKER_CROSS, 22, 2)
    if blob is None:
        return {"colours": [], "arrangement": "not_seen", "marked": marked, "labels": [], "parts": []}
    mask = np.zeros(img.shape[:2], np.uint8)
    cv2.fillPoly(mask, [np.array(blob.polygon, np.int32)], 255)
    parts = colour_parts(img, mask)
    for p in parts:
        p["high"], p["across"] = p["u"] * up[0] + p["v"] * up[1], p["u"] * -up[1] + p["v"] * up[0]
    parts.sort(key=lambda p: -p["high"])
    for p in parts:
        outline = max(cv2.findContours(p["mask"].astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], key=cv2.contourArea)
        cv2.polylines(marked, [outline], True, (0, 255, 0), 2)
    return {"colours": [p["color"] for p in parts], "arrangement": arrangement(parts), "marked": marked, "labels": [(p["color"], p["u"], p["v"]) for p in parts],
            "parts": [{k: p[k] for k in ("color", "high", "across")} for p in parts]}  # fmt: skip


def arrangement(parts: list[dict]) -> str:
    """How the colours of one view lie to each other. One above the other in the picture is a
    pile (or one standing behind the other, from this side); next to each other is two objects
    on the table."""
    if len(parts) < 2:
        return "one_colour" if parts else "not_seen"
    beside = any(abs(b["across"] - a["across"]) > 0.8 * abs(b["high"] - a["high"]) for a, b in zip(parts, parts[1:]))
    return "side_by_side" if beside else "stacked"


def inspect_object(ctx, object: str | None = None, x: float | None = None, y: float | None = None, views: int = 3, speed: float = 0.6, back: bool = True) -> dict:
    if (object is None) == (x is None or y is None):
        raise RobotError("give either object (a tracked name) or x and y")
    if require_ready(ctx, awake=True)["gripper"]["holding"]:
        raise RobotError("holding something: the jaws would be in the picture; place it first")
    plan = ctx.plan
    base = top = None
    tracks: list[dict] = []
    height = 0.03
    if object is not None:
        tracks = tracker.fresh()["tracks"]
        t = tracker.track({"tracks": tracks}, object)
        if t["state"] in ("held", "missing"):
            raise RobotError(f"{object} is {t['state']}: nothing to look at")
        under = t["on"] or t.get("beside")
        base = next((o for o in tracks if o["id"] == under), t) if under else t
        riders = [o for o in tracks if o["on"] == base["id"]]
        top = t if t is not base else (riders[0] if riders else None)
        x, y = base["x"], base["y"]
        height = max(0.03, ((top or base)["z_mm"] + ((top or base)["height_mm"] or 25)) / 1000)
    spot = np.array([x, y, plan.table_z + height / 2])  # the middle of what stands there

    frame = ctx.frame("wrist", settle=0.0)
    cam = capture.wrist_camera(ctx.root).scaled(frame.shape[1], frame.shape[0])
    ctx.arm.joints(np.radians(READY_DEG), speed=speed, label="going home")  # every view is planned from here
    seen, unreachable = [], []
    for azimuth in AZIMUTHS:
        if len(seen) >= max(1, min(views, 3)):
            break
        pose = viewpoint(plan, cam, spot, azimuth, np.radians(READY_DEG))
        if pose is None:
            unreachable.append(azimuth)
            continue
        # Where the robot side runs in this same process (the tests), the search above has kept
        # it from reading its servos: give it a moment to have fresh state before it is asked to move.
        deadline = time.time() + 3.0
        while ctx.arm.why_not_ready() and time.time() < deadline:
            time.sleep(0.1)
        try:
            if seen:  # by way of the ready pose: straight from one view to the next the hand can sweep through the base
                ctx.arm.joints(np.radians(READY_DEG), speed=speed, label="going home")
            ctx.arm.joints(pose["q"], speed=speed, label="looking closer")
        except RobotError as e:
            if "rejected" not in str(e):
                raise
            unreachable.append(azimuth)  # the pose is fine, the way there is not
            continue
        img = ctx.frame("wrist", settle=0.5)
        n = len(seen) + 1
        T = plan.fk(ctx.arm.q())
        expected = cam.project(spot, T) or (AT[0] * cam.width, AT[1] * cam.height)
        above = cam.project(spot + [0.0, 0.0, 0.02], T) or (expected[0], expected[1] - 1.0)
        up = np.array(above) - expected
        view = read_view(img, expected, up)
        labels = view.pop("labels")
        if up[1] > 0:  # the picture is upside down: turned round for whoever opens it
            img, view["marked"] = cv2.rotate(img, cv2.ROTATE_180), cv2.rotate(view["marked"], cv2.ROTATE_180)
            labels = [(name, img.shape[1] - 1 - u, img.shape[0] - 1 - v) for name, u, v in labels]
        for name, u, v in labels:
            vision._label(view["marked"], name, (int(u) - 14, int(v)), scale=0.5)
        seen.append({
            "from_deg": azimuth,
            "slant_deg": round(pose["slant_deg"]),
            "distance_mm": round(pose["distance"] * 1000),
            "colours": view["colours"],
            "arrangement": view["arrangement"],
            "parts": view["parts"],
            "image": ctx.save_image(f"view{n}.jpg", img, f"wrist view {n}, upright"),
            "marked": ctx.save_image(f"view{n}_marked.jpg", view.pop("marked"), f"wrist view {n}: a cross at the spot, each colour found there outlined"),
        })  # fmt: skip
    if not seen:
        raise RobotError(f"no viewpoint for the wrist camera on x={x:+.3f} y={y:+.3f} is reachable; look with look_at_table or the top camera")
    if back:
        ctx.arm.joints(np.radians(READY_DEG), speed=speed, label="going home")

    # Something standing behind the spot gets into the outline from one side and not from the
    # others. What is at the spot is in every view that found anything: the rest is left out,
    # and each view is read again without it (`colours`, `arrangement`; `also_in_view` is the rest).
    found = [v for v in seen if v["parts"]]
    always = set.intersection(*(set(v["colours"]) for v in found)) if found else set()
    for v in seen:
        parts = [p for p in v.pop("parts") if p["color"] in always]
        v["also_in_view"] = [c for c in v["colours"] if c not in always]
        v["colours"], v["arrangement"] = [p["color"] for p in parts], arrangement(parts)
    found = [v for v in seen if v["arrangement"] != "not_seen"]
    several = [v for v in found if len(v["colours"]) > 1]
    if not found:
        verdict, certain = "not_seen", False
    elif any(v["arrangement"] == "side_by_side" for v in several):
        verdict, certain = "side_by_side", True
    elif several:
        verdict, certain = "pile", len(several) >= 2 and len(several) == len(found)
    else:
        verdict, certain = "one_colour", True
    out = {
        "verdict": verdict,
        "certain": certain,
        "colours": max(found, key=lambda v: len(v["colours"]))["colours"] if found else [],
        "looked_at": {"x": round(float(x), 3), "y": round(float(y), 3), "object": base["id"] if base else None, "on_it": top["id"] if top else None},
        "views": seen,
        "unreachable_from_deg": unreachable,
    }

    # What the tracker should make of it, where it is sure. The other object here is the one
    # the tracker has on or beside this one, else one that went missing and whose colour is seen.
    tell = None
    if base is not None:
        other = top or next((o for o in tracks if o["state"] == "missing" and o["color"] != base["color"] and o["color"] in out["colours"]), None)
        if other is not None and verdict == "one_colour" and other["color"] != base["color"] and top is not None:
            tell = {"upper": other["id"], "lower": base["id"], "verdict": "alone"}
        elif other is not None and base["color"] in out["colours"] and other["color"] in out["colours"] and other["color"] != base["color"]:
            upper, lower = sorted((base, other), key=lambda o: out["colours"].index(o["color"]))
            if verdict == "side_by_side" or (verdict == "pile" and certain):
                tell = {"upper": upper["id"], "lower": lower["id"], "verdict": verdict}
        elif top is not None and verdict == "side_by_side":
            tell = {"upper": top["id"], "lower": base["id"], "verdict": verdict}
    if tell:
        tracker.RUNTIME.mkdir(exist_ok=True)
        tmp = tracker.NOTES.with_name(tracker.NOTES.name + ".tmp")
        tmp.write_text(json.dumps(tell))
        os.replace(tmp, tracker.NOTES)
    out["told_tracker"] = tell
    return out


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(inspect_object)
