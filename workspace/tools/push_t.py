"""Push a flat T-shaped block across the table until it lies on a target pose: Push-T.

Give the target as `x`, `y` (metres, base frame: where the T's area centroid should end up) and
`yaw_deg` (its heading: at 0 the bar is on the far side from the base and the stem points back
at it; positive turns it counter-clockwise seen from above). The loop, each step of which you
can read below and do yourself:

 1. turn the arm aside and look with the top camera. Every outline on the table is flattened to
    a top-down picture at the T's height and the T's known shape is fitted to it
    (util/pushing.py fit_pose): that gives x, y and heading, and tells the T from other things;
 2. if it is within `tolerance_mm` and `tolerance_deg` of the target, stop;
 3. choose a push (util/pushing.py push_candidates): a point on one of the T's edges and a
    short straight push inwards, the one expected to remove most of the position and heading
    error together, among those the arm can reach;
 4. close the jaws, come down beside the T clear of it, push at most `max_step_mm` in a straight
    line with the closed fingertips, lift, and go back to 1.

It is the planner of StationHarness's simulated Push-T, with this workspace's camera and arm.
The model of how the T answers a push is rough; looking after every push is what makes it work.
Several short pushes are normal: 5 to 15 for a few centimetres and 20 or 30 degrees.

`shape_mm` is the T: [bar width, bar depth, stem width, stem length], default [50, 14, 16, 46].
`height_mm` is how thick it is (default 20): the top camera needs it to place the outline, and
the fingertips push at about half of it. A T flatter than 12 mm cannot be pushed: the fingertips
stay 8 mm above the mat. `max_pushes: 0` only looks, and reports where the T is.

Returns `reached`, where the T was last seen (`pose`: x, y, yaw_deg), `position_error_mm`,
`yaw_error_deg`, `overlap` (how much of the T and the target footprint coincide, 1 = exactly),
and `pushes`, one entry per push made. `image` is the last look: cyan the T as measured,
magenta the target, yellow the last planned push. Check it: a fit drawn on the wrong object
means every number here is about that object.

Errors: no T in view (`best_overlap` says how close the nearest outline came: move things that
hide it, or correct `shape_mm`), or no useful push the arm can reach (the T is too far out, too
close to the base, or against something). Nothing here knows about other objects: clear the
path first. On the real arm positions are a centimetre or two off beyond 30 cm from the base;
keep the T and its target within about 30 cm and do not expect better than a centimetre.
"""

import math

import cv2
import numpy as np

import find_objects
from util import capture, pushing
from util.arm import RobotError
from util.motion import READY_DEG, require_ready

FINGERTIP_BELOW_TCP = 0.012  # the fingertips are this far beyond the tool point
LOWEST_FINGERTIP = 0.008  # above the table; lower and they drag on the mat
HOVER = 0.07
SEEN_ENOUGH = 0.55  # overlap of the fitted outline with what the camera shows, below which it is not the T


def turn_aside(ctx, away_from_x: float = 0.0) -> None:
    """The ready pose with the base turned 75 degrees, to the side the T is not on, so the top
    camera sees the T whole."""
    lo, hi = ctx.arm.limits()
    aside = np.radians(READY_DEG)
    turn = min(math.radians(75), hi[0] - 0.05, -lo[0] - 0.05)
    trial = aside.copy()
    trial[0] = turn
    side = 1.0 if (ctx.plan.fk(trial)[0, 3] > 0) != (away_from_x > 0) else -1.0
    aside[0] = side * turn
    if np.max(np.abs(ctx.arm.q() - aside)) > 0.03:
        ctx.arm.joints(aside, speed=0.6, label="turning aside")


def top_down(cam: capture.TopCamera, mask: np.ndarray, height: float) -> tuple[np.ndarray, np.ndarray]:
    """A mask of the top image redrawn as seen from straight above, for things `height` metres
    above the table: one cell per pushing.RASTER metres. Returns it and the base-frame x, y of
    its cell (0, 0)."""
    x, y, w, h = cv2.boundingRect(mask.astype(np.uint8))
    px = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], dtype=np.float32)
    xy = np.array([cam.to_base(u, v, height)[:2] for u, v in px])
    origin = xy.min(axis=0) - 0.01
    size = np.ceil((xy.max(axis=0) + 0.01 - origin) / pushing.RASTER).astype(int)
    H = cv2.getPerspectiveTransform(px, ((xy - origin) / pushing.RASTER).astype(np.float32))
    return cv2.warpPerspective(mask.astype(np.uint8) * 255, H, (int(size[0]), int(size[1])), flags=cv2.INTER_LINEAR) > 127, origin


def see_t(ctx, polygon: np.ndarray, height: float) -> dict:
    """Where the T is, from the top camera now. The arm should be out of the way (turn_aside)."""
    img = np.median([ctx.frame("top")] + [ctx.frame("top", settle=0.08) for _ in range(2)], axis=0).astype(np.uint8)
    robot = ctx.arm.meta()
    meta = {"table_z": robot["table_z"], "joints_rad": [float(v) for v in ctx.arm.q()]}
    cam = capture.TopCamera(ctx.root, robot["mode"], robot["table_z"])
    cam.to_pixel([0.0, 0.2, robot["table_z"]])  # fails with the reason if the calibration lacks the camera's position
    area = cv2.contourArea(polygon.astype(np.float32))
    best = {"overlap": 0.0, "pose": None}
    for blob in find_objects.table_blobs(ctx, cam, img, meta)[0]:
        mask = np.zeros(img.shape[:2], np.uint8)
        cv2.fillPoly(mask, [np.array(blob.polygon, np.int32)], 255)
        top = find_objects.top_face(img, mask)  # the lit top without the shaded sides, when the two can be told apart
        if top is not None:
            mask[:] = 0
            cv2.fillPoly(mask, [np.round(top).astype(np.int32)], 255)
        seen, origin = top_down(cam, mask > 0, height)
        if not 0.4 * area < seen.sum() * pushing.RASTER**2 < 2.5 * area:
            continue  # nothing like the T's size
        pose, value = pushing.fit_pose(seen, origin, polygon)
        if value > best["overlap"]:
            best = {"overlap": value, "pose": pose}
    return {**best, "img": img, "cam": cam}


def picture(seen: dict, polygon: np.ndarray, height: float, target, push: dict | None = None) -> np.ndarray:
    img, cam = seen["img"].copy(), seen["cam"]
    z = cam.table_z + height

    def px(xy) -> tuple[int, int]:
        u, v = cam.to_pixel([xy[0], xy[1], z])
        return int(round(u)), int(round(v))

    for pose, colour in ((target, (232, 85, 255)), (seen["pose"], (255, 240, 66))):
        if pose is None:
            continue
        cv2.polylines(img, [np.array([px(p) for p in pushing.placed(polygon, pose)], np.int32)], True, colour, 2, cv2.LINE_AA)
        cv2.drawMarker(img, px(pose[:2]), colour, cv2.MARKER_CROSS, 12, 2)
        cv2.line(img, px(pose[:2]), px(np.asarray(pose[:2]) + pushing.rotation(pose[2]) @ [0.0, 0.03]), colour, 1, cv2.LINE_AA)  # towards the bar
    if push:
        cv2.arrowedLine(img, px(push["start"]), px(push["end"]), (116, 228, 255), 2, cv2.LINE_AA, tipLength=0.3)
        cv2.circle(img, px(push["contact"]), 4, (116, 228, 255), 1, cv2.LINE_AA)
    return img


def plan_motion(ctx, c: dict, tcp_z: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Joint paths for one push: to the hover point, down beside the T, the push, and up again.
    Raises RobotError if any part cannot be done; nothing has moved then."""
    plan = ctx.plan
    along_edge = np.array([-c["direction"][1], c["direction"][0]])
    yaw = find_objects.jaw_yaw(along_edge)  # jaws closed across the push, so the two fingertips press side by side
    approach, _, q_hover, _, low = plan.approach_for(np.array([*c["start"], plan.table_z]), tcp_z, yaw, hover=HOVER)
    down = plan.line(q_hover, low, approach=approach, allow_low=True)
    push = plan.line(down[-1], np.array([*c["end"], low[2]]), approach=approach, allow_low=True)
    up = plan.line(push[-1], np.array([*c["end"], low[2] + HOVER]), approach=approach, allow_low=True)
    return q_hover, down, push, up


def push_t(ctx, x: float, y: float, yaw_deg: float = 0.0, max_pushes: int = 15, tolerance_mm: float = 6.0, tolerance_deg: float = 6.0,
           shape_mm: list | None = None, height_mm: float = 20.0, max_step_mm: float = 12.0, turn: bool = True) -> dict:  # fmt: skip
    require_ready(ctx, awake=True)
    polygon = pushing.t_polygon(*[v / 1000 for v in (shape_mm or [50, 14, 16, 46])])
    height = height_mm / 1000
    if max_pushes and height < LOWEST_FINGERTIP + 0.004:
        raise RobotError(f"a T {height_mm:.0f} mm thick is too flat to push: the fingertips stay {LOWEST_FINGERTIP * 1000:.0f} mm above the mat and would pass over it")
    tcp_z = FINGERTIP_BELOW_TCP + max(LOWEST_FINGERTIP, 0.4 * height)
    target = np.array([x, y, math.radians(yaw_deg)])
    pushes: list[dict] = []
    last_x, seen, push = 0.0, None, None
    while True:
        turn_aside(ctx, away_from_x=last_x)
        seen = see_t(ctx, polygon, height)
        if seen["overlap"] < SEEN_ENOUGH:
            ctx.save_image("view.jpg", picture(seen, polygon, height, target), "top view: magenta the target; no T was found")
            raise RobotError(f"no T in view of the top camera after {len(pushes)} pushes (best_overlap {seen['overlap']:.2f} with the shape given): "
                             "is it on the mat, uncovered, and is shape_mm right?")  # fmt: skip
        pose = seen["pose"]
        last_x = float(pose[0])
        off, heading = pushing.errors(pose, target)
        if pushes:
            pushes[-1]["then_position_error_mm"], pushes[-1]["then_yaw_error_deg"] = round(off * 1000, 1), round(math.degrees(heading), 1)
        reached = off * 1000 <= tolerance_mm and (math.degrees(heading) <= tolerance_deg or not turn)
        if reached or len(pushes) >= max_pushes:
            break
        push = None
        refusals = []
        for c in pushing.push_candidates(polygon, pose, target, max_push=max_step_mm / 1000, turn=turn)[:10]:
            try:
                motion = plan_motion(ctx, c, tcp_z)
            except RobotError as e:
                refusals.append(str(e))
                continue
            push = c
            break
        if push is None:
            ctx.save_image("view.jpg", picture(seen, polygon, height, target), "top view: cyan the T as measured, magenta the target")
            raise RobotError(f"no useful push that the arm can make from here, after {len(pushes)} pushes "
                             f"(the T is at x={pose[0]:+.3f} y={pose[1]:+.3f}, {off * 1000:.0f} mm and {math.degrees(heading):.0f} deg off)"
                             + (f": {refusals[0]}" if refusals else ""))  # fmt: skip
        ctx.save_image(f"push_{len(pushes) + 1:02d}.jpg", picture(seen, polygon, height, target, push), f"before push {len(pushes) + 1}: cyan the T, magenta the target, yellow the push")
        q_hover, down, line, up = motion
        ctx.arm.joints(q_hover, gripper=0.0, speed=0.7, label="reaching")
        ctx.arm.joints(down, speed=0.4, allow_low=True, label="lowering")
        ctx.arm.joints(line, speed=0.15, allow_low=True, label="pushing")
        ctx.arm.joints(up, speed=0.5, allow_low=True, label="lifting")
        pushes.append({
            "from": [round(float(v), 3) for v in push["start"]], "to": [round(float(v), 3) for v in push["end"]],
            "push_mm": round(push["push_m"] * 1000, 1), "lever_mm": round(push["lever_m"] * 1000, 1), "edge": push["edge"],
        })  # fmt: skip
    if pushes:
        ctx.arm.joints(gripper=1.0, label="opening")
    image = ctx.save_image("view.jpg", picture(seen, polygon, height, target, push), "top view at the end: cyan the T as measured, magenta the target, yellow the last push")
    return {
        "reached": bool(reached),
        "pose": {"x": round(float(pose[0]), 4), "y": round(float(pose[1]), 4), "yaw_deg": round(math.degrees(pose[2]), 1)},
        "position_error_mm": round(off * 1000, 1),
        "yaw_error_deg": round(math.degrees(heading), 1),
        "overlap": round(pushing.overlap(polygon, pose, target), 3),
        "seen_overlap": round(seen["overlap"], 2),
        "pushes": pushes,
        "image": image,
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(push_t)
