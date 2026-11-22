"""Pick up the object at x, y with the wrist camera checking the grasp before and after.

pick_at goes where it is told and reports only what the jaws felt. This adds two looks through
the wrist camera, each saved as an image to open:

 1. Before descending, from the hover point: is the object where the jaws are about to close?
    The object is taken to be the outline nearest to where x, y should appear. Its far top edge
    is at `object_height` above the surface and its near bottom edge is on the surface; half way
    between those two points is where it stands. That is compared with the aim:
    `lateral_mm` across the jaws (they only forgive about (53 - object width) / 2) and
    `along_mm` along the fingers. If either is more than `tolerance_mm` the aim is moved onto
    the object and the arm hovers again, at most `corrections` times, at most 4 cm each.
    `aligned_before_grasp` says whether the last look was within tolerance.
 2. After lifting: is the object between the jaws? The gap between the fingers in the wrist
    image should be filled with the colour the object had in the first look.
    `seen_in_jaws` is that, `jaws_filled_fraction` how much of the gap it fills.

Trust the pick when `holding`, `seen_in_jaws` and a `width_mm` that matches the object agree.
`holding` true with `seen_in_jaws` false means the jaws are held apart by something else, or
the object hangs from a corner: look at `after_image`.

`object_height` matters: a wrong height moves the measured position along the fingers by
about as much as the error. Prefer depth_scan(at=[[x,y]]) at fresh top-camera coordinates;
divide the corresponding heights_mm entry by 1000 for object_height. A stacked reading is total height above the
mat, not the upper object's thickness. Missing/occluded depth is not zero height. The first
look cannot see an object that is black or the colour of the mat, nor one hidden behind the
jaws; it then says `seen_before_grasp: false` and the grasp goes ahead at x, y as given.
`grasp_height` and `yaw_deg` are as in pick_at.
The real-arm approach includes the configured extra descent clearance, currently 15 mm on
top of the 10 mm correction. Do not undo it to compensate for an empty grasp.

Positions seen through the wrist camera are in the arm's own frame, so this also shows how far
the top camera's x, y was from where the arm found the object: `corrections_mm`. If that is
regularly more than a centimetre in one direction, `base_in_mat` in robot_config/camera_top.json
(where the robot stands on the mat) is off.
"""

import time

import cv2
import numpy as np

from util import capture, vision
from util.arm import RobotError
from util.motion import require_ready, tcp_report

GAP = (0.38, 0.60, 0.58, 0.92)  # where a held object shows in the wrist image: x0, x1, y0, y1 as fractions


def _jaws(img: np.ndarray) -> np.ndarray:
    """Pixels of the gripper's own black jaws: dark, in the lower part of the wrist view."""
    dark = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) < 55
    dark[: int(0.5 * img.shape[0])] = False
    return cv2.dilate(dark.astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)


def _objects(img: np.ndarray) -> list:
    """Outlines in the wrist view that can be an object to grasp: not the jaws, not the table edge
    or tape filling a side of the picture, and not the gripper's own shadow on the mat (darker
    than the mat and colourless)."""
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    mat_l = float(np.median(lab[..., 0]))
    keep = []
    for b in vision.segment_objects(img, ignore=_jaws(img), min_area=500):
        mask = np.zeros(img.shape[:2], np.uint8)
        cv2.fillPoly(mask, [np.array(b.polygon, np.int32)], 255)
        l, a, bb = cv2.mean(lab, mask=mask)[:3]
        shadow = l < mat_l and np.hypot(a - 128, bb - 128) < 12
        if b.area < 0.15 * img.shape[0] * img.shape[1] and not shadow:
            keep.append(b)
    return keep


def _same_colour(img: np.ndarray, blob, expected) -> tuple[np.ndarray, tuple]:
    """The part of an outline that is one object: objects standing close together, and the
    shadow between them, come out of the segmentation as a single outline. Starting from the
    pixel of it nearest the aim, keep what has that pixel's colour and is connected to it.
    Returns (mask, the colour in Lab)."""
    whole = np.zeros(img.shape[:2], np.uint8)
    cv2.fillPoly(whole, [np.array(blob.polygon, np.int32)], 255)
    ys, xs = np.nonzero(whole)
    i = int(np.argmin((xs - expected[0]) ** 2 + (ys - expected[1]) ** 2))
    seed = (int(xs[i]), int(ys[i]))
    lab = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2LAB).astype(np.float32)
    rows, cols = np.indices(whole.shape)
    near = (whole > 0) & ((cols - seed[0]) ** 2 + (rows - seed[1]) ** 2 <= 64)
    colour = tuple(float(v) for v in np.median(lab[near], axis=0))
    coloured = np.hypot(colour[1] - 128, colour[2] - 128) > 15
    d = np.sqrt((0.1 if coloured else 0.35) * (lab[..., 0] - colour[0]) ** 2 + (lab[..., 1] - colour[1]) ** 2 + (lab[..., 2] - colour[2]) ** 2)
    alike = ((d < 18) & (whole > 0)).astype(np.uint8)
    alike = cv2.morphologyEx(alike, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    _, labels = cv2.connectedComponents(alike)
    if labels[seed[1], seed[0]] == 0:  # the seed itself was filtered away: take the largest piece
        counts = np.bincount(labels.ravel())[1:]
        if not len(counts):
            return whole, colour
        return (labels == 1 + int(np.argmax(counts))).astype(np.uint8), colour
    return (labels == labels[seed[1], seed[0]]).astype(np.uint8), colour


def look(ctx, aim: np.ndarray, object_height: float, surface_z: float) -> dict | None:
    """From where the gripper is now: where the object nearest the aim stands, by the wrist camera."""
    img = ctx.frame("wrist", settle=0.5)
    cam = capture.wrist_camera(ctx.root).scaled(img.shape[1], img.shape[0])
    T = ctx.plan.fk(ctx.arm.q())
    expected = cam.project([aim[0], aim[1], surface_z + object_height], T)
    if expected is None:
        return None
    blob = vision.nearest_blob(_objects(img), *expected, max_dist=0.3 * img.shape[1])
    if blob is None:
        return None
    part, colour = _same_colour(img, blob, expected)
    ys, xs = np.nonzero(part)
    x1, y1, x2, y2 = xs.min(), ys.min(), xs.max(), ys.max()
    u = (x1 + x2) / 2
    far_top = cam.pixel_to_plane(u, y1, T, surface_z + object_height)
    near_bottom = cam.pixel_to_plane(u, y2, T, surface_z)
    if far_top is None or near_bottom is None:
        return None
    stands = (far_top[:2] + near_bottom[:2]) / 2
    jaw = T[:2, 0] / np.linalg.norm(T[:2, 0])
    along = T[:2, 2] / np.linalg.norm(T[:2, 2])
    off = stands - aim[:2]
    outline = max(cv2.findContours(part, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], key=cv2.contourArea)
    marked = img.copy()
    cv2.polylines(marked, [outline], True, (0, 255, 0), 2)
    cv2.drawMarker(marked, (int(expected[0]), int(expected[1])), (0, 255, 255), cv2.MARKER_CROSS, 22, 2)
    vision._label(marked, "aim", (int(expected[0]) + 12, int(expected[1]) - 8), scale=0.5, color=(0, 255, 255))
    return {"stands": stands, "lateral_mm": float(off @ jaw) * 1000, "along_mm": float(off @ along) * 1000, "colour": colour, "marked": marked}


def in_jaws(img: np.ndarray, colour) -> tuple[float, np.ndarray]:
    """How much of the gap between the fingers shows the object's colour, and the view with the gap drawn."""
    h, w = img.shape[:2]
    x0, x1, y0, y1 = int(GAP[0] * w), int(GAP[1] * w), int(GAP[2] * h), int(GAP[3] * h)
    lab = cv2.cvtColor(cv2.GaussianBlur(img[y0:y1, x0:x1], (5, 5), 0), cv2.COLOR_BGR2LAB).astype(np.float32)
    # between the fingers the object is in shadow: a coloured object is matched on colour alone,
    # only a white or grey one on lightness as well
    coloured = np.hypot(colour[1] - 128, colour[2] - 128) > 15
    d = np.sqrt((0.0 if coloured else 0.35) * (lab[..., 0] - colour[0]) ** 2 + (lab[..., 1] - colour[1]) ** 2 + (lab[..., 2] - colour[2]) ** 2)
    marked = img.copy()
    cv2.rectangle(marked, (x0, y0), (x1, y1), (0, 255, 255), 2)
    return float((d < (15 if coloured else 22)).mean()), marked


def pick_verified(ctx, x: float, y: float, yaw_deg: float | None = 0.0, grasp_height: float = 0.022, object_height: float = 0.025,
                  corrections: int = 2, tolerance_mm: float = 8.0) -> dict:  # fmt: skip
    arm, plan = ctx.arm, ctx.plan
    if require_ready(ctx, awake=True)["gripper"]["holding"]:
        raise RobotError("already holding something; place it first")
    aim = np.array([x, y, plan.table_z], dtype=float)
    approach, pitch, q_hover, _, grasp = plan.approach_for(aim, grasp_height, yaw_deg)
    down = plan.line(q_hover, grasp, approach=approach, allow_low=True)
    arm.joints(gripper=1.0, label="opening")
    arm.joints(q_hover, speed=0.9, label="reaching")

    moved, seen, before, colour = [], None, None, None  # colour: the object's, from the last look that found it
    for attempt in range(corrections + 1):
        seen = look(ctx, aim, object_height, plan.table_z)
        if seen is None:
            break
        colour = seen["colour"]
        before = ctx.save_image("before.jpg", seen["marked"], "wrist view from the hover point: the object outlined, a cross where the jaws aim")
        off = seen["stands"] - aim[:2]
        if (abs(seen["lateral_mm"]) <= tolerance_mm and abs(seen["along_mm"]) <= tolerance_mm) or attempt == corrections:
            break
        if np.linalg.norm(off) > 0.04:
            off *= 0.04 / np.linalg.norm(off)
        moved.append([round(float(off[0]) * 1000, 1), round(float(off[1]) * 1000, 1)])
        aim[:2] += off
        approach, pitch, q_hover, _, grasp = plan.approach_for(aim, grasp_height, yaw_deg)
        down = plan.line(q_hover, grasp, approach=approach, allow_low=True)
        arm.joints(q_hover, speed=0.6, label="lining up")
    aligned = bool(seen is not None and abs(seen["lateral_mm"]) <= tolerance_mm and abs(seen["along_mm"]) <= tolerance_mm)
    report = {
        "seen_before_grasp": seen is not None,
        "aligned_before_grasp": aligned,
        "alignment_mm": {"lateral": round(seen["lateral_mm"], 1), "along": round(seen["along_mm"], 1)} if seen else None,
        "corrections_mm": moved,
        "before_image": before,
        "aimed_at": [round(float(aim[0]), 3), round(float(aim[1]), 3)],
    }

    arm.joints(down, speed=0.5, allow_low=True, label="descending")
    grip = arm.joints(gripper=0.0, label="grasping")["gripper"]
    if not grip["holding"]:
        arm.joints(gripper=1.0, label="opening")
        arm.joints(down[::-1], speed=0.7, allow_low=True, label="backing off")
        return {"holding": False, **report, "note": "the jaws closed on nothing: open before_image to see where the object was against the aim"}
    res = arm.joints(down[::-1], speed=0.6, allow_low=True, label="lifting")
    time.sleep(0.3)
    holding = bool(arm.state()["gripper"]["holding"])
    out = {"holding": holding, "width_mm": grip["width_mm"], **report, "approach_pitch_deg": pitch, "tcp": tcp_report(res["tcp"], res["gripper"])}
    if colour is not None:
        fraction, marked = in_jaws(ctx.frame("wrist", settle=0.3), colour)
        out["jaws_filled_fraction"] = round(fraction, 2)
        out["seen_in_jaws"] = bool(fraction > 0.25)
        out["after_image"] = ctx.save_image("after.jpg", marked, "wrist view after lifting: the object should fill the yellow box between the fingers")
    return out


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(pick_verified)
