"""Outline, number and locate whatever stands out on the table, seen by a camera.

No object model and no labels: the table is taken to be the dominant colour of the search
area, and every connected region far from it in colour is an object. A colour word is attached
so objects can be told apart.

`camera: "top"` (default) is the one to trust: it sees the whole mat and is calibrated by the
corner tags. Only the part of the table the arm can work on is searched (SEARCH_X, SEARCH_Y
below), and the arm's own body is masked out using its joint angles. For each object the edge
where it meets the table on the camera's side is traced and turned into table positions:

* two sides visible (a box seen corner-on, or anything round): the footprint is completed from
  the two visible sides, so `x, y` is its centre and `footprint_mm` its two side lengths;
* one side visible (a box seen face-on, or nearly): the depth cannot be seen, and is taken to
  equal the width. `footprint_mm` then has one number, and for an object that is not square
  `x, y` is off by half the difference between its width and its depth.

`yaw_deg` is the jaw rotation for pick_at that closes the jaws across flat sides, and
`width_mm` what they would close on (they open 53 mm). Heights are not measured.

`camera: "wrist"` does the same from wherever the gripper is, with the wrist camera's rough
model: useful up close, centimetres off on the real arm.

Does not move the arm. Returns a new `image_id` and `annotated_image`: open it, and check that
each outline is a real object and each cross sits in the middle of where the object stands.
Shadows, reflections and things the colour of the table fool it; if it misses something, read
the pixel yourself and use pixel_to_table. Also writes perception/exteroception/objects.json.
"""

import json
import math

import cv2
import numpy as np

from elrobot import vision
from util import capture

# The part of the table that is searched from the top camera, metres in the base frame.
SEARCH_X = (-0.24, 0.24)
SEARCH_Y = (0.14, 0.40)


def color_name(img: np.ndarray, blob: vision.Blob) -> str:
    mask = np.zeros(img.shape[:2], np.uint8)
    cv2.fillPoly(mask, [np.array(blob.polygon, np.int32)], 255)
    h, s, v = [float(np.median(c[mask > 0])) for c in cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))]
    if s < 45:
        return "white" if v > 170 else ("black" if v < 70 else "grey")
    for name, lo, hi in (("red", 0, 8), ("orange", 8, 20), ("yellow", 20, 33), ("green", 33, 85), ("blue", 85, 130),
                         ("purple", 130, 160), ("pink", 160, 172), ("red", 172, 181)):  # fmt: skip
        if lo <= h < hi:
            return name
    return ""


def jaw_yaw(direction) -> int:
    """Jaw rotation (degrees, as pick_at takes it) that closes the jaws along a table direction."""
    return round((-math.degrees(math.atan2(direction[1], direction[0])) + 90) % 180 - 90)


def arm_mask(ctx, cam: capture.TopCamera, shape, q) -> np.ndarray:
    """Pixels of the top view covered by the arm itself, from its joint angles: each body
    segment is a capsule, drawn generously (cables and the wrist camera stick out)."""
    mask = np.zeros(shape[:2], np.uint8)
    for start, end, radius in ctx.plan.kin.body_segments(q):
        for t in np.linspace(0.0, 1.0, 8):
            c = start + (end - start) * t
            uv = cam.to_pixel(c)
            r = int(np.linalg.norm(cam.to_pixel(c + [radius, 0.0, 0.0]) - uv) * 1.8) + 10
            cv2.circle(mask, (int(round(uv[0])), int(round(uv[1]))), r, 255, -1)
    return mask > 0


def _side(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The two ends of the straight line through some edge points (least squares)."""
    mean = points.mean(axis=0)
    direction = np.linalg.svd(points - mean)[2][0]
    along = (points - mean) @ direction
    return mean + direction * along.min(), mean + direction * along.max()


def footprint(cam: capture.TopCamera, blob: vision.Blob, shape) -> dict | None:
    """Where an object stands, from the edge where it meets the table on the camera's side."""
    mask = np.zeros(shape[:2], np.uint8)
    cv2.fillPoly(mask, [np.array(blob.polygon, np.int32)], 255)
    ys, xs = np.nonzero(mask)
    cols = np.unique(xs)[2:-2]  # the outermost columns are the corners of the top, not the base
    if len(cols) < 6:
        return None
    base = np.array([cam.to_base(u, ys[xs == u].max())[:2] for u in cols])  # lowest pixel of each column, on the table
    chord = base[-1] - base[0]
    off = np.abs(chord[0] * (base[:, 1] - base[0, 1]) - chord[1] * (base[:, 0] - base[0, 0])) / (np.linalg.norm(chord) + 1e-9)
    k = int(np.argmax(off))
    sides = [base]
    if off[k] > max(0.004, 0.12 * float(np.linalg.norm(chord))):  # the edge bends: a corner (or a curve) faces the camera
        sides = [base[: k + 1], base[k:]]
        if min(len(s) for s in sides) >= 8:  # both sides seen well enough to measure
            (a0, a1), (b0, b1) = _side(sides[0]), _side(sides[1])
            ea, eb = a0 - base[k], b1 - base[k]
            la, lb = float(np.linalg.norm(ea)), float(np.linalg.norm(eb))
            jaw, width = (a1 - a0, la) if la <= lb else (b1 - b0, lb)  # close along the shorter side
            return {"centre": base[k] + (ea + eb) / 2, "jaw": jaw, "width": width, "sides": sorted([la, lb], reverse=True)}
    # One side to go on (the other is hidden, or seen too edge-on to measure): the footprint is
    # taken to be as deep as that side is wide.
    p0, p1 = _side(max(sides, key=len))
    mid, face = (p0 + p1) / 2, p1 - p0
    width = float(np.linalg.norm(face))
    normal = np.array([-face[1], face[0]]) / width
    if normal @ (mid - cam.eye_base()[:2]) < 0:
        normal = -normal  # away from the camera, into the object
    return {"centre": mid + normal * width / 2, "jaw": face, "width": width, "sides": [width]}


def from_top(ctx, img: np.ndarray, meta: dict, max_objects: int) -> tuple[list[dict], list[dict]]:
    cam = capture.TopCamera(ctx.root, meta["mode"], meta["table_z"])
    z = meta["table_z"]
    area = np.zeros(img.shape[:2], np.uint8)
    corners = [cam.to_pixel([x, y, z]) for x, y in ((SEARCH_X[0], SEARCH_Y[0]), (SEARCH_X[1], SEARCH_Y[0]), (SEARCH_X[1], SEARCH_Y[1]), (SEARCH_X[0], SEARCH_Y[1]))]
    cv2.fillConvexPoly(area, np.round(corners).astype(np.int32), 255)
    ignore = (area == 0) | arm_mask(ctx, cam, img.shape, meta["joints_rad"])
    objects, marks = [], []
    for blob in vision.segment_objects(img, ignore=ignore, min_area=120):
        if len(objects) >= max_objects:
            break
        f = footprint(cam, blob, img.shape)
        if f is None:
            continue
        n = len(objects) + 1
        centre_px = cam.to_pixel([*f["centre"], z])
        objects.append({
            "id": n,
            "color": color_name(img, blob),
            "x": round(float(f["centre"][0]), 3),
            "y": round(float(f["centre"][1]), 3),
            "yaw_deg": jaw_yaw(f["jaw"]),
            "width_mm": round(f["width"] * 1000),
            "footprint_mm": [round(s * 1000) for s in f["sides"]],
            "pixel": [round(float(centre_px[0])), round(float(centre_px[1]))],
        })  # fmt: skip
        marks.append({"label": f"{n}", "polygon": blob.polygon, "point": [float(centre_px[0]), float(centre_px[1])], "box": list(blob.box)})
    return objects, marks


def from_wrist(ctx, img: np.ndarray, meta: dict, max_objects: int) -> tuple[list[dict], list[dict]]:
    cam = capture.wrist_camera(ctx.root).scaled(meta["width"], meta["height"])
    T, z = np.array(meta["T_base_tool"]), meta["table_z"]
    objects, marks = [], []
    for blob in vision.segment_objects(img, ignore=capture.jaw_mask(img)):
        if len(objects) >= max_objects:
            break
        p = cam.pixel_to_plane(*blob.centroid, T, z)
        if p is None or not (-0.45 < p[0] < 0.45 and -0.05 < p[1] < 0.60):
            continue
        # physical size: cast both ends of the outline's long and short axes onto the table
        a = math.radians(blob.angle_deg)
        spans = []
        for dx, dy, half in ((math.cos(a), math.sin(a), blob.length_px / 2), (-math.sin(a), math.cos(a), blob.width_px / 2)):
            e1 = cam.pixel_to_plane(blob.centroid[0] + dx * half, blob.centroid[1] + dy * half, T, z)
            e2 = cam.pixel_to_plane(blob.centroid[0] - dx * half, blob.centroid[1] - dy * half, T, z)
            spans.append(None if e1 is None or e2 is None else e1 - e2)
        if any(s is None for s in spans):
            continue
        long_v, short_v = sorted(spans, key=lambda s: -np.linalg.norm(s))
        n = len(objects) + 1
        objects.append({
            "id": n,
            "color": color_name(img, blob),
            "x": round(float(p[0]), 3),
            "y": round(float(p[1]), 3),
            "yaw_deg": jaw_yaw(short_v),
            "width_mm": round(float(np.linalg.norm(short_v)) * 1000),
            "footprint_mm": [round(float(np.linalg.norm(long_v)) * 1000), round(float(np.linalg.norm(short_v)) * 1000)],
            "pixel": [round(blob.centroid[0]), round(blob.centroid[1])],
        })  # fmt: skip
        marks.append(blob.as_mark(f"{n}"))
    return objects, marks


def find_objects(ctx, camera: str = "top", max_objects: int = 8) -> dict:
    img = ctx.frame(camera)
    meta = capture.freeze(ctx, camera, img, grid=False)
    objects, marks = (from_top if camera == "top" else from_wrist)(ctx, img, meta, max_objects)
    annotated = ctx.save_image("objects.jpg", vision.draw_marks(vision.draw_grid(img), marks), f"{camera} view: objects outlined and numbered, a cross where each stands")
    ctx.emit("detections", camera=camera, width=meta["width"], height=meta["height"],
             items=[{"label": f"#{o['id']}", "score": 1.0, "box": m["box"], "point": m["point"], "polygon": m["polygon"]} for o, m in zip(objects, marks)])  # fmt: skip
    ctx.emit("markers", markers=[{"id": f"obj{o['id']}", "label": f"#{o['id']}", "pos": [o["x"], o["y"], meta["table_z"] + 0.01], "color": "#ffd166", "radius": 0.012} for o in objects])
    listing = ctx.root / "perception" / "exteroception" / "objects.json"
    listing.parent.mkdir(parents=True, exist_ok=True)
    listing.write_text(json.dumps({"image_id": meta["image_id"], "camera": camera, "objects": objects}, indent=1) + "\n")
    return {"image_id": meta["image_id"], "camera": camera, "annotated_image": annotated, "objects": objects}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(find_objects)
