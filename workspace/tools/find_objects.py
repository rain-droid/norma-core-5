"""Outline and number whatever stands out on the table in the wrist camera's current view.

No object model and no labels: the table is taken to be the dominant colour of the frame, and
every connected region far from it in colour is an object. Each one's centre pixel is cast onto
the table plane to get x, y in metres (base frame), its outline gives a size, and a colour word
is attached so objects can be told apart. `yaw_deg` is the jaw rotation that closes across the
narrow side, for pick_at; null when the outline is roughly square.

Does not move the arm: call look_at_table first for an overview. Returns a new `image_id`
(this call's capture) and `annotated_image`; open it and check each outline is a real object.
Shadows, reflections and things the same colour as the table fool it. If it misses something,
read the pixel yourself and use pixel_to_table. Also writes perception/exteroception/objects.json.
"""

import json
import math

import cv2
import numpy as np

from elrobot import vision
from util import capture

MOTION = False

# Table area worth reporting, metres in the base frame: nothing further out is reachable.
X_RANGE = (-0.45, 0.45)
Y_RANGE = (-0.05, 0.60)


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


async def find_objects(ctx, max_objects: int = 8) -> dict:
    img = await ctx.frame("wrist")
    meta = capture.freeze(ctx, "wrist", img, grid=False)
    cam = ctx.skills.cam.scaled(meta["width"], meta["height"])
    T = np.array(meta["T_base_tool"])
    table_z = ctx.cfg.table_z

    objects, marks = [], []
    for blob in vision.segment_objects(img, ignore=capture.jaw_mask(img)):
        if len(objects) >= max_objects:
            break
        p = cam.pixel_to_plane(*blob.centroid, T, table_z)
        if p is None or not (X_RANGE[0] < p[0] < X_RANGE[1] and Y_RANGE[0] < p[1] < Y_RANGE[1]):
            continue
        # physical size: cast both ends of the outline's long and short axes onto the table
        a = math.radians(blob.angle_deg)
        spans = []
        for dx, dy, half in ((math.cos(a), math.sin(a), blob.length_px / 2), (-math.sin(a), math.cos(a), blob.width_px / 2)):
            e1 = cam.pixel_to_plane(blob.centroid[0] + dx * half, blob.centroid[1] + dy * half, T, table_z)
            e2 = cam.pixel_to_plane(blob.centroid[0] - dx * half, blob.centroid[1] - dy * half, T, table_z)
            spans.append(None if e1 is None or e2 is None else e1 - e2)
        if any(s is None for s in spans):
            continue
        long_v, short_v = sorted(spans, key=lambda s: -np.linalg.norm(s))
        long_mm, short_mm = float(np.linalg.norm(long_v) * 1000), float(np.linalg.norm(short_v) * 1000)
        # jaws close along the short side; yaw is measured about a straight-down approach
        jaw = short_v[:2] / (np.linalg.norm(short_v[:2]) + 1e-9)
        yaw = (math.degrees(math.atan2(-jaw[1], jaw[0])) + 90) % 180 - 90  # the gripper is symmetric
        n = len(objects) + 1
        objects.append({
            "id": n,
            "color": color_name(img, blob),
            "x": round(float(p[0]), 3),
            "y": round(float(p[1]), 3),
            "size_mm": [round(long_mm), round(short_mm)],
            "yaw_deg": round(yaw) if long_mm > 1.25 * short_mm else None,
            "pixel": [round(blob.centroid[0]), round(blob.centroid[1])],
        })  # fmt: skip
        marks.append(blob.as_mark(f"{n}"))

    annotated = ctx.save_image("objects.jpg", vision.draw_marks(vision.draw_grid(img), marks), "wrist view, objects outlined and numbered")
    ctx.emit("detections", camera="wrist", width=meta["width"], height=meta["height"],
             items=[{"label": f"#{o['id']}", "score": 1.0, "box": m["box"], "point": m["point"], "polygon": m["polygon"]} for o, m in zip(objects, marks)])  # fmt: skip
    ctx.emit("markers", markers=[{"id": f"obj{o['id']}", "label": f"#{o['id']}", "pos": [o["x"], o["y"], table_z + 0.01], "color": "#ffd166", "radius": 0.012} for o in objects])
    listing = ctx.root / "perception" / "exteroception" / "objects.json"
    listing.parent.mkdir(parents=True, exist_ok=True)
    listing.write_text(json.dumps({"image_id": meta["image_id"], "objects": objects}, indent=1) + "\n")
    return {"image_id": meta["image_id"], "annotated_image": annotated, "objects": objects, "camera_calibrated": bool(ctx.skills.cam.calibrated)}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(find_objects)
