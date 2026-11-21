"""Outline, number and locate whatever stands out on the table, seen by a camera.

No object model and no labels: the table is taken to be the dominant colour of the search
area, and every connected region that differs from it is an object. A colour word is attached
so objects can be told apart.

`camera: "top"` (default) is the one to trust: it sees the whole mat and is calibrated by the
corner tags. Only the part of the table the arm can work on is searched (SEARCH_X, SEARCH_Y
below), and the arm's own body is masked out using its joint angles. Where each object stands
is worked out from what the camera's known position allows:

* an object with a flat top lit from above (a box, a cube, a can): the top is seen whole, its
  nearest corner stands on the base's nearest corner, which gives `height_mm`, and at that
  height the top's outline is the footprint: `x, y` is its centre, `footprint_mm` its sides;
* otherwise only the edge where the object meets the table on the camera's side is used. If
  that edge bends (a corner or a curve faces the camera) the footprint is completed from its
  two sides; if it is straight, the hidden depth is taken to equal the visible width, and
  `footprint_mm` has one number.

Expect about a centimetre: at this distance a pixel is 1.3 mm across the view and 2.6 mm along
it, and the outline is good to a pixel or two. Read several times, the numbers wander by that.

`yaw_deg` is the jaw rotation for pick_at that closes the jaws across flat sides, and
`width_mm` what they would close on (they open 53 mm). Heights are not measured.

`camera: "wrist"` does the same from wherever the gripper is, with the wrist camera's rough
model: useful up close, centimetres off on the real arm.

Does not move the arm. Returns a new `image_id` and `annotated_image`: open it, and check that
each outline is a real object and each cross sits in the middle of where the object stands.
From the top camera a region that is only darker than the table is taken for a shadow, so a
black object on a dark table is missed, as is anything the table's own colour; if it misses
something, read the pixel yourself and use pixel_to_table. Also writes perception/exteroception/objects.json.
"""

import json
import math

import cv2
import numpy as np

from util import capture, vision

# The part of the table that is searched from the top camera, metres in the base frame.
SEARCH_X = (-0.24, 0.24)
SEARCH_Y = (0.14, 0.40)
# How far from the table's colour a pixel must be to belong to an object (Lab units, 0..255).
COLOUR_STEP, LIGHTER_STEP, DARKER_STEP = 14.0, 30.0, 70.0


def color_name(img: np.ndarray, blob: vision.Blob) -> str:
    mask = np.zeros(img.shape[:2], np.uint8)
    cv2.fillPoly(mask, [np.array(blob.polygon, np.int32)], 255)
    h, s, v = [c[mask > 0].astype(float) for c in cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))]
    if np.median(s) < 45:
        return "white" if np.median(v) > 170 else ("black" if np.median(v) < 70 else "grey")
    # The hue of the more strongly coloured half, which is the object and not table seen through
    # a hole in it, averaged as a direction: hue is a circle with red at both ends of the scale,
    # and a plain median of a red thing lands in the middle, on green or blue.
    turn = h[s >= np.median(s)] * (math.pi / 90)
    h = math.degrees(math.atan2(np.sin(turn).mean(), np.cos(turn).mean())) / 2 % 180
    for name, lo, hi in (("red", 0, 8), ("orange", 8, 20), ("yellow", 20, 33), ("green", 33, 85), ("blue", 85, 130),
                         ("purple", 130, 160), ("pink", 160, 172), ("red", 172, 181)):  # fmt: skip
        if lo <= h < hi:
            return name
    return ""


def stands_out(img: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Pixels that are not table, by the top camera. The table is the median colour of the
    search area. A pixel counts if its colour differs (not just its brightness), or if it is
    clearly lighter, or very much darker. Merely darker is a shadow: the arm throws one right
    where it works, and a shadow has the table's colour."""
    lab = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2LAB).astype(np.float32)
    table = np.median(lab[valid], axis=0)
    lighter = lab[..., 0] - table[0]
    colour = np.hypot(lab[..., 1] - table[1], lab[..., 2] - table[2])
    # dark only counts in the bulk: the black line where an object meets the table is shadow too,
    # and would move its base towards the camera
    dark = cv2.morphologyEx((valid & (lighter < -DARKER_STEP)).astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)) > 0
    mask = ((valid & ((colour > COLOUR_STEP) | (lighter > LIGHTER_STEP))) | dark).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))


def jaw_yaw(direction) -> int:
    """Jaw rotation (degrees, as pick_at takes it) that closes the jaws along a table direction."""
    return round((-math.degrees(math.atan2(direction[1], direction[0])) + 90) % 180 - 90)


def arm_mask(ctx, cam: capture.TopCamera, shape, q) -> np.ndarray:
    """Pixels of the top view covered by the arm itself, from its joint angles: each body
    segment is a capsule, drawn a little fat. Not fatter: an object standing right in front of
    the arm must survive. Cables and the wrist camera stick out of it and can show up as objects."""
    mask = np.zeros(shape[:2], np.uint8)
    for start, end, radius in ctx.plan.kin.body_segments(q):
        for t in np.linspace(0.0, 1.0, 8):
            c = start + (end - start) * t
            uv = cam.to_pixel(c)
            r = int(np.linalg.norm(cam.to_pixel(c + [radius, 0.0, 0.0]) - uv) * 1.25) + 4
            cv2.circle(mask, (int(round(uv[0])), int(round(uv[1]))), r, 255, -1)
    return mask > 0


def _side(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The two ends of the straight line through some edge points (least squares)."""
    mean = points.mean(axis=0)
    direction = np.linalg.svd(points - mean)[2][0]
    along = (points - mean) @ direction
    return mean + direction * along.min(), mean + direction * along.max()


def top_face(img: np.ndarray, mask: np.ndarray) -> np.ndarray | None:
    """The outline of an object's flat top, if it shows one: under light from above the top is
    the brightest part of the object, set off from its sides."""
    value = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[..., 2]
    inside = value[mask > 0]
    cut, _ = cv2.threshold(inside.reshape(-1, 1), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    bright = cv2.morphologyEx(((mask > 0) & (value > cut)).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(bright, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours or not (inside <= cut).any():
        return None
    top = max(contours, key=cv2.contourArea)
    if cv2.contourArea(top) < 0.3 * inside.size or inside[inside > cut].mean() - inside[inside <= cut].mean() < 25:
        return None  # no clear two-tone split: not a lit top over shaded sides
    return top.reshape(-1, 2).astype(float)


def footprint(cam: capture.TopCamera, blob: vision.Blob, img: np.ndarray) -> dict | None:
    """Where an object stands, from the edge where it meets the table on the camera's side."""
    mask = np.zeros(img.shape[:2], np.uint8)
    cv2.fillPoly(mask, [np.array(blob.polygon, np.int32)], 255)
    ys, xs = np.nonzero(mask)
    # Upright edges do not look upright from this camera: they all point at one spot below the
    # image, where vertical lines meet. Sweeping a ray from that spot across the object, the
    # first pixel each ray meets is where the object stands on the table, and the outermost
    # rays run up its upright edges, so they meet it exactly at its base corners.
    foot = cam.P[:2, 2] / cam.P[2, 2]
    angle, reach = np.arctan2(xs - foot[0], foot[1] - ys), np.hypot(xs - foot[0], foot[1] - ys)
    ray = np.round((angle - angle.min()) * reach.mean()).astype(int)  # one pixel of arc per ray
    first = [np.flatnonzero(ray == k) for k in range(1, ray.max())]  # the two end rays graze it: too few pixels to trust
    if len(first) < 6:
        return None
    px = np.array([[xs[i[np.argmin(reach[i])]], ys[i[np.argmin(reach[i])]]] for i in first if len(i)], dtype=float)
    base = np.array([cam.to_base(u, v)[:2] for u, v in px])
    eye = cam.eye_base()
    top = top_face(img, mask)
    if top is not None:
        # A flat top says more than the base does: it is seen whole. Its nearest corner stands
        # on the base's nearest corner, which gives the height; at that height its outline is
        # the footprint.
        flat = np.array([cam.to_base(u, v)[:2] for u, v in top])  # the top's outline as if it lay on the table
        near_top = flat[np.argmin(np.linalg.norm(flat - eye[:2], axis=1))]
        near_base = base[np.argmin(np.linalg.norm(base - eye[:2], axis=1))]
        up = eye[2] - cam.table_z
        height = up * (1 - np.linalg.norm(near_base - eye[:2]) / np.linalg.norm(near_top - eye[:2]))
        if 0.005 < height < 0.15:
            outline = eye[:2] + (flat - eye[:2]) * (up - height) / up
            (cx, cy), (a, b), turn = cv2.minAreaRect(outline.astype(np.float32))
            t = math.radians(turn)
            along_a, along_b = np.array([math.cos(t), math.sin(t)]), np.array([-math.sin(t), math.cos(t)])
            jaw, width = (along_a, a) if a <= b else (along_b, b)  # close across the narrower side
            return {"centre": np.array([cx, cy]), "jaw": jaw, "width": float(width), "sides": sorted([float(a), float(b)], reverse=True), "height": float(height)}
    # Otherwise the base edge is all there is. Does it bend? Judged in the image, where the outline is good to a pixel or two; on
    # the table one pixel of height is several millimetres of depth, and every wobble a corner.
    chord = px[-1] - px[0]
    off = np.abs(chord[0] * (px[:, 1] - px[0, 1]) - chord[1] * (px[:, 0] - px[0, 0])) / (np.linalg.norm(chord) + 1e-9)
    k = int(np.argmax(off))
    sides = [base]
    if off[k] > max(3.0, 0.12 * float(np.linalg.norm(chord))):  # it does: a corner (or a curve) faces the camera
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
    if normal @ (mid - eye[:2]) < 0:
        normal = -normal  # away from the camera, into the object
    return {"centre": mid + normal * width / 2, "jaw": face, "width": width, "sides": [width]}


def table_blobs(ctx, cam: capture.TopCamera, img: np.ndarray, meta: dict, hide: np.ndarray | None = None) -> tuple[list[vision.Blob], np.ndarray, np.ndarray]:
    """Whatever stands out in the searched part of the top view, largest first. Also returns
    the mask of that part, and the mask of what hides the table: the arm, plus `hide`."""
    z = meta["table_z"]
    area = np.zeros(img.shape[:2], np.uint8)
    corners = [cam.to_pixel([x, y, z]) for x, y in ((SEARCH_X[0], SEARCH_Y[0]), (SEARCH_X[1], SEARCH_Y[0]), (SEARCH_X[1], SEARCH_Y[1]), (SEARCH_X[0], SEARCH_Y[1]))]
    cv2.fillConvexPoly(area, np.round(corners).astype(np.int32), 255)
    covered = arm_mask(ctx, cam, img.shape, meta["joints_rad"])
    if hide is not None:
        covered |= hide
    return vision.blobs_from_mask(stands_out(img, (area > 0) & ~covered), 120), area > 0, covered


def describe(cam: capture.TopCamera, blob: vision.Blob, img: np.ndarray, label) -> tuple[dict, dict] | None:
    """One outline as an object (where it stands, how to grasp it) and as a mark to draw."""
    f = footprint(cam, blob, img)
    if f is None:
        return None
    centre_px = cam.to_pixel([*f["centre"], cam.table_z])
    thing = {
        "id": label,
        "color": color_name(img, blob),
        "x": round(float(f["centre"][0]), 3),
        "y": round(float(f["centre"][1]), 3),
        "yaw_deg": jaw_yaw(f["jaw"]),
        "width_mm": round(f["width"] * 1000),
        "footprint_mm": [round(s * 1000) for s in f["sides"]],
        "height_mm": round(f["height"] * 1000) if "height" in f else None,
        "pixel": [round(float(centre_px[0])), round(float(centre_px[1]))],
    }  # fmt: skip
    return thing, {"label": f"{label}", "polygon": blob.polygon, "point": [float(centre_px[0]), float(centre_px[1])], "box": list(blob.box)}


def from_top(ctx, img: np.ndarray, meta: dict, max_objects: int) -> tuple[list[dict], list[dict]]:
    cam = capture.TopCamera(ctx.root, meta["mode"], meta["table_z"])
    cam.to_pixel([0.0, 0.2, meta["table_z"]])  # fails with the reason if the calibration lacks the camera's position
    found = []
    for blob in table_blobs(ctx, cam, img, meta)[0]:
        one = describe(cam, blob, img, len(found) + 1) if len(found) < max_objects else None
        if one:
            found.append(one)
    return [o for o, _ in found], [m for _, m in found]


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
    # the median of a few frames: outlines are measured to the pixel, and sensor noise moves them
    img = np.median([ctx.frame(camera)] + [ctx.frame(camera, settle=0.08) for _ in range(4)], axis=0).astype(np.uint8)
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
