"""Outline, number and locate the objects on the table, seen by a camera.

Which pixels are an object is decided by SAM 3 when its service is running (util/sam3.py: start
it with `uv run tools/util/sam3_server.py`), and `segmentation` in the result says "sam3". It
finds things by what they are called: by default every "object", so a black tape roll on the
dark mat, a ring and a cube standing on a box each get their own outline. `prompt` narrows it
to a noun phrase, or several separated by commas ("blue cube", "roll of tape, ring"): only
those come back, each with the phrase as its `label` and SAM 3's `score`. Without the service
(`segmentation: "colour"`) the old rule applies: the table is the dominant colour of the search
area and every connected region that differs from it is an object; that one misses black things
on the mat and merges things that touch. A colour word is attached either way.
The simulated arm always uses the colour rule.

With SAM 3, heights and stacks are measured from the side face this slanted camera sees below
each top face (split_top): `height_mm` is the object's own height, good to about 5 mm (a pixel
of side is 4 to 6 mm of height), `z_mm` how high its underside is above the table, and `on` the
id of the object it stands on (`z_mm` is null when that one's height could not be measured:
measure it yourself). A stack is only taken apart if the prompt names its parts
("blue cube, pink box"); to plain "object" it is one thing. To pick the upper one, give pick_at
`grasp_height` = (z_mm + height_mm / 2) / 1000. What stands underneath is partly hidden, so its
own numbers are rougher. Where no side shows (a flat or single-coloured thing), the rules below
apply, without their guess at a flat top, and `height_mm` is null.

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

`camera: "wrist"` does the same from wherever the gripper is, with the wrist camera's model:
useful up close; on the real arm about 1 cm off within 25 cm of the base, 2 cm or more beyond 30 cm.

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

from util import capture, sam3, vision
from util.arm import RobotError

# The part of the table that is searched from the top camera, metres in the base frame.
SEARCH_X = (-0.24, 0.24)
SEARCH_Y = (0.14, 0.40)
# How far from the table's colour a pixel must be to belong to an object (Lab units, 0..255).
COLOUR_STEP, LIGHTER_STEP, DARKER_STEP = 14.0, 30.0, 70.0
LOCAL_PX = 101  # window for the table's local brightness, pixels (about 15 cm): much wider than an object
LOCAL_SCALE, LOCAL_MIN = 8, 20  # it is computed on an image this many times smaller; fewer table pixels than this in a window and the overall median stands in


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
    clearly lighter than the table around it, or very much darker. Merely darker is a shadow:
    the arm throws one right where it works, and a shadow has the table's colour."""
    lab = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2LAB).astype(np.float32)
    table = np.median(lab[valid], axis=0)
    # Lighter than the table around it, not than the table as a whole: the lamp lights one part
    # of a pale table 25 to 30 units above its median, and that patch came and went as an object
    # ("white-1 is being moved") whenever the arm's shadow or the exposure shifted. The local
    # level is only ever raised above the median, so that lit table beside a shadow stays table.
    # The median is taken over table pixels only. Filling what the arm covers and what lies
    # outside the search area with the overall median made the local level fall back to it
    # wherever those fill more than half the window: next to the arm and along the edges,
    # which is where the lit patch then came back as "grey-N is being moved".
    small = cv2.resize(np.where(valid, lab[..., 0], np.nan), None, fx=1 / LOCAL_SCALE, fy=1 / LOCAL_SCALE, interpolation=cv2.INTER_NEAREST)
    half = LOCAL_PX // LOCAL_SCALE // 2
    windows = np.lib.stride_tricks.sliding_window_view(np.pad(small, half, constant_values=np.nan), (2 * half + 1, 2 * half + 1))
    windows = windows.reshape(*small.shape, -1)
    enough = np.isfinite(windows).sum(axis=-1) >= LOCAL_MIN
    local = np.full(small.shape, table[0], np.float32)
    local[enough] = np.nanmedian(windows[enough], axis=-1)
    local = cv2.resize(local, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_LINEAR)
    lighter = lab[..., 0] - np.maximum(local, table[0])
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


def footprint(cam: capture.TopCamera, blob: vision.Blob, img: np.ndarray, tops: bool = True) -> dict | None:
    """Where an object stands, from the edge where it meets the table on the camera's side.
    `tops: false` leaves out the brightness rule for a flat top (top_face): upright() has
    already looked for one with a stricter test and found none, and on a ring or a roll of tape
    the brightness rule takes the mat seen through the hole for a top and reports 10 cm of height."""
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
    top = top_face(img, mask) if tops else None
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


UP = 4  # split_top works on the outline at this many times the image's resolution
CLEAR_STEP = 20.0  # Lab units between a top face and the side below it, under which the split is not believed
SEE_THROUGH = 0.08  # share of an outline that has the colour of the mat around it, above which it is a ring and not a solid
AROUND = 0.4  # or this share of its whole rim
STANDS_ON = 0.5  # share of an outline's camera-side rim that must lie on another outline for it to stand on that one


def split_top(img: np.ndarray, mask: np.ndarray, foot: np.ndarray, max_px: float = 30.0, own: np.ndarray | None = None) -> tuple[np.ndarray, float, float, np.ndarray, np.ndarray] | None:
    """Split an upright object's outline into its top face and the side turned to the camera.

    Upright edges all point at `foot`, the pixel straight below the camera. So the outline of an
    upright object is its top face swept a little way towards `foot`, and taking that sweep back
    out leaves the top face: every pixel from which one can go `k` pixels towards `foot` and
    still be inside the outline. `k`, how long the upright edges look, is chosen where the two
    parts differ most in colour: the side is lit differently from the top. Unlike a brightness
    threshold this cannot cut the top face into odd shapes, and it works on an object standing
    on another, whose foot is not on the table.

    `own` is for an object with others standing on it: `mask` is then its outline together with
    theirs (they hide part of it), and `own` the part that shows the object itself, the only
    pixels whose colour is compared.

    Returns (top face mask, k in image pixels, colour difference in Lab units, the outline's
    centre pixel, the unit direction towards `foot`), or None for too small an outline. A small
    colour difference (below CLEAR_STEP) means no side was seen: a flat thing, one colour all
    over, or right under the camera."""
    x, y, w, h = cv2.boundingRect(mask)
    x0, y0, x1, y1 = max(0, x - 4), max(0, y - 4), min(mask.shape[1], x + w + 4), min(mask.shape[0], y + h + 4)
    big = cv2.resize(mask[y0:y1, x0:x1], None, fx=UP, fy=UP, interpolation=cv2.INTER_LINEAR) > 127
    big = cv2.erode(big.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0  # the outline's own edge pixels are mixed with the background
    lab = cv2.cvtColor(cv2.resize(img[y0:y1, x0:x1], None, fx=UP, fy=UP, interpolation=cv2.INTER_CUBIC), cv2.COLOR_BGR2LAB).astype(np.float32)
    mine = big if own is None else big & (cv2.resize(own[y0:y1, x0:x1], None, fx=UP, fy=UP, interpolation=cv2.INTER_LINEAR) > 127)
    ys, xs = np.nonzero(big)
    if len(xs) < 200 or mine.sum() < 100:
        return None
    centre = np.array([xs.mean() / UP + x0, ys.mean() / UP + y0])
    u = (np.asarray(foot) - centre) / np.linalg.norm(np.asarray(foot) - centre)
    top, best, total = big.copy(), None, len(xs)
    for step in range(1, int(max_px * UP) + 1):
        back = np.float32([[1, 0, -u[0] * step], [0, 1, -u[1] * step]])  # the outline moved back along the sweep
        top &= cv2.warpAffine(big.astype(np.uint8), back, (big.shape[1], big.shape[0]), flags=cv2.INTER_NEAREST) > 0
        n = int(top.sum())
        if n < 0.25 * total:
            break
        side, upper = mine & ~top, mine & top
        if side.sum() < 0.04 * total or upper.sum() < 0.04 * total:
            continue
        diff = lab[upper].mean(axis=0) - lab[side].mean(axis=0)
        score = (n / total) * (1 - n / total) * float(diff @ diff)  # as Otsu: how much of the colour variation the split explains
        if best is None or score > best[0]:
            best = (score, step / UP, top.copy(), float(np.linalg.norm(diff)))
    if best is None:
        return None
    _, k, top, diff = best
    out = np.zeros(mask.shape[:2], np.uint8)
    out[y0:y1, x0:x1] = cv2.resize(top.astype(np.uint8) * 255, (x1 - x0, y1 - y0), interpolation=cv2.INTER_AREA)
    return out, k, diff, centre, u


def own_height(cam: capture.TopCamera, centre: np.ndarray, u: np.ndarray, k: float, base_z: float) -> float:
    """How tall something is whose upright edge runs from pixel `centre` down to `centre + k u`,
    when it stands on a surface `base_z` metres above the table. One pixel of edge is 4 to 6 mm
    of height at this camera's distance and slant, so expect that much error."""
    under = cam.to_base(centre[0] + u[0] * k, centre[1] + u[1] * k, base_z)[:2]  # where the edge's lower end stands
    eye = cam.eye_base()
    flat = cam.to_base(centre[0], centre[1], 0.0)[:2]  # the upper end's ray, where it would meet the table
    up = eye[2] - cam.table_z
    d = flat - eye[:2]
    t = float((under - eye[:2]) @ d / (d @ d))  # along that ray, the point above `under`
    return up * (1 - t) - base_z


def see_through(img: np.ndarray, mask: np.ndarray) -> float:
    """The share of an outline's inside that has the colour of what lies just around it: the mat
    showing through the hole of a ring. A solid object has next to none."""
    solid = mask > 0
    rim = (cv2.dilate(mask, np.ones((15, 15), np.uint8)) > 0) & ~(cv2.dilate(mask, np.ones((5, 5), np.uint8)) > 0)
    inner = cv2.erode(mask, np.ones((5, 5), np.uint8)) > 0
    if rim.sum() < 20 or inner.sum() < 20:
        return 0.0
    lab = cv2.cvtColor(cv2.GaussianBlur(img, (3, 3), 0), cv2.COLOR_BGR2LAB).astype(np.float32)
    around = np.median(lab[rim], axis=0)
    d = np.sqrt(0.35 * (lab[..., 0] - around[0]) ** 2 + (lab[..., 1] - around[1]) ** 2 + (lab[..., 2] - around[2]) ** 2)
    return float((d[inner] < 10.0).mean())


def upright(cam: capture.TopCamera, mask: np.ndarray, img: np.ndarray, base_z: float, own: np.ndarray | None = None) -> dict | None:
    """Where an upright object stands and how tall it is, from its top face and the side below
    it (split_top), on a surface `base_z` above the table. `mask` and `own` as for split_top.
    None if no side can be told from the top: describe() then falls back to the edge where the
    object meets the table."""
    if see_through(img, mask if own is None else own) > SEE_THROUGH:
        return None  # a ring or a roll of tape: the mat in its hole would pass for a top face, a hand's width above the table
    found = split_top(img, mask, cam.P[:2, 2] / cam.P[2, 2], own=own)
    if found is None:
        return None
    top, k, diff, centre, u = found
    if diff < CLEAR_STEP or k < 1.5:
        return None
    height = own_height(cam, centre, u, k, base_z)
    contours, _ = cv2.findContours(top, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours or not 0.004 < height < 0.15:
        return None
    outline = np.array([cam.to_base(a, b, base_z + height)[:2] for a, b in max(contours, key=cv2.contourArea).reshape(-1, 2)], np.float32)
    (cx, cy), (a, b), turn = cv2.minAreaRect(outline)
    t = math.radians(turn)
    along_a, along_b = np.array([math.cos(t), math.sin(t)]), np.array([-math.sin(t), math.cos(t)])
    jaw, width = (along_a, a) if a <= b else (along_b, b)  # close across the narrower side
    return {"centre": np.array([cx, cy]), "jaw": jaw, "width": float(width), "sides": sorted([float(a), float(b)], reverse=True), "height": float(height)}


def supports(masks: list[np.ndarray], foot: np.ndarray) -> list[int | None]:
    """For each outline, the index of the outline it stands on, or None for the table. An object
    stands on another if the rim of its outline on the camera's side, where its foot is, lies
    on that other outline and not on the mat; or, when it hangs over the lower one's near edge,
    if that one shows around a good part of the rest of its rim (AROUND: more than a neighbour
    touching one side would); or if a smaller outline lies almost wholly against it, which is
    what the lower object looks like when little of it shows. That last one can be a small thing
    lying beside it instead: its height then usually cannot be measured and `z_mm` comes out
    null, which is the cue to look."""
    out: list[int | None] = []
    for i, m in enumerate(masks):
        ys, xs = np.nonzero(m)
        c = np.array([xs.mean(), ys.mean()])
        u = (foot - c) / np.linalg.norm(foot - c)
        rim = (cv2.dilate(m.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0) & ~m
        ry, rx = np.nonzero(rim)
        near = (rx - c[0]) * u[0] + (ry - c[1]) * u[1] > 0  # the half of the rim towards the camera
        best, share = None, STANDS_ON
        for j, other in enumerate(masks):
            if j != i and near.any():
                s_ = float(other[ry[near], rx[near]].mean())
                if s_ <= STANDS_ON and float(other[ry, rx].mean()) > AROUND:
                    s_ = STANDS_ON + float(other[ry, rx].mean())
                if s_ > share:
                    best, share = j, s_
        if best is None:
            # a smaller outline that hugs this one all along is most likely what it stands on, peeking out from under it
            close = cv2.dilate(m.astype(np.uint8), np.ones((19, 19), np.uint8)) > 0
            for j, other in enumerate(masks):
                if j != i and other.sum() < 0.8 * m.sum() and float(close[other].mean()) > 0.7:
                    best = j
        out.append(best)
    return out


def sam3_blobs(ctx, cam: capture.TopCamera, img: np.ndarray, meta: dict, prompts: list[str]) -> list[tuple[vision.Blob, str, float]]:
    """What SAM 3 finds for each prompt in the searched part of the top view, as (outline,
    prompt, score), best score first. Left out: anything whose middle lies outside the search
    area, the arm itself (more than half under the arm's mask), the mat and other things too big
    to be an object (over 12% of the search area), and a second outline of something already
    found (the same thing answering two prompts: the first prompt listed keeps it)."""
    z = meta["table_z"]
    area = np.zeros(img.shape[:2], np.uint8)
    corners = [cam.to_pixel([x, y, z]) for x, y in ((SEARCH_X[0], SEARCH_Y[0]), (SEARCH_X[1], SEARCH_Y[0]), (SEARCH_X[1], SEARCH_Y[1]), (SEARCH_X[0], SEARCH_Y[1]))]
    cv2.fillConvexPoly(area, np.round(corners).astype(np.int32), 255)
    covered = arm_mask(ctx, cam, img.shape, meta["joints_rad"])
    kept: list[tuple[np.ndarray, vision.Blob, str, float]] = []
    found = sam3.segment(img, prompts)
    for one in sorted(found, key=lambda o: (prompts.index(o["prompt"]), -o["score"])):
        mask = np.zeros(img.shape[:2], np.uint8)
        cv2.fillPoly(mask, [np.array(one["polygon"], np.int32)], 255)
        size = int((mask > 0).sum())
        if size < 120 or size > 0.12 * (area > 0).sum():
            continue
        blobs = vision.blobs_from_mask(mask, 120)
        if not blobs:
            continue
        cx, cy = blobs[0].centroid
        if not area[int(np.clip(cy, 0, area.shape[0] - 1)), int(np.clip(cx, 0, area.shape[1] - 1))] or (covered & (mask > 0)).sum() > 0.5 * size:
            continue
        if any((m & (mask > 0)).sum() > 0.6 * min(m.sum(), size) for m, *_ in kept):
            continue
        kept.append((mask > 0, blobs[0], one["prompt"], one["score"]))
    return [(b, p, s) for _, b, p, s in sorted(kept, key=lambda k: -k[3])]


def describe(cam: capture.TopCamera, blob: vision.Blob, img: np.ndarray, label, f: dict | None = None, base_z: float = 0.0, tops: bool = True) -> tuple[dict, dict] | None:
    """One outline as an object (where it stands, how to grasp it) and as a mark to draw. `f` is
    its footprint if already measured (upright), on a surface `base_z` above the table."""
    f = f or footprint(cam, blob, img, tops)
    if f is None:
        return None
    centre_px = cam.to_pixel([*f["centre"], cam.table_z + base_z])
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


def from_top(ctx, img: np.ndarray, meta: dict, max_objects: int, prompts: list[str] | None = None) -> tuple[list[dict], list[dict]]:
    cam = capture.TopCamera(ctx.root, meta["mode"], meta["table_z"])
    cam.to_pixel([0.0, 0.2, meta["table_z"]])  # fails with the reason if the calibration lacks the camera's position
    found = []
    if prompts is not None:
        blobs = sam3_blobs(ctx, cam, img, meta, prompts)[:max_objects]
        masks = []
        for blob, _, _ in blobs:
            m = np.zeros(img.shape[:2], np.uint8)
            cv2.fillPoly(m, [np.array(blob.polygon, np.int32)], 255)
            masks.append(m > 0)
        under = supports(masks, cam.P[:2, 2] / cam.P[2, 2])
        done: dict[int, tuple[dict, dict] | None] = {}

        def carried(i: int, seen: tuple = ()) -> list[int]:
            """Everything standing on object i, directly or on top of that."""
            return [j for j in range(len(blobs)) if under[j] == i and j not in seen for j in (j, *carried(j, (*seen, i, j)))]

        def measure(i: int, chain: tuple = ()) -> tuple[dict, dict] | None:
            """Object i, after whatever it stands on: its base is that one's top."""
            if i in done:
                return done[i]
            below = measure(under[i], (*chain, i)) if under[i] is not None and under[i] not in chain else None
            known = below is not None and below[0]["height_mm"] is not None and below[0]["z_mm"] is not None
            base_z = (below[0]["z_mm"] + below[0]["height_mm"]) / 1000 if known else 0.0
            blob, label, score = blobs[i]
            load = carried(i)
            whole = masks[i].copy()
            for j in load:
                whole |= masks[j]  # what stands on it hides part of it: its outline is the two together
            f = None
            if not load or masks[i].sum() >= 0.5 * whole.sum():  # with most of it hidden there is too little of its side to go by
                f = upright(cam, whole.astype(np.uint8) * 255, img, base_z, own=masks[i].astype(np.uint8) * 255 if load else None)
            one = describe(cam, blob, img, i + 1, f, base_z if f else 0.0, tops=False)
            if one:
                # z_mm None: it stands on something whose height could not be measured
                one[0].update(label=label, score=score, z_mm=(round(base_z * 1000) if known else None) if below else 0, on=below[0]["id"] if below else None)
            done[i] = one
            return one

        found = [one for one in (measure(i) for i in range(len(blobs))) if one]
    else:
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


def find_objects(ctx, camera: str = "top", max_objects: int = 8, prompt: str | None = None) -> dict:
    # the median of a few frames: outlines are measured to the pixel, and sensor noise moves them
    img = np.median([ctx.frame(camera)] + [ctx.frame(camera, settle=0.08) for _ in range(4)], axis=0).astype(np.uint8)
    meta = capture.freeze(ctx, camera, img, grid=False)
    use_sam3 = camera == "top" and meta["mode"] == "real" and sam3.available()  # the simulation's flat-coloured renders do not need it
    if prompt and not use_sam3:
        raise RobotError("`prompt` needs SAM 3 and the top camera: start it from the workspace directory with `uv run tools/util/sam3_server.py`")
    if use_sam3:
        objects, marks = from_top(ctx, img, meta, max_objects, [p.strip() for p in (prompt or "object").split(",") if p.strip()])
    else:
        objects, marks = (from_top if camera == "top" else from_wrist)(ctx, img, meta, max_objects)
    annotated = ctx.save_image("objects.jpg", vision.draw_marks(vision.draw_grid(img), marks), f"{camera} view: objects outlined and numbered, a cross where each stands")
    ctx.emit("detections", camera=camera, width=meta["width"], height=meta["height"],
             items=[{"label": f"#{o['id']}", "score": 1.0, "box": m["box"], "point": m["point"], "polygon": m["polygon"]} for o, m in zip(objects, marks)])  # fmt: skip
    ctx.emit("markers", markers=[{"id": f"obj{o['id']}", "label": f"#{o['id']}", "pos": [o["x"], o["y"], meta["table_z"] + 0.01], "color": "#ffd166", "radius": 0.012} for o in objects])
    listing = ctx.root / "perception" / "exteroception" / "objects.json"
    listing.parent.mkdir(parents=True, exist_ok=True)
    listing.write_text(json.dumps({"image_id": meta["image_id"], "camera": camera, "objects": objects}, indent=1) + "\n")
    return {"image_id": meta["image_id"], "camera": camera, "segmentation": "sam3" if use_sam3 else "colour", "annotated_image": annotated, "objects": objects}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(find_objects)
