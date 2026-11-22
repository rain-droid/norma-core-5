"""Pushing a flat T across the table: its shape, where it is, how well it sits on a target, and
which push to make next. Geometry only: nothing here touches the robot or a camera.

A T's pose is (x, y, yaw): the area centroid in the base frame, metres, and its heading in
radians. At yaw 0 the bar is on the +y side (away from the base) and the stem points back at
the base; positive yaw turns it counter-clockwise seen from above.

The push planner is the one from StationHarness (sim/pusht.py, MIT licence), which solved its
Push-T in simulation: every edge of the T is a place to push from, straight inwards. A push at a
point off the line through the centroid also turns the T, and the planner picks the contact
whose mix of sliding and turning removes most of what is left of the position and heading
error. One short push, then look again: the model is rough, and the looking makes up for it.
"""

import math

import cv2
import numpy as np

RASTER = 0.001  # metres per cell wherever a footprint is drawn to be compared with another


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def rotation(a: float) -> np.ndarray:
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def t_polygon(bar_w: float = 0.050, bar_h: float = 0.014, stem_w: float = 0.016, stem_h: float = 0.046) -> np.ndarray:
    """The T's outline, counter-clockwise, with its area centroid at the origin. The bar is
    `bar_w` wide and `bar_h` deep; the stem hangs `stem_h` below it and is `stem_w` wide."""
    area = bar_w * bar_h + stem_w * stem_h
    top = stem_h + bar_h
    com = (bar_w * bar_h * (stem_h + bar_h / 2) + stem_w * stem_h * stem_h / 2) / area  # above the stem's foot
    b, s = bar_w / 2, stem_w / 2
    return np.array([[-b, top], [-b, stem_h], [-s, stem_h], [-s, 0], [s, 0], [s, stem_h], [b, stem_h], [b, top]]) - [0, com]


def placed(polygon: np.ndarray, pose) -> np.ndarray:
    """The outline at a pose, in the base frame."""
    return polygon @ rotation(pose[2]).T + np.asarray(pose[:2])


def gyration2(polygon: np.ndarray) -> float:
    """Mean squared distance of the T's area from its centroid, m^2: how hard it is to turn
    compared with how hard it is to slide."""
    lo, hi = polygon.min(axis=0), polygon.max(axis=0)
    size = np.ceil((hi - lo) / RASTER).astype(int) + 1
    img = np.zeros((size[1], size[0]), np.uint8)
    cv2.fillPoly(img, [np.round((polygon - lo) / RASTER).astype(np.int32)], 1)
    ys, xs = np.nonzero(img)
    return float(np.mean((xs * RASTER + lo[0]) ** 2 + (ys * RASTER + lo[1]) ** 2))


def _raster(outline: np.ndarray, origin: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    img = np.zeros(shape, np.uint8)
    cv2.fillPoly(img, [np.round((outline - origin) / RASTER).astype(np.int32)], 1)
    return img > 0


def overlap(polygon: np.ndarray, pose, target) -> float:
    """How much of the T at `pose` and the T at `target` is common to both: intersection over
    union of the two footprints, 1 for a perfect fit. Position alone can be right with the T
    turned wrong; this cannot."""
    a, b = placed(polygon, pose), placed(polygon, target)
    lo = np.minimum(a.min(axis=0), b.min(axis=0)) - RASTER
    shape = tuple((np.ceil((np.maximum(a.max(axis=0), b.max(axis=0)) - lo) / RASTER).astype(int) + 2)[::-1])
    ra, rb = _raster(a, lo, shape), _raster(b, lo, shape)
    return float((ra & rb).sum() / max(1, (ra | rb).sum()))


def fit_pose(seen: np.ndarray, origin: np.ndarray, polygon: np.ndarray) -> tuple[np.ndarray, float]:
    """The pose at which the T's outline best covers what was seen.

    `seen` is a top-down picture of the T's top face, True where it is, one cell per RASTER
    metres, whose cell (0, 0) lies at `origin` in the base frame. Every heading is tried in
    5 degree steps from the seen centroid, and the best is then tightened in position and
    heading. Returns the pose and the overlap reached (intersection over union); a low overlap
    means that what was seen is not this T.
    """
    ys, xs = np.nonzero(seen)
    if len(xs) < 30:
        return np.zeros(3), 0.0
    centre = np.array([xs.mean(), ys.mean()]) * RASTER + origin

    def score(pose) -> float:
        drawn = _raster(placed(polygon, pose), origin, seen.shape)
        return float((drawn & seen).sum() / max(1, (drawn | seen).sum()))

    best = max(((score([*centre, a]), a) for a in np.radians(np.arange(0, 360, 5))), key=lambda s: s[0])
    pose, value = np.array([*centre, best[1]]), best[0]
    step = np.array([0.004, 0.004, math.radians(4)])
    while step[0] > 0.0002:
        moved = False
        for i in range(3):
            for sign in (1, -1):
                trial = pose.copy()
                trial[i] += sign * step[i]
                s = score(trial)
                if s > value:
                    pose, value, moved = trial, s, True
        if not moved:
            step = step / 2
    pose[2] = wrap(pose[2])
    return pose, value


def errors(pose, target) -> tuple[float, float]:
    """How far the T is from the target: metres between the centroids, radians of heading."""
    return float(np.linalg.norm(np.asarray(pose[:2]) - np.asarray(target[:2]))), abs(wrap(target[2] - pose[2]))


def _clear_run(polygon: np.ndarray, contact: np.ndarray, outward: np.ndarray, standoff: float, gap: float) -> bool:
    """Whether a pusher of radius `standoff` can come down `gap` away from the face and run up to
    it touching nothing else of the T. In the T's own frame."""
    outline = polygon.astype(np.float32)
    for k in np.linspace(0.0, 1.0, 6):
        p = contact + outward * (standoff + k * gap)
        free = -cv2.pointPolygonTest(outline, (float(p[0]), float(p[1])), True)  # distance to the T, negative inside it
        if free < standoff + k * gap * 0.5 - 0.0005:
            return False
    return True


def push_candidates(polygon: np.ndarray, pose, target, max_push: float = 0.012, standoff: float = 0.011, gap: float = 0.010,
                    turn: bool = True) -> list[dict]:  # fmt: skip
    """Pushes worth making from here, best first.

    Each is a straight push by the closed fingertips against one edge of the T, along that edge's
    inward normal. `standoff` is how far the pusher's centre stays from the face it presses on
    (half the closed fingers' depth) and `gap` the free run before it touches. A push of `d` at
    a contact with lever arm `l` about the centroid is taken to slide the T by d / (1 + l^2/c^2)
    along the push and turn it by l/c^2 times that, with c^2 the T's `gyration2`: a push through
    the centroid only slides, one far from it mostly turns. The error it should remove is the
    `predicted_gain`. `turn: false` ignores heading and keeps only pushes through the centroid.
    Contacts in the T's inner corners, where the pusher would not fit beside the stem, are left out.

    Fields: `contact` (x, y where the pusher meets the T), `direction` (unit, the way it
    pushes), `start`, `end` (x, y of the pusher's centre), `push_m` (how far the T is pressed),
    `lever_m`, `edge`.
    """
    pose, R = np.asarray(pose, dtype=float), rotation(pose[2])
    c2 = gyration2(polygon)
    weight = math.sqrt(c2)  # metres per radian: makes a heading error comparable with a position error
    error = np.array([*(np.asarray(target[:2]) - pose[:2]), wrap(target[2] - pose[2]) * weight])
    if not turn:
        error[2] = 0.0
    out = []
    for edge, (a, b) in enumerate(zip(polygon, np.roll(polygon, -1, axis=0))):
        v = b - a
        length = float(np.linalg.norm(v))
        if length < 0.012:
            continue  # too short a face to press on reliably
        outward = np.array([v[1], -v[0]]) / length  # the outline runs counter-clockwise
        u_local = -outward
        fractions = [0.25, 0.5, 0.75]
        denominator = v[0] * u_local[1] - v[1] * u_local[0]
        through = -(a[0] * u_local[1] - a[1] * u_local[0]) / denominator  # where this edge's normal passes through the centroid
        if 0.05 < through < 0.95:
            fractions.append(float(through))
        for fraction in fractions:
            r, u = R @ (a + fraction * v), R @ u_local
            lever = float(r[0] * u[1] - r[1] * u[0])
            if not turn and abs(lever) > 1e-6:
                continue
            response = np.array([*u, lever / c2 * weight]) / (1 + lever * lever / c2)
            distance = float(np.clip(error @ response / (response @ response), 0.0, max_push))
            if distance < 0.001:
                continue
            if not _clear_run(polygon, a + fraction * v, outward, standoff, gap):
                continue  # the fingertips would come down on, or scrape, another part of the T: an inner corner
            left = error - response * distance
            contact = pose[:2] + r
            start = contact - u * (standoff + gap)
            out.append({
                "contact": contact, "direction": u, "start": start, "end": start + u * (gap + distance), "push_m": distance,
                "predicted_gain": float(error @ error - left @ left), "lever_m": lever, "edge": edge,
            })  # fmt: skip
    return sorted(out, key=lambda c: -c["predicted_gain"])
