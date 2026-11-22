"""Pen strokes: turning a picture into them, laying them out on a sheet, showing them.

A drawing is a list of strokes, each a polyline, in a frame as wide as the drawing: x runs 0..1
from left to right, y runs upward from 0, and `height` is how tall that makes it. Written as
JSON an agent can also compose a drawing itself:

    {"strokes": [[[x, y], [x, y], ...], ...]}        any units; it is rescaled to width 1

The current drawing, the sheet and the pen are kept in `state/drawing.json`.
"""

import json
from pathlib import Path

import cv2
import numpy as np

from util.arm import RobotError

ROOT = Path(__file__).resolve().parents[2]
STATE = ROOT / "state" / "drawing.json"


def load() -> dict:
    return json.loads(STATE.read_text()) if STATE.exists() else {}


def save(st: dict) -> None:
    STATE.parent.mkdir(exist_ok=True)
    tmp = STATE.with_suffix(".tmp")
    tmp.write_text(json.dumps(st, indent=1) + "\n")
    tmp.replace(STATE)


# --------------------------------------------------------------------------- picture -> strokes


def _ink(img: np.ndarray, invert: bool) -> np.ndarray:
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    k = 700 / max(gray.shape)
    if k < 1:
        gray = cv2.resize(gray, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    if invert:
        gray = 255 - gray
    block = max(15, (min(gray.shape) // 12) | 1)  # a local threshold copes with uneven light on a photographed sheet
    mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, block, 12)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    for i in range(1, n):  # specks
        if stats[i, cv2.CC_STAT_AREA] < 25:
            mask[labels == i] = 0
    return mask


def _thin(mask: np.ndarray) -> np.ndarray:
    """Zhang-Suen thinning: every line of the mask reduced to one pixel wide."""
    img = np.pad((mask > 0).astype(np.uint8), 1)
    while True:
        changed = False
        for step in (0, 1):
            p2, p3, p4, p5 = img[:-2, 1:-1], img[:-2, 2:], img[1:-1, 2:], img[2:, 2:]
            p6, p7, p8, p9 = img[2:, 1:-1], img[2:, :-2], img[1:-1, :-2], img[:-2, :-2]
            ring = [p2, p3, p4, p5, p6, p7, p8, p9]
            neighbours = sum(ring)
            transitions = sum((a == 0) & (b == 1) for a, b in zip(ring, ring[1:] + ring[:1]))
            gone = (img[1:-1, 1:-1] == 1) & (neighbours >= 2) & (neighbours <= 6) & (transitions == 1)
            gone &= (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0) if step == 0 else (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            if gone.any():
                img[1:-1, 1:-1][gone] = 0
                changed = True
        if not changed:
            return img[1:-1, 1:-1] * 255


def _trace(skeleton: np.ndarray) -> list[np.ndarray]:
    """Walk a one-pixel-wide skeleton into polylines."""
    pts = {(int(x), int(y)) for y, x in zip(*np.nonzero(skeleton))}
    around = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]

    def neighbours(p):
        return [(p[0] + dx, p[1] + dy) for dx, dy in around if (p[0] + dx, p[1] + dy) in pts]

    paths = []
    while pts:
        ends = [p for p in pts if len(neighbours(p)) == 1]
        cur = ends[0] if ends else next(iter(pts))
        path, prev = [cur], None
        pts.discard(cur)
        while True:
            options = neighbours(cur)
            if not options:
                break
            if prev is not None:  # keep going as straight as possible through a crossing
                d = (cur[0] - prev[0], cur[1] - prev[1])
                options.sort(key=lambda o: -((o[0] - cur[0]) * d[0] + (o[1] - cur[1]) * d[1]))
            prev, cur = cur, options[0]
            pts.discard(cur)
            path.append(cur)
        if len(path) >= 2:
            paths.append(np.array(path, np.float32))
    return paths


def _order(strokes: list[np.ndarray], join_gap: float) -> list[np.ndarray]:
    """Nearest stroke next, either way round; strokes whose ends nearly touch become one."""
    todo, out, pos = list(strokes), [], np.zeros(2)
    while todo:
        gap, i, end = min(((float(np.linalg.norm(s[e] - pos)), i, e) for i, s in enumerate(todo) for e in (0, -1)), key=lambda t: t[0])
        s = todo.pop(i)
        if end == -1:
            s = s[::-1]
        if out and gap < join_gap:
            out[-1] = np.vstack([out[-1], s])
        else:
            out.append(s)
        pos = s[-1]
    return out


def vectorize(img: np.ndarray, quad=None, invert: bool = False, detail: float = 0.004, min_length: float = 0.02) -> dict:
    """A line drawing as strokes. `quad` (four pixels: top-left, top-right, bottom-right,
    bottom-left of the sheet) first straightens a sheet photographed at an angle."""
    if quad is not None:
        q = np.asarray(quad, np.float32)
        w = max(np.linalg.norm(q[1] - q[0]), np.linalg.norm(q[2] - q[3]))
        h = max(np.linalg.norm(q[3] - q[0]), np.linalg.norm(q[2] - q[1]))
        k = 700 / max(w, h)
        w, h = int(w * k), int(h * k)
        M = cv2.getPerspectiveTransform(q, np.float32([[0, 0], [w, 0], [w, h], [0, h]]))
        img = cv2.warpPerspective(img, M, (w, h), flags=cv2.INTER_CUBIC, borderValue=(255, 255, 255))
        m = int(0.03 * max(w, h))  # the sheet's own edge is not part of the drawing
        img = img[m : h - m, m : w - m]
    mask = _ink(img, invert)
    if not mask.any():
        raise RobotError("no lines found in the picture; for light lines on a dark background pass invert: true")
    skeleton = _thin(mask)
    ys, xs = np.nonzero(skeleton)
    x0, y0 = xs.min(), ys.min()
    scale = 1.0 / max(xs.max() - x0, 1)
    height = (ys.max() - y0) * scale
    strokes = []
    for p in _trace(skeleton):
        p = cv2.approxPolyDP(p.reshape(-1, 1, 2), detail / scale, False).reshape(-1, 2)
        s = np.column_stack([(p[:, 0] - x0) * scale, height - (p[:, 1] - y0) * scale])
        if np.linalg.norm(np.diff(s, axis=0), axis=1).sum() >= min_length:
            strokes.append(s)
    if not strokes:
        raise RobotError("the picture has no lines long enough to draw")
    return {"height": round(float(height), 4), "strokes": [np.round(s, 4).tolist() for s in _order(strokes, join_gap=0.012)]}


def normalise(strokes) -> dict:
    """Strokes in any units, rescaled to width 1 with y upward from 0."""
    try:
        polylines = [np.asarray(s, dtype=float) for s in strokes]
    except (TypeError, ValueError):
        polylines = []
    if not polylines or any(s.ndim != 2 or s.shape[1] != 2 or len(s) < 2 for s in polylines):
        raise RobotError("strokes is a list of polylines, each a list of at least two [x, y] points")
    points = np.vstack(polylines)
    lo, span = points.min(axis=0), max(float(np.ptp(points[:, 0])), 1e-9)
    return {"height": round(float(np.ptp(points[:, 1]) / span), 4), "strokes": [np.round((s - lo) / span, 4).tolist() for s in polylines]}


# --------------------------------------------------------------------------- showing and placing


def length(drawing: dict) -> float:
    return float(sum(np.linalg.norm(np.diff(np.asarray(s), axis=0), axis=1).sum() for s in drawing["strokes"]))


def preview(drawing: dict, size: int = 640) -> np.ndarray:
    """What the pen will draw: strokes in black, numbered where they start, pen-up travel in pale red."""
    pad = 30
    h = int(size * max(drawing["height"], 0.05))
    img = np.full((h + 2 * pad, size + 2 * pad, 3), 255, np.uint8)

    def px(p):
        return int(pad + p[0] * size), int(pad + h - p[1] * size)

    last = None
    for i, s in enumerate(drawing["strokes"]):
        if last is not None:
            cv2.line(img, px(last), px(s[0]), (200, 200, 255), 1, cv2.LINE_AA)
        cv2.polylines(img, [np.int32([px(p) for p in s])], False, (0, 0, 0), 2, cv2.LINE_AA)
        if len(drawing["strokes"]) <= 60:
            cv2.putText(img, str(i), px(s[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 0), 1, cv2.LINE_AA)
        last = s[-1]
    return img


def on_sheet(drawing: dict, corners, margin: float) -> tuple[list[np.ndarray], float]:
    """The strokes in the base frame, scaled to fit the sheet inside `margin` and centred.
    `corners` are the sheet's corners (x, y): bottom-left, bottom-right, top-right, top-left of
    the drawing as it should read. Returns (strokes as Nx2 arrays, the drawing's width in metres)."""
    bl, br, tr, tl = (np.asarray(c, dtype=float) for c in corners)
    w, h = float(np.linalg.norm(br - bl)), float(np.linalg.norm(tl - bl))
    ex, ey = (br - bl) / w, (tl - bl) / h
    fit_w, fit_h = w - 2 * margin, h - 2 * margin
    if fit_w < 0.02 or fit_h < 0.02:
        raise RobotError("the sheet is too small for that margin")
    scale = min(fit_w, fit_h / max(drawing["height"], 1e-6))
    ox, oy = margin + (fit_w - scale) / 2, margin + (fit_h - scale * drawing["height"]) / 2
    out = []
    for s in drawing["strokes"]:
        s = np.asarray(s, dtype=float) * scale
        out.append(bl + np.outer(ox + s[:, 0], ex) + np.outer(oy + s[:, 1], ey))
    return out, scale


def dense(points: np.ndarray, step: float) -> np.ndarray:
    """The polyline with a point at least every `step` metres."""
    out = [points[0]]
    for a, b in zip(points[:-1], points[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
    return np.array(out)
