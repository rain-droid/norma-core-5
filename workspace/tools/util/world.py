"""The world model: what is on the table, by name, and what has been done with it.

`state/world.json` holds one record per object and outlives any single look:

    {"holding": "blue-1" | null,
     "objects": {"blue-1": {"name": "blue cube", "kind": "cube", "colour": "blue",
                            "x": 0.05, "y": 0.24, "yaw_deg": 0, "height_mm": 28, "width_mm": 28, "footprint_mm": [29, 28],
                            "shape": "solid" | "ring", "ring": {"r_mid_mm": 22, "band_mm": 14} | null,
                            "state": "on_table" | "held" | "unseen", "on": "pink-1" | null, "seen_t": 1795300000.0,
                            "history": [{"t": ..., "action": "grasp", "outcome": "holding, 27 mm"}]}}}

Geometry comes from the top camera (tools/find_objects.py) each time someone looks; `sync` matches
what is seen now to the records by place and colour, so an object keeps its name when it has
been moved. Names and kinds come from whoever looked at the pictures (tools/name_objects.py).
What a tool did to an object and how it went is appended to its `history`.

The key (`blue-1`) never changes; the name is free text and can. `resolve` takes either.
"""

import json
import time
from pathlib import Path

import cv2
import numpy as np

from util.arm import RobotError

ROOT = Path(__file__).resolve().parents[2]
FILE = ROOT / "state" / "world.json"
SAME_PLACE = 0.025  # metres: closer than this to where it was, it is the same object whatever its colour reads as
SAME_OBJECT = 0.30  # an object of the same colour this near is taken to be the one that moved
HISTORY = 12  # entries kept per object


def load() -> dict:
    if FILE.exists():
        return json.loads(FILE.read_text())
    return {"holding": None, "objects": {}}


def save(world: dict) -> None:
    FILE.parent.mkdir(exist_ok=True)
    tmp = FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps(world, indent=1) + "\n")
    tmp.replace(FILE)


def note(world: dict, key: str, action: str, outcome: str) -> None:
    """Add to an object's history what was done to it and how it went."""
    h = world["objects"][key].setdefault("history", [])
    h.append({"t": round(time.time(), 1), "action": action, "outcome": outcome})
    del h[:-HISTORY]


def resolve(world: dict, name: str) -> str:
    """The key of the object meant by `name`: a key, a name, or words of a name ("the ring")."""
    objects = world["objects"]
    if name in objects:
        return name
    want = name.lower().replace("the ", "").strip()
    exact = [k for k, o in objects.items() if (o.get("name") or "").lower() == want]
    if len(exact) == 1:
        return exact[0]
    words = set(want.split())
    scored = []
    for k, o in objects.items():
        have = set(f"{o.get('name') or ''} {o.get('kind') or ''} {o.get('colour') or ''}".lower().split())
        if words and words <= have:
            scored.append(k)
    if len(scored) == 1:
        return scored[0]
    listing = ", ".join(f"{k} ({o.get('name') or 'unnamed'})" for k, o in objects.items()) or "nothing"
    if scored or exact:
        raise RobotError(f"'{name}' fits more than one object ({', '.join(scored or exact)}): use its key. Known: {listing}")
    raise RobotError(f"no object called '{name}'. Known: {listing}. Run tools/describe_objects.py to look again.")


def sync(world: dict, seen: list[dict]) -> list[str]:
    """Bring the records up to date with what the top camera sees now (find_objects' list, each
    entry optionally carrying `shape` and `ring`). Returns the keys in the order of `seen`."""
    objects, now = world["objects"], time.time()
    free = {k for k, o in objects.items() if o["state"] != "held"}
    keys: list[str | None] = [None] * len(seen)

    def dist(o, d):
        return float(np.hypot(o["x"] - d["x"], o["y"] - d["y"]))

    # first what has not moved, then what moved but kept its colour: nearest pairs first
    for limit, same_colour in ((SAME_PLACE, False), (SAME_OBJECT, True)):
        pairs = sorted((dist(objects[k], d), k, i) for k in free for i, d in enumerate(seen)
                       if keys[i] is None and (not same_colour or objects[k]["colour"] == d["color"]))  # fmt: skip
        for gap, k, i in pairs:
            if gap <= limit and k in free and keys[i] is None:
                keys[i] = k
                free.discard(k)
    for i, d in enumerate(seen):
        k = keys[i]
        if k is None:
            n = 1 + max([int(x.rsplit("-", 1)[1]) for x in objects if x.rsplit("-", 1)[0] == d["color"]], default=0)
            k = keys[i] = f"{d['color']}-{n}"
            objects[k] = {"name": None, "kind": None, "colour": d["color"], "history": [], "on": None}
        o = objects[k]
        moved = "x" in o and dist(o, d) > SAME_PLACE
        o.update({f: d.get(f) for f in ("x", "y", "yaw_deg", "width_mm", "footprint_mm")})
        if d.get("height_mm"):
            o["height_mm"] = d["height_mm"]
        o.setdefault("height_mm", None)
        o.update(shape=d.get("shape", o.get("shape", "solid")), ring=d.get("ring", o.get("ring")), state="on_table", seen_t=round(now, 1))
        if moved and o.get("on"):
            o["on"] = None  # it is somewhere else now: whatever it stood on, it no longer does
    for k in free:  # known, not held, and not in this look
        objects[k]["state"] = "unseen"
    return keys


def ring_of(img: np.ndarray, polygon, cam, height: float) -> dict | None:
    """If the outline has a hole showing the table through it (a ring, a roll of tape), the hole's
    centre and the ring's size on the table. `cam` is the top camera; `height` the object's, metres."""
    h, w = img.shape[:2]
    outer = np.zeros((h, w), np.uint8)
    cv2.fillPoly(outer, [np.array(polygon, np.int32)], 255)
    area = int(np.count_nonzero(outer))
    if area < 200:
        return None
    lab = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2LAB).astype(np.float32)
    ring = cv2.dilate(outer, np.ones((15, 15), np.uint8)) & ~cv2.dilate(outer, np.ones((5, 5), np.uint8))
    table = np.median(lab[ring > 0], axis=0)  # the table just outside the object
    body = np.median(lab[cv2.erode(outer, np.ones((3, 3), np.uint8)) > 0], axis=0)
    d_table = np.linalg.norm((lab - table) * [0.5, 1, 1], axis=2)
    d_body = np.linalg.norm((lab - body) * [0.5, 1, 1], axis=2)
    like_table = ((d_table < d_body) & (outer > 0)).astype(np.uint8)
    like_table = cv2.morphologyEx(like_table, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, centres = cv2.connectedComponentsWithStats(like_table)
    edge = cv2.dilate(outer, np.ones((3, 3), np.uint8)) & ~cv2.erode(outer, np.ones((7, 7), np.uint8))
    best = None
    for i in range(1, n):
        a = stats[i, cv2.CC_STAT_AREA]
        if a < 0.06 * area or np.any((labels == i) & (edge > 0)):
            continue  # too small to be a hole, or open to the outside: a notch, not a hole
        if best is None or a > stats[best, cv2.CC_STAT_AREA]:
            best = i
    if best is None:
        return None
    hole = (labels == best).astype(np.uint8)
    contour = max(cv2.findContours(hole, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0], key=len).reshape(-1, 2)
    centre = cam.to_base(*centres[best], height)[:2]
    rim = np.array([cam.to_base(u, v, height)[:2] for u, v in contour[:: max(1, len(contour) // 40)]])
    r_inner = float(np.median(np.linalg.norm(rim - centre, axis=1)))
    outline = np.array([cam.to_base(u, v, height)[:2] for u, v in np.array(polygon)[:: max(1, len(polygon) // 40)]])
    r_outer = float(np.percentile(np.linalg.norm(outline - centre, axis=1), 25))  # the low side: the far wall shows in the outline
    band = r_outer - r_inner
    if not 0.006 <= band <= 0.045:
        band = 0.015
    return {"x": round(float(centre[0]), 3), "y": round(float(centre[1]), 3), "r_mid_mm": round((r_inner + band / 2) * 1000), "band_mm": round(band * 1000),
            "hole_mm": round(2 * r_inner * 1000)}  # fmt: skip


def summary(world: dict) -> list[dict]:
    """The records without their histories' bulk: what an agent needs to choose and to speak."""
    out = []
    for k, o in world["objects"].items():
        last = (o.get("history") or [None])[-1]
        out.append({"key": k, "name": o.get("name"), "kind": o.get("kind"), "colour": o.get("colour"), "state": o.get("state"), "on": o.get("on"),
                    "x": o.get("x"), "y": o.get("y"), "height_mm": o.get("height_mm"), "width_mm": o.get("width_mm"), "shape": o.get("shape"),
                    "last": f"{last['action']}: {last['outcome']}" if last else None})  # fmt: skip
    return out
