"""Look at the table and say what is on it: a numbered picture sheet for you to name the objects from.

find_objects knows where things are and what colour they are, not what they are: to it a rubber
ring and a cube are both `red`. This tool looks with the top camera, brings the world model
(state/world.json) up to date, and returns `sheet_image`: one enlarged cut-out per object,
labelled with its key (`blue-1`). Open it, decide what each thing is, and write that down with
name_objects. From then on tools take the name ("the ring"), and the object keeps it when it is
moved.

It also notices a hole: an object the table shows through (a ring, a roll of tape) gets
`shape: "ring"` and is later grasped by its rim, not at its empty centre.

Each entry of `objects` has the key, the name and kind if it has been named before, colour,
position (metres, base frame), size, shape, whether it is `new` since the last look, and
`reachable`: whether the gripper can get to it at all; say so to the person before promising
to pick up something that is not.
`unseen` lists known objects this look did not find (hidden, taken away, or held).

Does not move the arm; bring it `home` or to `rest` first if it covers what you want to see.
`max_objects` as in find_objects.
"""

import cv2
import numpy as np

import find_objects as fo
from util import capture, vision, world as W
from util.arm import RobotError


def _sheet(img: np.ndarray, items: list[tuple[str, dict, dict]]) -> np.ndarray:
    """One enlarged cut-out per object, in a row or two, each under its key."""
    cell, pad = 220, 8
    tiles = []
    for key, o, mark in items:
        x1, y1, x2, y2 = mark["box"]
        m = int(0.45 * max(x2 - x1, y2 - y1)) + 6
        cut = img[max(y1 - m, 0) : y2 + m, max(x1 - m, 0) : x2 + m]
        k = cell / max(cut.shape[:2])
        cut = cv2.resize(cut, None, fx=k, fy=k, interpolation=cv2.INTER_CUBIC)
        tile = np.full((cell + 46, cell, 3), 28, np.uint8)
        tile[: cut.shape[0], : cut.shape[1]] = cut
        name = o.get("name") or "not named yet"
        vision._label(tile, key, (6, cell + 18), scale=0.6, color=(0, 255, 255))
        vision._label(tile, name[:26], (6, cell + 38), scale=0.42)
        tiles.append(tile)
    per_row = 4 if len(tiles) > 3 else max(1, len(tiles))
    rows = []
    for i in range(0, len(tiles), per_row):
        row = tiles[i : i + per_row] + [np.full_like(tiles[0], 28)] * (per_row - len(tiles[i : i + per_row]))
        rows.append(np.hstack([np.pad(t, ((pad, pad), (pad, pad), (0, 0)), constant_values=28) for t in row]))
    return np.vstack(rows)


def look(ctx, max_objects: int = 8) -> tuple[dict, list[str], np.ndarray, list[dict], dict]:
    """find_objects' look, with rings noticed and the world model brought up to date.
    Returns (world, keys in the order seen, the image, the marks, the capture's meta)."""
    img = np.median([ctx.frame("top")] + [ctx.frame("top", settle=0.08) for _ in range(4)], axis=0).astype(np.uint8)
    meta = capture.freeze(ctx, "top", img, grid=False)
    objects, marks = fo.from_top(ctx, img, meta, max_objects)
    cam = capture.TopCamera(ctx.root, meta["mode"], meta["table_z"])
    for o, m in zip(objects, marks):
        ring = W.ring_of(img, m["polygon"], cam, (o.get("height_mm") or 25) / 1000)
        o["shape"], o["ring"] = ("ring", ring) if ring else ("solid", None)
        if ring:  # a ring stands where its hole is, whatever the outline's middle says
            o["x"], o["y"] = ring["x"], ring["y"]
    world = W.load()
    keys = W.sync(world, objects)
    W.save(world)
    return world, keys, img, marks, meta


def describe_objects(ctx, max_objects: int = 8) -> dict:
    before = set(W.load()["objects"])
    world, keys, img, marks, meta = look(ctx, max_objects)
    items = [(k, world["objects"][k], m) for k, m in zip(keys, marks)]
    listed = []
    for k, o, _ in items:
        try:  # can the gripper get to it?
            ctx.plan.approach_for(np.array([o["x"], o["y"], ctx.plan.table_z]), 0.025, float(o.get("yaw_deg") or 0.0))
            reachable = True
        except RobotError:
            reachable = False
        listed.append({"key": k, "name": o.get("name"), "kind": o.get("kind"), "colour": o["colour"], "x": o["x"], "y": o["y"],
                       "width_mm": o.get("width_mm"), "height_mm": o.get("height_mm"), "shape": o["shape"], "new": k not in before,
                       "reachable": reachable})  # fmt: skip
    out = {
        "image_id": meta["image_id"],
        "objects": listed,
        "unseen": [{"key": k, "name": o.get("name"), "state": o["state"]} for k, o in world["objects"].items() if k not in keys],
        "holding": world.get("holding"),
    }
    if items:
        out["sheet_image"] = ctx.save_image("sheet.jpg", _sheet(img, items), "one cut-out per object, labelled with its key: name them with name_objects")
        ctx.emit("detections", camera="top", width=meta["width"], height=meta["height"],
                 items=[{"label": o.get("name") or k, "score": 1.0, "box": m["box"], "point": m["point"], "polygon": m["polygon"]} for k, o, m in items])  # fmt: skip
    unnamed = [e["key"] for e in listed if not e["name"]]
    if unnamed:
        out["next"] = f"open sheet_image and name {', '.join(unnamed)} with tools/name_objects.py"
    return out


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(describe_objects)
