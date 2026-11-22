"""Measure height with the Orbbec DaBai DCW2 depth camera across the table (USB 2bc5:06a0).

Its separate RGB device is 2bc5:0561, available as capture_camera(camera="across").

    depth_scan()                                  everything standing on the mat
    depth_scan(at=[[0.05, 0.25], [-0.1, 0.3]])    also the height at those spots

`objects`: each thing higher than `min_height_mm` above the mat, tallest first: x, y of the middle
of its top (base frame, metres), `height_mm` of that top above the mat, `size_mm` (long and short
side) of what the camera sees of it. `heights_mm`: for each spot in `at`, the height of what
stands there, or null where the camera has no reading. `image`: the table from above coloured by
height, base at the bottom, +x right, 10 cm grid; grey is where the camera sees nothing.

What it is good for: the height of a stack or of one object on another, whether a place is free,
whether something was really put down or lifted (scan before and after). Heights are good to
about 3 mm. Positions are good to about a centimetre: for aiming a grasp use find_objects (the
top camera), and pass its x, y here in `at` to get that object's height.

Use this before table descents and after a lift or placement. Wait until the arm has finished
moving home before scanning. Heights are relative to the mat: a stack's reading is its total
top height, not the upper object's thickness. Divide mm by 1000 for grasp_height or place z.
Null/grey is unknown, not a clear spot; inspect capture_camera(camera="across") and re-scan
from an unobstructed view. A scan measures only: call scan_obstacles separately to install the
collision map. It does not correct robot fingertip calibration or replace the shared extra
descent-clearance limit in robot_config/motion.json.

What it does not see: the side of things facing away from the camera, and the strip of mat behind
each thing (its shadow, grey in the image). The arm over the mat shows as a tall thing too,
marked `"arm": true`, and hides what is under and behind it: for a clear view send the arm home,
or to rest, first.

The camera must have been fitted once with calibrate_depth. No arm motion; takes about 3 s.
"""

import numpy as np

from util import depth as depth_camera


def _off_line(p: np.ndarray, end: np.ndarray) -> float:
    """How far p is from the line between the base (0, 0) and `end`."""
    along = float(np.clip(p @ end / max(float(end @ end), 1e-9), 0.0, 1.0))
    return float(np.linalg.norm(p - along * end))


def depth_scan(ctx, at: list | None = None, min_height_mm: float = 8.0) -> dict:
    state = ctx.arm.state()
    table_z = float(ctx.arm.meta().get("table_z", 0.0))
    T = depth_camera.load(ctx.root)
    depth, cal = depth_camera.grab(ctx.root)
    pts = depth_camera.to_base(depth_camera.points(depth, cal["depth_K"]), T)
    grid = depth_camera.height_map(pts, table_z)
    tool = np.array(state["ee"]["pos"]) if state.get("ee") else None
    objects = []
    for b in depth_camera.bumps(grid, min_height_mm / 1000.0):
        o = {k: v for k, v in b.items() if k != "mask"}
        # the arm over the mat: a tall patch near the line from the base to the tool point
        if tool is not None and o["height_mm"] > 60 and _off_line(np.array([o["x"], o["y"]]), tool[:2]) < 0.09:
            o["arm"] = True
        objects.append(o)
    marks = [{"x": o["x"], "y": o["y"], "label": ("arm " if o.get("arm") else "") + f"{o['height_mm']} mm"} for o in objects]
    out = {"objects": objects}
    if at:
        heights = [depth_camera.height_at(grid, float(x), float(y)) for x, y in at]
        out["heights_mm"] = [None if h is None else round(h * 1000) for h in heights]
        marks += [{"x": float(x), "y": float(y), "label": "?" if h is None else f"{round(h * 1000)}"} for (x, y), h in zip(at, heights)]
    (x0, x1), (y0, y1) = depth_camera.AREA
    out["seen_share"] = round(float(np.isfinite(grid).mean()), 2)
    out["area"] = {"x": [x0, x1], "y": [y0, y1]}
    out["image"] = ctx.save_image("heights.jpg", depth_camera.picture(grid, marks), "the table from above, coloured by height above the mat (blue 0, red 80 mm and more)")
    return out


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(depth_scan)
