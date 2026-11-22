"""Look at the table with the depth camera and keep what stands on it as obstacles for both arms.

The height map of depth_scan (4 mm cells over the mat) is kept in state/obstacles.npz, with both
arms, where they stand now, cut out of it. For the next two minutes every move of this arm, and
the second arm's home, move_to and move_joints, is also walked against it
(util/shared_space.py): a part of an arm may not dip into something on the table, except around
where the move starts and where it ends. After two minutes, or once things have been moved, scan
again: a stale map refuses moves for no reason, or allows one it should not.

`things` are what was kept, tallest first (x, y, height_mm). The web UI's 3D view shows them as
columns coloured by height, beside both arms. What is under or behind an arm is
not seen: send the arms home or to rest first if that matters. No motion.

Wait for the arms to stop before scanning. Use depth_scan(at=[[x, y]]) for the target's height
and the top camera for x/y; this tool installs the depth obstacle map rather than choosing a
grasp height. Refresh before carrying and after objects move. Missing depth and the excluded
start/end regions are not proof of free space. This map supplements, and never relaxes, the
extra mat-clearance floor applied to every ElRobot joint command in util/arm.py.
"""

import numpy as np

from util import depth as depth_camera
from util import second_arm, shared_space
from util.arm import RobotError


def scan_obstacles(ctx) -> dict:
    table_z = float(ctx.arm.meta().get("table_z", 0.0))
    T = depth_camera.load(ctx.root)
    depth, cal = depth_camera.grab(ctx.root)
    grid = depth_camera.height_map(depth_camera.to_base(depth_camera.points(depth, cal["depth_K"]), T), table_z)
    space = shared_space.space(ctx.root)
    try:
        q_second = second_arm.joints_now()
    except RobotError:
        q_second = None
    kept = space.keep_obstacles(ctx.root, grid, depth_camera.AREA, depth_camera.CELL, table_z, ctx.arm.q(), q_second)
    space.use_obstacles(ctx.root)
    things = [{k: v for k, v in b.items() if k != "mask"} for b in depth_camera.bumps(np.where(kept > 0, kept, np.nan))]
    marks = [{"x": o["x"], "y": o["y"], "label": f"{o['height_mm']} mm"} for o in things]
    # the web UI's 3D view: a column for every 8 mm of table that has something on it, and a label on each thing
    h, w = kept.shape[0] // 2 * 2, kept.shape[1] // 2 * 2
    coarse = kept[:h, :w].reshape(h // 2, 2, w // 2, 2).max(axis=(1, 3))
    (x0, _), (y0, _) = depth_camera.AREA
    cell = 2 * depth_camera.CELL
    boxes = [[round(float(x0 + (c + 0.5) * cell), 3), round(float(y0 + (r + 0.5) * cell), 3), round(float(coarse[r, c]), 3)] for r, c in zip(*np.nonzero(coarse >= shared_space.LOW))]
    ctx.emit("obstacles", cell=cell, table_z=table_z, boxes=boxes)
    ctx.emit("markers", markers=[{"id": f"thing{i}", "label": f"{o['height_mm']} mm", "pos": [o["x"], o["y"], table_z + o["height_mm"] / 1000 + 0.012],
                                  "color": "#ff8a3d", "radius": 0.006} for i, o in enumerate(things)])  # fmt: skip
    return {
        "things": things,
        "image": ctx.save_image("obstacles.jpg", depth_camera.picture(np.where(kept > 0, kept, 0.0), marks), "the things on the table kept as obstacles, from above, coloured by height; the arms are cut out"),
        "used_for_s": shared_space.OBSTACLES_LAST_S,
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(scan_obstacles)
