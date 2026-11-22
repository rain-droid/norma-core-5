"""Fit the depth camera across the table to the robot: where it stands, in the base frame.

Run it once after the camera or the mat has been moved, with the mat mostly clear and at least two
of its corner tags uncovered in the `across` view. No arm motion. Two things fix the camera:

* the mat is the largest flat surface the depth sensor sees: that gives its height and tilt;
* the mat's corner tags, seen by the camera's own colour picture (`across`), give the turn and
  the position on the mat. Their places on the mat are the ones calibrate_top uses, and the
  robot's place on the mat comes from the top camera's calibration (robot_config/camera_top.json).

Writes robot_config/camera_depth.json. Check the result: `flatness_mm` is how flat the mat is in
the depth picture (1 to 2 mm is normal), `tag_fit_mm` how far each tag lands from its known place
(with two tags this is the error of the distance between them, split in two: under 5 mm is good),
`camera_base` where the camera is. Then depth_scan gives heights.
"""

import json
import time

import cv2
import numpy as np

from calibrate_top import TAGS, detect
from util import depth as depth_camera
from util.arm import RobotError


def calibrate_depth(ctx, frames: int = 12) -> dict:
    meta = ctx.arm.meta()
    if meta["mode"] == "sim":
        raise RobotError("the simulated table has no depth camera")
    top = json.loads((ctx.root / "robot_config" / "camera_top.json").read_text())
    if not top.get("base_in_mat"):
        raise RobotError("the top camera's calibration has no base_in_mat, which places the robot on the mat: run tools/calibrate_top.py first")
    seen: dict[int, list] = {}
    for _ in range(frames):
        img = ctx.frame("across", settle=0.1)
        for i, c in detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)).items():
            if i in TAGS:
                seen.setdefault(i, []).append(c.mean(axis=0))
    tags_px = {i: np.median(v, axis=0) for i, v in seen.items()}
    depth, cal = depth_camera.grab(ctx.root)
    n, d, flatness = depth_camera.table_plane(depth_camera.points(depth, cal["depth_K"]))
    T, fit = depth_camera.register(n, d, cal, tags_px, (img.shape[1], img.shape[0]), TAGS, top["base_in_mat"], float(meta.get("table_z", 0.0)))
    path = ctx.root / "robot_config" / "camera_depth.json"
    path.write_text(json.dumps({
        "T_base_depth": [[round(float(v), 6) for v in row] for row in T],
        "depth_K": cal["depth_K"],
        "tags_px": {str(i): [round(float(v), 1) for v in tags_px[i]] for i in fit["tags"]},
        "flatness_mm": round(flatness * 1000, 2),
        "tag_fit_mm": fit["tag_fit_mm"],
        "fitted": time.strftime("%Y-%m-%d %H:%M"),
    }, indent=1) + "\n")  # fmt: skip
    marked = img.copy()
    for i, c in tags_px.items():
        cv2.circle(marked, (int(c[0]), int(c[1])), 8, (0, 255, 0), 2)
        cv2.putText(marked, str(i), (int(c[0]) + 10, int(c[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return {
        "tags_used": fit["tags"],
        "tag_fit_mm": fit["tag_fit_mm"],
        "flatness_mm": round(flatness * 1000, 2),
        "camera_base": fit["camera_base"],
        "depth_coverage": round(float(np.isfinite(depth).mean()), 2),
        "image": ctx.save_image("tags_across.jpg", marked, "the depth camera's colour view, the tags it was fitted to circled"),
        "saved": str(path.relative_to(ctx.root)),
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(calibrate_depth)
