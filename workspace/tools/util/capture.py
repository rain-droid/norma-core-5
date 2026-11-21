"""Frozen camera captures: an image bound to the pose it was taken from.

A pixel only means something together with where the camera was. So every capture is stored in
its call's history folder with the tool pose at that instant, and is named by that folder: the
`image_id`. Tools that take a pixel take the image_id too, and stay correct after the arm moves.
"""

import json
import re
import time

import cv2
import numpy as np

from elrobot import vision
from elrobot.robot import RobotError


def freeze(ctx, camera: str, img: np.ndarray, grid: bool = True) -> dict:
    """Save `img` as this call's capture. One capture per call."""
    h, w = img.shape[:2]
    meta = {
        "image_id": ctx.history.name,
        "camera": camera,
        "width": w,
        "height": h,
        "t": time.time(),
        "joints_rad": [round(float(v), 5) for v in ctx.robot.q()],
        # only the wrist camera has a model (it rides on the tool); the top camera is view-only
        "T_base_tool": ctx.robot.tcp().tolist() if camera == "wrist" else None,
    }
    meta["image"] = ctx.save_image("rgb.jpg", img, f"{camera} camera, unmodified, {w}x{h}")
    if grid:
        meta["grid_image"] = ctx.save_image("rgb_grid.jpg", vision.draw_grid(img), f"{camera} camera with pixel gridlines every 100 px")
    ctx.save_json("capture.json", meta)
    return meta


def load(ctx, image_id: str) -> tuple[np.ndarray, dict]:
    if not re.fullmatch(r"[0-9]{6}_[a-z0-9_]+", str(image_id)):
        raise RobotError(f"'{image_id}' is not an image_id; use the one returned by capture_camera / find_objects / look_at_table")
    folder = ctx.root / "tool_call_history" / image_id
    if not (folder / "capture.json").is_file():
        raise RobotError(f"no capture named '{image_id}' in tool_call_history/")
    meta = json.loads((folder / "capture.json").read_text())
    img = cv2.imread(str(folder / "rgb.jpg"))
    return img, meta


def table_point(ctx, meta: dict, u: float, v: float, height: float = 0.0) -> np.ndarray:
    """Base-frame point where the ray through pixel (u, v) of a wrist capture meets the
    horizontal plane `height` metres above the table."""
    if meta.get("T_base_tool") is None:
        raise RobotError(f"capture '{meta['image_id']}' is from the {meta['camera']} camera, which has no calibration; only wrist captures give table positions")
    if not (0 <= u < meta["width"] and 0 <= v < meta["height"]):
        raise RobotError(f"pixel ({u}, {v}) is outside the {meta['width']}x{meta['height']} image")
    cam = ctx.skills.cam.scaled(meta["width"], meta["height"])
    p = cam.pixel_to_plane(u, v, np.array(meta["T_base_tool"]), ctx.cfg.table_z + height)
    if p is None:
        raise RobotError(f"pixel ({u:.0f}, {v:.0f}) looks above the horizon, not at the table")
    return p


def jaw_mask(img: np.ndarray) -> np.ndarray:
    """Pixels covered by the gripper's own jaws in the wrist view (bottom centre)."""
    h, w = img.shape[:2]
    mask = np.zeros((h, w), bool)
    mask[int(0.58 * h) :, int(0.08 * w) : int(0.92 * w)] = True
    return mask


def crosshair(img: np.ndarray, points: list[tuple[float, float, str]]) -> np.ndarray:
    """Mark pixels on a copy of the image, so a click can be checked against what was meant."""
    out = img.copy()
    for u, v, label in points:
        c = (int(round(u)), int(round(v)))
        for width, color in ((5, (0, 0, 0)), (2, (255, 255, 0))):
            cv2.line(out, (c[0] - 14, c[1]), (c[0] + 14, c[1]), color, width, cv2.LINE_AA)
            cv2.line(out, (c[0], c[1] - 14), (c[0], c[1] + 14), color, width, cv2.LINE_AA)
        vision._label(out, label, (min(c[0] + 16, out.shape[1] - 120), max(14, c[1] - 10)), scale=0.5)
    return out
