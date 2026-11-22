"""Frozen camera captures, and what a pixel in one means on the table.

A pixel only means something together with where the camera was. So every capture is stored in
its call's history folder with what is needed to read it, and is named by that folder: the
`image_id`. Tools that take a pixel take the image_id too, and stay correct after the arm moves.

Two cameras, two models:

* `top` is fixed, above the mat, looking down at it. Its calibration
  (robot_config/camera_top.json, written by tools/calibrate_top.py) maps a pixel to the mat
  surface with a homography fitted to the four corner tags, and holds the camera's position,
  which extends that to points at a known height. This is the camera to measure with.
* `wrist` rides on the gripper. A pinhole model placed in the tool frame
  (robot_config/camera_wrist.json if someone has calibrated it, else a rough default that is
  centimetres off on the real arm) casts a ray from wherever the gripper was.
"""

import json
import math
import re
import time
from pathlib import Path

import cv2
import numpy as np

from util import vision
from util.arm import RobotError


def pose_matrix(ee: dict) -> np.ndarray:
    """4x4 tool pose from the state's `ee` (pos, quat [x, y, z, w])."""
    x, y, z, w = ee["quat"]
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]  # fmt: skip
    T[:3, 3] = ee["pos"]
    return T


def freeze(ctx, camera: str, img: np.ndarray, grid: bool = True) -> dict:
    """Save `img` as this call's capture. One capture per call."""
    h, w = img.shape[:2]
    state, robot = ctx.arm.state(), ctx.arm.meta()
    meta = {
        "image_id": ctx.history.name,
        "camera": camera,
        "width": w,
        "height": h,
        "t": time.time(),
        "mode": robot["mode"],  # "real" or "sim": which calibration applies
        "table_z": robot["table_z"],
        "joints_rad": [j["q"] for j in state["joints"][:7]],
        "T_base_tool": pose_matrix(state["ee"]).tolist() if state["ee"] else None,
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
    return cv2.imread(str(folder / "rgb.jpg")), json.loads((folder / "capture.json").read_text())


# --------------------------------------------------------------------------- the top camera


class TopCamera:
    """The fixed camera's calibration: pixels <-> points in the robot's base frame.

    The file works in mat coordinates (metres on the mat surface, z up from it); `base_in_mat`
    places the robot on the mat: x, y of the base origin and its rotation in degrees.
    """

    def __init__(self, root: Path, mode: str, table_z: float):
        self.path = root / "robot_config" / ("camera_top.sim.json" if mode == "sim" else "camera_top.json")
        cal = json.loads(self.path.read_text()) if self.path.exists() else {}
        if not cal.get("H_mat_px") or not cal.get("base_in_mat"):
            raise RobotError(f"the top camera is not calibrated ({self.path.name} is missing or incomplete): run tools/calibrate_top.py")
        self.H = np.array(cal["H_mat_px"])  # pixel -> mat surface
        self.P = np.array(cal["P_px_mat"]) if cal.get("P_px_mat") else None  # mat (x, y, z) -> pixel
        self.eye = np.array(cal["camera_mat"]) if cal.get("camera_mat") else None  # camera position on the mat
        bx, by, yaw = cal["base_in_mat"]
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        self.R = np.array([[c, -s], [s, c]])  # base axes on the mat
        self.b = np.array([bx, by])
        self.table_z = table_z

    def to_base(self, u: float, v: float, height: float = 0.0) -> np.ndarray:
        """The point `height` metres above the mat that pixel (u, v) shows, in the base frame."""
        m = cv2.perspectiveTransform(np.array([[[u, v]]], dtype=np.float64), self.H)[0, 0]
        if height:
            if self.eye is None:
                raise RobotError("this calibration has no camera position, so it only gives positions on the mat surface: re-run tools/calibrate_top.py")
            m = self.eye[:2] + (m - self.eye[:2]) * (self.eye[2] - height) / self.eye[2]  # slide along the ray up to that height
        return np.array([*(self.R.T @ (m - self.b)), self.table_z + height])

    def to_pixel(self, p) -> np.ndarray:
        """Where a base-frame point appears in the image."""
        if self.P is None:
            raise RobotError("this calibration has no camera position: re-run tools/calibrate_top.py")
        p = np.asarray(p, dtype=float)
        x = self.P @ np.array([*(self.R @ p[:2] + self.b), p[2] - self.table_z, 1.0])
        return x[:2] / x[2]

    def eye_base(self) -> np.ndarray:
        """Where the camera is, in the base frame."""
        return np.array([*(self.R.T @ (self.eye[:2] - self.b)), self.table_z + self.eye[2]])


def camera_from_homography(H_mat_px, image_size) -> tuple[np.ndarray, np.ndarray]:
    """Where the camera is, from the pixel -> mat homography alone.

    One plane's homography fixes the whole camera if the lens is taken to be ideal: square
    pixels, the optical axis through the image centre, no distortion. The mat's two axes are
    perpendicular and equally long in space; asking that of their images gives the focal length,
    and the pose follows. Returns (P, eye): the 3x4 projection from mat coordinates (z up from
    the mat) to pixels, and the camera's position in mat coordinates.
    """
    w, h = image_size
    centred = np.array([[1, 0, -w / 2], [0, 1, -h / 2], [0, 0, 1.0]]) @ np.linalg.inv(np.asarray(H_mat_px, dtype=float))
    h1, h2, h3 = centred.T  # images of the mat's x axis, y axis and origin, pixels from the image centre
    cross, same = h1[2] * h2[2], h2[2] ** 2 - h1[2] ** 2
    if abs(cross) > abs(same):  # perpendicular ...
        f2 = -(h1[0] * h2[0] + h1[1] * h2[1]) / cross
    else:  # ... or equally long, whichever constraint is the better conditioned
        f2 = (h1[0] ** 2 + h1[1] ** 2 - h2[0] ** 2 - h2[1] ** 2) / same
    if not (np.isfinite(f2) and (0.2 * w) ** 2 < f2 < (5 * w) ** 2):
        raise RobotError("the tag fit does not pin down where the camera is (a camera looking straight down at the mat cannot be located this way)")
    f = math.sqrt(f2)
    b1, b2, b3 = (np.diag([1 / f, 1 / f, 1.0]) @ centred).T
    k = 2.0 / (np.linalg.norm(b1) + np.linalg.norm(b2))
    if b3[2] < 0:
        k = -k  # the mat is in front of the camera
    U, _, Vt = np.linalg.svd(np.column_stack([k * b1, k * b2, np.cross(k * b1, k * b2)]))
    R, t = U @ Vt, k * b3
    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])
    return K @ np.column_stack([R, t]), -R.T @ t


# --------------------------------------------------------------------------- the wrist camera


def wrist_camera(root: Path) -> vision.CameraModel:
    return vision.CameraModel.load(root / "robot_config" / "camera_wrist.json")


def table_point(ctx, meta: dict, u: float, v: float, height: float = 0.0) -> np.ndarray:
    """The base-frame point `height` metres above the table that pixel (u, v) of a capture shows."""
    if not (0 <= u < meta["width"] and 0 <= v < meta["height"]):
        raise RobotError(f"pixel ({u}, {v}) is outside the {meta['width']}x{meta['height']} image")
    if meta["camera"] == "top":
        return TopCamera(ctx.root, meta["mode"], meta["table_z"]).to_base(u, v, height)
    if meta["camera"] != "wrist" or meta.get("T_base_tool") is None:
        raise RobotError(f"capture '{meta['image_id']}' is from the {meta['camera']} camera, which has no calibration")
    cam = wrist_camera(ctx.root).scaled(meta["width"], meta["height"])
    p = cam.pixel_to_plane(u, v, np.array(meta["T_base_tool"]), meta["table_z"] + height)
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
