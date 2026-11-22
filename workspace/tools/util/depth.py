"""The depth camera across the table (Orbbec DaBai DCW2): depth pictures, and points in the base frame.

The camera (the user's "Orbbec depth cam 2") is two USB devices: RGB 2bc5:0561 and depth
2bc5:06a0. USB bus/device numbers can change. Its colour camera is an ordinary webcam and reaches the tools
through the Station as camera `across`. Its depth sensor is read here, directly, with Orbbec's
SDK (built into bin/orbbec/ by scripts/orbbec_setup.sh): 640x400, every pixel the distance along
the lens axis. A grab takes about two seconds, most of it starting the sensor.

Where the depth sensor stands is fitted by tools/calibrate_depth.py and kept in
robot_config/camera_depth.json as `T_base_depth`: the mat is the largest flat thing it sees, which
fixes its height and tilt, and the mat's corner tags seen by its colour camera fix the rest.
Heights measured this way are true heights: unlike the top camera, which sees a tall object's top
shifted away from itself, a depth point is where the surface is.
"""

from __future__ import annotations

import ctypes
import json
import math
import os
import sys
import time
import warnings
from pathlib import Path

import cv2
import numpy as np

from util.arm import RobotError

SIZE = (640, 400)  # the depth picture
COLOUR_SIZE = (1280, 720)  # the size the colour calibration below is for; the Station shows the same view smaller
CELL = 0.004  # the height map's grid, metres
AREA = ((-0.34, 0.34), (0.02, 0.50))  # the part of the table the height map covers, base frame x and y


def _sdk(root: Path):
    d = Path(os.environ.get("ELROBOT_ORBBEC") or root.parent / "bin" / "orbbec")
    if not list(d.glob("pyorbbecsdk.*.so")):
        raise RobotError(f"the depth camera's SDK is not built ({d} is empty): run scripts/orbbec_setup.sh once")
    ctypes.CDLL(str(d / "libOrbbecSDK.so"), mode=ctypes.RTLD_GLOBAL)  # so the module finds it without LD_LIBRARY_PATH
    if str(d) not in sys.path:
        sys.path.insert(0, str(d))
    import pyorbbecsdk as ob

    ob.Context.set_logger_level(ob.OBLogLevel.NONE)  # otherwise it writes a Log/ folder wherever it runs
    return ob


def grab(root: Path, frames: int = 8) -> tuple[np.ndarray, dict]:
    """One depth picture in metres (nan where the sensor saw nothing), the median of `frames`
    pictures, and the camera's own calibration: depth and colour intrinsics, and the colour
    camera's place relative to the depth sensor."""
    ob = _sdk(root)
    devices = ob.Context().query_devices()
    if devices.get_count() == 0:
        raise RobotError("no depth camera found: is the Orbbec plugged in (lsusb shows 2bc5:06a0), and may this user open it "
                         "(the udev rule in scripts/orbbec_setup.sh)?")  # fmt: skip
    device = devices.get_device_by_index(0)
    pipe, cfg = ob.Pipeline(device), ob.Config()
    profiles = pipe.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
    cfg.enable_stream(profiles.get_video_stream_profile(SIZE[0], SIZE[1], ob.OBFormat.Y11, 15))
    cal = _calibration(device)
    pipe.start(cfg)
    try:
        stack, t0 = [], time.time()
        while len(stack) < frames and time.time() - t0 < 10:
            got = pipe.wait_for_frames(1000)
            d = got.get_depth_frame() if got is not None else None
            if d is None or time.time() - t0 < 0.8:  # the first pictures are empty or still settling
                continue
            stack.append(np.frombuffer(d.get_data(), np.uint16).reshape(d.get_height(), d.get_width()).astype(np.float32) * d.get_depth_scale())
    finally:
        pipe.stop()
    if not stack:
        raise RobotError("the depth camera gave no picture in 10 s. If /sys/module/usbcore/parameters/usbfs_memory_mb says 16, raise it "
                         "(scripts/orbbec_setup.sh says how); otherwise unplug the camera and plug it in again")  # fmt: skip
    mm = np.stack(stack)
    mm[mm <= 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # a pixel with no reading in any picture stays nan
        depth = np.nanmedian(mm, axis=0) / 1000.0
    depth[np.isfinite(mm).sum(axis=0) < max(1, len(stack) // 2)] = np.nan  # a pixel seen in under half the pictures is an edge flickering
    return depth.astype(np.float32), cal


def _calibration(device) -> dict:
    params = device.get_calibration_camera_param_list()
    for i in range(params.get_count()):
        p = params.get_camera_param(i)
        if (p.depth_intrinsic.width, p.depth_intrinsic.height) == SIZE and (p.rgb_intrinsic.width, p.rgb_intrinsic.height) == COLOUR_SIZE:
            d, c, k = p.depth_intrinsic, p.rgb_intrinsic, p.rgb_distortion
            return {
                "depth_K": [[d.fx, 0, d.cx], [0, d.fy, d.cy], [0, 0, 1]],
                "colour_K": [[c.fx, 0, c.cx], [0, c.fy, c.cy], [0, 0, 1]],
                "colour_dist": [k.k1, k.k2, k.p1, k.p2, k.k3],
                "R_colour_depth": np.asarray(p.transform.rot, dtype=float).tolist(),  # a depth-frame point p is at R p + t in the colour camera
                "t_colour_depth": (np.asarray(p.transform.transform, dtype=float) / 1000.0).tolist(),
            }
    raise RobotError(f"the depth camera has no calibration for depth {SIZE} with colour {COLOUR_SIZE}")


def points(depth: np.ndarray, K) -> np.ndarray:
    """Every pixel with a reading as a point in the depth sensor's frame (x right, y down, z out), metres. N x 3."""
    K = np.asarray(K, dtype=float)
    v, u = np.nonzero(np.isfinite(depth))
    z = depth[v, u].astype(float)
    return np.column_stack([(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z])


def table_plane(pts: np.ndarray, within: float = 0.9, tol: float = 0.004, seed: int = 0) -> tuple[np.ndarray, float, float]:
    """The largest flat surface within `within` metres of the sensor: its unit normal pointing at
    the sensor, its offset d (n . p = d on the plane), and how flat the points on it are (rms, metres)."""
    near = pts[np.linalg.norm(pts, axis=1) < within]
    if len(near) < 500:
        raise RobotError("the depth camera sees almost nothing within reach: is it pointed at the mat?")
    rng = np.random.default_rng(seed)
    sample = near[rng.choice(len(near), min(len(near), 20000), replace=False)]
    best, best_n = None, 0
    for _ in range(400):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        n = np.cross(b - a, c - a)
        length = np.linalg.norm(n)
        if length < 1e-9:
            continue
        n /= length
        count = int((np.abs((sample - a) @ n) < tol).sum())
        if count > best_n:
            best, best_n = (n, float(n @ a)), count
    n, d = best
    for _ in range(3):  # least squares on the points near it
        on = near[np.abs(near @ n - d) < tol]
        centre = on.mean(axis=0)
        n = np.linalg.svd(on - centre, full_matrices=False)[2][-1]
        d = float(n @ centre)
    if d > 0:  # the sensor is at the origin: make the normal point at it, so heights above the plane are positive
        n, d = -n, -d
    on = near[np.abs(near @ n - d) < tol]
    return n, d, float(np.sqrt(np.mean((on @ n - d) ** 2)))


def register(n: np.ndarray, d: float, cal: dict, tags_px: dict[int, np.ndarray], image_size, tags_mat: dict[int, tuple[float, float]],
             base_in_mat, table_z: float) -> tuple[np.ndarray, dict]:  # fmt: skip
    """The depth sensor's place in the base frame, as a 4x4 `T_base_depth`, and how well it fits.

    `n`, `d`: the mat's plane in the depth frame (table_plane). `tags_px`: the centres of the mat's
    corner tags in the colour picture, of size `image_size`. Each tag's sight line is followed down
    to the plane; the turn and shift on the plane that put those points on the tags' known places
    (`tags_mat`, metres on the mat) complete the fit. Two tags are enough.
    """
    ids = [i for i in tags_px if i in tags_mat]
    if len(ids) < 2:
        raise RobotError(f"the depth camera's colour picture shows {len(ids)} of the mat's corner tags and the fit needs two: uncover them, or turn the camera so two are in view")
    K = np.asarray(cal["colour_K"], dtype=float).copy()
    K[0] *= image_size[0] / COLOUR_SIZE[0]
    K[1] *= image_size[1] / COLOUR_SIZE[1]
    R, t = np.asarray(cal["R_colour_depth"], dtype=float), np.asarray(cal["t_colour_depth"], dtype=float)
    px = np.array([tags_px[i] for i in ids], dtype=np.float64).reshape(-1, 1, 2)
    rays = cv2.undistortPoints(px, K, np.asarray(cal["colour_dist"], dtype=float)).reshape(-1, 2)
    origin = -R.T @ t  # the colour camera, in the depth frame
    on_plane = []
    for x, y in rays:
        direction = R.T @ np.array([x, y, 1.0])
        on_plane.append(origin + direction * (d - n @ origin) / (n @ direction))
    on_plane = np.array(on_plane)
    # a right-handed frame on the plane: e1, e2, n
    e1 = np.cross([0.0, 1.0, 0.0], n)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    flat = np.column_stack([on_plane @ e1, on_plane @ e2])
    want = np.array([tags_mat[i] for i in ids], dtype=float)
    fc, wc = flat.mean(axis=0), want.mean(axis=0)
    cross = sum(a[0] * b[1] - a[1] * b[0] for a, b in zip(flat - fc, want - wc))
    dot = sum(a @ b for a, b in zip(flat - fc, want - wc))
    turn = math.atan2(cross, dot)
    c, s = math.cos(turn), math.sin(turn)
    Rz = np.array([[c, -s], [s, c]])
    shift = wc - Rz @ fc
    fit = np.linalg.norm((flat @ Rz.T + shift) - want, axis=1)
    # depth frame -> mat (x, y on the mat, z above it) -> base
    T_mat = np.eye(4)
    T_mat[:2, :3] = Rz @ np.array([e1, e2])
    T_mat[:2, 3] = shift
    T_mat[2, :3], T_mat[2, 3] = n, -d
    bx, by, yaw = base_in_mat
    cb, sb = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    Rb = np.array([[cb, -sb], [sb, cb]])  # base axes on the mat
    T_base = np.eye(4)
    T_base[:2, :2] = Rb.T
    T_base[:2, 3] = -Rb.T @ np.array([bx, by])
    T_base[2, 3] = table_z
    T = T_base @ T_mat
    return T, {"tags": ids, "tag_fit_mm": [round(float(v) * 1000, 1) for v in fit], "camera_base": [round(float(v), 3) for v in T[:3, 3]]}


COLOURS = {"blue": ((95, 110, 50), (130, 255, 255)), "green": ((40, 80, 50), (85, 255, 255)), "pink": ((145, 60, 80), (175, 255, 255)),
           "orange": ((5, 130, 80), (20, 255, 255)), "yellow": ((22, 100, 80), (35, 255, 255))}  # HSV ranges, OpenCV scale  # fmt: skip


def colour_of_points(pts: np.ndarray, cal: dict, colour_img: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """For depth-frame points: the HSV colour the camera's colour picture shows there, and which points fall inside that picture."""
    h, w = colour_img.shape[:2]
    K = np.asarray(cal["colour_K"], dtype=float).copy()
    K[0] *= w / COLOUR_SIZE[0]
    K[1] *= h / COLOUR_SIZE[1]
    rvec = cv2.Rodrigues(np.asarray(cal["R_colour_depth"], dtype=float))[0]
    px = cv2.projectPoints(pts.astype(np.float64), rvec, np.asarray(cal["t_colour_depth"], dtype=float), K, np.asarray(cal["colour_dist"], dtype=float))[0].reshape(-1, 2)
    u, v = np.round(px[:, 0]).astype(int), np.round(px[:, 1]).astype(int)
    inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    hsv = np.zeros((len(pts), 3), np.uint8)
    hsv[inside] = cv2.cvtColor(colour_img, cv2.COLOR_BGR2HSV)[v[inside], u[inside]]
    return hsv, inside


def find_in_air(ctx, colour: str = "blue", min_height: float = 0.045, near=None, radius: float = 0.12) -> dict | None:
    """Something of one colour held above the mat: the middle of what the depth camera sees of it,
    in the base frame, and how many depth points that is. None if there is no such thing.

    `min_height` keeps out things of that colour lying on the mat; `near` (x, y, z) with `radius`
    looks only around one spot. What the camera sees is the faces turned to it, so the middle
    reported sits a few millimetres on the camera's side of the thing's true centre, the same
    way for either arm holding it: compare such readings with each other, not with a ruler.
    """
    if colour not in COLOURS:
        raise RobotError(f"no colour range for {colour!r}; known: {sorted(COLOURS)}")
    T = load(ctx.root)
    table_z = float(ctx.arm.meta().get("table_z", 0.0))
    img = ctx.frame("across", settle=0.2)
    depth, cal = grab(ctx.root, frames=5)
    pts = points(depth, cal["depth_K"])
    hsv, inside = colour_of_points(pts, cal, img)
    lo, hi = COLOURS[colour]
    match = inside & np.all(hsv >= lo, axis=1) & np.all(hsv <= hi, axis=1)
    base = to_base(pts[match], T)
    base = base[base[:, 2] - table_z > min_height]
    if near is not None:
        base = base[np.linalg.norm(base - np.asarray(near, dtype=float), axis=1) < radius]
    if len(base) < 30:
        return None
    centre = np.median(base, axis=0)
    for _ in range(4):  # the thing itself, without stray points of the same colour elsewhere
        close = base[np.linalg.norm(base - centre, axis=1) < 0.035]
        if len(close) < 30:
            return None
        centre = np.median(close, axis=0)
    return {"pos": centre, "points": int(len(close))}


def load(root: Path) -> np.ndarray:
    path = root / "robot_config" / "camera_depth.json"
    if not path.exists():
        raise RobotError("the depth camera is not calibrated (robot_config/camera_depth.json is missing): run tools/calibrate_depth.py")
    return np.array(json.loads(path.read_text())["T_base_depth"])


def to_base(pts: np.ndarray, T: np.ndarray) -> np.ndarray:
    return pts @ T[:3, :3].T + T[:3, 3]


def height_map(pts_base: np.ndarray, table_z: float, top: float = 0.20) -> np.ndarray:
    """Height above the mat on a grid over AREA, metres; nan where the sensor has no point.
    Row 0 is the smallest y, column 0 the smallest x. A cell holds its highest point, up to `top`:
    above that is the arm or a hand, not something lying on the mat."""
    (x0, x1), (y0, y1) = AREA
    w, h = round((x1 - x0) / CELL), round((y1 - y0) / CELL)
    col = np.floor((pts_base[:, 0] - x0) / CELL).astype(int)
    row = np.floor((pts_base[:, 1] - y0) / CELL).astype(int)
    z = pts_base[:, 2] - table_z
    keep = (col >= 0) & (col < w) & (row >= 0) & (row < h) & (z < top)
    grid = np.full((h, w), -np.inf)
    np.maximum.at(grid, (row[keep], col[keep]), z[keep])
    grid[np.isinf(grid)] = np.nan
    return grid


def cell_xy(row: float, col: float) -> tuple[float, float]:
    return AREA[0][0] + (col + 0.5) * CELL, AREA[1][0] + (row + 0.5) * CELL


def bumps(grid: np.ndarray, min_height: float = 0.008, min_cells: int = 12) -> list[dict]:
    """Things standing on the mat: connected patches of the height map above `min_height`.
    Each: x, y of the middle of its top, height_mm of that top, size_mm (long side, short side) of
    all that is seen of it, and the patch's cells as a mask."""
    high = np.where(np.isfinite(grid), grid, 0.0) > min_height
    opened = cv2.morphologyEx(high.astype(np.uint8), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8)) & high  # drops single cells: noise
    count, labels = cv2.connectedComponents(opened.astype(np.uint8), connectivity=8)
    out = []
    for i in range(1, count):
        mask = labels == i
        if mask.sum() < min_cells:
            continue
        heights = grid[mask]
        peak = float(np.percentile(heights, 95))
        top = mask & (grid > 0.75 * peak)  # the top face, without the side the sensor also sees
        rows, cols = np.nonzero(top)
        x, y = cell_xy(float(rows.mean()), float(cols.mean()))
        (_, (a, b), _) = cv2.minAreaRect(np.column_stack(np.nonzero(mask)[::-1]).astype(np.float32))
        out.append({"x": round(x, 3), "y": round(y, 3), "height_mm": round(float(np.median(grid[top])) * 1000),
                    "size_mm": [round((max(a, b) + 1) * CELL * 1000), round((min(a, b) + 1) * CELL * 1000)], "mask": mask})  # fmt: skip
    return sorted(out, key=lambda o: -o["height_mm"])


def height_at(grid: np.ndarray, x: float, y: float, radius: float = 0.012) -> float | None:
    """The height of what stands at (x, y): the upper part of the readings within `radius`. None if the sensor has none there."""
    (x0, _), (y0, _) = AREA
    rows, cols = np.mgrid[0 : grid.shape[0], 0 : grid.shape[1]]
    near = np.hypot(x0 + (cols + 0.5) * CELL - x, y0 + (rows + 0.5) * CELL - y) <= radius
    values = grid[near & np.isfinite(grid)]
    return float(np.percentile(values, 80)) if len(values) >= 3 else None


def picture(grid: np.ndarray, marks: list[dict] | None = None, scale: int = 4, top: float = 0.08) -> np.ndarray:
    """The height map as the robot sees the table from above: the base at the bottom, +x to the
    right, colour by height (dark blue the mat, red `top` metres and more), grey where the sensor sees nothing."""
    shade = np.clip(np.nan_to_num(grid, nan=0.0) / top, 0, 1)
    img = cv2.applyColorMap((shade * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[~np.isfinite(grid)] = (70, 70, 70)
    img = cv2.resize(img[::-1], None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)  # far side up
    h = img.shape[0]
    (x0, x1), (y0, y1) = AREA
    for gx in np.arange(math.ceil(x0 * 10) / 10, x1, 0.1):  # a 10 cm grid, labelled in metres
        u = int((gx - x0) / CELL * scale)
        cv2.line(img, (u, 0), (u, h), (255, 255, 255), 1)
        cv2.putText(img, f"x{gx:+.1f}", (u + 3, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
    for gy in np.arange(math.ceil(y0 * 10) / 10, y1, 0.1):
        v = h - int((gy - y0) / CELL * scale)
        cv2.line(img, (0, v), (img.shape[1], v), (255, 255, 255), 1)
        cv2.putText(img, f"y{gy:.1f}", (3, v - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
    for m in marks or []:
        u, v = int((m["x"] - x0) / CELL * scale), h - int((m["y"] - y0) / CELL * scale)
        cv2.circle(img, (u, v), 5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img, m["label"], (u + 8, v + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, m["label"], (u + 8, v + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return img
