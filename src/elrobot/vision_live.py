"""Live vision for the web UI: what a camera's newest frame shows, worked out when asked.

`GET /api/camera/{name}/vision` returns layers of shapes in that frame's pixels, for the camera
panel to draw over the live picture:

  objects   whatever stands out from the table: outline (the segmentation), box, centre, colour
  tags      AprilTags (25h9) in view: the mat's corner tags, tags on objects
  frame     top camera only: the robot's base frame on the mat (axes, reach limits), from the
            saved calibration: it shows where the tools believe the robot stands
  arm       top camera only: the arm as the kinematics place it, joint to joint. Where this line
            leaves the real arm in the picture, calibration or joint zero is off

Nothing here commands the robot or feeds the tools: it is for people to see what the vision
sees. It runs only while a browser asks (a few times a second) and is cached per frame.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

from . import vision

_cache: dict[str, tuple[int, dict]] = {}
_COLOURS = [("red", 0), ("orange", 15), ("yellow", 30), ("green", 60), ("cyan", 90), ("blue", 112), ("purple", 140), ("pink", 165), ("red", 180)]


def _colour(bgr) -> str:
    h, s, v = (int(c) for c in cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0, 0])
    if v < 55:
        return "black"
    if s < 45:
        return "white" if v > 175 else "grey"
    return min(_COLOURS, key=lambda c: abs(c[1] - h))[0]


class _TopCalibration:
    """Base-frame points to pixels of the top camera, from robot_config/camera_top.json."""

    def __init__(self, path: Path, table_z: float):
        cal = json.loads(path.read_text())
        self.P = np.array(cal["P_px_mat"])
        bx, by, yaw = cal["base_in_mat"]
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        self.R, self.b, self.table_z = np.array([[c, -s], [s, c]]), np.array([bx, by]), table_z
        self.size = cal.get("image_size")
        self.H_px_mat = np.linalg.inv(np.array(cal["H_mat_px"]))

    def pixel(self, p) -> list[float]:
        x = self.P @ np.array([*(self.R @ np.asarray(p[:2], dtype=float) + self.b), p[2] - self.table_z, 1.0])
        return [round(float(x[0] / x[2]), 1), round(float(x[1] / x[2]), 1)]

    def mat_quad(self, w: float = 0.585, h: float = 0.315, inset: float = 0.012) -> np.ndarray:
        """The mat between its corner tags, in pixels."""
        corners = np.array([[[inset, inset]], [[w - inset, inset]], [[w - inset, h - inset]], [[inset, h - inset]]], dtype=np.float64)
        return cv2.perspectiveTransform(corners, self.H_px_mat).reshape(4, 2)


def _top_calibration(hub, shape) -> _TopCalibration | None:
    root = hub.cfg.workspace_dir / "robot_config"
    path = root / ("camera_top.sim.json" if hub.cfg.sim else "camera_top.json")
    try:
        cal = _TopCalibration(path, hub.cfg.table_z)
    except (OSError, KeyError, TypeError, ValueError):
        return None
    if cal.size and list(cal.size) != [shape[1], shape[0]]:
        return None  # calibrated at another frame size: its pixels would be wrong here
    return cal


def _objects(img: np.ndarray, camera: str, cal: _TopCalibration | None, area=None, arm=()) -> list[dict]:
    h, w = img.shape[:2]
    ignore = None
    if camera == "wrist":  # the gripper's own black jaws, low in the picture
        ignore = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) < 55
        ignore[: h // 2] = False
        ignore = cv2.dilate(ignore.astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
    elif cal is not None:  # only the work surface: the table edge and the room are not objects
        on_mat = np.zeros((h, w), np.uint8)
        cv2.fillPoly(on_mat, [area.astype(np.int32)], 1)
        ignore = on_mat == 0
        for line in arm:  # nor is the arm itself, drawn fat along its joints
            cv2.polylines(on_mat, [np.array(line["points"], np.int32)], False, 0, max(18, w // 22), cv2.LINE_AA)
        ignore = on_mat == 0
    out = []
    for b in vision.segment_objects(img, ignore=ignore, min_area=max(180.0, 0.0006 * w * h)):
        if b.area > 0.2 * w * h or len(out) >= 14:
            continue
        mask = np.zeros((h, w), np.uint8)
        cv2.fillPoly(mask, [np.array(b.polygon, np.int32)], 255)
        out.append({"label": _colour(cv2.mean(img, mask=mask)[:3]), "polygon": b.polygon, "box": list(b.box),
                    "point": [round(b.centroid[0], 1), round(b.centroid[1], 1)]})  # fmt: skip
    return out


def _tags(img: np.ndarray) -> list[dict]:
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin, params.adaptiveThreshWinSizeMax, params.adaptiveThreshWinSizeStep = 3, 33, 6
    params.minMarkerPerimeterRate = 0.008
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9), params)
    scale = 2.0
    gray = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    corners, ids, _ = detector.detectMarkers(gray)
    return [{"label": f"tag {int(i)}", "polygon": (c[0] / scale).round(1).tolist()} for i, c in zip(np.ravel(ids) if ids is not None else [], corners)]


def _frame(cal: _TopCalibration) -> list[dict]:
    z = cal.table_z

    def arc(r):
        return [cal.pixel([r * math.sin(a), 0.04 + r * math.cos(a), z]) for a in np.radians(np.arange(-90, 91, 5))]

    return [
        {"label": "x 10 cm", "color": "x", "arrow": True, "points": [cal.pixel([0, 0, z]), cal.pixel([0.10, 0, z])]},
        {"label": "y 10 cm", "color": "y", "arrow": True, "points": [cal.pixel([0, 0, z]), cal.pixel([0, 0.10, z])]},
        {"label": "reach 36 cm", "color": "reach", "points": arc(0.36)},
        {"label": "", "color": "reach", "points": arc(0.09)},
    ]


def _arm(hub, cal: _TopCalibration) -> list[dict]:
    robot = hub.robot
    if not robot.ready:
        return []
    g = robot.kin.geometry(robot.q())
    return [
        {"label": "arm (kinematics)", "color": "arm", "points": [cal.pixel(p) for p in [*g.joints, g.wrist, g.tcp]], "joints": True},
        {"label": "", "color": "arm", "points": [cal.pixel(g.tips[0]), cal.pixel(g.tcp), cal.pixel(g.tips[1])]},
    ]


def analyse(hub, name: str) -> dict | None:
    """The layers for camera `name`'s newest frame, or None if it has none."""
    fr = hub.link.frames.get(name)
    if fr is None:
        return None
    cached = _cache.get(name)
    if cached and cached[0] == fr.seq:
        return cached[1]
    t0 = time.monotonic()
    img = vision.decode(fr.jpeg)
    cal = _top_calibration(hub, img.shape) if name == "top" else None
    arm = _arm(hub, cal) if cal is not None else []
    area = None
    if cal is not None:  # the real mat between its tags; the simulated table has none, so the reachable area
        z = cal.table_z
        area = np.array([cal.pixel([x, y, z]) for x, y in ((-0.26, 0.02), (0.26, 0.02), (0.26, 0.44), (-0.26, 0.44))]) if hub.cfg.sim else cal.mat_quad()
    layers = {"objects": _objects(img, name, cal, area, arm), "tags": _tags(img)}
    if cal is not None:
        layers["frame"] = _frame(cal)
        layers["arm"] = arm
    out = {"camera": name, "width": img.shape[1], "height": img.shape[0], "seq": fr.seq, "ms": round((time.monotonic() - t0) * 1000), "layers": layers}
    _cache[name] = (fr.seq, out)
    return out
