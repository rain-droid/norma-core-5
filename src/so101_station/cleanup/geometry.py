"""SO-101 FK/IK and measured planar camera grounding. No ROS runtime required."""
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from ..calibration import DEFAULT_PROFILE, CalibrationError, load_calibration


class CleanupError(ValueError):
    pass


def vector(value, size, label):
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise CleanupError(f"invalid {label}") from exc
    if array.shape != (size,) or not np.isfinite(array).all():
        raise CleanupError(f"{label} must contain {size} finite numbers")
    return array


def transform(xyz, rpy):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    out[:3, 3] = xyz
    return out


class ArmModel:
    def __init__(self, calibration=None):
        self.calibration = calibration or load_calibration()
        self.joints = self.calibration.joints[:5]
        self.lower = np.array([j.lower for j in self.joints]) + 0.015
        self.upper = np.array([j.upper for j in self.joints]) - 0.015
        root = ET.parse(DEFAULT_PROFILE / "model/so101_arm_macro.xacro").getroot()
        self.origins, self.axes = [], []
        for joint in self.joints:
            node = root.find(f".//joint[@name='${{prefix}}{joint.name}']")
            origin = node.find("origin")
            self.origins.append(transform(np.fromstring(origin.get("xyz"), sep=" "), np.fromstring(origin.get("rpy"), sep=" ")))
            self.axes.append(np.fromstring(node.find("axis").get("xyz"), sep=" "))
        # Exact default mount and tool0 transforms from the pinned pgripper Xacro.
        self.mount = transform([0, 0, 0], [-math.pi / 2, 0, -math.pi / 2])
        self.tip = transform([0, 0.104, 0], [math.pi / 2, 0, -math.pi])

    def frames(self, q):
        q = vector(q, 5, "arm angles")
        T = np.eye(4)
        points = []
        for origin, axis, angle in zip(self.origins, self.axes, q):
            T = T @ origin
            points.append(T[:3, 3].copy())
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            T = T @ rotation
        T = T @ self.mount
        points.append(T[:3, 3].copy())
        T = T @ self.tip
        points.append(T[:3, 3].copy())
        return T, np.array(points)

    def fk(self, q):
        return self.frames(q)[0]

    def validate(self, q, floor_z=0.0):
        q = vector(q, 5, "arm angles")
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise CleanupError("arm joint limit exceeded")
        _, points = self.frames(q)
        if points[-1, 2] < floor_z + 0.003 or np.min(points[1:-1, 2]) < floor_z + 0.025:
            raise CleanupError("planned arm/fingertip is too close to the table")
        # Conservative centerline clearance, not mesh/environment collision checking.
        samples = [np.linspace(a, b, 10) for a, b in zip(points[:-1], points[1:]) if np.linalg.norm(a - b) > 0.005]
        for i in range(len(samples)):
            for j in range(i + 3, len(samples)):
                distance = np.linalg.norm(samples[i][:, None] - samples[j][None, :], axis=2).min()
                if distance < 0.025:
                    raise CleanupError("planned arm has insufficient self-clearance")

    def solve(self, xyz, seed, floor_z=0.0):
        target, seed = vector(xyz, 3, "target"), vector(seed, 5, "seed")
        # Five-DOF arm: constrain position and downward approach; free jaw yaw.
        def residual(q):
            T = self.fk(q)
            return np.r_[T[:3, 3] - target, 0.06 * (T[:3, 2] - [0, 0, -1]), 0.0001 * (q - seed)]
        starts = [seed, np.zeros(5), (self.lower + self.upper) / 2]
        best = None
        for initial in starts:
            fit = least_squares(residual, np.clip(initial, self.lower + 1e-6, self.upper - 1e-6),
                                bounds=(self.lower, self.upper), max_nfev=100, ftol=1e-8, xtol=1e-8, gtol=1e-8)
            T = self.fk(fit.x)
            error = float(np.linalg.norm(T[:3, 3] - target))
            angle = math.acos(np.clip(T[:3, 2] @ [0, 0, -1], -1, 1))
            if error <= 0.004 and angle <= math.radians(10):
                try:
                    self.validate(fit.x, floor_z)
                except CleanupError:
                    continue
                cost = np.linalg.norm(fit.x - seed)
                if initial is seed and cost < 0.4:
                    return fit.x
                if best is None or cost < best[0]:
                    best = cost, fit.x
        if best is None:
            raise CleanupError(f"no checked downward grasp pose reaches {target.round(4).tolist()}")
        return best[1]

    def line(self, seed, xyz, floor_z=0.0):
        seed = vector(seed, 5, "seed")
        start, target = self.fk(seed)[:3, 3], vector(xyz, 3, "target")
        count = max(1, math.ceil(np.linalg.norm(target - start) / 0.008))
        if count > 100:
            raise CleanupError("Cartesian segment is too long")
        path, q = [], seed
        for pos in np.linspace(start, target, count + 1)[1:]:
            next_q = self.solve(pos, q, floor_z)
            if np.max(np.abs(next_q - q)) > 0.25:
                raise CleanupError("IK would jump configuration; choose a different approach")
            for sample in np.linspace(q, next_q, 5):
                self.validate(sample, floor_z)
            path.append(next_q.tolist())
            q = next_q
        return path

    def ticks(self, q, gripper_tick):
        return {**{j.motor_id: j.radians_to_ticks(float(v)) for j, v in zip(self.joints, q)}, 6: int(gripper_tick)}


class TableMap:
    def __init__(self, config):
        try:
            self._load(config)
        except CleanupError:
            raise
        except (KeyError, TypeError, ValueError, cv2.error) as exc:
            raise CleanupError(f"invalid/incomplete workspace calibration: {exc}") from exc

    def _load(self, config):
        self.config = config
        if not isinstance(config.get("camera_id"), str) or not config["camera_id"].strip():
            raise CleanupError("workspace requires the exact Station camera_id")
        if config.get("camera_fixed") is not True:
            raise CleanupError("arbitrary tabletop targeting requires a fixed, calibrated camera")
        self.size = tuple(config["image_size"])
        if len(self.size) != 2 or any(type(n) is not int or n <= 0 for n in self.size):
            raise CleanupError("image_size must be [width, height]")
        pixels, world = np.asarray(config["pixel_points"], float), np.asarray(config["table_points_m"], float)
        if pixels.ndim != 2 or pixels.shape[1] != 2 or len(pixels) < 6 or world.shape != pixels.shape:
            raise CleanupError("provide at least six measured pixel/table point pairs")
        if not np.isfinite(pixels).all() or not np.isfinite(world).all():
            raise CleanupError("calibration points must be finite")
        self.hull = cv2.convexHull(world.astype(np.float32))
        if cv2.contourArea(self.hull) < 0.005:
            raise CleanupError("calibration points are collinear or cover too little table")
        self.H, _ = cv2.findHomography(pixels, world, method=0)
        if self.H is None or not np.isfinite(self.H).all() or np.linalg.cond(self.H) > 1e8:
            raise CleanupError("degenerate table calibration")
        predicted = cv2.perspectiveTransform(pixels[None], self.H)[0]
        self.max_error_m = float(np.linalg.norm(predicted - world, axis=1).max())
        if self.max_error_m > 0.005:
            raise CleanupError("table calibration reprojection error exceeds 5 mm")
        self.polygon = np.asarray(config["workspace_polygon_m"], np.float32)
        if self.polygon.ndim != 2 or self.polygon.shape[1] != 2 or len(self.polygon) < 3 or not np.isfinite(self.polygon).all():
            raise CleanupError("invalid workspace polygon")
        if not cv2.isContourConvex(self.polygon):
            raise CleanupError("workspace polygon must be convex")
        for xy in self.polygon:
            if cv2.pointPolygonTest(self.hull, tuple(map(float, xy)), False) < 0:
                raise CleanupError("workspace extends outside measured calibration points")
        self.drop = vector(config["drop_xy_m"], 2, "drop point")
        self.check_xy(self.drop)
        self.floor = float(config.get("table_z_m", 0.0))
        self.grasp_z = float(config.get("grasp_tip_z_m", 0.008))
        self.travel_z = float(config.get("travel_z_m", 0.10))
        self.drop_z = float(config.get("drop_tip_z_m", 0.045))
        if not all(math.isfinite(n) for n in (self.floor, self.grasp_z, self.travel_z, self.drop_z)):
            raise CleanupError("heights must be finite")
        if not self.floor + 0.003 <= self.grasp_z < self.drop_z < self.travel_z <= self.floor + 0.25:
            raise CleanupError("require table < grasp < drop < travel heights, travel <=25 cm")

    def check_xy(self, xy):
        xy = vector(xy, 2, "table point")
        if cv2.pointPolygonTest(self.polygon, tuple(map(float, xy)), False) < 0:
            raise CleanupError("target outside calibrated workspace")

    def project(self, pixel):
        pixel = vector(pixel, 2, "grasp pixel")
        if not (0 <= pixel[0] < self.size[0] and 0 <= pixel[1] < self.size[1]):
            raise CleanupError("pixel outside calibrated image")
        p = self.H @ np.r_[pixel, 1]
        if abs(p[2]) < 1e-8:
            raise CleanupError("pixel maps to infinity")
        xy = p[:2] / p[2]
        self.check_xy(xy)
        if np.linalg.norm(xy - self.drop) < 0.05:
            raise CleanupError("target is already in the drop zone")
        return xy

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text()))
