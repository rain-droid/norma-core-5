"""Camera frames, overlays, the wrist-camera model, and simple table-top segmentation."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

import cv2
import numpy as np


def decode(jpeg: bytes) -> np.ndarray:
    """JPEG bytes -> BGR image."""
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("could not decode camera frame")
    return img


def encode(img: np.ndarray, quality: int = 85) -> bytes:
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError("could not encode image")
    return buf.tobytes()


def draw_grid(img: np.ndarray, step: int = 100) -> np.ndarray:
    """Overlay labelled pixel gridlines so a vision model can name a pixel accurately."""
    out = img.copy()
    h, w = out.shape[:2]
    overlay = out.copy()
    for x in range(step, w, step):
        cv2.line(overlay, (x, 0), (x, h), (255, 255, 255), 1, cv2.LINE_AA)
    for y in range(step, h, step):
        cv2.line(overlay, (0, y), (w, y), (255, 255, 255), 1, cv2.LINE_AA)
    out = cv2.addWeighted(overlay, 0.55, out, 0.45, 0)
    font = cv2.FONT_HERSHEY_SIMPLEX
    for x in range(step, w, step):
        _label(out, str(x), (x + 3, 13), font)
    for y in range(step, h, step):
        _label(out, str(y), (3, y - 4), font)
    return out


def _label(img, text, org, font=cv2.FONT_HERSHEY_SIMPLEX, scale=0.42, color=(255, 255, 255)):
    cv2.putText(img, text, org, font, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, font, scale, color, 1, cv2.LINE_AA)


def draw_marks(img: np.ndarray, marks: list[dict]) -> np.ndarray:
    """Draw detections/points: each mark may have box [x1,y1,x2,y2], point [u,v], polygon, label."""
    out = img.copy()
    for i, m in enumerate(marks):
        color = m.get("bgr", ((80, 220, 255), (120, 255, 120), (255, 160, 80), (255, 120, 220))[i % 4])
        if m.get("polygon"):
            cv2.polylines(out, [np.array(m["polygon"], np.int32)], True, color, 2, cv2.LINE_AA)
        if m.get("box"):
            x1, y1, x2, y2 = [int(v) for v in m["box"]]
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
        if m.get("point"):
            u, v = [int(round(c)) for c in m["point"]]
            cv2.drawMarker(out, (u, v), color, cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)
        if m.get("label"):
            anchor = m.get("box", None)
            org = (int(anchor[0]), max(14, int(anchor[1]) - 6)) if anchor else (int(m["point"][0]) + 10, int(m["point"][1]) - 8)
            _label(out, str(m["label"]), org, scale=0.5, color=color)
    return out


# --------------------------------------------------------------------------- camera model


@dataclass
class CameraModel:
    """Pinhole camera with radial distortion, rigidly attached to the tool frame.

    `R_tool_cam`/`t_tool_cam` place the camera in the tool (TCP) frame: a point p_cam in camera
    coordinates (x right, y down, z forward) is at R_tool_cam @ p_cam + t_tool_cam in the tool
    frame. Defaults are a rough guess for the stock gripper mount; `scripts/calibrate_camera.py`
    replaces them with measured values.
    """

    width: int = 853
    height: int = 480
    fx: float = 330.0
    fy: float = 330.0
    cx: float = 426.5
    cy: float = 240.0
    k1: float = 0.0
    k2: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    # camera looks along the tool's approach axis (z), image right = tool x, image down = tool y
    rvec_tool_cam: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    t_tool_cam: list[float] = field(default_factory=lambda: [0.0, -0.045, -0.10])
    calibrated: bool = False
    rms_px: float | None = None

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])

    @property
    def dist(self) -> np.ndarray:
        return np.array([self.k1, self.k2, self.p1, self.p2, 0.0])

    @property
    def R_tool_cam(self) -> np.ndarray:
        return cv2.Rodrigues(np.asarray(self.rvec_tool_cam, dtype=float))[0]

    def T_tool_cam(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = self.R_tool_cam
        T[:3, 3] = self.t_tool_cam
        return T

    def scaled(self, width: int, height: int) -> "CameraModel":
        """The same camera at another image size (the Station can be set to other resolutions)."""
        if (width, height) == (self.width, self.height):
            return self
        sx, sy = width / self.width, height / self.height
        d = asdict(self)
        d.update(width=width, height=height, fx=self.fx * sx, fy=self.fy * sy, cx=self.cx * sx, cy=self.cy * sy)
        return CameraModel(**d)

    # --- projection

    def ray_cam(self, u: float, v: float) -> np.ndarray:
        """Unit ray in camera coordinates through pixel (u, v), with distortion removed."""
        pt = cv2.undistortPoints(np.array([[[u, v]]], dtype=np.float64), self.K, self.dist)[0, 0]
        r = np.array([pt[0], pt[1], 1.0])
        return r / np.linalg.norm(r)

    def pixel_to_plane(self, u: float, v: float, T_base_tool: np.ndarray, plane_z: float) -> np.ndarray | None:
        """Where the ray through pixel (u, v) meets the horizontal plane z = plane_z (base frame)."""
        T = T_base_tool @ self.T_tool_cam()
        d = T[:3, :3] @ self.ray_cam(u, v)
        o = T[:3, 3]
        if d[2] > -1e-3:  # ray does not go down towards the table
            return None
        s = (plane_z - o[2]) / d[2]
        return o + s * d

    def project(self, p_base, T_base_tool: np.ndarray) -> tuple[float, float] | None:
        """Pixel where a base-frame point appears, or None if it is behind the camera."""
        T = np.linalg.inv(T_base_tool @ self.T_tool_cam())
        pc = T[:3, :3] @ np.asarray(p_base, dtype=float) + T[:3, 3]
        if pc[2] <= 1e-4:
            return None
        uv = cv2.projectPoints(pc.reshape(1, 1, 3), np.zeros(3), np.zeros(3), self.K, self.dist)[0][0, 0]
        return float(uv[0]), float(uv[1])

    # --- persistence

    @classmethod
    def load(cls, path: Path) -> "CameraModel":
        if path.exists():
            return cls(**json.loads(path.read_text()))
        return cls()

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(self), indent=2))


# --------------------------------------------------------------------------- table-top segmentation


@dataclass
class Blob:
    centroid: tuple[float, float]
    box: tuple[int, int, int, int]  # x1, y1, x2, y2
    area: float
    angle_deg: float  # orientation of the long axis in the image, 0 = along +u, CCW positive in image coords
    length_px: float
    width_px: float
    polygon: list[list[int]]

    def as_mark(self, label: str = "") -> dict:
        return {"label": label, "box": list(self.box), "point": [round(self.centroid[0], 1), round(self.centroid[1], 1)], "polygon": self.polygon}


def _blobs_from_mask(mask: np.ndarray, min_area: float) -> list[Blob]:
    out = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        m = cv2.moments(c)
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        (_, _), (w, h), ang = cv2.minAreaRect(c)
        if w < h:
            w, h, ang = h, w, ang + 90.0
        x, y, bw, bh = cv2.boundingRect(c)
        poly = cv2.approxPolyDP(c, 0.01 * cv2.arcLength(c, True), True).reshape(-1, 2).tolist()
        out.append(Blob((cx, cy), (x, y, x + bw, y + bh), float(area), float(ang), float(w), float(h), poly))
    return sorted(out, key=lambda b: -b.area)


def segment_color(img: np.ndarray, hsv_lo, hsv_hi, min_area: float = 150.0) -> list[Blob]:
    """Blobs within an HSV range, largest first."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_lo, np.uint8), np.array(hsv_hi, np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    return _blobs_from_mask(mask, min_area)


def segment_objects(img: np.ndarray, ignore: np.ndarray | None = None, min_area: float = 250.0, thresh: float = 22.0) -> list[Blob]:
    """Things that stand out from the table, without knowing what they are.

    The table is modelled as the dominant colour of the image (median in Lab); anything far
    from it in colour is an object. `ignore` is a mask of pixels to skip (the gripper's own
    jaws at the bottom of the wrist view).
    """
    lab = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2LAB).astype(np.float32)
    valid = np.ones(img.shape[:2], bool) if ignore is None else ~ignore.astype(bool)
    med = np.median(lab[valid], axis=0)
    # weight lightness less than colour so shadows and the wood grain do not count as objects
    d = np.sqrt(0.35 * (lab[..., 0] - med[0]) ** 2 + (lab[..., 1] - med[1]) ** 2 + (lab[..., 2] - med[2]) ** 2)
    mask = ((d > thresh) & valid).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    return _blobs_from_mask(mask, min_area)


def nearest_blob(blobs: list[Blob], u: float, v: float, max_dist: float = 1e9) -> Blob | None:
    """The blob containing (u, v), else the one whose centroid is closest."""
    best, best_d = None, max_dist
    for b in blobs:
        x1, y1, x2, y2 = b.box
        if x1 <= u <= x2 and y1 <= v <= y2 and cv2.pointPolygonTest(np.array(b.polygon, np.int32), (float(u), float(v)), False) >= 0:
            return b
        d = math.hypot(b.centroid[0] - u, b.centroid[1] - v)
        if d < best_d:
            best, best_d = b, d
    return best
