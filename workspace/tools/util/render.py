"""Draw the robot's body from its URDF and meshes, with no camera and no scene.

Used for the on-demand posture view (read_proprioception) and the joint manual
(robot_config/build_visuals.py). Orthographic views, numpy only: every mesh triangle is sampled
with points, the nearest point per pixel wins. These pictures show the model at given joint
angles. They are not camera images: no table objects, no held object, and pixels in them mean
nothing to pixel_to_table.
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np

ARM_JOINTS = [f"rev_motor_{i:02d}" for i in range(1, 8)]
GRIPPER_JOINT = "rev_motor_08"

# name -> (image right, image up, towards the viewer), all in the base frame
VIEWS = {
    "from behind": (np.array([1.0, 0, 0]), np.array([0, 0, 1.0]), np.array([0, -1.0, 0])),
    "from the right": (np.array([0, 1.0, 0]), np.array([0, 0, 1.0]), np.array([1.0, 0, 0])),
    "from above": (np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])),
}
AXIS_NAMES = {"from behind": ("x right", "z up"), "from the right": ("y forward", "z up"), "from above": ("x right", "y forward")}

BACKGROUND = (250, 250, 250)
SERVO = np.array([70.0, 70, 70])
BRACKET = np.array([205.0, 205, 200])
HIGHLIGHT = np.array([40.0, 140, 255])  # orange, BGR
CYAN = (230, 200, 0)


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def _origin(el) -> np.ndarray:
    T = np.eye(4)
    if el is not None:
        T[:3, 3] = [float(v) for v in el.get("xyz", "0 0 0").split()]
        T[:3, :3] = _rpy(*[float(v) for v in el.get("rpy", "0 0 0").split()])
    return T


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis / np.linalg.norm(axis)
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s], [y * x * C + z * s, c + y * y * C, y * z * C - x * s], [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def _load_stl(path: Path) -> np.ndarray:
    """Binary STL -> (N, 3, 3) triangle vertices."""
    record = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    raw = path.read_bytes()
    count = int(np.frombuffer(raw, "<u4", 1, 80)[0])
    return np.frombuffer(raw, record, count, 84)["v"].astype(np.float64)


class Model:
    def __init__(self, urdf: Path):
        urdf = Path(urdf)
        root = ET.parse(urdf).getroot()
        self.meshes: dict[str, np.ndarray] = {}
        for link in root.findall("link"):
            visual = link.find("visual")
            if visual is None:
                continue
            mesh = visual.find("geometry/mesh")
            scale = np.array([float(v) for v in mesh.get("scale", "1 1 1").split()])
            tris = _load_stl(urdf.parent / mesh.get("filename")) * scale
            T = _origin(visual.find("origin"))
            self.meshes[link.get("name")] = tris @ T[:3, :3].T + T[:3, 3]
        self.joints = []
        children = set()
        for j in root.findall("joint"):
            axis = j.find("axis")
            mimic = j.find("mimic")
            limit = j.find("limit")
            self.joints.append({
                "name": j.get("name"),
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "origin": _origin(j.find("origin")),
                "axis": np.array([float(v) for v in axis.get("xyz").split()]) if axis is not None else None,
                "mimic": (mimic.get("joint"), float(mimic.get("multiplier", 1)), float(mimic.get("offset", 0))) if mimic is not None else None,
                "lower": float(limit.get("lower")) if limit is not None else 0.0,
                "upper": float(limit.get("upper")) if limit is not None else 0.0,
            })  # fmt: skip
            children.add(j.find("child").get("link"))
        self.root = next(link.get("name") for link in root.findall("link") if link.get("name") not in children)
        self.gripper_upper = next(j["upper"] for j in self.joints if j["name"] == GRIPPER_JOINT)

    def values(self, q, gripper_open: float = 1.0) -> dict[str, float]:
        """Joint values from seven arm angles (rad) and a jaw opening (0..1)."""
        return {**dict(zip(ARM_JOINTS, [float(v) for v in q])), GRIPPER_JOINT: float(gripper_open) * self.gripper_upper}

    def poses(self, values: dict[str, float]) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
        """Base-frame pose of every link, and of every joint frame before its own motion."""
        links = {self.root: np.eye(4)}
        frames = {}
        pending = list(self.joints)
        while pending:
            for j in pending:
                if j["parent"] in links:
                    break
            else:
                raise ValueError("URDF joints do not form a tree")
            pending.remove(j)
            T = links[j["parent"]] @ j["origin"]
            frames[j["name"]] = T
            value = values.get(j["name"], 0.0)
            if j["mimic"]:
                value = values.get(j["mimic"][0], 0.0) * j["mimic"][1] + j["mimic"][2]
            M = np.eye(4)
            if j["type"] == "revolute":
                M[:3, :3] = _axis_rotation(j["axis"], value)
            elif j["type"] == "prismatic":
                M[:3, 3] = j["axis"] / np.linalg.norm(j["axis"]) * value
            links[j["child"]] = T @ M
        return links, frames

    def triangles(self, values: dict[str, float], highlight: str | None = None) -> tuple[np.ndarray, np.ndarray]:
        """All mesh triangles in the base frame, with a base colour each."""
        links, _ = self.poses(values)
        tris, colors = [], []
        for name, mesh in self.meshes.items():
            T = links[name]
            tris.append(mesh @ T[:3, :3].T + T[:3, 3])
            color = HIGHLIGHT if name == highlight else (SERVO if name.startswith("ST3215") else BRACKET)
            colors.append(np.tile(color, (len(mesh), 1)))
        return np.concatenate(tris), np.concatenate(colors)


def bounds_of(points: np.ndarray, margin: float = 0.03) -> tuple[np.ndarray, float]:
    """Centre and half-size of a cube that contains the points."""
    lo, hi = points.reshape(-1, 3).min(axis=0), points.reshape(-1, 3).max(axis=0)
    return (lo + hi) / 2, float((hi - lo).max()) / 2 + margin


class Panel:
    """One orthographic view onto a square image."""

    def __init__(self, view: str, centre: np.ndarray, half: float, size: int = 420):
        self.view = view
        self.right, self.up, self.toward = VIEWS[view]
        self.centre, self.half, self.size = np.asarray(centre, dtype=float), half, size
        self.scale = size / (2 * half)
        self.img = np.full((size, size, 3), BACKGROUND, np.uint8)

    def px(self, points: np.ndarray) -> np.ndarray:
        d = np.asarray(points, dtype=float) - self.centre
        return np.stack([(d @ self.right) * self.scale + self.size / 2, self.size / 2 - (d @ self.up) * self.scale], axis=-1)

    def grid(self, step: float = 0.1) -> None:
        """Faint lines every `step` metres, labelled in metres, and the table as a dark line."""
        cr, cu = float(self.centre @ self.right), float(self.centre @ self.up)
        for k in range(math.ceil((cr - self.half) / step), math.floor((cr + self.half) / step) + 1):
            x = int(round((k * step - cr) * self.scale + self.size / 2))
            cv2.line(self.img, (x, 0), (x, self.size), (228, 228, 228), 1)
            cv2.putText(self.img, f"{k * step:+.1f}", (x + 2, self.size - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (150, 150, 150), 1, cv2.LINE_AA)
        for k in range(math.ceil((cu - self.half) / step), math.floor((cu + self.half) / step) + 1):
            y = int(round(self.size / 2 - (k * step - cu) * self.scale))
            is_table = k == 0 and self.view != "from above"
            cv2.line(self.img, (0, y), (self.size, y), (120, 120, 120) if is_table else (228, 228, 228), 2 if is_table else 1)
            cv2.putText(self.img, f"{k * step:+.1f}", (3, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (150, 150, 150), 1, cv2.LINE_AA)

    def solids(self, tris: np.ndarray, colors: np.ndarray, density: float = 2.5) -> None:
        normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        area = np.linalg.norm(normals, axis=1)
        keep = area > 0
        tris, colors, normals, area = tris[keep], colors[keep], normals[keep] / area[keep, None], area[keep]
        # flat shading, lit from the viewer's upper left; both faces count, CAD exports mix windings
        light = self.toward + 0.5 * self.up - 0.3 * self.right
        shade = 0.35 + 0.65 * np.abs(normals @ (light / np.linalg.norm(light)))
        n = np.clip(np.ceil(0.5 * area * self.scale**2 * density), 1, 5000).astype(int)
        idx = np.repeat(np.arange(len(tris)), n)
        rng = np.random.default_rng(0)
        a, b = rng.random(len(idx)), rng.random(len(idx))
        flip = a + b > 1
        a[flip], b[flip] = 1 - a[flip], 1 - b[flip]
        pts = tris[idx, 0] + a[:, None] * (tris[idx, 1] - tris[idx, 0]) + b[:, None] * (tris[idx, 2] - tris[idx, 0])
        uv = np.round(self.px(pts)).astype(int)
        ok = (uv[:, 0] >= 0) & (uv[:, 0] < self.size) & (uv[:, 1] >= 0) & (uv[:, 1] < self.size)
        uv, idx, depth = uv[ok], idx[ok], (pts[ok] @ self.toward)
        order = np.argsort(depth)  # far first: the nearest sample is written last
        rgb = np.clip(colors[idx] * shade[idx, None], 0, 255).astype(np.uint8)
        self.img[uv[order, 1], uv[order, 0]] = rgb[order]

    def dot(self, point: np.ndarray, color, label: str = "") -> None:
        c = tuple(int(v) for v in np.round(self.px(point)))
        cv2.circle(self.img, c, 5, (0, 0, 0), -1, cv2.LINE_AA)
        cv2.circle(self.img, c, 3, color, -1, cv2.LINE_AA)
        if label:
            cv2.putText(self.img, label, (c[0] + 8, c[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)

    def axis(self, origin: np.ndarray, direction: np.ndarray, length: float = 0.09) -> None:
        """A joint axis in cyan; seen end-on it becomes a ringed dot."""
        p = tuple(int(v) for v in np.round(self.px(origin)))
        if abs(float(direction @ self.toward)) > 0.9:
            cv2.circle(self.img, p, 9, CYAN, 2, cv2.LINE_AA)
            cv2.circle(self.img, p, 3, CYAN, -1, cv2.LINE_AA)
        else:
            q = tuple(int(v) for v in np.round(self.px(origin + direction * length)))
            cv2.arrowedLine(self.img, p, q, CYAN, 2, cv2.LINE_AA, tipLength=0.2)

    def title(self, text: str) -> np.ndarray:
        """The panel with a caption strip above it."""
        strip = np.full((40, self.size, 3), 255, np.uint8)
        cv2.putText(strip, text, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
        axes = AXIS_NAMES[self.view]
        cv2.putText(strip, f"{self.view}: right = {axes[0]}, up = {axes[1]} (metres)", (6, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (110, 110, 110), 1, cv2.LINE_AA)
        return np.vstack([strip, self.img])


def three_views(model: Model, q, gripper_open: float, tcp: np.ndarray | None = None) -> np.ndarray:
    """The body at the given joint angles from behind, from the right and from above."""
    tris, colors = model.triangles(model.values(q, gripper_open))
    centre, half = np.array([0.0, 0.17, 0.20]), 0.31  # fixed frame, so views compare across calls
    panels = []
    for view in VIEWS:
        panel = Panel(view, centre, half)
        panel.grid()
        panel.solids(tris, colors)
        if tcp is not None:
            panel.dot(tcp, CYAN, "tool point")
        panels.append(panel.title("body " + view))
    return np.hstack(panels)
