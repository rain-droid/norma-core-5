"""A table with a few blocks for `--sim`: gives the simulated arm cameras and things to grasp.

Deliberately crude. It exists so the whole see -> decide -> act loop can be exercised with no
hardware: the wrist and top cameras render the blocks, the jaws stall on a block that is
between them, a held block follows the tool, and a released one lands on whatever is below.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import cv2
import numpy as np

from .kinematics import GRIPPER_MAX_WIDTH
from .link import Frame, SimLink
from .robot import Robot
from .vision import CameraModel, encode

TABLE_BGR = (150, 185, 210)
ROOM_BGR = (70, 66, 62)
TABLE_X = (-0.45, 0.45)
TABLE_Y = (-0.10, 0.60)


@dataclass
class Block:
    name: str
    xy: np.ndarray
    size: tuple[float, float]  # extent along x and y, metres
    height: float
    bgr: tuple[int, int, int]
    z: float = 0.0  # height of its underside above the table

    @property
    def grip_width(self) -> float:
        return min(self.size)

    def corners(self, table_z: float) -> np.ndarray:
        hx, hy = self.size[0] / 2, self.size[1] / 2
        base = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
        return np.array([[self.xy[0] + dx, self.xy[1] + dy, table_z + z] for z in (self.z, self.z + self.height) for dx, dy in base])


class SimWorld:
    def __init__(self, robot: Robot, wrist_cam: CameraModel):
        self.robot = robot
        self.wrist_cam = wrist_cam
        self.table_z = robot.cfg.table_z
        self.top_cam = CameraModel(width=640, height=480, fx=480.0, fy=480.0, cx=320.0, cy=240.0, t_tool_cam=[0.0, 0.0, 0.0])
        # Fixed camera placed like the real one: across the table from the robot, 42 cm up,
        # looking back and down at 45 degrees. Columns: image right, image down, view direction.
        s = math.sqrt(0.5)
        self.top_pose = np.array([[-1.0, 0, 0, 0.0], [0, s, -s, 0.62], [0, -s, -s, self.table_z + 0.42], [0, 0, 0, 1]])
        self.blocks = [
            Block("red cube", np.array([0.07, 0.20]), (0.025, 0.025), 0.025, (60, 60, 220)),
            Block("blue cube", np.array([-0.08, 0.22]), (0.025, 0.025), 0.025, (210, 120, 40)),
            Block("green block", np.array([0.0, 0.28]), (0.045, 0.022), 0.022, (80, 180, 70)),
        ]
        self.held: Block | None = None
        self._held_dz = 0.0
        self._seq = 0

    def attach(self, link: SimLink) -> None:
        link.frame_provider = self.frames
        link.gripper_block = self.gripper_block

    # ---- physics

    def _between_jaws(self) -> Block | None:
        tcp = self.robot.tcp()[:3, 3]
        up = tcp[2] - self.table_z
        for b in self.blocks:
            if np.linalg.norm(tcp[:2] - b.xy) < 0.014 and b.z - 0.005 < up < b.z + b.height + 0.012:
                return b
        return None

    def gripper_block(self, tick: float) -> float | None:
        b = self.held or self._between_jaws()
        if b is None:
            return None
        return float(self.robot._gripper_tick(b.grip_width / GRIPPER_MAX_WIDTH))

    def _step(self) -> None:
        if not self.robot.ready:
            return
        width = self.robot.gripper_open() * GRIPPER_MAX_WIDTH
        tcp = self.robot.tcp()[:3, 3]
        if self.held is None:
            b = self._between_jaws()
            if b is not None and width <= b.grip_width + 0.0015:
                self.held, self._held_dz = b, b.z - (tcp[2] - self.table_z)
        elif width > self.held.grip_width + 0.004:
            b, self.held = self.held, None
            under = [o.z + o.height for o in self.blocks if o is not b and np.all(np.abs(o.xy - b.xy) < (np.array(o.size) + b.size) / 2)]
            b.z = max(under, default=0.0)
        if self.held is not None:
            self.held.xy = tcp[:2].copy()
            self.held.z = max(0.0, tcp[2] - self.table_z + self._held_dz)

    # ---- rendering

    def _render(self, cam: CameraModel, T_base_tool: np.ndarray) -> np.ndarray:
        T = T_base_tool @ cam.T_tool_cam()
        u, v = np.meshgrid(np.arange(cam.width), np.arange(cam.height))
        rays = np.stack([(u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, np.ones_like(u, dtype=float)], axis=-1) @ T[:3, :3].T
        with np.errstate(divide="ignore", invalid="ignore"):
            s = (self.table_z - T[2, 3]) / rays[..., 2]
        hit = T[:3, 3] + s[..., None] * rays
        on_table = (rays[..., 2] < -1e-6) & (hit[..., 0] > TABLE_X[0]) & (hit[..., 0] < TABLE_X[1]) & (hit[..., 1] > TABLE_Y[0]) & (hit[..., 1] < TABLE_Y[1])
        img = np.where(on_table[..., None], np.array(TABLE_BGR, np.uint8), np.array(ROOM_BGR, np.uint8)).astype(np.uint8)
        eye = T[:3, 3]
        for b in sorted(self.blocks, key=lambda b: -np.linalg.norm(np.array([*b.xy, b.z]) - eye)):
            px = [cam.project(c, T_base_tool) for c in b.corners(self.table_z)]
            if any(p is None for p in px):
                continue
            pts = np.array(px, dtype=np.float32)
            side = tuple(int(c * 0.6) for c in b.bgr)
            cv2.fillConvexPoly(img, cv2.convexHull(pts).astype(np.int32), side, cv2.LINE_AA)
            cv2.fillConvexPoly(img, pts[4:].astype(np.int32), b.bgr, cv2.LINE_AA)
        return img

    def _draw_arm(self, img: np.ndarray) -> None:
        g = self.robot.kin.geometry(self.robot.q())
        chain = [*g.joints, g.wrist, g.tcp]
        px = [self.top_cam.project(p, self.top_pose) for p in chain]
        if all(p is not None for p in px):
            cv2.polylines(img, [np.array(px, np.int32)], False, (40, 40, 40), 12, cv2.LINE_AA)

    def calibration(self) -> dict:
        """The overhead camera's calibration, in the format workspace/tools/calibrate_top.py writes.
        The mat frame is the base frame dropped onto the table surface."""
        c = self.top_cam
        K = np.array([[c.fx, 0, c.cx], [0, c.fy, c.cy], [0, 0, 1.0]])
        lift = np.eye(4)
        lift[2, 3] = self.table_z
        P = K @ (np.linalg.inv(self.top_pose) @ lift)[:3]
        return {
            "H_mat_px": np.linalg.inv(P[:, [0, 1, 3]]).tolist(),
            "P_px_mat": P.tolist(),
            "camera_mat": (self.top_pose[:3, 3] - [0.0, 0.0, self.table_z]).tolist(),
            "base_in_mat": [0.0, 0.0, 0.0],
            "image_size": [c.width, c.height],
        }

    def frames(self) -> dict[str, Frame]:
        self._step()
        self._seq += 1
        now = time.monotonic()
        out = {}
        if self.robot.ready:
            wrist = self._render(self.wrist_cam, self.robot.tcp())
            h, w = wrist.shape[:2]
            for x0, x1 in ((0.30, 0.40), (0.60, 0.70)):  # the gripper's own jaws
                cv2.rectangle(wrist, (int(x0 * w), int(0.84 * h)), (int(x1 * w), h), (25, 25, 25), -1)
            out["wrist"] = Frame(encode(wrist), w, h, now, self._seq)
            top = self._render(self.top_cam, self.top_pose)
            self._draw_arm(top)
            out["top"] = Frame(encode(top), top.shape[1], top.shape[0], now, self._seq)
        return out
