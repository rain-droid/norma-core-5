"""Both arms in one space: this arm and the second arm (the SO-101), for moving one without hitting the other.

Everything is in this arm's base frame. This arm is the model planning already uses
(util/kinematics.py). The second arm is read from its URDF (robot_config/second_arm.urdf) as a
plain chain of five joints and stood on the table where calibrate_second_arm found it: at the
fitted position and turn, upright (the fit itself is of points in the air and comes out tilted).

Each arm is a handful of capsules (a line with a radius): base, upper arm, forearm, wrist,
gripper, and the wrist camera on its mount, which both arms carry on the same parallel gripper. `between` gives the smallest gap between any capsule of one arm and any of the other; for a
hand-over the two grippers, and a gripper and the other arm's wrist, may be left out.
`path_clear` walks a joint path of one arm in small steps against the other arm standing still
and says where it would come too close. `second_ik` finds joint angles for the second arm that
put its tool point at a place in this arm's frame, so its moves can be checked before they are
made: its own daemon plans without knowing this arm exists.

Things on the table come from the depth camera: tools/scan_obstacles.py keeps its height map
(a grid of 4 mm cells over the mat, each the height of what stands there) in
state/obstacles.npz, with both arms cut out of it. A capsule may not dip into a cell's column.
Around where a move starts and where it ends the cells are left out: an arm may touch what it
goes to and what it comes from. `travel_height` is how high the things along a straight line
stand, for carrying something over them. A map older than two minutes is not used: things get
moved, and a stale map refuses moves for no reason. Scan again before a move that matters.

What this does not know: a thing held in a gripper, the other arm while it is moving, and
anything the depth camera does not see (behind things, off the mat). Only one arm moves at a time.
"""

from __future__ import annotations

import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from util.arm import RobotError
from util.kinematics import Kinematics, _transform, axis_angle_matrix, segment_distance, unit

SECOND_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
SECOND_TIP = "pgripper_base_link"
SECOND_TOOL = np.array([0.0, 0.092, 0.0])  # the tool point in the gripper base frame: 12 mm back from the fingertips, as its daemon has it
PARTS = ("base", "upper arm", "forearm", "wrist", "gripper", "wrist camera")


class Chain:
    """A serial arm read from a URDF: the fixed and revolute joints from its root to a tip link."""

    def __init__(self, urdf: Path, joints: tuple[str, ...], tip: str, tool):
        root = ET.parse(urdf).getroot()
        by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        chain, link = [], tip
        while link in by_child:
            chain.append(by_child[link])
            link = by_child[link].find("parent").get("link")
        chain.reverse()
        self.steps, pending = [], np.eye(4)  # (fixed transform before the joint, its axis) per moving joint
        self.lower, self.upper = [], []
        for j in chain:
            o = j.find("origin")
            pending = pending @ _transform([float(v) for v in o.get("xyz", "0 0 0").split()], [float(v) for v in o.get("rpy", "0 0 0").split()])
            if j.get("type") == "fixed":
                continue
            self.steps.append((pending, unit([float(v) for v in j.find("axis").get("xyz").split()])))
            self.lower.append(float(j.find("limit").get("lower")))
            self.upper.append(float(j.find("limit").get("upper")))
            pending = np.eye(4)
        if len(self.steps) != len(joints):
            raise ValueError(f"expected the joints {joints} between the root and {tip}")
        self.after = pending
        self.tool = np.asarray(tool, dtype=float)
        # the wrist camera on its mount, when the URDF has it: the foot of the mount and the front of the lens, in the tip link's frame
        named, self.camera = {j.get("name"): j for j in root.findall("joint")}, None
        if "camera_mount_joint" in named and "camera_joint" in named:
            fixed = lambda n: _transform([float(v) for v in named[n].find("origin").get("xyz", "0 0 0").split()], [float(v) for v in named[n].find("origin").get("rpy", "0 0 0").split()])  # noqa: E731
            mount = fixed("camera_mount_joint")
            camera = mount @ fixed("camera_joint")
            self.camera = np.array([mount[:3, 3], camera[:3, :3] @ np.array([0.025, 0.0, 0.0]) + camera[:3, 3]])
        self.lower, self.upper = np.array(self.lower), np.array(self.upper)

    def frames(self, q) -> list[np.ndarray]:
        T, out = np.eye(4), []
        for (pre, axis), qi in zip(self.steps, q):
            R = np.eye(4)
            R[:3, :3] = axis_angle_matrix(axis, float(qi))
            T = T @ pre @ R
            out.append(T)
        return out

    def tip(self, q) -> np.ndarray:
        return self.frames(q)[-1] @ self.after

    def tool_point(self, q) -> np.ndarray:
        T = self.tip(q)
        return T[:3, :3] @ self.tool + T[:3, 3]

    def points_along(self, q) -> np.ndarray:
        """The way it points: the gripper base's y axis."""
        return self.tip(q)[:3, 1]


OBSTACLES = "state/obstacles.npz"
OBSTACLES_LAST_S = 120.0
LOW = 0.012  # cells lower than this are the mat and the depth camera's noise on it

_space: dict = {}


def space(root: Path) -> "SharedSpace":
    """The shared space, built once per process and again when the second arm has been fitted anew."""
    stamp = (root / "robot_config" / "second_arm.json").stat().st_mtime
    if _space.get("stamp") != stamp:
        _space.update(stamp=stamp, space=SharedSpace(root))
    _space["space"].use_obstacles(root)
    return _space["space"]


class SharedSpace:
    def __init__(self, root: Path):
        self.this = Kinematics(root / "robot_config" / "model.urdf")
        self.second = Chain(root / "robot_config" / "second_arm.urdf", SECOND_JOINTS, SECOND_TIP, SECOND_TOOL)
        fit = root / "robot_config" / "second_arm.json"
        if not fit.exists():
            raise RobotError("where the second arm stands is not known (robot_config/second_arm.json is missing): run tools/calibrate_second_arm.py")
        T = np.array(json.loads(fit.read_text())["T_base_second"])
        yaw = math.atan2(T[1, 0], T[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        self.T = np.array([[c, -s, 0, T[0, 3]], [s, c, 0, T[1, 3]], [0, 0, 1, 0], [0, 0, 0, 1.0]])  # upright on the table
        self.obstacles = None  # the things on the table, once use_obstacles has found a recent map

    # ---------------------------------------------------------------- shapes

    def segments(self, arm: str, q) -> list[tuple[np.ndarray, np.ndarray, float]]:
        """That arm as capsules (start, end, radius) in this arm's base frame, base first, gripper last."""
        if arm == "this":
            s = self.this.body_segments(np.asarray(q, dtype=float)[:7])
            return [s[0], s[1], (s[2][0], s[3][1], 0.03), s[4], s[5], *s[6:]]  # elbow bracket and forearm as one, to match PARTS; then the wrist camera
        f = [self.T @ F for F in self.second.frames(np.asarray(q, dtype=float)[:5])]
        tip = self.T @ self.second.tip(q)
        o = [F[:3, 3] for F in f]
        fingertips = tip[:3, :3] @ (self.second.tool + [0, 0.012, 0]) + tip[:3, 3]
        ground = np.array([o[0][0], o[0][1], 0.0])
        parts = [(ground, o[1], 0.05), (o[1], o[2], 0.032), (o[2], o[3], 0.03), (o[3], tip[:3, 3], 0.032), (tip[:3, 3], fingertips, 0.04)]
        if self.second.camera is not None:
            foot, lens = (tip[:3, :3] @ self.second.camera.T).T + tip[:3, 3]
            parts.append((foot, lens, 0.022))
        return parts

    def second_tool(self, q) -> np.ndarray:
        """The second arm's tool point in this arm's base frame."""
        return self.T[:3, :3] @ self.second.tool_point(q) + self.T[:3, 3]

    def second_jaws_along(self, q) -> np.ndarray:
        """The direction the second arm's jaws close along, in this arm's base frame (the gripper base's x axis)."""
        return (self.T @ self.second.tip(q))[:3, 0]

    def between(self, q_this, q_second, grippers_may_meet: bool = False) -> tuple[float, str]:
        """The smallest gap between the two arms, metres (negative: overlapping), and between which parts."""
        best, where = math.inf, ""
        for i, (a0, a1, ra) in enumerate(self.segments("this", q_this)):
            for k, (b0, b1, rb) in enumerate(self.segments("second", q_second)):
                if grippers_may_meet and min(i, k) >= 3 and max(i, k) >= 4:
                    continue  # the two hands: gripper, wrist camera, and the wrist just behind them
                gap = segment_distance(a0, a1, b0, b1) - ra - rb
                if gap < best:
                    best, where = gap, f"this arm's {PARTS[i]} and the second arm's {PARTS[k]}"
        return float(best), where

    def path_clear(self, arm: str, path, other_q, margin: float = 0.02, grippers_may_meet: bool = False) -> str | None:
        """Why `arm` may not go along `path` (joint angles, the first being where it is now) with the
        other arm standing at `other_q`, or None if it stays `margin` clear all the way."""
        path = [np.asarray(q, dtype=float) for q in path]
        if other_q is not None:  # None: where the other arm is, is not known; only the things on the table are checked
            start = self.between(path[0], other_q, grippers_may_meet)[0] if arm == "this" else self.between(other_q, path[0], grippers_may_meet)[0]
            margin = min(margin, start - 0.003)  # an arm that is already closer than that may still move away, not closer
        tool = (lambda q: self.this.fk(q[:7])[:3, 3]) if arm == "this" else self.second_tool
        ends = ((tool(path[0])[:2], 0.07), (tool(path[-1])[:2], 0.07))  # it may touch what it comes from and what it goes to
        for a, b in zip(path, path[1:]):
            n = max(1, int(np.abs(b - a).max() / 0.06))
            for t in np.linspace(0.0, 1.0, n + 1)[1:]:
                q = a + (b - a) * t
                if other_q is not None:
                    gap, where = self.between(q, other_q, grippers_may_meet) if arm == "this" else self.between(other_q, q, grippers_may_meet)
                    if gap < margin:
                        return f"on the way {where} would come within {max(0.0, gap) * 1000:.0f} mm of each other"
                hit = self.hits(arm, q, ends)
                if hit:
                    return f"on the way {hit}"
        return None

    # ---------------------------------------------------------------- things on the table

    def keep_obstacles(self, root: Path, grid: np.ndarray, area, cell: float, table_z: float, q_this, q_second) -> np.ndarray:
        """Keep a height map (metres above the mat, nan where nothing was seen) as the things on the
        table, with both arms, standing at those joint angles, cut out of it. Returns what was kept."""
        (x0, _), (y0, _) = area
        rows, cols = np.mgrid[0 : grid.shape[0], 0 : grid.shape[1]]
        xy = np.stack([x0 + (cols + 0.5) * cell, y0 + (rows + 0.5) * cell], axis=-1)
        kept = np.nan_to_num(grid, nan=0.0)
        for arm, q in (("this", q_this), ("second", q_second)):
            if q is None:
                continue
            for a0, a1, r in self.segments(arm, q):
                d = a1[:2] - a0[:2]
                t = np.clip(((xy - a0[:2]) @ d) / max(float(d @ d), 1e-9), 0.0, 1.0)
                kept[np.linalg.norm(xy - (a0[:2] + t[..., None] * d), axis=-1) < r + 0.03] = 0.0
        path = root / OBSTACLES
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(path, grid=kept.astype(np.float32), origin=np.array([x0, y0]), cell=cell, table_z=table_z, t=time.time())
        return kept

    def use_obstacles(self, root: Path) -> bool:
        """Take up the kept height map if there is one and it is recent. Says whether there is."""
        path = root / OBSTACLES
        self.obstacles = None
        if path.exists():
            data = np.load(path)
            if time.time() - float(data["t"]) < OBSTACLES_LAST_S:
                self.obstacles = {"grid": data["grid"], "origin": data["origin"], "cell": float(data["cell"]), "table_z": float(data["table_z"])}
        return self.obstacles is not None

    def _tallest_near(self, xy, radius: float, leave_out=()) -> tuple[float, tuple[float, float] | None]:
        """The tallest thing within `radius` of a spot: its height above the mat and where, leaving out the cells near the spots in `leave_out`."""
        o = self.obstacles
        g, cell = o["grid"], o["cell"]
        c0, r0 = (np.asarray(xy) - o["origin"]) / cell
        n = int(math.ceil(radius / cell))
        rows = np.arange(max(0, int(r0) - n), min(g.shape[0], int(r0) + n + 1))
        cols = np.arange(max(0, int(c0) - n), min(g.shape[1], int(c0) + n + 1))
        if not len(rows) or not len(cols):
            return 0.0, None
        R, C = np.meshgrid(rows, cols, indexing="ij")
        cx, cy = o["origin"][0] + (C + 0.5) * cell, o["origin"][1] + (R + 0.5) * cell
        ok = np.hypot(cx - xy[0], cy - xy[1]) <= radius
        for spot, around in leave_out:
            ok &= np.hypot(cx - spot[0], cy - spot[1]) > around
        h = np.where(ok, g[R, C], 0.0)
        if h.max() < LOW:
            return 0.0, None
        k = np.unravel_index(int(h.argmax()), h.shape)
        return float(h[k]), (float(cx[k]), float(cy[k]))

    def hits(self, arm: str, q, leave_out=(), margin: float = 0.008) -> str | None:
        """Which part of that arm, at those joint angles, dips into something on the table, or None."""
        if getattr(self, "obstacles", None) is None:
            return None
        table_z = self.obstacles["table_z"]
        for part, (a0, a1, r) in list(zip(PARTS, self.segments(arm, q)))[1:]:  # the base stands on the table: not it
            for t in np.linspace(0.0, 1.0, max(2, int(np.linalg.norm(a1 - a0) / 0.012) + 1)):
                p = a0 + (a1 - a0) * t
                tall, where = self._tallest_near(p[:2], r + margin, leave_out)
                if where is not None and table_z + tall > p[2] - r - margin:
                    whose = "this arm's" if arm == "this" else "the second arm's"
                    return f"{whose} {part} would hit the thing {tall * 1000:.0f} mm tall at x={where[0]:+.3f} y={where[1]:+.3f}"
        return None

    def travel_height(self, start_xy, end_xy, width: float = 0.05) -> float:
        """How high the tallest thing stands, above the mat, along the straight line between two spots on the table. 0 with no map."""
        if getattr(self, "obstacles", None) is None:
            return 0.0
        a, b = np.asarray(start_xy, dtype=float), np.asarray(end_xy, dtype=float)
        steps = max(2, int(np.linalg.norm(b - a) / 0.01) + 1)
        return max(self._tallest_near(a + (b - a) * t, width)[0] for t in np.linspace(0.0, 1.0, steps))

    # ---------------------------------------------------------------- the second arm's inverse kinematics

    def second_ik(self, pos, pitch_deg: float = 90.0, yaw_deg: float = 0.0, seed=None) -> np.ndarray:
        """Joint angles that put the second arm's tool point at `pos` (this arm's base frame),
        pointing `pitch_deg` below level, jaws turned by `yaw_deg`. Five joints: the direction it
        points sideways follows from where the place is. Raises if it cannot get there."""
        target = np.asarray(pos, dtype=float)
        local = np.linalg.inv(self.T)[:3, :3] @ (target - self.T[:3, 3])
        down = -math.sin(math.radians(pitch_deg))
        starts = [np.asarray(seed, dtype=float)[:5]] if seed is not None else []
        pan = math.atan2(local[0], local[1])  # its straight ahead is +y
        starts += [np.array([s * pan, -0.8, 0.8, 1.0, 0.0]) for s in (-1.0, 1.0)] + [np.array([0.0, -1.2, 1.2, 0.5, 0.0])]
        chain = self.second

        def miss(q4):
            q = np.array([*q4, math.radians(yaw_deg)])
            return np.array([*(chain.tool_point(q) - local), 0.05 * (chain.points_along(q)[2] - down)])

        best = None
        for q0 in starts:
            q4 = q0[:4].copy()
            for _ in range(80):
                e = miss(q4)
                J = np.column_stack([(miss(q4 + d) - e) / 1e-5 for d in np.eye(4) * 1e-5])
                q4 = np.clip(q4 - np.linalg.solve(J.T @ J + 1e-6 * np.eye(4), J.T @ e), chain.lower[:4], chain.upper[:4])
            e = miss(q4)
            score = float(np.linalg.norm(e[:3])) + abs(float(e[3])) / 0.05 * 0.02
            if best is None or score < best[0]:
                best = (score, q4, e)
        _, q4, e = best
        if np.linalg.norm(e[:3]) > 0.004 or abs(e[3]) / 0.05 > math.sin(math.radians(8)):
            raise RobotError(f"the second arm cannot put its tool point there pointing {pitch_deg:.0f} deg down: closest is {np.linalg.norm(e[:3]) * 1000:.0f} mm off. "
                             "It reaches about 30 cm, and has no level pose closer than about 24 cm to its own base.")  # fmt: skip
        return np.array([*q4, math.radians(yaw_deg)])
