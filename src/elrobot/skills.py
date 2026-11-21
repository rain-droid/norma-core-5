"""Behaviours built on Robot: named poses, looking, picking, placing, touching, gestures.

Each public coroutine is one thing an agent can ask for. They raise RobotError with a message
written for the agent: what went wrong and, where possible, what to try instead.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

from . import vision
from .config import Config
from .kinematics import approach_from_pitch, jaw_from_yaw
from .robot import ContactGuard, Robot, RobotError
from .servo import ARM_IDS

log = logging.getLogger("elrobot.skills")

# Joint angles in degrees. READY keeps the gripper over the table in front of the base,
# pointing down and forward; a good place to start and end any task.
READY_DEG = [0.0, -21.0, -29.0, 36.0, 0.0, 73.5, 0.0]

GESTURES = ("dap", "wave", "nod", "shake", "bow", "dance", "shrug", "peek")


@dataclass
class Seen:
    """An object found on the table in the last scan."""

    id: int
    xyz: list[float]  # table point under the object's centre, base frame
    size_mm: tuple[float, float]  # long side, short side as seen from above
    yaw_deg: float | None  # jaw rotation that closes across the short side (top-down grasp)
    pixel: tuple[float, float]
    box: tuple[int, int, int, int]
    color: str = ""


class Skills:
    def __init__(self, robot: Robot, cfg: Config, emit: Callable[..., None] | None = None):
        self.robot = robot
        self.cfg = cfg
        self.emit = emit or (lambda kind, **f: None)
        self.cam = vision.CameraModel.load(cfg.data_dir / "camera_wrist.json")
        self.seen: dict[int, Seen] = {}
        self._sit_file = cfg.data_dir / "sit_pose.json"

    # ================================================================== helpers

    @property
    def table_z(self) -> float:
        return self.cfg.table_z

    def _frame(self, camera: str = "wrist") -> tuple[np.ndarray, float]:
        fr = self.robot.link.frames.get(camera)
        if fr is None:
            have = sorted(self.robot.link.frames) or "none"
            raise RobotError(f"no frame from camera '{camera}' (cameras with frames: {have})")
        age = time.monotonic() - fr.stamp
        if age > 2.0:
            raise RobotError(f"camera '{camera}' is stale ({age:.1f}s since last frame)")
        return vision.decode(fr.jpeg), age

    async def _fresh_frame(self, camera: str = "wrist", settle: float = 0.35) -> np.ndarray:
        """A frame taken after the arm has stopped (the cameras lag by ~100 ms)."""
        await asyncio.sleep(settle)
        return self._frame(camera)[0]

    def _cam(self, img: np.ndarray) -> vision.CameraModel:
        return self.cam.scaled(img.shape[1], img.shape[0])

    def _jaw_mask(self, img: np.ndarray) -> np.ndarray:
        """Pixels covered by the gripper's own jaws in the wrist view (bottom centre)."""
        h, w = img.shape[:2]
        mask = np.zeros((h, w), bool)
        mask[int(0.58 * h) :, int(0.08 * w) : int(0.92 * w)] = True
        return mask

    def describe_pose(self) -> str:
        r = self.robot
        if not r.ready:
            return r.why_not_ready() or "not ready"
        T = r.tcp()
        pitch = math.degrees(math.asin(float(np.clip(-T[2, 2], -1, 1))))
        return (
            f"tool at x={T[0, 3]:+.3f} y={T[1, 3]:+.3f} z={T[2, 3]:+.3f} m, pointing {pitch:.0f} deg below horizontal; "
            f"gripper {r.gripper_open() * 100:.0f}% open{', holding something' if r.holding else ''}; "
            f"torque {'on' if r.torque_on else 'off'}"
        )

    # ================================================================== poses

    async def home(self, speed: float = 0.8) -> str:
        await self.robot.move_joints(np.radians(READY_DEG), speed=speed, label="going home")
        return "at the ready pose. " + self.describe_pose()

    def _observe_q(self) -> np.ndarray:
        r = self.robot.solve([0.0, 0.13, 0.215], pitch=68, yaw=0, q_seed=np.radians(READY_DEG))
        if not r.ok:
            raise RobotError(f"can't reach the observe pose ({r.describe()})")
        return r.q

    async def observe(self, speed: float = 0.8) -> str:
        """Raise the wrist camera so it looks down over the workspace."""
        await self.robot.move_joints(self._observe_q(), speed=speed, label="looking around")
        return "camera raised over the workspace. " + self.describe_pose()

    async def rest(self) -> str:
        """Fold into the sitting pose and turn torque off."""
        if not self._sit_file.exists():
            raise RobotError("no sitting pose saved (run `scripts/arm.py savesit` with the arm resting)")
        q_sit = np.array(json.loads(self._sit_file.read_text())["q"])
        above = q_sit.copy()
        above[1] += math.radians(22)  # shoulder raised: come down onto the rest from above
        r = self.robot
        if r.holding:
            raise RobotError("holding something; place it or open the gripper before resting")
        await r.move_joints(above, speed=0.6, check=False, label="sitting down")
        await r.move_joints(q_sit, speed=0.3, check=False, label="sitting down")
        await r.release()
        return "sitting, torque off"

    # ================================================================== seeing

    async def look(self, camera: str = "wrist", grid: bool = True) -> tuple[str, bytes]:
        img = await self._fresh_frame(camera, settle=0.15)
        h, w = img.shape[:2]
        out = vision.draw_grid(img) if grid else img
        text = f"{camera} camera, {w}x{h} px. " + self.describe_pose()
        if camera == "wrist":
            text += ". The black shapes at the bottom are your own gripper jaws; the grasp point is between their tips."
        return text, vision.encode(out)

    def pixel_to_table(self, u: float, v: float, img_shape=None, height: float = 0.0) -> np.ndarray:
        cam = self.cam if img_shape is None else self.cam.scaled(img_shape[1], img_shape[0])
        p = cam.pixel_to_plane(u, v, self.robot.tcp(), self.table_z + height)
        if p is None:
            raise RobotError(f"pixel ({u:.0f}, {v:.0f}) looks above the horizon, not at the table")
        return p

    async def scan(self, max_objects: int = 8) -> tuple[str, bytes, list[Seen]]:
        """Find things on the table in the current wrist view and number them."""
        img = await self._fresh_frame("wrist")
        cam = self._cam(img)
        T = self.robot.tcp()
        blobs = vision.segment_objects(img, ignore=self._jaw_mask(img))
        seen: list[Seen] = []
        marks = []
        for b in blobs:
            if len(seen) >= max_objects:
                break
            p = cam.pixel_to_plane(*b.centroid, T, self.table_z)
            if p is None:
                continue
            if not (-0.45 < p[0] < 0.45 and -0.05 < p[1] < 0.60):
                continue  # far outside the table area the arm could ever reach
            # physical size: project the long and short half-axes onto the table
            a = math.radians(b.angle_deg)
            ends = []
            for dx, dy, half in ((math.cos(a), math.sin(a), b.length_px / 2), (-math.sin(a), math.cos(a), b.width_px / 2)):
                q1 = cam.pixel_to_plane(b.centroid[0] + dx * half, b.centroid[1] + dy * half, T, self.table_z)
                q2 = cam.pixel_to_plane(b.centroid[0] - dx * half, b.centroid[1] - dy * half, T, self.table_z)
                ends.append((q1, q2))
            if any(q is None for pair in ends for q in pair):
                continue
            long_v = ends[0][0] - ends[0][1]
            short_v = ends[1][0] - ends[1][1]
            long_mm, short_mm = float(np.linalg.norm(long_v) * 1000), float(np.linalg.norm(short_v) * 1000)
            if long_mm < short_mm:
                long_mm, short_mm, long_v, short_v = short_mm, long_mm, short_v, long_v
            # jaws close along the short side; yaw is measured about a straight-down approach
            jaw = short_v[:2] / (np.linalg.norm(short_v[:2]) + 1e-9)
            yaw = math.degrees(math.atan2(-jaw[1], jaw[0]))
            yaw = (yaw + 90) % 180 - 90  # the gripper is symmetric: keep it in [-90, 90)
            s = Seen(
                id=len(seen) + 1,
                xyz=[round(float(p[0]), 3), round(float(p[1]), 3), round(self.table_z, 3)],
                size_mm=(round(long_mm), round(short_mm)),
                yaw_deg=round(yaw) if long_mm > 1.25 * short_mm else None,
                pixel=(round(b.centroid[0], 1), round(b.centroid[1], 1)),
                box=b.box,
                color=_color_name(img, b),
            )
            seen.append(s)
            marks.append({**b.as_mark(f"#{s.id}"), "polygon": b.polygon})
        self.seen = {s.id: s for s in seen}
        self.emit("detections", camera="wrist", width=img.shape[1], height=img.shape[0],
                  items=[{"label": f"#{s.id} {s.color}".strip(), "score": 1.0, "box": list(s.box), "point": list(s.pixel)} for s in seen])  # fmt: skip
        self.emit("markers", markers=[{"id": f"obj{s.id}", "label": f"#{s.id} {s.color}".strip(), "pos": s.xyz, "color": "#ffb020", "radius": 0.012} for s in seen])
        if not seen:
            text = "nothing stands out on the table in this view. Try observe() for a wider view, or move the camera."
        else:
            lines = [
                f"#{s.id}: {s.color} object, about {s.size_mm[0]}x{s.size_mm[1]} mm, at x={s.xyz[0]:+.3f} y={s.xyz[1]:+.3f} "
                f"({math.hypot(s.xyz[0], s.xyz[1] - 0.04) * 100:.0f} cm from the base), pixel ({s.pixel[0]:.0f}, {s.pixel[1]:.0f})"
                for s in seen
            ]
            text = "objects on the table (numbers match the image):\n" + "\n".join(lines)
            if not self.cam.calibrated:
                text += "\n(camera not calibrated yet: positions are rough, pick() refines them up close)"
        return text, vision.encode(vision.draw_marks(img, marks)), seen

    # ================================================================== manipulating

    def _resolve(self, obj: int | None, x: float | None, y: float | None) -> tuple[np.ndarray, Seen | None]:
        if obj is not None:
            s = self.seen.get(int(obj))
            if s is None:
                raise RobotError(f"no object #{obj} from the last scan (known: {sorted(self.seen) or 'none'}). Call scan() first.")
            return np.array(s.xyz, dtype=float), s
        if x is None or y is None:
            raise RobotError("give either an object number from scan() or x and y in metres")
        return np.array([x, y, self.table_z], dtype=float), None

    def _approach_for(self, target: np.ndarray, grasp_z: float, yaw: float | None, hover: float = 0.07):
        """Pick the most vertical approach that reaches both the grasp point and a hover point
        backed off along the approach. Returns (approach, pitch, q_hover, hover_pos, grasp_pos)."""
        r = self.robot
        grasp = np.array([target[0], target[1], self.table_z + grasp_z])
        last = None
        for pitch in (90, 80, 70, 60, 50, 40, 30):
            a = approach_from_pitch(grasp[:2], pitch)
            jaw = jaw_from_yaw(a, yaw if yaw is not None else 0.0)
            hover_pos = grasp - a * hover
            rg = r.kin.ik(grasp, approach=a, jaw=jaw, q_seed=np.radians(READY_DEG), limit_margin=self.cfg.limit_margin,
                          accept=lambda q: r.check_config(q, allow_low=True) is None)  # fmt: skip
            if not rg.ok:
                last = rg
                continue
            rh = r.kin.ik(hover_pos, approach=a, jaw=jaw, q_seed=rg.q, limit_margin=self.cfg.limit_margin,
                          accept=lambda q: r.check_config(q) is None)  # fmt: skip
            if rh.ok:
                return a, float(pitch), rh.q, hover_pos, grasp
            last = rh
        d = math.hypot(target[0], target[1] - 0.04)
        why = f"it is {d * 100:.0f} cm from the base (reach is about 38 cm on the table)" if d > 0.36 else (
            "it is too close to the base" if d < 0.09 else (last.describe() if last else "no approach found"))  # fmt: skip
        raise RobotError(f"can't reach a grasp at x={target[0]:+.3f} y={target[1]:+.3f}: {why}")

    async def _refine(self, target: np.ndarray, max_shift: float = 0.05) -> np.ndarray:
        """From the hover pose, look again and return a better table position for the target.

        The object closest to where the target is expected to appear is taken to be the target.
        """
        img = await self._fresh_frame("wrist")
        cam = self._cam(img)
        T = self.robot.tcp()
        expect = cam.project([target[0], target[1], self.table_z], T)
        if expect is None:
            return target
        blobs = vision.segment_objects(img, ignore=self._jaw_mask(img), min_area=400)
        b = vision.nearest_blob(blobs, *expect, max_dist=0.3 * img.shape[1])
        if b is None:
            return target
        p = cam.pixel_to_plane(*b.centroid, T, self.table_z)
        if p is None or np.linalg.norm(p[:2] - target[:2]) > max_shift:
            return target
        self.emit("detections", camera="wrist", width=img.shape[1], height=img.shape[0],
                  items=[{"label": "target", "score": 1.0, "box": list(b.box), "point": [round(b.centroid[0]), round(b.centroid[1])]}])  # fmt: skip
        return np.array([p[0], p[1], self.table_z])

    async def pick(self, obj: int | None = None, x: float | None = None, y: float | None = None,
                   yaw: float | None = None, grasp_height: float = 0.018, refine: bool = True) -> dict:  # fmt: skip
        """Grasp the thing at a table position and lift it."""
        r = self.robot
        target, s = self._resolve(obj, x, y)
        if yaw is None and s is not None:
            yaw = s.yaw_deg
        if s is not None and s.size_mm[1] > 50:
            raise RobotError(f"object #{s.id} is about {s.size_mm[1]} mm across its narrow side; the gripper opens 53 mm. Try another angle with yaw, or push it instead.")
        if r.holding:
            raise RobotError("already holding something; place it first")
        self.emit("markers", markers=[{"id": "target", "label": "pick", "pos": target.tolist(), "color": "#3ddc84", "radius": 0.014}])
        await r.set_gripper(1.0, label="opening")
        a, pitch, q_hover, hover_pos, grasp = self._approach_for(target, grasp_height, yaw)
        await r.move_joints(q_hover, speed=0.9, label="reaching")
        if refine:
            for _ in range(2):
                better = await self._refine(target)
                shift = float(np.linalg.norm(better[:2] - target[:2]))
                if shift < 0.004:
                    break
                target = better
                a, pitch, q_hover, hover_pos, grasp = self._approach_for(target, grasp_height, yaw)
                await r.move_joints(q_hover, speed=0.6, label="lining up")
        self.emit("markers", markers=[{"id": "target", "label": "pick", "pos": target.tolist(), "color": "#3ddc84", "radius": 0.014}])
        await r.move_to(grasp, approach=a, linear=True, speed=0.5, allow_low=True, label="descending")
        g = await r.grasp()
        if not g["holding"]:
            await r.set_gripper(1.0, label="opening")
            await r.move_to(hover_pos, approach=a, linear=True, speed=0.7, allow_low=True, label="backing off")
            raise RobotError(
                f"closed on nothing at x={target[0]:+.3f} y={target[1]:+.3f}: the jaws shut completely. "
                "The object is probably a little off from there; scan() again and retry, or adjust x/y."
            )
        await r.move_to(grasp + np.array([0, 0, 0.09]), approach=a, linear=True, speed=0.6, allow_low=True, label="lifting")
        await asyncio.sleep(0.25)
        if not r.still_holding():
            raise RobotError("had it, but it slipped out while lifting. scan() to see where it ended up.")
        return {"holding": True, "width_mm": g["width_mm"], "picked_from": [round(float(v), 3) for v in target[:2]], "pitch": pitch}

    async def place(self, x: float, y: float, z: float = 0.0, yaw: float | None = None) -> dict:
        """Put down whatever is held at a table position (z = height of the surface there)."""
        r = self.robot
        target = np.array([x, y, self.table_z + z])
        self.emit("markers", markers=[{"id": "target", "label": "place", "pos": target.tolist(), "color": "#5ab0ff", "radius": 0.014}])
        a, pitch, q_hover, hover_pos, drop = self._approach_for(target, z + 0.022, yaw)
        await r.move_joints(q_hover, speed=0.8, label="carrying")
        await r.move_to(drop, approach=a, linear=True, speed=0.5, allow_low=True, label="lowering")
        was_holding = r.holding
        await r.set_gripper(1.0, label="releasing")
        await r.move_to(hover_pos, approach=a, linear=True, speed=0.7, allow_low=True, label="backing off")
        return {"placed_at": [round(x, 3), round(y, 3)], "released_something": was_holding}

    async def touch(self, obj: int | None = None, x: float | None = None, y: float | None = None, height: float = 0.0) -> dict:
        """Reach out and gently touch a point on the table (or the top of something on it)."""
        r = self.robot
        target, _ = self._resolve(obj, x, y)
        self.emit("markers", markers=[{"id": "target", "label": "touch", "pos": target.tolist(), "color": "#ff6ad5", "radius": 0.014}])
        if not r.holding:
            await r.set_gripper(0.0, label="closing")  # closed jaws make one blunt fingertip
        # the fingertips are 12 mm ahead of the tool point
        a, pitch, q_hover, hover_pos, low = self._approach_for(target, max(0.014, height * 0.5), None, hover=0.08)
        await r.move_joints(q_hover, speed=0.9, label="reaching")
        guard = ContactGuard()
        res = await r.move_to(low, approach=a, linear=True, speed=0.22, allow_low=True, label="touching", guard=guard)
        contact = res.outcome == "contact"
        tip = r.tcp()[:3, 3] + r.tcp()[:3, 2] * 0.012
        await asyncio.sleep(0.5)
        await r.move_to(hover_pos, approach=a, linear=True, speed=0.6, allow_low=True, label="backing off")
        return {"touched": True, "felt_contact": contact, "fingertip_height_mm": round(float(tip[2] - self.table_z) * 1000)}

    async def point_at(self, obj: int | None = None, x: float | None = None, y: float | None = None) -> dict:
        """Hover the gripper over a spot, pointing at it."""
        target, _ = self._resolve(obj, x, y)
        self.emit("markers", markers=[{"id": "target", "label": "here", "pos": target.tolist(), "color": "#ff6ad5", "radius": 0.014}])
        a, pitch, q_hover, hover_pos, _ = self._approach_for(target, 0.02, None, hover=0.07)
        await self.robot.move_joints(q_hover, speed=0.9, label="pointing")
        return {"pointing_at": [round(float(target[0]), 3), round(float(target[1]), 3)]}

    # ================================================================== gestures

    async def gesture(self, name: str) -> str:
        name = name.lower().strip()
        fn = getattr(self, f"_g_{name}", None)
        if fn is None:
            raise RobotError(f"unknown gesture '{name}'. Available: {', '.join(GESTURES)}")
        if self.robot.holding and name in ("dap", "dance"):
            raise RobotError("holding something; put it down before that")
        return await fn()

    async def _keys(self, frames: list[tuple[list[float], float]], label: str) -> None:
        """Play joint keyframes given as (offsets in degrees from READY, speed)."""
        base = np.radians(READY_DEG)
        for off, speed in frames:
            await self.robot.move_joints(base + np.radians(off), speed=speed, label=label, check=True)

    async def _g_wave(self) -> str:
        up = [0, 25, 10, -40, 0, -75, 0]  # forearm up, hand raised
        await self._keys([(up, 1.2)], "waving")
        for _ in range(3):
            await self._keys([([20, 25, 10, -40, 0, -75, 35], 1.8), ([-20, 25, 10, -40, 0, -75, -35], 1.8)], "waving")
        await self.home(1.2)
        return "waved"

    async def _g_nod(self) -> str:
        for _ in range(2):
            await self._keys([([0, 0, 0, 0, 0, 24, 0], 1.6), ([0, 0, 0, 0, 0, -22, 0], 1.6)], "nodding")
        await self.home(1.2)
        return "nodded"

    async def _g_shake(self) -> str:
        up = [0, 10, 0, -15, 0, -45, 0]
        await self._keys([(up, 1.2)], "shaking head")
        for _ in range(2):
            await self._keys([([0, 10, 0, -15, 0, -45, 55], 2.0), ([0, 10, 0, -15, 0, -45, -55], 2.0)], "shaking head")
        await self.home(1.2)
        return "shook head"

    async def _g_bow(self) -> str:
        await self._keys([([0, 28, 8, 10, 0, 12, 0], 0.7)], "bowing")
        await asyncio.sleep(0.5)
        await self.home(0.8)
        return "bowed"

    async def _g_shrug(self) -> str:
        await self.robot.set_gripper(1.0, label="shrugging")
        await self._keys([([0, -10, -12, 8, 0, -35, 0], 1.5), ([0, 0, 0, 0, 0, 0, 0], 1.0)], "shrugging")
        return "shrugged"

    async def _g_peek(self) -> str:
        await self._keys([([35, 5, 0, -10, 0, -40, 0], 1.0)], "looking around")
        await asyncio.sleep(0.4)
        await self._keys([([-35, 5, 0, -10, 0, -40, 0], 1.0)], "looking around")
        await asyncio.sleep(0.4)
        await self.home(1.0)
        return "looked left and right"

    async def _g_dance(self) -> str:
        r = self.robot
        for i in range(3):
            s = 1 if i % 2 == 0 else -1
            await self._keys([([28 * s, 12, -10, -15, 0, -30, 40 * s], 1.8)], "dancing")
            await r.set_gripper(0.2, label="dancing")
            await self._keys([([-28 * s, -5, 12, 12, 0, 10, -40 * s], 1.8)], "dancing")
            await r.set_gripper(1.0, label="dancing")
        await self.home(1.2)
        return "danced"

    async def _g_dap(self) -> str:
        """Fist bump: hold a closed fist out towards the person and wait to be bumped."""
        r = self.robot
        await r.set_gripper(0.0, label="dap")
        # fist out in front, level, at about chest height for someone sitting at the table
        sol = r.solve([0.0, 0.31, 0.20], pitch=0, yaw=90, q_seed=np.radians(READY_DEG))
        if not sol.ok:
            sol = r.solve([0.0, 0.29, 0.18], pitch=5, q_seed=np.radians(READY_DEG))
        if not sol.ok:
            raise RobotError(f"can't reach the fist-bump pose ({sol.describe()})")
        await r.move_joints(sol.q, speed=1.3, label="dap")
        bumped = await self._wait_for_bump(6.0)
        if bumped:
            # recoil and "blow it up"
            await r.move_relative([0, -0.05, 0.02], speed=1.4, label="dap")
            await r.set_gripper(1.0, label="dap")
        await self.home(1.2)
        return "got the fist bump" if bumped else "held my fist out but nobody bumped it"

    async def _wait_for_bump(self, timeout: float) -> bool:
        """While holding still, watch for the arm being pushed: joints deflect and load jumps."""
        r = self.robot
        link = r.link
        await asyncio.sleep(0.35)
        q0 = r.q()
        load0 = np.array([link.motors[i].load for i in ARM_IDS], dtype=float)
        t0 = time.monotonic()
        hits = 0
        self.robot.activity = "waiting for a bump"
        try:
            while time.monotonic() - t0 < timeout:
                if r._stop_evt.is_set():
                    return False
                await asyncio.sleep(0.02)
                dq = float(np.max(np.abs(r.q() - q0)))
                dl = float(np.max(np.abs(np.array([link.motors[i].load for i in ARM_IDS], dtype=float) - load0)))
                hits = hits + 1 if (dq > 0.02 or dl > 90) else 0
                if hits >= 2:
                    return True
            return False
        finally:
            self.robot.activity = "idle"


def _color_name(img: np.ndarray, b: vision.Blob) -> str:
    """Rough colour word for a blob, to help an agent tell objects apart."""
    import cv2

    mask = np.zeros(img.shape[:2], np.uint8)
    cv2.fillPoly(mask, [np.array(b.polygon, np.int32)], 255)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = [float(np.median(c[mask > 0])) for c in cv2.split(hsv)]
    if s < 45:
        return "white" if v > 170 else ("black" if v < 70 else "grey")
    for name, lo, hi in (("red", 0, 8), ("orange", 8, 20), ("yellow", 20, 33), ("green", 33, 85), ("blue", 85, 130), ("purple", 130, 160), ("pink", 160, 172), ("red", 172, 181)):
        if lo <= h < hi:
            return name
    return ""
