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
from .profile import ACTIVE as PROFILE
from .robot import ContactGuard, Robot, RobotError, Stopped
from .servo import ARM_IDS

log = logging.getLogger("elrobot.skills")

# Joint angles in degrees. READY keeps the gripper over the table in front of the base,
# pointing down and forward; a good place to start and end any task.
READY_DEG = list(PROFILE.ready_deg)

GESTURES = ("dap", "handshake", "wave", "nod", "shake", "bow", "dance", "shrug", "peek", "happy", "sad", "curious", "celebrate", "oops")


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
        self.cam = self._load_camera()
        self.seen: dict[int, Seen] = {}
        self._sit_file = cfg.data_dir / ("sit_pose.json" if PROFILE.name == "elrobot" else f"sit_pose_{PROFILE.name}.json")

    # ================================================================== helpers

    def _load_camera(self) -> vision.CameraModel:
        """Wrist camera model: a saved calibration if there is one, else the arm's nominal one
        (factory intrinsics + the camera mount pose from the URDF)."""
        import cv2

        f = self.cfg.data_dir / ("camera_wrist.json" if PROFILE.name == "elrobot" else f"camera_wrist_{PROFILE.name}.json")
        if f.exists() or PROFILE.camera is None:
            return vision.CameraModel.load(f)
        T = self.robot.kin.T_tool_cam
        return vision.CameraModel(
            **PROFILE.camera,
            rvec_tool_cam=[float(v) for v in cv2.Rodrigues(T[:3, :3])[0].ravel()],
            t_tool_cam=[float(v) for v in T[:3, 3]],
            calibrated=True,
        )

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
        if PROFILE.name == "so101":
            # salmon-coloured jaws poking up from the bottom of the frame: mask them by colour so
            # things lying between or beside them stay visible
            import cv2

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            hch, sch, vch = hsv[..., 0], hsv[..., 1], hsv[..., 2]
            jaw = (((hch < 12) | (hch > 160)) & (sch > 50) & (vch > 110)).astype(np.uint8)
            jaw[: int(0.55 * h)] = 0
            jaw = cv2.dilate(jaw, np.ones((15, 15), np.uint8))
            return jaw.astype(bool)
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
        try:
            await self.robot.move_joints(np.radians(READY_DEG), speed=speed, label="going home")
        except RobotError as e:
            if "rejected" not in str(e):
                raise
            await self.robot.move_joints(np.radians(READY_DEG), speed=0.6, check=False, label="going home")
        return "at the ready pose. " + self.describe_pose()

    # Camera-up poses, best view first: (tool point, pitch). Top-down is out of reach this high,
    # so the camera looks forward and down over the table.
    OBSERVE_POSES = PROFILE.observe_poses

    def _observe_q(self) -> np.ndarray:
        last = None
        for xyz, pitch in self.OBSERVE_POSES:
            last = self.robot.solve(xyz, pitch=pitch, yaw=0, q_seed=np.radians(READY_DEG))
            if last.ok:
                return last.q
        raise RobotError(f"can't reach an observe pose ({last.describe()})")

    async def observe(self, speed: float = 0.8) -> str:
        """Raise the wrist camera so it looks down over the workspace."""
        await self._safe_joints(self._observe_q(), speed, "looking around")
        return "camera raised over the workspace. " + self.describe_pose()

    async def _safe_joints(self, q, speed: float, label: str) -> None:
        """Joint move that first returns to the ready pose if the direct path is rejected
        (as it is from an odd pose, e.g. where an interrupted throw left the arm)."""
        r = self.robot
        try:
            await r.move_joints(q, speed=speed, label=label)
        except RobotError as e:
            if "rejected" not in str(e):
                raise
            await r.move_joints(np.radians(READY_DEG), speed=0.6, check=False, label="getting back to ready")
            await r.move_joints(q, speed=speed, label=label)

    async def rest(self) -> str:
        """Fold into the sitting pose and turn torque off."""
        if not self._sit_file.exists():
            raise RobotError("no sitting pose saved (run `scripts/arm.py savesit` with the arm resting)")
        q_sit = np.array(json.loads(self._sit_file.read_text())["q"])
        above = q_sit.copy()
        if PROFILE.name == "so101":
            above[2] -= math.radians(18)  # forearm lifted: fold down onto the rest from above
        else:
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

    def _approach_for(self, target: np.ndarray, grasp_z: float, yaw: float | None, hover: float = 0.07, prefer_pitch: float | None = None):
        """Pick the most vertical approach that reaches both the grasp point and a hover point
        backed off along the approach. Returns (approach, pitch, q_hover, hover_pos, grasp_pos)."""
        r = self.robot
        grasp = np.array([target[0], target[1], self.table_z + grasp_z])
        last = None
        best = None
        for pitch in PROFILE.pitches:
            a = approach_from_pitch(grasp[:2], pitch)
            jaw0 = jaw_from_yaw(a, yaw if yaw is not None else 0.0)
            hover_pos = grasp - a * hover
            # The jaws are symmetric, so a grasp turned half a turn is the same grasp; the wrist roll
            # range is not symmetric (-157..+56 deg), so one of the two is often out of range.
            # With no yaw asked for, the roll is also allowed to settle wherever is easiest.
            jaws = [jaw0, -jaw0] if (yaw is not None or PROFILE.name != "so101") else [jaw0, -jaw0, None]
            for jaw in jaws:
                rg = r.kin.ik(grasp, approach=a, jaw=jaw, q_seed=np.radians(READY_DEG), limit_margin=self.cfg.limit_margin,
                              accept=lambda q: r.check_config(q, allow_low=True) is None)  # fmt: skip
                if not rg.ok:
                    last = rg
                    continue
                rh = r.kin.ik(hover_pos, approach=a, jaw=jaw, q_seed=rg.q, limit_margin=self.cfg.limit_margin,
                              accept=lambda q: r.check_config(q) is None)  # fmt: skip
                if not rh.ok:
                    last = rh
                    continue
                cand = (a, float(pitch), rh.q, hover_pos, grasp)
                if PROFILE.name != "so101":
                    return cand
                # With 5 joints the steepest reachable approach leaves the wrist on its limit and
                # the straight descent has nowhere to go: take the approach with the most joint room.
                room = min(float(np.min(np.minimum(q - r.kin.lower, r.kin.upper - q))) for q in (rg.q, rh.q))
                jump = float(np.max(np.abs(rg.q - rh.q)))
                # when a pitch is asked for (e.g. to put a thing down the way it was picked up),
                # closeness to it matters more than joint room
                score = room if prefer_pitch is None else (-abs(pitch - prefer_pitch) if room > 0.04 else -1e3)
                if jump < 0.9 and (best is None or score > best[0]):
                    best = (score, cand)
        if best is not None:
            return best[1]
        d = math.hypot(target[0], target[1] - PROFILE.base_y)
        why = f"it is {d * 100:.0f} cm from the base (reach is about {PROFILE.reach_m * 100:.0f} cm on the table)" if d > PROFILE.reach_m - 0.02 else (
            "it is too close to the base" if d < 0.09 else (last.describe() if last else "no approach found"))  # fmt: skip
        raise RobotError(f"can't reach a grasp at x={target[0]:+.3f} y={target[1]:+.3f}: {why}")

    def _color_blobs(self, img: np.ndarray, color: str, min_area: float = 180.0) -> list:
        """Blobs of a named colour, largest first (jaws excluded)."""
        if color not in COLOR_HSV:
            raise RobotError(f"don't know the colour '{color}'. Known: {', '.join(COLOR_HSV)}")
        import cv2

        hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
        mask = np.zeros(img.shape[:2], np.uint8)
        for lo, hi in COLOR_HSV[color]:
            mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        if color not in ("red", "pink", "orange"):
            pass
        else:
            mask[self._jaw_mask(img)] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        return vision._blobs_from_mask(mask, min_area)

    async def find_color(self, color: str) -> tuple[np.ndarray, bytes]:
        """Look over the table for something of this colour; returns its table position and the view."""
        r = self.robot
        base = self._observe_q()
        for pan in (0, 35, -35):
            q = base.copy()
            q[0] += math.radians(pan)
            await r.move_joints(q, speed=0.9, label=f"looking for something {color}")
            img = await self._fresh_frame("wrist", settle=0.45)
            cam, T = self._cam(img), r.tcp()
            # small bright specks (status LEDs on a USB hub, reflections) are not objects to pick up
            for b in self._color_blobs(img, color, min_area=420.0):
                p = cam.pixel_to_plane(*b.centroid, T, self.table_z + 0.012)
                if p is None or math.hypot(p[0], p[1] - PROFILE.base_y) > PROFILE.reach_m + 0.06 or p[1] < 0.02:
                    continue
                jpeg = vision.encode(vision.draw_marks(img, [b.as_mark(color)]))
                self.emit("detections", camera="wrist", width=img.shape[1], height=img.shape[0],
                          items=[{"label": color, "score": 1.0, "box": list(b.box), "point": [round(b.centroid[0]), round(b.centroid[1])]}])  # fmt: skip
                return np.array([p[0], p[1], self.table_z]), jpeg
        raise RobotError(f"looked left, right and centre and saw nothing {color} within reach on the table")

    async def fetch(self, color: str, hand_over: bool = False) -> dict:
        """The whole job in one go: find the thing of this colour, pick it up, hold it up.

        On the so101 it is first measured in 3D, so it does not matter whether it lies on the table
        or is held up in a hand: low things get the table pick, anything higher is taken from the air.
        """
        color = color.lower().strip()
        if PROFILE.name == "so101":
            out = await self.fetch_air(color)
        else:
            target, _ = await self.find_color(color)
            self._hint_color = color
            try:
                out = await self.pick(x=float(target[0]), y=float(target[1]))
            finally:
                self._hint_color = None
        if hand_over:
            sol = self.robot.solve([0.0, 0.30, 0.20], pitch=10)
            if sol.ok:
                await self.robot.move_joints(sol.q, speed=0.8, label="handing it over")
        return {**out, "color": color}

    # ---- things that are not on the table: held in a hand, on top of something

    def _blob_rays(self, img: np.ndarray, color: str, min_area: float = 350.0) -> list:
        """Object-like blobs of the colour that are fully in view, each as (camera position, unit ray, blob)
        in the base frame. Clothing and screens of the same colour are dropped: too big, too ragged or too long."""
        h, w = img.shape[:2]
        cam = self._cam(img)
        T = self.robot.tcp() @ cam.T_tool_cam()
        out = []
        for b in self._color_blobs(img, color, min_area=min_area):
            x1, y1, x2, y2 = b.box
            if x1 <= 2 or y1 <= 2 or x2 >= w - 3 or y2 >= h - 3:
                continue  # cut off by the edge of the picture: its centre is not where it looks
            fill = b.area / max(1.0, b.length_px * b.width_px)
            if b.area > 0.07 * w * h or fill < 0.55 or b.length_px > 2.6 * max(1.0, b.width_px):
                continue
            d = T[:3, :3] @ cam.ray_cam(*b.centroid)
            out.append((T[:3, 3].copy(), d / np.linalg.norm(d), b))
        return out

    @staticmethod
    def _meet(o1, d1, o2, d2):
        """Where two rays come closest: (midpoint, gap between them, angle between them in degrees)."""
        w0 = o1 - o2
        b = float(d1 @ d2)
        den = 1.0 - b * b
        if den < 1e-6:
            return None, float("inf"), 0.0
        t1 = (b * float(d2 @ w0) - float(d1 @ w0)) / den
        t2 = (float(d2 @ w0) - b * float(d1 @ w0)) / den
        p1, p2 = o1 + t1 * d1, o2 + t2 * d2
        if t1 <= 0 or t2 <= 0:
            return None, float("inf"), 0.0
        return 0.5 * (p1 + p2), float(np.linalg.norm(p1 - p2)), math.degrees(math.acos(max(-1.0, min(1.0, b))))

    # (pan, lift, elbow, wrist): looking out level, then lower, then further round to each side
    AIR_LOOKS = ((0.0, -40.0, 20.0, 15.0), (0.0, -30.0, 35.0, 30.0), (40.0, -40.0, 20.0, 15.0), (-40.0, -40.0, 20.0, 15.0),
                 (40.0, -30.0, 35.0, 30.0), (-40.0, -30.0, 35.0, 30.0), (78.0, -38.0, 22.0, 18.0), (-78.0, -38.0, 22.0, 18.0))

    def _hypotheses(self, views: list, cam_f: float) -> list:
        """Every way of pairing a blob in the first view with one in a later view, kept only if the two
        lines of sight really cross at a graspable-sized thing inside the workspace. Each is scored by how
        many of the other views also have a line of sight through that point."""
        out = []
        first = views[0]
        for k in range(1, len(views)):
            for o1, d1, b1 in first:
                for o2, d2, b2 in views[k]:
                    p, gap, ang = self._meet(o1, d1, o2, d2)
                    if p is None or gap > 0.02 or not (2.0 < ang < 34.0):
                        continue
                    reach = math.hypot(p[0], p[1] - PROFILE.base_y)
                    z1, z2 = float(np.linalg.norm(p - o1)), float(np.linalg.norm(p - o2))
                    if reach > PROFILE.reach_m + 0.14 or not (-0.03 < p[2] < 0.48) or min(z1, z2) < 0.06 or p[1] < -0.12:
                        continue  # the desk behind, the room, the ceiling: not something to pick up
                    # how big the thing really is, from how big it looks at that distance
                    s1, s2 = math.sqrt(b1.area) * z1 / cam_f, math.sqrt(b2.area) * z2 / cam_f
                    if not (0.008 < s1 < 0.08) or not (0.6 < s1 / s2 < 1.65):
                        continue
                    support = 0
                    for m, view in enumerate(views):
                        if m in (0, k):
                            continue
                        if any(float(np.linalg.norm(np.cross(p - o, d))) < 0.022 for o, d, _ in view):
                            support += 1
                    out.append((support, -gap / ang, p, gap, ang, s1))
        return sorted(out, key=lambda h: (h[0], h[1]), reverse=True)

    async def locate_air(self, color: str) -> np.ndarray:
        """Find something of this colour anywhere in front of the arm and measure where it is in 3D,
        by looking at it from several camera positions and seeing where the lines of sight cross."""
        r = self.robot
        looks = [np.radians([*look, 0.0]) for look in self.AIR_LOOKS]
        base = self._observe_q()  # and down over the table, for things lying on it
        for n, pan in enumerate((0.0, 35.0, -35.0)):
            q = base.copy()
            q[0] += math.radians(pan)
            looks.insert(1 + 2 * n, q)
        saw_any = False
        for q1 in looks:
            await self._safe_joints(q1, 1.0, f"looking for something {color}")
            img = await self._fresh_frame("wrist", settle=0.45)
            cam_f = self._cam(img).fx
            first = self._blob_rays(img, color)
            log.info("locate %s: look %s -> %s", color, np.degrees(q1).round(0), [(round(b.centroid[0]), round(b.centroid[1]), round(b.area)) for _, _, b in first])
            if not first:
                continue
            saw_any = True
            views = [first]
            side = -1.0 if first[0][2].centroid[0] < img.shape[1] / 2 else 1.0
            best = None
            for dpan, dlift in ((10.0 * side, 0.0), (0.0, 9.0), (-10.0 * side, 0.0)):
                q = q1.copy()
                q[0] += math.radians(dpan)
                q[1] += math.radians(dlift)
                await self._safe_joints(q, 0.9, "measuring where it is")
                views.append(self._blob_rays(await self._fresh_frame("wrist", settle=0.45), color))
                hyp = self._hypotheses(views, cam_f)
                if hyp:
                    best = hyp[0]
                    log.info("locate %s: %d view(s): best %s support %d gap %.1f mm angle %.1f deg size %.0f mm (%d candidates)", color, len(views),
                             best[2].round(3), best[0], best[3] * 1000, best[4], best[5] * 1000, len(hyp))
                    if best[0] >= 1 and best[3] < 0.012:
                        break
                if len(views) == 3 and not hyp:
                    break  # two more looks and nothing within reach lines up: it is background, move on
            if best is not None and (best[0] >= 1 or len(views) >= 3):
                p = best[2]
                self.emit("markers", markers=[{"id": "target", "label": color, "pos": p.tolist(), "color": "#3ddc84", "radius": 0.014}])
                return p
        if saw_any:
            raise RobotError(f"I can see {color} things, but none of them within my reach (or it would not hold still long enough to measure). Bring it within about 35 cm and hold it still.")
        raise RobotError(f"looked all around and saw nothing {color}, on the table or in the air")

    async def fetch_air(self, color: str) -> dict:
        """Take something of this colour from wherever it is held: out of a hand, off the top of a box."""
        r = self.robot
        if r.holding:
            raise RobotError("already holding something; place it first")
        color = color.lower().strip()
        p = await self.locate_air(color)
        if p[2] < 0.05:
            # it is lying on the table: the table pick lines up on it from above and is more exact
            self._hint_color = color
            try:
                return {**await self.pick(x=float(p[0]), y=float(p[1])), "from": "the table"}
            finally:
                self._hint_color = None
        dist = math.hypot(p[0], p[1] - PROFILE.base_y)
        if p[2] < -0.02 or p[2] > 0.45 or dist > PROFILE.reach_m + 0.08:
            raise RobotError(f"it is at x={p[0]:+.2f} y={p[1]:+.2f} z={p[2]:.2f}, {dist * 100:.0f} cm out: beyond my reach. Bring it closer.")

        def plan(point):
            last = None
            for pitch in (20, 35, 5, 50, 65, 80):
                a = approach_from_pitch(point[:2], pitch)
                g = r.solve(point - a * 0.004, approach=a, q_seed=np.radians(READY_DEG), allow_low=point[2] < 0.05)
                if not g.ok:
                    last = g
                    continue
                st = r.solve(point - a * 0.085, approach=a, q_seed=g.q, allow_low=point[2] < 0.05)
                if st.ok:
                    return a, st.q, g.q
                last = st
            raise RobotError(f"it is at x={point[0]:+.2f} y={point[1]:+.2f} z={point[2]:.2f} but I can't get my gripper there ({last.describe() if last else 'no approach'})")

        a, q_stand, q_grasp = plan(p)
        await r.set_gripper(1.0, label="opening")
        await self._safe_joints(q_stand, 0.9, "reaching for it")
        # one more look from close up: it may have drifted, and near lines of sight are sharper
        nears = [(o + d * float((p - o) @ d)) for o, d, _ in self._blob_rays(await self._fresh_frame("wrist", settle=0.4), color, min_area=500.0)]
        if nears:
            # the point on the new line of sight nearest to the first estimate
            p2 = min(nears, key=lambda c: float(np.linalg.norm(c - p)))
            log.info("fetch_air: close-up puts it %.0f mm from the first estimate", float(np.linalg.norm(p2 - p)) * 1000)
            if np.linalg.norm(p2 - p) < 0.05:
                p = 0.3 * p + 0.7 * p2
                try:
                    a, _, q_grasp = plan(p)
                except RobotError:
                    pass
        await r.move_joints(q_grasp, speed=0.5, allow_low=p[2] < 0.05, label="taking it")
        g = await r.grasp()
        if not g["holding"]:
            await r.set_gripper(1.0, label="opening")
            await r.move_joints(q_stand, speed=0.7, check=False, label="backing off")
            raise RobotError(f"reached x={p[0]:+.2f} y={p[1]:+.2f} z={p[2]:.2f} and closed on nothing. Hold it still and I will try again.")
        await asyncio.sleep(0.6)  # give the hand a moment to let go
        await r.move_joints(q_stand, speed=0.6, check=False, label="taking it")
        if not r.still_holding():
            raise RobotError("had it, but it slipped out as I pulled back")
        await self.home(0.9)
        return {"holding": True, "width_mm": g["width_mm"], "picked_from": [round(float(v), 3) for v in p], "from": "the air"}

    # Overhand lob for the so101: cocked back above the base, whipped forward, let go on the way up.
    # (shoulder_lift, elbow_flex, wrist_flex) in degrees.
    THROW_WIND = (-96.0, -60.0, -91.0)
    THROW_END = (-10.0, 20.0, 0.0)

    THROW_STYLES = {"normal": 0.09, "high": 0.065, "flat": 0.125}  # when the jaws are told to open: later = flatter

    async def throw(self, color: str | None = None, pan: float | None = 0.0, elevation: float = 50.0, dry: bool = False,
                    style: str = "normal") -> dict:  # fmt: skip
        """Lob whatever is in the gripper. `pan` aims it (degrees, + is the robot's right; None = where the arm
        is facing now), `style` picks a high lob, the normal arc or a flatter pass. With `color`, fetch first."""
        r = self.robot
        if PROFILE.name != "so101":
            raise RobotError("throwing is only set up for the so101 arm")
        if pan is None:
            pan = math.degrees(float(r.q()[0]))  # before any fetch turns the arm away
        if color:
            await self.fetch(color)
        if not (r.holding or dry):
            raise RobotError("nothing in the gripper to throw; fetch something first")
        pan = max(-85.0, min(85.0, float(pan)))
        wind = np.radians([pan, *self.THROW_WIND, 0.0])
        end = np.radians([pan, *self.THROW_END, 0.0])
        await self._safe_joints(np.radians([pan, -45, -20, 30, 0]), 1.0, "winding up")
        await r.move_joints(wind, speed=0.7, check=False, label="winding up")
        await asyncio.sleep(0.25)
        release_at = self.cfg.throw_release_at + self.THROW_STYLES.get(style, 0.09) - 0.09
        out = await r.throw(end, release_at=release_at, pan=math.radians(pan), release=not dry)
        await self.home(1.0)
        return {**out, "pan": round(pan), "style": style}

    GIVE_PAN = {"left": -38.0, "center": 0.0, "right": 38.0}

    async def give(self, side: str = "center", wait: float = 7.0) -> str:
        """Hold the gripper out towards the person's hand and let go when they touch or tug it.

        If nobody touches it the object stays held: it is never dropped on the off chance.
        """
        r = self.robot
        was_holding = r.holding
        side = side if side in self.GIVE_PAN else "center"
        pan = math.radians(self.GIVE_PAN[side])
        sol = None
        for reach, pitch in ((0.31, 15), (0.28, 30), (0.25, 40)):
            sol = r.solve([reach * math.sin(pan), reach * math.cos(pan), 0.17], pitch=pitch)
            if sol.ok:
                break
        if not sol.ok:
            raise RobotError(f"can't reach the hand-over pose ({sol.describe()})")
        await self._safe_joints(sol.q, 1.0, "handing it over")
        where = f"held out to my {side}, about 30 cm from the base and 17 cm up"
        if not was_holding:
            # told to give while the jaws read empty: something light may still be stuck in them
            await r.set_gripper(1.0, label="handing it over")
            return f"my gripper read empty, but I {where} and opened the jaws anyway"
        touched = await self._wait_for_bump(wait)
        if not touched:
            return f"FAILED to hand over: {where} for {wait:.0f} s but nobody touched the gripper, so I am still holding it. Tell them to tap or pull it."
        await r.set_gripper(1.0, label="handing it over")
        await asyncio.sleep(0.4)
        await self.home(1.0)
        return f"{where}, felt their hand and let go"

    async def lineup(self, seconds: float = 25.0) -> str:
        """Big, slow, wavy sweeps through the air in front of the arm, for as long as asked.

        Whatever is in or on the gripper stays there, and the camera is not used: it is expected to be
        covered by whatever has been attached.
        """
        r = self.robot
        rng = np.random.default_rng()
        lo = np.array([-55.0, -58.0, -30.0, -15.0, -45.0])
        hi = np.array([55.0, -22.0, 18.0, 45.0, 40.0])
        t0, n, side = time.monotonic(), 0, 1.0
        seconds = max(6.0, min(60.0, float(seconds)))
        while time.monotonic() - t0 < seconds:
            q = lo + rng.random(5) * (hi - lo)
            q[0] = side * (15.0 + rng.random() * 40.0)  # swing from one side to the other each time
            side = -side
            try:
                await r.move_joints(np.radians(q), speed=0.38, label="lining you up")
                n += 1
            except Stopped:
                raise
            except RobotError:
                continue  # that pose was too low or folded: pick another
        await self.home(0.6)
        return f"did {n} big slow sweeps over {time.monotonic() - t0:.0f} s" + ("; still holding on" if r.holding else "")

    _search_i = 0

    async def search(self) -> str:
        """Turn the raised camera to the next direction to look around. Whatever is held stays held."""
        pans = (-45.0, 45.0, 0.0)
        pan = pans[self._search_i % len(pans)]
        self._search_i += 1
        q = np.radians([pan, -40.0, 20.0, 15.0, 0.0])  # head up, looking out level rather than down at the table
        await self._safe_joints(q, 1.0, "looking around")
        where = "my left" if pan < 0 else "my right" if pan > 0 else "straight ahead"
        return f"camera now looking out towards {where}" + ("; still holding the object" if self.robot.holding else "")

    TURNS = {"left": (0, -25.0), "right": (0, 25.0), "up": (3, -20.0), "down": (3, 20.0)}

    async def turn(self, direction: str, degrees: float | None = None) -> str:
        """Turn the head: left/right swings the base, up/down tilts the wrist camera. Holds on to what is held."""
        if direction not in self.TURNS:
            raise RobotError(f"turn takes left, right, up or down, not '{direction}'")
        r = self.robot
        j, step = self.TURNS[direction]
        if degrees:
            step = math.copysign(min(60.0, abs(float(degrees))), step)
        q = r.q().copy()
        lo, hi = r.kin.lower[j] + r.cfg.limit_margin, r.kin.upper[j] - r.cfg.limit_margin
        target = min(hi, max(lo, q[j] + math.radians(step)))
        if abs(target - q[j]) < math.radians(3):
            raise RobotError(f"already turned as far {direction} as I can go")
        moved = math.degrees(target - q[j])
        q[j] = target
        await r.move_joints(q, speed=1.0, check=j != 0, label=f"turning {direction}")
        return f"turned {direction} by {abs(moved):.0f} degrees" + ("; still holding the object" if r.holding else "")

    _hint_color: str | None = None
    _held_pitch: float | None = None

    def _inspect_q(self, target: np.ndarray, cam_height: float = 0.17) -> np.ndarray | None:
        """A pose that puts the wrist camera right above `target`, looking down at it."""
        r = self.robot
        Tc = self.cam.T_tool_cam()
        # tool pitch that makes the camera axis vertical
        tilt = math.degrees(math.atan2(Tc[1, 2], Tc[2, 2]))  # camera axis leans this far towards tool +y
        tcp = np.array([target[0], target[1], self.table_z + cam_height * 0.6])
        sol = None
        for pitch in (90 - tilt, 80 - tilt, 70 - tilt, 58 - tilt):
            for _ in range(4):
                sol = r.solve(tcp, pitch=pitch, yaw=0, q_seed=np.radians(READY_DEG))
                if not sol.ok:
                    break
                T = r.kin.fk(sol.q) @ Tc
                o, d = T[:3, 3], T[:3, 2]
                if d[2] > -0.3:
                    break
                hit = o + d * ((self.table_z - o[2]) / d[2])
                err = np.array([target[0] - hit[0], target[1] - hit[1], cam_height - (o[2] - self.table_z)])
                if np.linalg.norm(err[:2]) < 0.004 and abs(err[2]) < 0.02:
                    return sol.q
                tcp = tcp + err
            tcp = np.array([target[0], target[1], self.table_z + cam_height * 0.6])
        return None

    async def _inspect(self, target: np.ndarray, color: str | None = None, max_shift: float = 0.08) -> np.ndarray:
        """Move the camera over the target, find it again from close up, return its position."""
        q = self._inspect_q(target)
        if q is None:
            return target
        await self.robot.move_joints(q, speed=0.9, label="taking a closer look")
        img = await self._fresh_frame("wrist", settle=0.45)
        cam = self._cam(img)
        T = self.robot.tcp()
        color = color or self._hint_color
        blobs = self._color_blobs(img, color, 400) if color in COLOR_HSV else []
        if not blobs:
            blobs = vision.segment_objects(img, ignore=self._jaw_mask(img), min_area=500)
        expect = cam.project([target[0], target[1], self.table_z], T) or (img.shape[1] / 2, img.shape[0] / 2)
        b = vision.nearest_blob(blobs, *expect, max_dist=0.45 * img.shape[1])
        if b is None:
            return target
        p = cam.pixel_to_plane(*b.centroid, T, self.table_z + self.inspect_plane)
        if p is None or np.linalg.norm(p[:2] - target[:2]) > max_shift:
            return target
        self.emit("detections", camera="wrist", width=img.shape[1], height=img.shape[0],
                  items=[{"label": "target", "score": 1.0, "box": list(b.box), "point": [round(b.centroid[0]), round(b.centroid[1])]}])  # fmt: skip
        self.last_inspect = {"pixel": [round(b.centroid[0], 1), round(b.centroid[1], 1)], "xy": [round(float(p[0]), 4), round(float(p[1]), 4)]}
        return np.array([p[0], p[1], self.table_z])

    grasp_offset = (0.0, 0.0)  # measured correction (x, y) added to camera-derived grasp points
    inspect_plane = 0.02  # height of the surface the camera actually sees (top of a small object)
    last_inspect: dict | None = None

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
        if refine and PROFILE.name == "so101":
            # this arm's camera cannot see the grasp point from the hover pose: look first, then reach
            for _ in range(2):
                better = await self._inspect(target, s.color if s is not None else None)
                moved = float(np.linalg.norm(better[:2] - target[:2]))
                target = better
                if moved < 0.006:
                    break
            target = target + np.array([*self.grasp_offset, 0.0])
            refine = False
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
        await self._line_or_joint(grasp, a, 0.5, "descending")
        g = await r.grasp()
        if not g["holding"]:
            await r.set_gripper(1.0, label="opening")
            await self._line_or_joint(hover_pos, a, 0.7, "backing off")
            raise RobotError(
                f"closed on nothing at x={target[0]:+.3f} y={target[1]:+.3f}: the jaws shut completely. "
                "The object is probably a little off from there; scan() again and retry, or adjust x/y."
            )
        await self._line_or_joint(grasp + np.array([0, 0, 0.09]), a, 0.6, "lifting")
        await asyncio.sleep(0.25)
        if not r.still_holding():
            raise RobotError("had it, but it slipped out while lifting. scan() to see where it ended up.")
        self._held_pitch = pitch
        return {"holding": True, "width_mm": g["width_mm"], "picked_from": [round(float(v), 3) for v in target[:2]], "pitch": pitch}

    async def _line_or_joint(self, pos, approach, speed: float, label: str) -> None:
        """Straight-line move, or failing that a plain joint move to the same point.

        A 5-joint arm cannot always keep its approach direction along a line ("would have to flip
        configuration"); the point is still reachable, just not along that exact line.
        """
        r = self.robot
        try:
            await r.move_to(pos, approach=approach, linear=True, speed=speed, allow_low=True, label=label)
            return
        except Stopped:
            raise
        except RobotError as e:
            if "straight" not in str(e):
                raise
        sol = r.solve(pos, approach=approach, allow_low=True)
        if not sol.ok:
            sol, _ = r.solve_reach(pos, allow_low=True)
        if not sol.ok:
            raise RobotError(r._unreachable(np.asarray(pos, dtype=float), sol))
        await r.move_joints(sol.q, speed=speed * 0.8, allow_low=True, label=label)

    async def place(self, x: float, y: float, z: float = 0.0, yaw: float | None = None, pitch: float | None = None) -> dict:
        """Put down whatever is held at a table position (z = height of the surface there)."""
        r = self.robot
        target = np.array([x, y, self.table_z + z])
        self.emit("markers", markers=[{"id": "target", "label": "place", "pos": target.tolist(), "color": "#5ab0ff", "radius": 0.014}])
        a, pitch_used, q_hover, hover_pos, drop = self._approach_for(target, z + 0.022, yaw, prefer_pitch=pitch if pitch is not None else (self._held_pitch if r.holding else None))
        # onto something tall: go up first so the carry does not sweep through it
        rise = float(hover_pos[2] - r.tcp()[2, 3]) + 0.03
        if z > 0.03 and rise > 0.02:
            try:
                await r.move_relative([0, 0, rise], speed=0.7, label="lifting higher")
            except RobotError:
                await r.move_joints(np.radians(_UP if PROFILE.name == "so101" else READY_DEG), speed=0.7, label="lifting higher")
        await r.move_joints(q_hover, speed=0.7, label="carrying")
        await r.move_to(drop, approach=a, linear=True, speed=0.4, allow_low=True, label="lowering")
        was_holding = r.holding
        await r.set_gripper(1.0, label="releasing")
        await r.move_to(hover_pos, approach=a, linear=True, speed=0.7, allow_low=True, label="backing off")
        self._held_pitch = None
        return {"placed_at": [round(x, 3), round(y, 3)], "released_something": was_holding, "pitch": pitch_used}

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

    async def gesture(self, name: str, side: str = "center") -> str:
        name = name.lower().strip()
        fn = getattr(self, f"_g_{name}", None)
        self._side = side if side in self.GIVE_PAN else "center"  # where the person is, for dap and handshake
        if name not in GESTURES:
            raise RobotError(f"unknown gesture '{name}'. Available: {', '.join(GESTURES)}")
        if self.robot.holding and name in ("dap", "handshake", "dance", "celebrate"):
            raise RobotError("holding something; put it down before that")
        if PROFILE.name == "so101" and name in SO101_GESTURES:
            return await self._play(name, SO101_GESTURES[name])
        if fn is None:
            raise RobotError(f"gesture '{name}' is not available on this arm")
        return await fn()

    async def _play(self, name: str, steps: list) -> str:
        """Play a gesture written as data: ("q", [deg...], speed) | ("grip", open) | ("wait", s) | ("home", speed)."""
        r = self.robot
        for st in steps:
            if st[0] == "q":
                await r.move_joints(np.radians(st[1]), speed=st[2], label=name)
            elif st[0] == "grip":
                if not r.holding:
                    await r.set_gripper(st[1], label=name)
            elif st[0] == "wait":
                await asyncio.sleep(st[1])
            elif st[0] == "home":
                await self.home(st[1])
        return f"did the {name} gesture"

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

    _side = "center"

    def _reach_out_q(self, reach: float = 0.33, height: float = 0.21) -> np.ndarray:
        """Hand held out level towards the person on `self._side`."""
        r = self.robot
        pan = math.radians(self.GIVE_PAN[self._side])
        for rr, pitch in ((reach, 0), (reach - 0.03, 5), (reach - 0.06, 10)):
            sol = r.solve([rr * math.sin(pan), rr * math.cos(pan), height], pitch=pitch, q_seed=np.radians([math.degrees(pan), -20, 15, 5, 0]))
            if sol.ok:
                return sol.q
        raise RobotError(f"can't reach out that far to my {self._side} ({sol.describe()})")

    async def _g_dap(self) -> str:
        """Fist bump: hold a closed fist out towards the person and wait to be bumped."""
        r = self.robot
        await r.set_gripper(0.0, label="dap")
        await self._safe_joints(self._reach_out_q(), 1.3, "dap")
        bumped = await self._wait_for_bump(6.0)
        if bumped:
            # recoil and "blow it up"
            q = r.q().copy()
            q[1] -= math.radians(10)
            await r.move_joints(q, speed=1.4, label="dap")
            await r.set_gripper(1.0, label="dap")
        await self.home(1.2)
        return "got the fist bump" if bumped else "held my fist out but nobody bumped it"

    async def _g_handshake(self) -> str:
        """Hold the open hand out, wait for the person to take it, then shake up and down."""
        r = self.robot
        await r.set_gripper(0.55, label="handshake")
        q0 = self._reach_out_q(reach=0.31, height=0.19)
        await self._safe_joints(q0, 1.2, "handshake")
        taken = await self._wait_for_bump(7.0)
        if taken:
            for _ in range(3):
                for d in (-9.0, 9.0):
                    q = q0.copy()
                    q[3] += math.radians(d)
                    q[2] += math.radians(d * 0.6)
                    await r.move_joints(q, speed=1.8, label="handshake")
            await r.move_joints(q0, speed=1.2, label="handshake")
            await asyncio.sleep(0.3)
        await r.set_gripper(1.0, label="handshake")
        await self.home(1.1)
        return "shook hands" if taken else "held my hand out but nobody took it"

    async def take(self, side: str = "center", wait: float = 9.0) -> str:
        """Hold the open gripper out; when the person puts something between the jaws and taps, close on it."""
        r = self.robot
        if r.holding:
            raise RobotError("already holding something; give or place it first")
        self._side = side if side in self.GIVE_PAN else "center"
        await r.set_gripper(1.0, label="reaching for it")
        await self._safe_joints(self._reach_out_q(reach=0.31, height=0.18), 1.2, "reaching for it")
        if not await self._wait_for_bump(wait):
            await self.home(1.0)
            raise RobotError(f"held my open gripper out to my {self._side} for {wait:.0f} s but felt nothing put in it. Tell them to push it between the jaws.")
        await asyncio.sleep(0.25)
        g = await r.grasp()
        if not g["holding"]:
            await r.set_gripper(1.0, label="opening")
            await self.home(1.0)
            raise RobotError("felt a touch and closed, but there was nothing between the jaws")
        await self.home(1.0)
        return f"took it from their hand, holding it ({g['width_mm']:.0f} mm between the jaws)"

    async def catch(self, side: str = "center", wait: float = 10.0) -> str:
        """Goalkeeper: open jaws on the table facing the person, snap shut when something rolls in between them."""
        import cv2

        r = self.robot
        if r.holding:
            raise RobotError("already holding something; give or place it first")
        self._side = side if side in self.GIVE_PAN else "center"
        pan = math.radians(self.GIVE_PAN[self._side] * 0.6)
        await r.set_gripper(1.0, label="getting ready to catch")
        sol = None
        for z in (0.022, 0.03, 0.04):
            for reach, pitch in ((0.24, 50), (0.22, 60), (0.26, 45), (0.22, 75), (0.25, 35)):
                sol = r.solve([reach * math.sin(pan), reach * math.cos(pan), z], pitch=pitch, allow_low=True)
                if sol.ok:
                    break
            if sol.ok:
                break
        if not sol.ok:
            raise RobotError(f"can't get the gripper down to the table there ({sol.describe()})")
        await self._safe_joints(np.radians(READY_DEG), 1.0, "getting ready to catch")
        await r.move_joints(sol.q, speed=0.9, allow_low=True, label="getting ready to catch")

        def gap(img: np.ndarray) -> np.ndarray:
            # the strip of table between the two jaws, low in the wrist picture
            h, w = img.shape[:2]
            return cv2.GaussianBlur(cv2.cvtColor(img[int(0.62 * h) : int(0.97 * h), int(0.36 * w) : int(0.64 * w)], cv2.COLOR_BGR2GRAY), (9, 9), 0).astype(np.float32)

        await asyncio.sleep(0.5)
        base = gap(self._frame("wrist")[0])
        t0, hits = time.monotonic(), 0
        r.activity = "waiting to catch"
        try:
            while time.monotonic() - t0 < wait:
                if r._stop_evt.is_set():
                    raise Stopped("stopped")
                await asyncio.sleep(0.03)
                changed = float(np.mean(np.abs(gap(self._frame("wrist")[0]) - base) > 28))
                hits = hits + 1 if changed > 0.12 else 0
                if hits >= 2:
                    break
            else:
                await self.home(1.0)
                raise RobotError(f"waited {wait:.0f} s with my jaws open on the table and nothing rolled in")
        finally:
            r.activity = "idle"
        g = await r.grasp()
        if not g["holding"]:
            await r.set_gripper(1.0, label="opening")
            await self.home(1.0)
            raise RobotError("something came by and I snapped shut, but I missed it")
        await self.home(1.0)
        return f"caught it ({g['width_mm']:.0f} mm between the jaws)"

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


# Gestures for the SO-101, as absolute joint angles in degrees: [pan, lift, elbow, wrist, roll].
# The gripper is the "head" (the camera is its eye): wrist = nod, roll = head tilt, pan = turn.
# lift/elbow/wrist: positive bends down and forward. Roll range is -157..+56.
_UP = [0, -35, 5, 30, 0]  # head up, looking at the person across the table
SO101_GESTURES: dict[str, list] = {
    "nod": [("q", _UP, 1.2), ("q", [0, -35, 5, 55, 0], 1.6), ("q", [0, -35, 5, 12, 0], 1.6), ("q", [0, -35, 5, 55, 0], 1.6), ("q", _UP, 1.6), ("home", 1.0)],
    "shake": [("q", _UP, 1.2), ("q", [22, -35, 5, 30, 0], 1.8), ("q", [-22, -35, 5, 30, 0], 1.8), ("q", [22, -35, 5, 30, 0], 1.8), ("q", [-22, -35, 5, 30, 0], 1.8), ("q", _UP, 1.5), ("home", 1.0)],
    "wave": [("q", [0, -40, -45, 0, 0], 1.2), ("grip", 1.0),
             ("q", [0, -40, -45, 0, 40], 2.0), ("q", [0, -40, -45, 0, -45], 2.0), ("q", [0, -40, -45, 0, 40], 2.0), ("q", [0, -40, -45, 0, -45], 2.0),
             ("q", [0, -40, -45, 0, 0], 1.6), ("home", 1.0)],
    "bow": [("q", _UP, 1.0), ("q", [0, -8, 15, 55, 0], 0.6), ("wait", 0.7), ("q", _UP, 0.7), ("home", 0.9)],
    "shrug": [("q", _UP, 1.2), ("grip", 1.0), ("q", [0, -55, -5, 45, -35], 1.6), ("wait", 0.35), ("q", _UP, 1.2), ("home", 1.0)],
    "peek": [("q", _UP, 1.0), ("q", [45, -35, 5, 30, 0], 1.0), ("wait", 0.5), ("q", [-45, -35, 5, 30, 0], 1.0), ("wait", 0.5), ("q", _UP, 1.0), ("home", 1.0)],
    "curious": [("q", _UP, 1.0), ("q", [0, -15, 0, 25, 40], 0.9), ("wait", 0.8), ("q", [0, -15, 0, 25, -40], 0.9), ("wait", 0.6), ("q", _UP, 1.0), ("home", 1.0)],
    "happy": [("q", _UP, 1.4), ("q", [0, -45, -10, 25, 35], 2.0), ("q", [0, -30, 10, 40, -35], 2.0), ("q", [0, -45, -10, 25, 35], 2.0), ("q", [0, -30, 10, 40, -35], 2.0),
              ("grip", 0.2), ("grip", 1.0), ("q", _UP, 1.4), ("home", 1.0)],
    "sad": [("q", _UP, 0.8), ("q", [0, -28, 45, 78, 0], 0.35), ("wait", 0.8), ("q", [10, -28, 45, 78, 0], 0.4), ("q", [-10, -28, 45, 78, 0], 0.4), ("wait", 0.4), ("home", 0.6)],
    "oops": [("q", [0, -50, 0, 20, 0], 1.8), ("q", [0, -20, 30, 80, 0], 0.5), ("q", [14, -20, 30, 80, 0], 1.4), ("q", [-14, -20, 30, 80, 0], 1.4), ("q", [0, -20, 30, 80, 0], 1.2), ("home", 0.9)],
    "celebrate": [("q", [0, -40, -45, 0, 0], 1.6), ("grip", 0.1), ("grip", 1.0), ("q", [30, -40, -45, 0, 40], 2.0), ("q", [-30, -40, -45, 0, -45], 2.0),
                  ("q", [30, -40, -45, 0, 40], 2.0), ("q", [0, -40, -45, 0, 0], 1.8), ("grip", 0.1), ("grip", 1.0), ("home", 1.1)],
    "dance": [("q", _UP, 1.4),
              ("q", [30, -45, -10, 20, 40], 1.9), ("grip", 0.2), ("q", [-30, -25, 20, 50, -45], 1.9), ("grip", 1.0),
              ("q", [30, -25, 20, 50, 40], 1.9), ("grip", 0.2), ("q", [-30, -45, -10, 20, -45], 1.9), ("grip", 1.0),
              ("q", [0, -40, -45, 0, 0], 1.8), ("q", [0, -40, -45, 0, 45], 2.2), ("q", [0, -40, -45, 0, -50], 2.2), ("q", [0, -40, -45, 0, 0], 2.2),
              ("q", [25, -20, 25, 60, 0], 1.8), ("q", [-25, -20, 25, 60, 0], 1.8), ("q", _UP, 1.6), ("home", 1.1)],
}


# HSV ranges (OpenCV: H 0..179) for colours an agent can ask for by name.
COLOR_HSV: dict[str, list] = {
    "blue": [((96, 110, 50), (130, 255, 255))],
    "green": [((38, 80, 50), (88, 255, 255))],
    "yellow": [((20, 110, 110), (34, 255, 255))],
    "orange": [((8, 140, 120), (19, 255, 255))],
    "red": [((0, 140, 80), (7, 255, 255)), ((172, 140, 80), (179, 255, 255))],
    "pink": [((150, 70, 120), (171, 255, 255))],
    "purple": [((131, 70, 50), (149, 255, 255))],
}


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
