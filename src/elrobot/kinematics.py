"""Forward and inverse kinematics for the ElRobot arm, built straight from its URDF.

Frames
------
Base frame is the URDF base frame: origin on the table under the base, x right, y forward,
z up (seen from behind the robot). The 7 arm joints are servo ids 1..7; servo 8 is the gripper
and does not affect the tool pose.

Tool frame (TCP) sits between the finger pads:
  z_tool = approach direction (the way the gripper points),
  x_tool = jaw closing axis (the fingers slide along it),
  y_tool = z_tool x x_tool.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from .profile import ACTIVE as PROFILE

ARM_JOINT_NAMES = list(PROFILE.arm_joint_names)
GRIPPER_JOINT_NAME = "rev_motor_08"
TIP_LINK = PROFILE.tip_link

# Measured from the jaw meshes (see docs/NOTES.md): in the Gripper_Base frame the finger pads
# run from y=0.045 to y=0.1035, centred at x=0.0017, z=0. The TCP sits 12 mm back from the tips.
FINGERTIP_Y = 0.1035
TCP_BACKOFF = 0.012
JAW_CENTER_X = 0.0017
# Full jaw opening between the pads (27 mm travel per jaw).
GRIPPER_MAX_WIDTH = 0.0536


def _rpy_matrix(rpy: tuple[float, float, float]) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _transform(xyz, rpy=(0.0, 0.0, 0.0)) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy_matrix(tuple(rpy))
    T[:3, 3] = xyz
    return T


def axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation; `axis` must be unit length."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ]
    )


def quat_from_matrix(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> quaternion [x, y, z, w]."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("zero-length vector")
    return v / n


@dataclass
class Joint:
    id: int
    name: str
    pre: np.ndarray  # fixed transform from the previous joint's moving frame to this joint's frame
    axis: np.ndarray  # unit axis in this joint's frame
    lower: float
    upper: float


@dataclass
class IKResult:
    q: np.ndarray
    ok: bool
    pos_err: float  # metres
    ang_err: float  # radians (0 when no orientation was requested)
    reason: str = ""
    tried: int = 0

    def describe(self) -> str:
        return f"pos_err={self.pos_err * 1000:.1f}mm ang_err={math.degrees(self.ang_err):.1f}deg {self.reason}".strip()


@dataclass
class ArmGeometry:
    """Points used for clearance checks, all in the base frame."""

    joints: np.ndarray  # (7, 3) joint origins
    wrist: np.ndarray  # gripper base origin
    tcp: np.ndarray
    tips: np.ndarray  # (2, 3) the two fingertips (at full opening)
    extras: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))  # e.g. camera bump

    def all_points(self) -> np.ndarray:
        return np.vstack([self.joints, self.wrist[None], self.tcp[None], self.tips, self.extras])


class Kinematics:
    def __init__(self, urdf_path: str | Path):
        self.urdf_path = Path(urdf_path)
        root = ET.parse(self.urdf_path).getroot()
        by_child = {j.find("child").get("link"): j for j in root.findall("joint")}

        # Walk from the tip link back to the base to get the serial chain.
        chain = []
        link = TIP_LINK
        while link in by_child:
            j = by_child[link]
            chain.append(j)
            link = j.find("parent").get("link")
        chain.reverse()
        self.base_link = link

        self.joints: list[Joint] = []
        pending = np.eye(4)
        for j in chain:
            o = j.find("origin")
            xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
            rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
            pending = pending @ _transform(xyz, rpy)
            if j.get("type") == "fixed":
                continue
            if j.get("type") != "revolute":
                raise ValueError(f"unexpected joint type in arm chain: {j.get('name')} {j.get('type')}")
            lim = j.find("limit")
            self.joints.append(
                Joint(
                    id=len(self.joints) + 1,
                    name=j.get("name"),
                    pre=pending,
                    axis=unit([float(v) for v in j.find("axis").get("xyz").split()]),
                    lower=float(lim.get("lower")),
                    upper=float(lim.get("upper")),
                )
            )
            pending = np.eye(4)
        if [j.name for j in self.joints] != ARM_JOINT_NAMES:
            raise ValueError(f"unexpected arm chain: {[j.name for j in self.joints]}")
        self.tip_post = pending  # fixed transform after the last joint (identity for this URDF)

        grip = next(j for j in root.findall("joint") if j.get("name") == GRIPPER_JOINT_NAME)
        glim = grip.find("limit")
        self.gripper_lower = float(glim.get("lower"))
        self.gripper_upper = float(glim.get("upper"))

        self.lower = np.array([j.lower for j in self.joints])
        self.upper = np.array([j.upper for j in self.joints])
        self.n = len(self.joints)

        # Tool frame in the Gripper_Base frame. Approach is the wrist-roll axis pointing out of the
        # gripper; the jaw axis comes from the prismatic jaw joints. Both are a degree or so off the
        # link's own axes, so take them from the URDF rather than assuming.
        approach = unit(self.tip_post[:3, :3].T @ -self.joints[-1].axis)
        jaw_joint = next(j for j in root.findall("joint") if j.get("name") == "rev_motor_08_1")
        jaw = -unit([float(v) for v in jaw_joint.find("axis").get("xyz").split()])
        jaw = unit(jaw - approach * float(jaw @ approach))
        R_tool = np.column_stack([jaw, np.cross(approach, jaw), approach])
        self.T_tip_tool = np.eye(4)
        self.T_tip_tool[:3, :3] = R_tool
        self.T_tip_tool[:3, 3] = np.array([JAW_CENTER_X, 0.0, 0.0]) + approach * (FINGERTIP_Y - TCP_BACKOFF)
        half = GRIPPER_MAX_WIDTH / 2
        self._tips_tool = np.array([[half, 0, TCP_BACKOFF], [-half, 0, TCP_BACKOFF]])
        # Extra body points (tool frame) included in clearance checks, e.g. the wrist camera.
        self.extra_points_tool = np.zeros((0, 3))
        # Wrist camera pose in the tool frame, when the URDF has an optical frame for it.
        self.T_tool_cam: np.ndarray | None = None
        if PROFILE.camera_link:
            T = np.eye(4)
            link = PROFILE.camera_link
            while link != TIP_LINK:
                j = by_child[link]
                o = j.find("origin")
                T = _transform([float(v) for v in o.get("xyz", "0 0 0").split()], [float(v) for v in o.get("rpy", "0 0 0").split()]) @ T
                link = j.find("parent").get("link")
            self.T_tool_cam = np.linalg.inv(self.T_tip_tool) @ T
            # the camera body sticks out of the gripper: keep it off the table too
            self.extra_points_tool = np.array([self.T_tool_cam[:3, 3]])

    # ------------------------------------------------------------------ forward

    def joint_frames(self, q) -> list[np.ndarray]:
        """World transform of each joint's moving frame (after its rotation)."""
        T = np.eye(4)
        out = []
        for j, qi in zip(self.joints, q):
            T = T @ j.pre
            R = np.eye(4)
            R[:3, :3] = axis_angle_matrix(j.axis, float(qi))
            T = T @ R
            out.append(T)
        return out

    def fk_tip(self, q) -> np.ndarray:
        """Pose of the Gripper_Base link."""
        return self.joint_frames(q)[-1] @ self.tip_post

    def fk(self, q) -> np.ndarray:
        """Pose of the tool frame (TCP) as a 4x4 matrix in the base frame."""
        return self.fk_tip(q) @ self.T_tip_tool

    def geometry(self, q) -> ArmGeometry:
        frames = self.joint_frames(q)
        T_tip = frames[-1] @ self.tip_post
        T_tool = T_tip @ self.T_tip_tool
        tips = (T_tool[:3, :3] @ self._tips_tool.T).T + T_tool[:3, 3]
        extras = (
            (T_tool[:3, :3] @ self.extra_points_tool.T).T + T_tool[:3, 3]
            if len(self.extra_points_tool)
            else np.zeros((0, 3))
        )
        return ArmGeometry(
            joints=np.array([F[:3, 3] for F in frames]),
            wrist=T_tip[:3, 3].copy(),
            tcp=T_tool[:3, 3].copy(),
            tips=tips,
            extras=extras,
        )

    def _fk_and_axes(self, q):
        """TCP pose plus each joint's world axis and origin, for Jacobians."""
        T = np.eye(4)
        axes = np.zeros((self.n, 3))
        origins = np.zeros((self.n, 3))
        for i, (j, qi) in enumerate(zip(self.joints, q)):
            T = T @ j.pre
            axes[i] = T[:3, :3] @ j.axis
            origins[i] = T[:3, 3]
            R = np.eye(4)
            R[:3, :3] = axis_angle_matrix(j.axis, float(qi))
            T = T @ R
        return T @ self.tip_post @ self.T_tip_tool, axes, origins

    def jacobian(self, q) -> np.ndarray:
        """6x7 geometric Jacobian of the TCP (linear rows first) in the base frame."""
        T, axes, origins = self._fk_and_axes(q)
        p = T[:3, 3]
        J = np.zeros((6, self.n))
        for i in range(self.n):
            J[:3, i] = np.cross(axes[i], p - origins[i])
            J[3:, i] = axes[i]
        return J

    # ------------------------------------------------------------------ clearance

    def body_segments(self, q) -> list[tuple[np.ndarray, np.ndarray, float]]:
        """The arm as capsules (start, end, radius), base first."""
        g = self.geometry(q)
        j = g.joints
        tip_mid = g.tips.mean(axis=0)
        base_top = j[1].copy()
        base_bottom = np.array([j[0][0], j[0][1], 0.0])
        if self.n == 5:
            return [
                (base_bottom, base_top, 0.045),  # base + shoulder servos
                (j[1], j[2], 0.025),  # upper arm
                (j[2], j[3], 0.025),  # forearm
                (j[3], g.wrist, 0.025),  # wrist
                (g.wrist, tip_mid, 0.034),  # gripper
            ]
        return [
            (base_bottom, base_top, 0.045),  # base + shoulder servos
            (j[1], j[2], 0.028),  # upper arm
            (j[2], j[3], 0.028),  # elbow bracket
            (j[3], j[5], 0.028),  # forearm
            (j[5], g.wrist, 0.028),  # wrist
            (g.wrist, tip_mid, 0.036),  # gripper
        ]

    # Body-part pairs that can actually meet. Neighbours, and parts separated by one short
    # link, always sit within their radii of each other and are excluded.
    _COLLISION_PAIRS = ((0, 3), (0, 4), (0, 5), (1, 4), (1, 5), (2, 5))
    _PART_NAMES = ("base", "upper arm", "elbow", "forearm", "wrist", "gripper")

    # the wrist sits right above the base whenever the arm is folded, so that pair is not checked
    _COLLISION_PAIRS_5 = ((0, 4), (1, 4))
    _PART_NAMES_5 = ("base", "upper arm", "forearm", "wrist", "gripper")

    def self_collision(self, q, margin: float = 0.0) -> str | None:
        """Name the first pair of body parts that overlap, or None."""
        segs = self.body_segments(q)
        if self.n == 5:
            for a, b in self._COLLISION_PAIRS_5:
                if segment_distance(segs[a][0], segs[a][1], segs[b][0], segs[b][1]) < segs[a][2] + segs[b][2] + margin:
                    return f"{self._PART_NAMES_5[b]} would hit the {self._PART_NAMES_5[a]}"
            return None
        for a, b in self._COLLISION_PAIRS:
            pa, qa, ra = segs[a]
            pb, qb, rb = segs[b]
            if segment_distance(pa, qa, pb, qb) < ra + rb + margin:
                return f"{self._PART_NAMES[b]} would hit the {self._PART_NAMES[a]}"
        return None

    # ------------------------------------------------------------------ limits

    def set_limits(self, lower, upper) -> None:
        """Narrow the joint limits to what the calibrated servos can actually reach."""
        self.lower = np.asarray(lower, dtype=float).copy()
        self.upper = np.asarray(upper, dtype=float).copy()

    def clamp(self, q, margin: float = 0.0) -> np.ndarray:
        return np.clip(np.asarray(q, dtype=float), self.lower + margin, self.upper - margin)

    def within_limits(self, q, margin: float = 0.0) -> bool:
        q = np.asarray(q)
        return bool(np.all(q >= self.lower + margin - 1e-9) and np.all(q <= self.upper - margin + 1e-9))

    # ------------------------------------------------------------------ inverse

    def ik(
        self,
        pos,
        approach=None,
        jaw=None,
        *,
        q_seed=None,
        q_ref=None,
        extra_seeds=(),
        limit_margin: float = 0.05,
        pos_tol: float = 0.003,
        ang_tol: float = math.radians(PROFILE.ik_ang_tol_deg),
        approach_weight: float = PROFILE.ik_approach_weight,
        ref_weight: float = 0.01,
        random_seeds: int = 8,
        accept=None,
        rng: np.random.Generator | None = None,
    ) -> IKResult:
        """Solve for joint angles that put the TCP at `pos`.

        approach: optional unit vector the gripper should point along (5-DoF constraint).
        jaw:      optional unit vector for the jaw closing axis (adds the 6th constraint). The
                  gripper is symmetric, so `jaw` and `-jaw` are the same grasp; both are tried.
        q_seed:   start here first, and prefer solutions near it (pass the current pose).
        q_ref:    posture the null space is pulled towards (defaults to q_seed).
        accept:   optional callable(q) -> bool to reject solutions (e.g. collision checks).

        Weights are in metres-per-radian terms: approach_weight=0.15 means a 1 deg pointing error
        costs as much as a 2.6 mm position error.
        """
        pos = np.asarray(pos, dtype=float)
        a_t = unit(approach) if approach is not None else None
        jaw_targets = [None]
        if jaw is not None:
            if a_t is None:
                raise ValueError("jaw direction needs an approach direction")
            j_t = np.asarray(jaw, dtype=float)
            j_t = unit(j_t - a_t * float(j_t @ a_t))
            jaw_targets = [j_t, -j_t]

        lo = self.lower + limit_margin
        hi = self.upper - limit_margin
        mid = 0.5 * (lo + hi)
        q_seed = self.clamp(q_seed, limit_margin) if q_seed is not None else mid.copy()
        q_ref = self.clamp(q_ref, limit_margin) if q_ref is not None else q_seed
        rng = rng or np.random.default_rng(0)

        seeds = [q_seed] + [self.clamp(s, limit_margin) for s in extra_seeds]
        seeds += [rng.uniform(lo, hi) for _ in range(random_seeds)]

        def solve(q0, j_t):
            def make(w_ref):
                def residual(q):
                    T, _, _ = self._fk_and_axes(q)
                    r = [T[:3, 3] - pos]
                    if a_t is not None:
                        r.append(approach_weight * (T[:3, 2] - a_t))
                    if j_t is not None:
                        r.append(approach_weight * (T[:3, 0] - j_t))
                    r.append(w_ref * (q - q_ref))
                    return np.concatenate(r)

                def jac(q):
                    T, axes, origins = self._fk_and_axes(q)
                    p = T[:3, 3]
                    rows = [np.cross(axes, p - origins).T]
                    if a_t is not None:
                        rows.append(approach_weight * np.cross(axes, T[:3, 2]).T)
                    if j_t is not None:
                        rows.append(approach_weight * np.cross(axes, T[:3, 0]).T)
                    rows.append(w_ref * np.eye(self.n))
                    return np.vstack(rows)

                return residual, jac

            # Stage 1 picks a posture: the pull towards q_ref decides between the many joint
            # solutions a 7-joint arm has. That pull also drags the tool a few millimetres off
            # target, so stage 2 repeats the solve from there with the pull almost removed.
            res, jac = make(ref_weight)
            q1 = least_squares(res, q0, jac=jac, bounds=(lo, hi), method="trf", xtol=1e-9, ftol=1e-10, gtol=1e-10, max_nfev=60).x
            res, jac = make(ref_weight * 0.02)
            return least_squares(res, np.clip(q1, lo + 1e-9, hi - 1e-9), jac=jac, bounds=(lo, hi), method="trf", xtol=1e-11, ftol=1e-13, gtol=1e-13, max_nfev=40).x

        def errors(q, j_t):
            T = self.fk(q)
            pe = float(np.linalg.norm(T[:3, 3] - pos))
            ae = 0.0
            if a_t is not None:
                ae = math.acos(float(np.clip(T[:3, 2] @ a_t, -1.0, 1.0)))
            if j_t is not None:
                ae = max(ae, math.acos(float(np.clip(T[:3, 0] @ j_t, -1.0, 1.0))))
            return pe, ae

        best_ok: tuple[float, np.ndarray, float, float] | None = None
        best_any: tuple[float, np.ndarray, float, float] | None = None
        tried = 0
        rejected = 0
        for si, q0 in enumerate(seeds):
            for j_t in jaw_targets:
                tried += 1
                q = solve(q0, j_t)
                pe, ae = errors(q, j_t)
                score_any = pe + approach_weight * ae
                if best_any is None or score_any < best_any[0]:
                    best_any = (score_any, q, pe, ae)
                if pe <= pos_tol and ae <= ang_tol:
                    if accept is not None and not accept(q):
                        rejected += 1
                        continue
                    # Among valid solutions prefer the one that moves the joints least.
                    cost = float(np.max(np.abs(q - q_seed)))
                    if best_ok is None or cost < best_ok[0]:
                        best_ok = (cost, q, pe, ae)
            # The first seed is the current pose: if it converged, later seeds rarely move less.
            if best_ok is not None and si >= 2:
                break

        if best_ok is not None:
            _, q, pe, ae = best_ok
            return IKResult(q=q, ok=True, pos_err=pe, ang_err=ae, tried=tried)
        _, q, pe, ae = best_any
        reason = "no solution within tolerance"
        if rejected:
            reason = f"{rejected} solution(s) rejected by clearance checks"
        return IKResult(q=q, ok=False, pos_err=pe, ang_err=ae, reason=reason, tried=tried)


def segment_distance(p1, q1, p2, q2) -> float:
    """Shortest distance between segments p1-q1 and p2-q2."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    if a <= 1e-12 and e <= 1e-12:
        return float(np.linalg.norm(r))
    if a <= 1e-12:
        s, t = 0.0, min(1.0, max(0.0, f / e))
    else:
        c = float(d1 @ r)
        if e <= 1e-12:
            t, s = 0.0, min(1.0, max(0.0, -c / a))
        else:
            b = float(d1 @ d2)
            den = a * e - b * b
            s = min(1.0, max(0.0, (b * f - c * e) / den)) if den > 1e-12 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, min(1.0, max(0.0, -c / a))
            elif t > 1.0:
                t, s = 1.0, min(1.0, max(0.0, (b - c) / a))
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


def approach_from_pitch(target_xy, pitch_deg: float) -> np.ndarray:
    """Approach vector for a target at horizontal position `target_xy`.

    pitch_deg = 90 points straight down; 0 points horizontally away from the base towards the
    target; values in between tilt the gripper so it reaches out and down.
    """
    p = math.radians(pitch_deg)
    xy = np.asarray(target_xy, dtype=float)[:2]
    n = np.linalg.norm(xy)
    radial = xy / n if n > 1e-6 else np.array([0.0, 1.0])
    return np.array([radial[0] * math.cos(p), radial[1] * math.cos(p), -math.sin(p)])


def jaw_from_yaw(approach, yaw_deg: float) -> np.ndarray:
    """Jaw closing axis for a given approach, rotated `yaw_deg` about the approach direction.

    yaw 0 puts the jaw axis horizontal (fingers left/right of the target as seen along the
    approach), which is the natural grasp for something lying on the table.
    """
    a = unit(approach)
    up = np.array([0.0, 0.0, 1.0])
    ref = np.cross(up, a)
    if np.linalg.norm(ref) < 1e-6:  # approach is vertical: take "horizontal" as the base x axis
        ref = np.array([1.0, 0.0, 0.0])
    ref = unit(ref)
    return axis_angle_matrix(a, math.radians(yaw_deg)) @ ref
