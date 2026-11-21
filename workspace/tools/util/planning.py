"""From a tool-point pose to joint angles: inverse kinematics, straight lines, grasp approaches.

The robot takes joint angles only, so everything Cartesian is worked out here, in the
workspace, and can be read and changed like any other tool code. The mathematics underneath
(forward kinematics from robot_config/model.urdf, IK solved by pink) is util/kinematics.py.

Frames: metres in the base frame, origin on the table under the base, x to the robot's right,
y forward, z up. The tool point (TCP) is between the finger pads. `pitch` is how far below
horizontal the gripper points (90 = straight down); `yaw` turns the jaws about that direction.

The robot side checks every joint command against its own copy of the clearance envelope used
here, so a plan that slips through this file is refused there, not executed.
"""

import math
from pathlib import Path

import numpy as np

from util.arm import Arm, RobotError
from util.kinematics import IKResult, Kinematics, approach_from_pitch, jaw_from_yaw
from util.motion import READY_DEG

ROOT = Path(__file__).resolve().parents[2]


class Planner:
    def __init__(self, arm: Arm):
        meta = arm.meta()
        self.arm = arm
        self.table_z: float = meta["table_z"]  # height of the table surface in the base frame
        self.envelope: dict = meta["clearance"]  # floor, body, min_y, limit_margin: what the robot side enforces
        self.margin: float = self.envelope["limit_margin"]  # rad kept clear of each joint limit when planning
        self.kin = Kinematics(ROOT / "robot_config" / "model.urdf")
        self.kin.set_limits(*arm.limits())

    def fk(self, q) -> np.ndarray:
        """Tool pose for joint angles `q`: 4x4, columns jaw axis, y, approach direction, position."""
        return self.kin.fk(q)

    def blocked(self, q, allow_low: bool = False) -> str | None:
        """Why the robot side would refuse this configuration, or None. Plans keep half a
        millimetre more than it asks for, so rounding on the way over cannot tip one into a refusal."""
        e = self.envelope
        return self.kin.clearance(q, table_z=self.table_z, floor=0.0005 if allow_low else e["floor"] + 0.0005, body=e["body"], min_y=e["min_y"])

    def solve(self, pos, *, pitch: float | None = None, yaw: float | None = None, approach=None, q_seed=None,
              allow_low: bool = False) -> IKResult:  # fmt: skip
        """Joint angles that put the tool point at `pos`, optionally pointing a given way.
        With neither `pitch` nor `approach` any orientation will do. Prefers solutions near `q_seed`."""
        pos = np.asarray(pos, dtype=float)
        if approach is None and pitch is not None:
            approach = approach_from_pitch(pos[:2], pitch)
        jaw = jaw_from_yaw(approach, yaw) if (yaw is not None and approach is not None) else None
        seed = self.arm.q() if q_seed is None else q_seed
        return self.kin.ik(pos, approach=approach, jaw=jaw, q_seed=seed, limit_margin=self.margin,
                           accept=lambda q: self.blocked(q, allow_low) is None)  # fmt: skip

    def reach(self, pos, *, yaw: float | None = None, pitches=(90, 75, 60, 45, 30, 15), **kw) -> tuple[IKResult, float | None]:
        """Try progressively flatter approaches until one reaches `pos`: straight-down poses only
        exist close in, further out the gripper has to lean forward."""
        best: IKResult | None = None
        for pitch in pitches:
            r = self.solve(pos, pitch=pitch, yaw=yaw, **kw)
            if r.ok:
                return r, float(pitch)
            if best is None or r.pos_err < best.pos_err:
                best = r
        return best, None

    def unreachable(self, pos, r: IKResult) -> str:
        d = float(np.linalg.norm(np.asarray(pos)[:2] - self.kin.joints[0].pre[:2, 3]))
        hint = ""
        if d > 0.40:
            hint = f" It is {d * 100:.0f} cm from the base and the arm reaches about 40 cm."
        elif d < 0.08:
            hint = " It is too close to the base."
        return f"can't reach ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}): {r.describe()}.{hint}"

    def line(self, q_start, pos, *, approach=None, allow_low: bool = False) -> np.ndarray:
        """Joint waypoints that carry the tool point in a straight line from where `q_start` puts
        it to `pos`, one every 8 mm, pointing along `approach` throughout (default: the way it
        points now). Send the result as one joint command. Reverse it to come back the same way."""
        path, r = self.kin.ik_line(q_start, pos, approach=approach, limit_margin=self.margin)
        if not r.ok:
            raise RobotError(f"can't move in a straight line to there: {r.reason} (pos_err={r.pos_err * 1000:.1f}mm ang_err={math.degrees(r.ang_err):.1f}deg)")
        for q in path[1:]:
            why = self.blocked(q, allow_low)
            if why:
                raise RobotError(f"straight-line move rejected: {why}")
        return path

    def approach_for(self, target, grasp_z: float, yaw: float | None, hover: float = 0.07):
        """Choose how to come down on a table point.

        Tries the most vertical approach first and leans forward in 10 degree steps until IK
        reaches both the grasp point (`grasp_z` above the table) and a hover point backed off
        `hover` metres along the approach. Straight-down only works within about 27 cm of the base.

        Returns (approach unit vector, pitch in degrees, hover joint angles, hover xyz, grasp xyz).
        """
        grasp = np.array([target[0], target[1], self.table_z + grasp_z])
        seed = np.radians(READY_DEG)
        last = None
        refused: list[str] = []  # what the envelope said about each pose IK reached but may not use

        def clear(q, allow_low: bool) -> bool:
            why = self.blocked(q, allow_low)
            if why:
                refused.append(why)
            return why is None

        for pitch in (90, 80, 70, 60, 50, 40, 30):
            a = approach_from_pitch(grasp[:2], pitch)
            jaw = jaw_from_yaw(a, yaw if yaw is not None else 0.0)
            hover_pos = grasp - a * hover
            rg = self.kin.ik(grasp, approach=a, jaw=jaw, q_seed=seed, limit_margin=self.margin, accept=lambda q: clear(q, True))
            if not rg.ok:
                last = rg
                continue
            rh = self.kin.ik(hover_pos, approach=a, jaw=jaw, q_seed=rg.q, limit_margin=self.margin, accept=lambda q: clear(q, False))
            if rh.ok:
                return a, float(pitch), rh.q, hover_pos, grasp
            last = rh
        d = math.hypot(target[0], target[1] - 0.04)
        if d > 0.36:
            why = f"it is {d * 100:.0f} cm from the base (reach is about 38 cm on the table)"
        elif d < 0.09:
            why = "it is too close to the base"
        elif refused and last is not None and "rejected" in last.reason:
            # reachable, but not allowed: most often a fingertip of leaning, turned jaws dipping
            # below the table, which a higher grasp or yaw 0 avoids
            why = f"every pose that gets there was refused, most recently: {refused[-1]}. Try a larger grasp height, or yaw 0"
        else:
            why = last.describe() if last else "no approach found"
        raise RobotError(f"can't reach a grasp at x={target[0]:+.3f} y={target[1]:+.3f}: {why}")
