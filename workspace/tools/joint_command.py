"""The atomic action, and the only thing the robot understands: joint angles.

`joints` is seven angles in radians for servos 1..7, in order: base rotation, shoulder, lower
elbow, upper elbow, forearm roll, wrist bend, wrist roll. It can also be a list of such
waypoints, which the arm travels through in one smooth move. `gripper` is the jaw opening, 0
(closed) to 1 (fully open, 53 mm); leave it out to leave the jaws alone. Targets are absolute,
not deltas: read perception/proprioception/state.json first and change only what you mean to.

Nothing is planned here. Between waypoints the arm moves in a straight line in joint space,
under one trapezoidal speed profile scaled by `speed` (0.05 slow .. 2 fast). The robot side
refuses targets outside the calibrated joint limits and, with `check` true (default), any
waypoint that would put the fingertips into the table, the arm into itself, or the tool behind
the base. `allow_low` lets the fingertips come down to the table surface, as a grasp needs.
`check: false` skips that envelope: the arm goes wherever it is sent, so look at the current
pose first (read_proprioception) and move in small steps.
Neither flag bypasses this workspace's real-arm mat floor: util/arm.py checks the entire
joint-space sweep before sending, including retries, for the model correction plus
extra_descent_clearance_m (25 mm total with current settings). Use top-camera x/y and depth_scan
heights before descending near the mat; depth does not calibrate the physical fingertips.

A gripper opening of 0 is a grasp: the jaws close until they stall, then hold just past that
point, and `gripper.holding` in the result says whether something stopped them.

Every other tool in this folder ends in this same command (util/arm.py, Arm.joints). Returns
the measured joint angles after settling: a joint more than a degree or so from its command is
pressing on something or overloaded.
"""

import math

import numpy as np

from util.arm import RobotError
from util.motion import tcp_report


def joint_command(ctx, joints=None, gripper: float | None = None, speed: float = 0.8, check: bool = True, allow_low: bool = False) -> dict:
    path = None
    if joints is not None:
        try:
            path = np.asarray(joints, dtype=float)
        except (TypeError, ValueError):
            path = np.zeros(0)
        path = path[None] if path.ndim == 1 else path
        if path.ndim != 2 or path.shape[1] != 7:
            raise RobotError("joints must be seven angles in radians (servos 1..7), or a list of such waypoints; the gripper is separate")
    res = ctx.arm.joints(path, gripper=gripper, speed=speed, check=check, allow_low=allow_low, label="joint command")
    out = {"measured_rad": res["joints"], "gripper": res["gripper"], "tcp": tcp_report(res["tcp"], res["gripper"]), "collision_checked": bool(check)}
    if path is not None:
        out["commanded_rad"] = [round(float(v), 4) for v in path[-1]]
        out["worst_joint_error_deg"] = round(math.degrees(float(np.max(np.abs(path[-1] - np.array(res["joints"]))))), 2)
    return out


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(joint_command)
