"""The atomic action: one absolute target for every joint. Everything else is built on this.

`joints` is seven angles in radians for servos 1..7, in order: base rotation, shoulder, lower
elbow, upper elbow, forearm roll, wrist bend, wrist roll. `gripper` is the jaw opening, 0
(closed) to 1 (fully open, 53 mm); omit it to leave the jaws alone. This is a target, not a
delta: read perception/proprioception/state.json first and change only what you mean to.

No IK and no planning happen here. The arm travels in a straight line in joint space, with a
smooth speed profile scaled by `speed` (0.05 slow .. 2 fast), and the servo droop under load
is trimmed out at the end. Targets outside the calibrated joint limits are refused, not clamped.
With `check` true (default) the path is also refused if it would put the fingertips into the
table, the arm into itself, or the tool behind the base. `check: false` skips that: the arm
will go wherever you send it, so look at robot_config/visuals/ and the current pose first.

Returns the measured joint angles after settling. A joint more than a degree or so from its
command is pressing on something or overloaded.
"""

import math

import numpy as np

from elrobot.robot import RobotError
from util.motion import require_ready, tcp_report


async def joint_command(ctx, joints: list, gripper: float | None = None, speed: float = 0.8, check: bool = True) -> dict:
    robot = ctx.robot
    require_ready(ctx)
    if not isinstance(joints, (list, tuple)) or len(joints) != 7:
        raise RobotError("joints must be exactly 7 angles in radians (servos 1..7); the gripper is separate")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in joints):
        raise RobotError("joint angles must be finite numbers")
    if gripper is not None and not 0.0 <= gripper <= 1.0:
        raise RobotError("gripper is an opening fraction between 0 (closed) and 1 (open)")
    q = np.array(joints, dtype=float)
    robot.ranges()  # refreshes kin.lower/upper from the servo calibration
    lo, hi = robot.kin.lower, robot.kin.upper
    over = [f"joint {i + 1}: {q[i]:+.3f} is outside [{lo[i]:+.3f}, {hi[i]:+.3f}]" for i in range(7) if not lo[i] - 1e-6 <= q[i] <= hi[i] + 1e-6]
    if over:
        raise RobotError("outside joint limits (radians). " + "; ".join(over))

    await robot.move_joints(q, speed=speed, check=check, label="joint command")
    if gripper is not None:
        await robot.set_gripper(gripper)
    measured = robot.q()
    return {
        "commanded_rad": [round(float(v), 4) for v in q],
        "measured_rad": [round(float(v), 4) for v in measured],
        "worst_joint_error_deg": round(math.degrees(float(np.max(np.abs(q - measured)))), 2),
        "collision_checked": bool(check),
        "tcp": tcp_report(ctx),
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(joint_command)
