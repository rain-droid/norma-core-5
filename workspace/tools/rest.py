"""Fold the arm into its resting seat and switch the motors off.

Two joint commands: to the pose just above the seat (robot_config/sit_pose.json with the
shoulder raised), then slowly down onto it; then torque off, so the servos cool down. The seat
rests the arm on its own base, which the clearance envelope would refuse, so both moves go
with `check` off. Refuses while holding something. Use it when a task is finished, or to wait:
a stretched-out arm keeps its shoulder servo working hard. `home` wakes it up again.
"""

from util.arm import RobotError
from util.motion import require_ready, seat


def rest(ctx) -> dict:
    state = require_ready(ctx)
    if state["gripper"]["holding"]:
        raise RobotError("holding something; place it or open the gripper before resting")
    sit, above = seat(ctx.arm)
    ctx.arm.joints(above, speed=0.6, check=False, label="sitting down")
    ctx.arm.joints(sit, speed=0.3, check=False, label="sitting down")
    ctx.arm.torque(False)
    return {"pose": "sitting", "torque": ctx.arm.state()["torque"]}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(rest)
