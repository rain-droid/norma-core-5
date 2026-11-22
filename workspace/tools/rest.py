"""Fold the arm into its resting seat and switch the motors off.

The jaws are brought together first: fully open they are the widest and lowest part of the
folded arm, and they have ended up pressed on the table, from where `home` is refused. Then two
joint commands: to the pose just above the seat (robot_config/sit_pose.json with the shoulder
raised), then down onto it; then torque off, so the servos cool down. Both go slowly: the
wrist folds through 60 degrees and at speed 0.6 it has been refused as blocked. The seat
rests the arm on its own base, which the clearance envelope would refuse, so both moves go
with `check` off. Refuses while holding something. Use it when a task is finished, or to wait:
a stretched-out arm keeps its shoulder servo working hard. `home` wakes it up again.
"""

from util.arm import RobotError
from util.motion import require_ready, seat


JAWS_SEATED = 0.03  # jaw opening the arm sits down with
FOLD_SPEED = 0.3


def rest(ctx) -> dict:
    state = require_ready(ctx)
    if state["gripper"]["holding"]:
        raise RobotError("holding something; place it or open the gripper before resting")
    if state["gripper"]["open"] > JAWS_SEATED + 0.02:
        ctx.arm.joints(gripper=JAWS_SEATED, label="jaws together")  # not 0: that is a grasp, and reads as holding
    sit, above = seat(ctx.arm)
    ctx.arm.joints(above, speed=FOLD_SPEED, check=False, label="sitting down")
    ctx.arm.joints(sit, speed=FOLD_SPEED, check=False, label="sitting down")
    ctx.arm.torque(False)
    return {"pose": "sitting", "torque": ctx.arm.state()["torque"]}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(rest)
