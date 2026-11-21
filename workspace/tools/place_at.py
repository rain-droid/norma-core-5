"""Put down what the gripper holds at table position x, y, then back away.

Steps: choose an approach as pick_at does, carry to a hover point 7 cm back along it, descend in
a straight line until the tool point is 22 mm above the surface, open the jaws, retreat to the
hover point. `z` is the height of the surface being placed on, metres above the table: 0 for the
table, the height of the object underneath when stacking. `yaw` rotates the jaws, degrees.

`released_something` only reports that the jaws were held apart before opening. Where the object
came to rest is for a camera to say: look_at_table afterwards.
"""

import numpy as np

from elrobot.robot import RobotError

from util.motion import approach_for, require_ready, tcp_report

RELEASE_ABOVE = 0.022  # metres between the surface and the tool point when the jaws open


async def place_at(ctx, x: float, y: float, z: float = 0.0, yaw: float | None = None) -> dict:
    robot = ctx.robot
    require_ready(ctx)
    target = np.array([x, y, ctx.cfg.table_z + z], dtype=float)
    ctx.emit("markers", markers=[{"id": "target", "label": "place", "pos": target.tolist(), "color": "#5ab0ff", "radius": 0.014}])
    approach, pitch, q_hover, hover_pos, drop = approach_for(ctx, target, z + RELEASE_ABOVE, yaw)
    await robot.move_joints(q_hover, speed=0.8, label="carrying")
    await robot.move_path(robot.plan_linear(robot._plan_start(), drop, approach=approach, allow_low=True), speed=0.5, label="lowering")
    was_holding = bool(robot.holding)
    await robot.set_gripper(1.0, label="releasing")
    try:
        await robot.move_path(robot.plan_linear(robot._plan_start(), hover_pos, approach=approach, allow_low=True), speed=0.7, label="backing off")
    except RobotError:
        # the straight retreat is sometimes unplannable from where the arm settled; the object is
        # already released, so go back to the hover pose in joint space instead
        await robot.move_joints(q_hover, speed=0.5, allow_low=True, label="backing off")
    return {
        "placed_at": [round(x, 3), round(y, 3), round(z, 3)],
        "released_something": was_holding,
        "approach_pitch_deg": pitch,
        "tcp": tcp_report(ctx),
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(place_at)
