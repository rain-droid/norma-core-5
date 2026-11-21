"""Put down what the gripper holds at table position x, y, then back away.

Steps: choose an approach as pick_at does, carry to a hover point 7 cm back along it, descend in
a straight line until the tool point is 22 mm above the surface, open the jaws, retreat up the
same line. `z` is the height of the surface being placed on, metres above the table: 0 for the
table, the height of the object underneath when stacking. `yaw_deg` rotates the jaws, in degrees.

`released_something` only reports that the jaws were held apart before opening. Where the object
came to rest is for a camera to say: capture_camera or find_objects afterwards. An object
gripped off-centre, or released from leaning jaws, lands a centimetre or two off.
"""

import numpy as np

from util.motion import require_ready, tcp_report

RELEASE_ABOVE = 0.022  # metres between the surface and the tool point when the jaws open


def place_at(ctx, x: float, y: float, z: float = 0.0, yaw_deg: float | None = None) -> dict:
    arm, plan = ctx.arm, ctx.plan
    was_holding = require_ready(ctx, awake=True)["gripper"]["holding"]
    target = np.array([x, y, plan.table_z + z], dtype=float)
    ctx.emit("markers", markers=[{"id": "target", "label": "place", "pos": target.tolist(), "color": "#5ab0ff", "radius": 0.014}])
    approach, pitch, q_hover, hover_pos, drop = plan.approach_for(target, z + RELEASE_ABOVE, yaw_deg)
    down = plan.line(q_hover, drop, approach=approach, allow_low=True)  # planned before anything moves
    arm.joints(q_hover, speed=0.8, label="carrying")
    arm.joints(down, speed=0.5, allow_low=True, label="lowering")
    arm.joints(gripper=1.0, label="releasing")
    # back out along the same straight line, so the gripper cannot sweep what it just put down
    res = arm.joints(down[::-1], speed=0.5, allow_low=True, label="backing off")
    return {
        "placed_at": [round(x, 3), round(y, 3), round(z, 3)],
        "released_something": was_holding,
        "approach_pitch_deg": pitch,
        "tcp": tcp_report(res["tcp"], res["gripper"]),
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(place_at)
