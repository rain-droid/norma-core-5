"""Report the measured robot state and draw the body's current posture.

Returns joint angles (radians and degrees, with each joint's calibrated limits, load and
temperature), the tool point and the jaw opening. Also renders the robot model at exactly those
angles from behind, from the right and from above, with the tool point marked: open
`body_image` to see the posture. Sends no command and captures no camera.

The same numbers are always in perception/proprioception/state.json; call this when you need
to *see* the posture, for instance before commanding joints directly. The picture is the
model, not a camera: it shows no table objects and no held object, and proves no grasp.
"""

import math

import numpy as np

from util import render


def read_proprioception(ctx) -> dict:
    s = ctx.arm.state()
    out = {
        "connected": s["connected"],
        "calibrated": s["calibrated"],
        "torque": s["torque"],
        "joints": [{"id": j["id"], "rad": j["q"], "deg": round(math.degrees(j["q"]), 1), "lower_rad": j["lower"], "upper_rad": j["upper"],
                    "load": j["load"], "temp_c": j["temp"], "error": j["error"]} for j in s["joints"][:7]],  # fmt: skip
        "gripper": s["gripper"],
        "tcp_xyz_m": s["ee"]["pos"] if s["ee"] else None,
        "body_image": None,
    }
    if s["ee"]:
        model = render.Model(ctx.root / "robot_config" / "model.urdf")
        q = np.array([j["q"] for j in s["joints"][:7]])
        img = render.three_views(model, q, s["gripper"]["open"], tcp=np.array(s["ee"]["pos"]))
        out["body_image"] = ctx.save_image("body.jpg", img, "robot model at the measured joint angles: from behind, from the right, from above")
    else:
        out["note"] = ctx.arm.why_not_ready(s)
    return out


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(read_proprioception)
