"""Report the measured robot state and draw the body's current posture.

Reads the servo positions and returns joint angles (radians and degrees), the tool point, the
jaw opening and per-joint load and temperature. Also renders the robot model at exactly those
angles from behind, from the right and from above, with the tool point marked: open
`body_image` to see the posture. Sends no motion and captures no camera.

The numbers alone are always current in perception/proprioception/state.json; call this when
you need to *see* the posture, for instance before commanding joints directly. The picture is
the model, not a camera: it shows no table objects and no held object, and proves no grasp.
"""

from util import render

MOTION = False


async def read_proprioception(ctx) -> dict:
    state = ctx.ws.state()
    if not state["calibrated"] or not state["connected"]:
        return {"state": state, "body_image": None, "note": ctx.robot.why_not_ready()}
    model = render.Model(ctx.cfg.urdf)
    img = render.three_views(model, ctx.robot.q(), ctx.robot.gripper_open(), tcp=ctx.robot.tcp()[:3, 3])
    return {"state": state, "body_image": ctx.save_image("body.jpg", img, "robot model at the measured joint angles: from behind, from the right, from above")}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(read_proprioception)
