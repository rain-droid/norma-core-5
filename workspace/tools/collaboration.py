"""Switch collaboration mode on or off: for working with a person's hands near the arm.

While it is on the arm pushes less hard (a lower torque limit on the arm servos), moves at no
more than about 0.6 of its planned speed, and watches every joint during a move: a touch shows
as a joint's load or lag stepping away from its recent level. When that happens the arm stops,
backs off a short way along the path it came by, holds, and the command that was running ends
with an error starting `contact:` (`kind: contact`). That move is not sent again by itself.

After a `contact:` error: look (top camera, then the wrist camera) and say what happened. If a
person touched the arm on purpose, wait for them to say go on. Do not repeat the same move
unchanged, and do not switch this mode off to get past it.

`enable` true or false; left out, it only reports. Returns `{"collaboration": bool}`.

This makes contact gentler. It is not a safety guarantee: the arm is felt before it yields,
guarded touch moves (grasping, pressing a pen down) do not use the reflex, and a held object
changes what the joints feel. Keep faces and anything fragile out of reach all the same.
"""


def collaboration(ctx, enable: bool | None = None) -> dict:
    if enable is not None:
        ctx.arm.collab(bool(enable))
    return {"collaboration": bool(ctx.arm.state().get("collab", False))}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(collaboration)
