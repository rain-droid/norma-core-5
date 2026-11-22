"""Put down what the gripper holds: onto a named object, or at a place.

Give `on` (a name or key: "the pink box") to stand it on that object, or `x`, `y` (metres, base
frame) to put it on the table. With `on`, it looks with the top camera first so it goes to
where that object is now, and releases at its height.

The world model is updated: the held object is `on_table` again at that place, `on` says what
it stands on, and the world's `holding` is cleared. `released_something` is false if the jaws
were not holding anything when they opened.

Where it came to rest is for a camera to say: describe_objects afterwards (the arm is then over
the spot; `home` first). Expect a centimetre or two of error; a stack is checked by the top
object having moved to the lower one's place.
"""

from describe_objects import look
from place_at import place_at
from util import world as W
from util.arm import RobotError
from util.motion import require_ready


def put_object(ctx, on: str | None = None, x: float | None = None, y: float | None = None, yaw_deg: float = 0.0) -> dict:
    require_ready(ctx, awake=True)
    world = W.load()
    held = world.get("holding")
    z, under = 0.0, None
    if on is not None:
        under = W.resolve(world, on)
        if under == held:
            raise RobotError(f"{under} is the object being held: it cannot be put on itself")
        world = look(ctx)[0]
        target = world["objects"][under]
        if target["state"] != "on_table":
            raise RobotError(f"{under} ({target.get('name') or 'unnamed'}) is not in view of the top camera: say where with x, y, or look for it")
        x, y, z = target["x"], target["y"], (target.get("height_mm") or 25) / 1000
    elif x is None or y is None:
        raise RobotError("say where: `on` (an object's name) or `x` and `y`")
    result = ctx.call(place_at, x=float(x), y=float(y), z=z, yaw_deg=yaw_deg)
    world = W.load()
    if held and held in world["objects"]:
        o = world["objects"][held]
        o.update(state="on_table", x=round(float(x), 3), y=round(float(y), 3), on=under)
        W.note(world, held, "put", f"on {under}" if under else f"at x={x:+.3f} y={y:+.3f}")
    world["holding"] = None
    W.save(world)
    return {"object": held, "on": under, "placed_at": result["placed_at"], "released_something": result["released_something"],
            "next": "home, then describe_objects to see where it came to rest"}  # fmt: skip


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(put_object)
