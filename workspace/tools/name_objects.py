"""Write down what the objects are, after looking at describe_objects' picture sheet.

`names` maps each key to what you see: either a name, or an object with any of
`name` (what to call it: "blue cube", "rubber ring"), `kind` (one word for what sort of thing it
is: cube, ring, box, pen, block), `height_mm` (if you know it better than the camera's guess: it
decides how high the gripper closes), and `notes` (anything worth knowing next time: "slippery",
"hollow, grip the rim").

    {"names": {"blue-1": {"name": "blue cube", "kind": "cube"}, "red-1": "rubber ring"}}

Names are kept in the world model (state/world.json) and stay with the object when it is moved
or hidden. Tools that take an object take the name, the kind or the key: "the ring", "blue cube",
"red-1". Give two objects different names. `forget` is a list of keys or names to drop
altogether (things taken off the table for good).

Does not look and does not move the arm. Returns the world model as it now stands.
"""

from util import world as W
from util.arm import RobotError


def name_objects(ctx, names: dict | None = None, forget: list | None = None) -> dict:
    world = W.load()
    for key, value in (names or {}).items():
        k = W.resolve(world, key)
        fields = {"name": value} if isinstance(value, str) else dict(value)
        unknown = set(fields) - {"name", "kind", "height_mm", "notes"}
        if unknown:
            raise RobotError(f"cannot set {sorted(unknown)} on an object: only name, kind, height_mm, notes")
        taken = [other for other, o in world["objects"].items() if other != k and fields.get("name") and (o.get("name") or "").lower() == fields["name"].lower()]
        if taken:
            raise RobotError(f"'{fields['name']}' is already the name of {taken[0]}: give {k} a name that tells them apart")
        world["objects"][k].update(fields)
    for name in forget or []:
        k = W.resolve(world, name)
        del world["objects"][k]
        if world.get("holding") == k:
            world["holding"] = None
    W.save(world)
    return {"objects": W.summary(world), "holding": world.get("holding")}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(name_objects)
