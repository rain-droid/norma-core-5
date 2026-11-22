"""Read the world model: what is on the table by name, where, and what has been done with it.

The world model (state/world.json) is the workspace's memory of the table between looks. Each
object has a key that never changes (`blue-1`), a name and kind once someone has named it
(name_objects), its last measured place and size, its shape (`ring` for things with a hole),
its `state` (`on_table`, `held`, or `unseen` when the last look did not find it), what it stands
`on`, and a short `history` of what was done to it and how that went.

`look: true` first takes a fresh look with the top camera (as describe_objects does, without
the picture sheet), so places are current. Without it you get what was last known: fine for
"what did I do with the ring?", wrong for "where is it now?" after anyone has touched the table.

`object` narrows the answer to one object, with `log`: everything recorded as done to it. `reset: true` forgets
everything (a new table, a new session).

Does not move the arm.
"""

from describe_objects import look as take_look
from util import world as W


def world_model(ctx, look: bool = False, object: str | None = None, reset: bool = False) -> dict:
    if reset:
        W.save({"holding": None, "objects": {}})
        return {"objects": [], "holding": None, "note": "the world model is empty: describe_objects to fill it"}
    world = take_look(ctx)[0] if look else W.load()
    if object is not None:
        k = W.resolve(world, object)
        record = dict(world["objects"][k])
        # `history` in a tool's result is the call's own folder, so the object's goes out as `log`
        return {"key": k, **{f: v for f, v in record.items() if f != "history"}, "log": record.get("history", []), "holding": world.get("holding")}
    return {"objects": W.summary(world), "holding": world.get("holding")}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(world_model)
