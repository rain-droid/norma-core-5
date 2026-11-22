"""Pick up an object by name: "the ring", "blue cube", or its key.

Looks with the top camera first, so it goes to where the object is now, then grasps the way
that suits it and writes the outcome into the world model:

* a solid object: pick_verified at its centre, jaws across its flat sides, closing at about half
  its height. The wrist camera lines the jaws up on it before descending and checks afterwards
  that it is between the fingers;
* a ring (anything the table shows through): the jaws cannot close on the empty middle. It is
  gripped across the band at the side of the ring, on the side nearer the arm's centre line,
  with the jaws closing left-right. The wrist camera's line-up is skipped (it would centre on
  the whole ring) but its check after lifting is kept.

If the first try comes up empty it looks again and tries once more (`attempts`), for a ring
on the other side of it. `grasp_height` overrides how high above the table the jaws close.

Returns `holding`, `width_mm` (what the jaws closed on: compare it with the object),
`seen_in_jaws` when the wrist camera could tell, and `tried`: each attempt and how it went.
The object's record gets `state: held`, and the world model's `holding` its key. Name the
objects first (describe_objects, name_objects); an unknown name lists what is known.
"""

import numpy as np

from describe_objects import look
from pick_verified import pick_verified
from util import world as W
from util.arm import RobotError
from util.motion import require_ready


def rim_points(o: dict) -> list[np.ndarray]:
    """Where to grip a ring: the two points of its band left and right of its centre as the arm
    reaches out to it, the one nearer the arm's centre line first. There the band runs along the
    reach, so jaws closing left-right close across it."""
    c = np.array([o["x"], o["y"]])
    radial = c / (np.linalg.norm(c) + 1e-9)
    lateral = np.array([-radial[1], radial[0]])
    r = o["ring"]["r_mid_mm"] / 1000
    return sorted((c + s * r * lateral for s in (1, -1)), key=lambda p: abs(p[0]))


def grasp_object(ctx, object: str, attempts: int = 2, grasp_height: float | None = None) -> dict:
    key = W.resolve(W.load(), object)  # an unknown name is said first: it is the more useful thing to hear
    if require_ready(ctx, awake=True)["gripper"]["holding"]:
        raise RobotError("already holding something: put it down first (put_object)")
    tried, result = [], None
    for attempt in range(max(1, attempts)):
        world, _, _, _, _ = look(ctx)
        o = world["objects"].get(key)
        if o is None or o["state"] != "on_table":
            raise RobotError(f"{key} ({(o or {}).get('name') or 'unnamed'}) is not in view of the top camera: "
                             "move the arm out of the way (home, rest) or look for it (describe_objects)")  # fmt: skip
        height = (o.get("height_mm") or 25) / 1000
        close_at = grasp_height if grasp_height is not None else float(np.clip(0.5 * height, 0.022, 0.030))
        if o["shape"] == "ring" and o.get("ring"):
            points = rim_points(o)
            if attempt == 0:  # the side the gripper can actually get to first
                def within_reach(q) -> bool:
                    try:
                        ctx.plan.approach_for(np.array([q[0], q[1], ctx.plan.table_z]), close_at, 0.0)
                        return True
                    except RobotError:
                        return False
                points.sort(key=lambda q: not within_reach(q))
            p = points[attempt % 2]
            how = {"x": round(float(p[0]), 3), "y": round(float(p[1]), 3), "yaw_deg": 0.0, "grasp_height": close_at, "object_height": height, "corrections": 0}
            way = "by the rim"
        else:
            how = {"x": o["x"], "y": o["y"], "yaw_deg": float(o.get("yaw_deg") or 0.0), "grasp_height": close_at, "object_height": height}
            way = "at its centre"
        try:
            result = ctx.call(pick_verified, **how)
        except RobotError as e:
            tried.append({"way": way, "at": [how["x"], how["y"]], "outcome": str(e)})
            W.note(world, key, "grasp", f"refused: {e}")
            W.save(world)
            if how.get("yaw_deg"):  # leaning, turned jaws are the usual refusal: once more with them straight
                o["yaw_deg"] = 0.0
                W.save(world)
            continue
        outcome = f"holding, {result['width_mm']:.0f} mm" if result.get("holding") else "closed on nothing"
        tried.append({"way": way, "at": result.get("aimed_at", [how["x"], how["y"]]), "outcome": outcome,
                      "corrections_mm": result.get("corrections_mm"), "seen_in_jaws": result.get("seen_in_jaws")})  # fmt: skip
        W.note(world, key, "grasp", f"{outcome} ({way})")
        if result.get("holding"):
            o.update(state="held", on=None)
            world["holding"] = key
            W.save(world)
            break
        W.save(world)
    held = bool(result and result.get("holding"))
    out = {"object": key, "name": W.load()["objects"][key].get("name"), "holding": held, "tried": tried}
    if held:
        out.update({k: result[k] for k in ("width_mm", "seen_in_jaws", "before_image", "after_image") if k in result})
    else:
        out["note"] = "not picked up: open the images of the last pick_verified call (nested in this call's history), or describe_objects to see where it is now"
    return out


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(grasp_object)
