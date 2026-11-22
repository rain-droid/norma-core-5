"""Put what the gripper holds onto a tracked object, or at x, y, and check where it ended up.

What is placed is whatever the gripper holds; `object` may name it, and is then checked.
Give `on` (a name from track_objects, to stack on that object) or `x` and `y` (a place on the
table; `z` if the surface there is higher than the table). The motion is place_at's: hover,
lower in a straight line, open the jaws 22 mm above the surface, back out along the same line.
What this adds:

* it waits until the table is quiet, and aims at where `on` is then;
* at the hover point it asks whether `on` has been moved since, and aims again if so;
* if the tracker stops the arm because something moved on the table (a hand, most likely), it
  waits for quiet and carries on;
* afterwards it waits for the top camera to see the object again (sending the arm home if the
  arm is in the way) and compares. On the table, it must lie within 40 mm of x, y. On another
  object, it must not be lying anywhere else, and the pile must look taller or bigger than
  what was underneath did alone;
* if that check fails and the object is in view, it is picked up and placed again, `retries`
  times (default 1).

Stacking needs the height of what is underneath: its measured `height_mm`, or `z` if you know
better (the measurement tends to read up to a centimetre high on the real arm).

The result says where it landed and how far off. An error after the object has been let go
says where it is now. Something the tracker never followed (picked up before tracking
started) is placed, but cannot be checked: `checked` is false and you have to look yourself.
"""

import time

import numpy as np

from home import home
from pick_object import pick_object
from place_at import RELEASE_ABOVE
from util import tracker
from util.arm import RobotError
from util.motion import require_ready, tcp_report

NEAR_ENOUGH = 0.04  # metres from x, y that still counts as placed there: the real arm puts down 1 to 4 cm off, and the camera measures to about 1


def settle(ctx, name: str) -> tuple[dict, dict]:
    """The tracker's report and this object's track once the camera has seen where it lies."""
    went_home, t0 = False, time.time()
    while True:
        data = tracker.fresh()
        t = tracker.track(data, name)
        if t["state"] != "hidden" and t["still_s"] > 0.5 and not data["intrusion"]:
            return data, t
        if time.time() - t0 > 0.4 and not went_home:
            went_home = True
            try:
                ctx.call(home)  # the arm is what hides it
            except RobotError as e:
                if not tracker.stopped_by_tracker(e):
                    raise
                went_home = False
        if time.time() - t0 > 20.0:
            raise RobotError(f"{name} was put down but stays out of the camera's sight; last known at x={t['x']:+.3f} y={t['y']:+.3f}")


def place_object(ctx, object: str = "", on: str = "", x: float | None = None, y: float | None = None, z: float | None = None,
                 yaw_deg: float | None = None, attempts: int = 4, retries: int = 1) -> dict:  # fmt: skip
    arm, plan = ctx.arm, ctx.plan
    if not require_ready(ctx, awake=True)["gripper"]["holding"]:
        raise RobotError("the gripper is empty: pick something first")
    if bool(on) == (x is not None and y is not None):
        raise RobotError("give either `on` (a tracked object to put it on) or both `x` and `y`")
    mine = tracker.fresh()["holding"]  # the name of what is in the gripper; "" if it never was a track
    if object and object != mine:
        raise RobotError(f"the gripper holds {mine or 'something the tracker never followed'}, not {object}")
    if on and on == mine:
        raise RobotError(f"{on} is what the gripper holds")
    tried: list[str] = []
    target = surface = under = None
    left, waits = attempts, 0
    while left > 0:
        left -= 1
        try:
            if not arm.state()["gripper"]["holding"] and target is not None:
                break  # stopped after the jaws had opened: it is down, what is left is to see where
            if on:
                base = tracker.wait_for(ctx, on)
                if z is None and not base["height_mm"]:
                    raise RobotError(f"the height of {on} could not be measured: give `z`, its height in metres")
                target, surface, under = np.array([base["x"], base["y"]]), (z if z is not None else base["height_mm"] / 1000), base
            else:
                tracker.wait_for(ctx)
                target, surface = np.array([x, y], dtype=float), z or 0.0
            spot = np.array([*target, plan.table_z + surface])
            ctx.emit("markers", markers=[{"id": "target", "label": f"place on {on}" if on else "place", "pos": spot.tolist(), "color": "#5ab0ff", "radius": 0.014}])
            a, pitch, q_hover, _, drop = plan.approach_for(spot, surface + RELEASE_ABOVE, yaw_deg)
            down = plan.line(q_hover, drop, approach=a, allow_low=True)
            arm.joints(q_hover, speed=0.8, label="carrying")
            if on:
                now = tracker.track(tracker.fresh(), on)
                if now["state"] == "missing" or now["moves"] != base["moves"]:
                    moved = float(np.hypot(now["x"] - target[0], now["y"] - target[1]))
                    tried.append(f"{on} was moved {moved * 1000:.0f} mm while the arm was carrying: aiming again")
                    continue
            arm.joints(down, speed=0.5, allow_low=True, label="lowering")
            arm.joints(gripper=1.0, label="releasing")
            arm.joints(down[::-1], speed=0.5, allow_low=True, label="backing off")
            break
        except RobotError as e:
            if not tracker.stopped_by_tracker(e):
                raise
            tried.append("the arm was stopped because something moved in the work area: waiting for it to go quiet")
            waits += 1
            left += waits <= 6  # being interrupted is not a failed attempt, up to a point
    else:
        raise RobotError(f"could not place it in {attempts} attempts, and still hold it: " + "; ".join(tried))

    out = {"placed": mine or None, "on": on or None, "target": [round(float(target[0]), 3), round(float(target[1]), 3), round(surface, 3)], "tried": tried}
    if not mine:
        return {**out, "checked": False}
    data, t = settle(ctx, mine)
    off = float(np.hypot(t["x"] - target[0], t["y"] - target[1]))
    if on:
        pile = tracker.track(data, on)
        # a pile stands taller in the image than its base did; one whose top object is small can be no larger in area
        grew = pile["box"][1] < under["box"][1] - 6 or pile["area_px"] > 1.2 * under["area_px"]
        good = t["on"] == on and pile["state"] == "visible" and grew
        problem = f"{mine} is not on {on}: " + (f"it lies at x={t['x']:+.3f} y={t['y']:+.3f}" if t["on"] != on else f"{on} looks no taller than before, so it may have fallen out of view")
    else:
        good = t["state"] == "visible" and off < NEAR_ENOUGH
        problem = f"{mine} ended up {off * 1000:.0f} mm from where it was meant to go, at x={t['x']:+.3f} y={t['y']:+.3f}" if t["state"] == "visible" else f"{mine} is {t['state']} after being put down"
    if good:
        return {**out, "checked": True, "landed_at": [t["x"], t["y"]], "off_target_mm": round(off * 1000), "state": tcp_report(arm.state()["ee"], arm.state()["gripper"])}
    if retries > 0 and t["state"] == "visible" and t["on"] is None:
        try:
            again = ctx.call(pick_object, object=mine)
        except RobotError as e:
            raise RobotError(f"{problem}. It was put down; picking it up to place it again failed: {e}") from None
        more = ctx.call(place_object, on=on, x=x, y=y, z=z, yaw_deg=yaw_deg, attempts=attempts, retries=retries - 1)
        return {**more, "tried": [*tried, problem + ": picked it up and placed it again", *again["tried"], *more["tried"]]}
    raise RobotError(problem)


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(place_object)
