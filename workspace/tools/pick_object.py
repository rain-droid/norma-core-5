"""Pick up a tracked object by name, wherever it is by the time the gripper gets there.

`object` is a name from track_objects (`red-1`). The grasp itself is pick_at's: open, hover
7 cm back along the approach, descend in a straight line, close until the jaws stall, lift.
What this adds is that it keeps reading the tracker while it works:

* it waits until the object is in view and lying still;
* it aims at where the object is then, with the jaws across its flat sides;
* at the hover point it asks whether the object has been moved since, and if so aims again
  instead of descending;
* while something moves on the table that the arm did not move (a hand, most likely), the
  tracker slows the arm down and it carries on. A tracker set to stop instead
  (track_objects `on_intrusion: "stop"`) halts it: then this waits for quiet and starts over
  from where the object is now, which does not count against `attempts`;
* if the jaws close on nothing, or the object slips out while lifting, it looks and tries again;
* if the grasp is refused because leaning, turned jaws would dip a fingertip into the table,
  it grasps 5 mm higher, and then with the jaws unturned.

Each pass through that counts as one of `attempts`. `tried` in the result says what happened.
An object out of sight behind the arm sends the arm home once to uncover it.

`grasp_height` is the tool point's height above whatever the object stands on; left out, it is
half the object's measured height, between 22 and 30 mm. The fingertips reach 12 mm below the
tool point and the arm sags a few millimetres when stretched, so lower than that they dig
into the mat. Give it yourself, lower, only for something flatter than about 15 mm. An object
lying on another one (`on` in its track) is picked from on top of it.

Errors: the object's name is unknown (the names there are get listed), it cannot be reached,
or it could not be picked in `attempts` passes, with what happened in each. As with pick_at,
success means the jaws are held apart after the lift.
"""

import time

import numpy as np

from home import home
from util import tracker
from util.arm import RobotError
from util.motion import require_ready, tcp_report


def approach(plan, target, height: float, yaw: float, tried: list[str]):
    """plan.approach_for, giving way when the first choice is refused by the clearance check:
    a little higher, then also with the jaws unturned."""
    choices = [(height, yaw), (height + 0.005, yaw), (height + 0.005, 0.0)]
    for n, (h, w) in enumerate(choices):
        try:
            found = plan.approach_for(target, h, w)
        except RobotError as e:
            if "refused" not in str(e) or n == len(choices) - 1:
                raise
            continue
        if n:
            tried.append(f"grasping at {h * 1000:.0f} mm with yaw {w:.0f}: lower and turned, a fingertip would have dipped into the table")
        return found


def pick_object(ctx, object: str, grasp_height: float | None = None, attempts: int = 4) -> dict:
    arm, plan = ctx.arm, ctx.plan
    if require_ready(ctx, awake=True)["gripper"]["holding"]:
        raise RobotError("already holding something; place it first")
    tried: list[str] = []
    aim = None  # where the object was seen to have gone, when that is fresh enough to go straight there
    left, waits = attempts, 0
    while left > 0:
        left -= 1
        try:
            if arm.state()["gripper"]["holding"]:  # stopped after the jaws had closed: all that is left is to lift it clear
                tracker.wait_for(ctx)
                res = ctx.call(home)
                return {"holding": True, "object": object, "tried": tried, "tcp": res["tcp"]}
            t, aim = aim or tracker.wait_for(ctx, object), None
            target = np.array([t["x"], t["y"], plan.table_z])
            own = grasp_height if grasp_height is not None else float(np.clip((t["height_mm"] or 44) / 2000, 0.022, 0.030))
            a, pitch, q_hover, _, grasp = approach(plan, target, t["z_mm"] / 1000 + own, t["yaw_deg"], tried)
            down = plan.line(q_hover, grasp, approach=a, allow_low=True)
            ctx.emit("markers", markers=[{"id": "target", "label": f"pick {object}", "pos": target.tolist(), "color": "#3ddc84", "radius": 0.014}])
            arm.joints(gripper=1.0, label="opening")
            arm.joints(q_hover, speed=0.9, label="reaching")
            now = tracker.track(tracker.fresh(), object)
            if now["state"] == "missing" or now["moves"] != t["moves"]:
                moved = float(np.hypot(now["x"] - t["x"], now["y"] - t["y"]))
                tried.append(f"{object} was moved {moved * 1000:.0f} mm while the arm was reaching: aiming again")
                if now["state"] != "missing" and now["seen_s"] < 1.5 and now["still_s"] > 0.4:
                    aim = now  # measured at rest in its new place just before the arm came over it
                continue
            arm.joints(down, speed=0.5, allow_low=True, label="descending")
            grip = arm.joints(gripper=0.0, label="grasping")["gripper"]
            if not grip["holding"]:
                arm.joints(gripper=1.0, label="opening")
                arm.joints(down[::-1], speed=0.7, allow_low=True, label="backing off")
                tried.append(f"closed on nothing at x={target[0]:+.3f} y={target[1]:+.3f}: looking again")
                continue
            res = arm.joints(down[::-1], speed=0.6, allow_low=True, label="lifting")
            time.sleep(0.25)
            if not arm.state()["gripper"]["holding"]:
                tried.append("it slipped out while lifting: looking again")
                continue
            return {
                "holding": True,
                "object": object,
                "width_mm": grip["width_mm"],
                "picked_from": [round(float(target[0]), 3), round(float(target[1]), 3)],
                "approach_pitch_deg": pitch,
                "tried": tried,
                "tcp": tcp_report(res["tcp"], res["gripper"]),
            }
        except RobotError as e:
            if not tracker.stopped_by_tracker(e):
                raise
            tried.append("the arm was stopped because something moved in the work area: waiting for it to go quiet")
            waits += 1
            left += waits <= 6  # being interrupted is not a failed attempt, up to a point
    raise RobotError(f"could not pick {object} in {attempts} attempts: " + "; ".join(tried))


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(pick_object)
