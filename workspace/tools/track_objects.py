"""Live tracking of what is on the table: read it, and start or stop it.

`action: "read"` (default) returns every object the top camera is following, by a name that
stays the same while the object is on the table (`red-1`, `blue-1`, ...): where it is now,
whether it is `visible`, `hidden` behind the arm or a hand, `held` in the gripper or `missing`,
how long it has been lying still (`still_s`), and what it has been put on (`on`). It starts the
tracker if it is not running. Open `image` to see which outline has which name.

Use these names with pick_object and place_object: they read the positions at the moment
they act, so an object that has been moved since you looked is picked up where it is.

The same numbers are in perception/exteroception/tracks.json, rewritten ten times a second;
tracks.jpg beside it is the view. util/tracker.py is the process that writes them.

`intrusion` is set while something is moving on the table that the arm did not move: a thing
coming in over the edge of the work area, something new appearing, a tracked object sliding.
From this camera that is what a hand looks like. `clear_s` says how long the table has been
quiet. While it is not quiet the tracker stops any motion of the arm; pick_object and
place_object wait and carry on, other tools return a "stopped" error. A thing lying still is
never an intrusion: a motionless hand is not seen as one. Nor is anything within a few
centimetres of the arm, where its own parts and what it holds move. To see what set it off
last, open perception/exteroception/intrusion.jpg: the spot is circled.
`stop_arm_on_intrusion: false` restarts the tracker without the stopping, for when it stops
the arm for no reason; say so to whoever is standing nearby. It stays off until set to true.

`at_edge` lists things lying across the edge of the searched area. They cannot be measured
there or picked by name: move them in, or use capture_camera and pixel_to_table.

`action: "stop"` ends the tracker. It also ends by itself when the robot side goes away.

Positions are what find_objects measures, one frame at a time: good to about a centimetre.
What the table's own colour hides from find_objects is hidden here too.
"""

import cv2

from util import tracker
from util.arm import RobotError


def track_objects(ctx, action: str = "read", stop_arm_on_intrusion: bool | None = None) -> dict:
    if action == "stop":
        return {"tracking": False, "was_running": tracker.stop()}
    if action != "read":
        raise RobotError(f"action is 'read' or 'stop', not '{action}'")
    now = tracker.read()
    if stop_arm_on_intrusion is not None:
        tracker.RUNTIME.mkdir(exist_ok=True)
        (tracker.NO_STOP.unlink(missing_ok=True) if stop_arm_on_intrusion else tracker.NO_STOP.touch())
        if now and now["stops_arm_on_intrusion"] != stop_arm_on_intrusion:
            tracker.stop()
    data = tracker.start()
    view = cv2.imread(str(tracker.FILE.with_suffix(".jpg")))
    return {
        "tracking": True,
        "image": ctx.save_image("tracks.jpg", view, "top view: each tracked object outlined and named") if view is not None else None,
        "clear_s": data["clear_s"],
        "intrusion": data["intrusion"],
        "stops_arm_on_intrusion": data["stops_arm_on_intrusion"],
        "holding": data["holding"],
        "tracks": [{k: v for k, v in t.items() if k not in ("pixel", "area_px")} for t in data["tracks"]],
        "at_edge": data["at_edge"],
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(track_objects)
