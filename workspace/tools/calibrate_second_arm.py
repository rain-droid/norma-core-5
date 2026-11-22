"""Find where the second arm (the SO-101) stands in this arm's base frame, with the depth camera.

The second arm must be holding something of one colour that nothing else in the air has (the
pink foam block: its own parts are blue, black and orange), level, in its jaws. It then carries
it through a dozen places on this arm's side of its reach; at each the depth camera measures
where the thing is (tools/util/depth.py, find_in_air) and the second arm reports where its tool
point is in its own frame. The rigid transform that best carries the one set of points onto the
other is `T_base_second`, written to robot_config/second_arm.json.

Because the thing is measured in the second arm's jaws by the same camera, in the same way, as
it later is in this arm's jaws at a hand-over, what the camera gets wrong about its middle
cancels. `miss_mm` is how far each place lies from the fit: a few millimetres is good, and a
place more than 15 mm off is left out and the fit redone. Run it again whenever either arm or
the depth camera has been moved. This arm goes to rest first, out of the way.
"""

import json
import math
import time

import numpy as np

from rest import rest
from util import depth as depth_camera
from util import second_arm
from util.arm import RobotError

# degrees left (negative) of straight ahead, metres out, metres up. This arm's gripper hangs over the
# mat even at rest, about 30 cm to the second arm's left: nothing here goes further left than 16 cm.
PLACES = [(pan, r, z) for pan in (-38, -25, -12, 0, 12) for r, z in ((0.25, 0.15), (0.28, 0.19), (0.24, 0.20)) if r * math.sin(math.radians(pan)) > -0.16]


def calibrate_second_arm(ctx, colour: str = "pink") -> dict:
    if ctx.arm.meta()["mode"] == "sim":
        raise RobotError("the simulated table has no second arm")
    if ctx.arm.state()["torque"]:
        ctx.call(rest)  # folded and out of the second arm's way
    seen_second, seen_base, skipped = [], [], []
    for pan, r, z in PLACES:
        x, y = r * math.sin(math.radians(pan)), r * math.cos(math.radians(pan))
        try:
            second_arm.call("move_to", x=round(x, 3), y=round(y, 3), z=z, pitch=0, yaw=-90, speed=0.5)
        except RobotError as e:
            skipped.append({"place": [pan, r, z], "why": str(e)[:90]})
            continue
        time.sleep(0.6)
        found = depth_camera.find_in_air(ctx, colour, min_height=0.06)
        if found is None or found["points"] < 80:
            skipped.append({"place": [pan, r, z], "why": f"the depth camera does not see the {colour} thing there"})
            continue
        seen_second.append(second_arm.tool_point())
        seen_base.append(found["pos"])
    if len(seen_base) < 5:
        raise RobotError(f"only {len(seen_base)} places were measured and the fit needs five: {skipped}")
    a, b = np.array(seen_second), np.array(seen_base)
    T, miss = second_arm.fit(a, b)
    keep = miss < 0.015
    if 5 <= keep.sum() < len(miss):
        T, miss_kept = second_arm.fit(a[keep], b[keep])
    else:
        keep, miss_kept = np.ones(len(miss), bool), miss
    yaw = math.degrees(math.atan2(T[1, 0], T[0, 0]))
    path = ctx.root / "robot_config" / "second_arm.json"
    path.write_text(json.dumps({
        "T_base_second": [[round(float(v), 6) for v in row] for row in T],
        "miss_mm": [round(float(m) * 1000, 1) for m in miss_kept],
        "places": int(keep.sum()),
        "colour": colour,
        "fitted": time.strftime("%Y-%m-%d %H:%M"),
    }, indent=1) + "\n")  # fmt: skip
    return {
        "second_arm_base": [round(float(v), 3) for v in T[:3, 3]],
        "turned_deg": round(yaw, 1),
        "tilt_deg": round(math.degrees(math.acos(min(1.0, float(T[2, 2])))), 1),
        "miss_mm": [round(float(m) * 1000, 1) for m in miss_kept],
        "places_used": int(keep.sum()),
        "left_out": int((~keep).sum()),
        "skipped": skipped,
        "saved": str(path.relative_to(ctx.root)),
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(calibrate_second_arm)
