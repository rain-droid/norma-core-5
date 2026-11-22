"""The second arm (the SO-101) picks a thing off the mat and sets it on top of another: its half of a stack built by both arms.

    second_arm_stack(thing=[0.115, 0.176], on=[0.20, 0.16], on_height_mm=22)

`thing`: x, y of the thing to pick (this arm's base frame); `thing_height_mm` its height, 30 for
the cube. `jaws_along_deg` turns the jaws to close along a direction on the table (degrees from
this arm's x axis), for a thing that has something right beside it. `on`: x, y of the middle of what it goes on, and `on_height_mm` how high that stands:
take both from depth_scan or scan_obstacles, made while no arm hangs over the place. This arm
should be out of the way (home or rest); it is not moved.

What happens: the second arm opens and goes above the thing. Where its jaws are by its own
joints is wrong by up to 5 cm down at the table (its place was fitted in the air, with level
jaws), so the top camera, which looks straight down on its orange gripper, says where they
really are, and the arm is shifted by the difference, up to three times. Then it comes down and
closes, lifts, goes above the base, is brought over its middle the same way, comes down until
the thing rests on the base, opens, rises and goes home. Every move is planned in the shared
space and checked against this arm and, after a scan_obstacles, against the things on the
table; that check places the second arm by its joints, so near the table it is as wrong as they are.

Returns `holding` after the pick, how far the jaws were off before each shift (`pick_off_mm`, `off_mm`), and
`stack_height_mm`, what the depth camera then measures at the place: it should be the base's
height plus the thing's. The second arm leans where it cannot point straight down
(`pitch_deg`), and a thing set down from leaning jaws lands a little turned.
"""

import math
import time

import cv2
import numpy as np

from util import capture
from util import depth as depth_camera
from util import second_arm as arm2
from util import shared_space
from util.arm import RobotError


def _yaw_for(root, pos, jaws_along_deg: float) -> float:
    """The jaw turn that makes the second arm's jaws, pointing down at `pos`, close along a direction on the table (degrees from this arm's x axis)."""
    space = shared_space.space(root)
    want = np.array([math.cos(math.radians(jaws_along_deg)), math.sin(math.radians(jaws_along_deg))])
    best = (-1.0, 0.0)
    for yaw in range(-90, 51, 5):  # its wrist roll turns from -157 to +56 degrees
        for pitch in (90.0, 75.0, 60.0):
            try:
                q = space.second_ik(pos, pitch, yaw)
            except RobotError:
                continue
            along = space.second_jaws_along(q)[:2]
            best = max(best, (abs(float(along @ want)) / max(float(np.linalg.norm(along)), 1e-9), float(yaw)))
            break
    return best[1]


def _jaws_from_above(ctx, believed, height: float) -> np.ndarray | None:
    """Where the second arm's orange gripper really is, x and y, as the top camera sees it: the
    middle of the orange patch nearest to where its joints say it is, taken `height` metres above
    the mat (the middle of the gripper's body when it points down)."""
    img = ctx.frame("top", settle=0.2)
    top = capture.TopCamera(ctx.root, "real", ctx.plan.table_z)
    # the gripper's orange is bright; the maroon ring is the same hue and dark, and once passed for it
    mask = cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), np.array([4, 120, 150]), np.array([22, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, _, stats, centres = cv2.connectedComponentsWithStats(mask)
    expect = top.to_pixel([believed[0], believed[1], ctx.plan.table_z + height])
    best = None
    for i in range(1, count):
        # From above most of the gripper is under its own camera: what shows is a few hundred pixels of orange.
        # Its joints are never more than about 3 cm out, which is 40 pixels with the height's parallax.
        if stats[i, cv2.CC_STAT_AREA] < 100 or np.linalg.norm(centres[i] - expect) > 60:
            continue
        if best is None or np.linalg.norm(centres[i] - expect) < np.linalg.norm(centres[best] - expect):
            best = i  # the patch nearest to where the joints say the gripper is
    if best is None:
        return None
    return top.to_base(float(centres[best][0]), float(centres[best][1]), height)[:2]


def _bring_over(ctx, this_q, xy, z: float, yaw: float, note: list) -> np.ndarray:
    """Put the jaws over a spot at tool height z, by the top camera: go where the joints say that is,
    look where the gripper really is, shift by the difference, up to three times. Returns the
    place asked for in the end, which carries the correction for the way down."""
    root, table_z = ctx.root, ctx.plan.table_z
    target = np.array([xy[0], xy[1], z], dtype=float)
    for _ in range(4):
        arm2.reach(root, target, this_q, yaw, speed=0.3)
        time.sleep(0.9)
        seen = _jaws_from_above(ctx, arm2.tool_in_base(root), z - table_z + 0.045)
        if seen is None:
            note.append(None)  # the top camera does not see the gripper there: its joints have to do
            break
        off = np.asarray(xy[:2], dtype=float) - seen
        note.append(round(float(np.linalg.norm(off)) * 1000))
        if note[-1] < 5 or note[-1] > 100 or len(note) >= 4:
            break
        target[:2] += off
    return target


def second_arm_stack(ctx, thing: list, on: list, on_height_mm: float, thing_height_mm: float = 30.0, colour: str = "blue",
                     jaws_along_deg: float | None = None, speed: float = 0.5) -> dict:  # fmt: skip
    root, table_z = ctx.root, ctx.plan.table_z
    this_q = ctx.arm.q()
    tall, base = thing_height_mm / 1000.0, on_height_mm / 1000.0
    if not arm2.state()["torque"]:
        arm2.call("home")
    grasp_at = [thing[0], thing[1], table_z + max(0.02, tall - 0.008)]
    yaw = 0.0 if jaws_along_deg is None else _yaw_for(root, grasp_at, jaws_along_deg)

    # pick: over the thing by the top camera, down, close
    arm2.call("gripper", open=1.0)
    pick_off: list = []
    above = _bring_over(ctx, this_q, thing, table_z + tall + 0.05, yaw, pick_off)
    pitch = arm2.reach(root, [above[0], above[1], grasp_at[2]], this_q, yaw, speed=0.3)
    time.sleep(0.5)
    held = arm2.call("grasp").get("data") or {}
    if not held.get("holding"):
        arm2.call("gripper", open=1.0)
        arm2.reach(root, above, this_q, yaw, speed=speed)
        raise RobotError(f"the second arm closed on nothing at x={thing[0]:+.3f} y={thing[1]:+.3f} (the top camera had it {pick_off} mm off before each shift): "
                         "it is back above the place, jaws open. Look again where the thing is.")  # fmt: skip
    arm2.reach(root, [above[0], above[1], table_z + base + tall + 0.07], this_q, yaw, speed=0.4)

    # over the base by the top camera, down onto it, let go, away
    off_mm: list = []
    over = _bring_over(ctx, this_q, on, table_z + base + tall + 0.05, 0.0, off_mm)
    arm2.reach(root, [over[0], over[1], table_z + base + max(0.02, tall - 0.008) + 0.004], this_q, speed=0.25)
    time.sleep(0.5)
    arm2.call("gripper", open=1.0)
    time.sleep(0.4)
    arm2.reach(root, over, this_q, speed=0.3)
    arm2.home_checked(root, this_q, speed)
    time.sleep(0.6)

    depth, cal = depth_camera.grab(root)
    grid = depth_camera.height_map(depth_camera.to_base(depth_camera.points(depth, cal["depth_K"]), depth_camera.load(root)), table_z)
    height = depth_camera.height_at(grid, float(on[0]), float(on[1]), radius=0.015)
    return {
        "holding": True,
        "held_width_mm": held.get("width_mm"),
        "pitch_deg": pitch,
        "pick_off_mm": pick_off,
        "off_mm": off_mm,
        "stack_height_mm": None if height is None else round(height * 1000),
        "expected_height_mm": round(on_height_mm + thing_height_mm),
        "image": ctx.save_image("stack.jpg", ctx.frame("across"), "front view after the second arm set the thing down"),
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(second_arm_stack)
