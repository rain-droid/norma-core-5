"""Pick up the object at table position x, y: the whole grasp, built from the simpler tools.

Steps, each of which you can also do yourself with the tool named:
 1. open the jaws (open_gripper);
 2. choose an approach: straight down if the spot is within about 27 cm, otherwise leaning
    forward in 10 degree steps (util/motion.py approach_for), and move to a hover point 7 cm
    back along it (joint_command);
 3. with `refine`, look again from the hover point: the outline nearest to where the target
    should appear in the wrist image is taken to be the object and x, y are corrected, at most
    5 cm, up to twice (this is what absorbs camera calibration error);
 4. descend in a straight line to `grasp_height` above the table (move_tcp straight);
 5. close until the jaws stall (close_gripper);
 6. retreat to the hover point, lifting the object 7 cm, and check the jaws are still held apart.

`yaw` rotates the jaws about the vertical, degrees; 0 closes left-right. Pass the `yaw_deg`
find_objects reported so the jaws close across the narrow side. The jaws open 53 mm.
`grasp_height` is where the tool point ends up above the table: about half the object's height,
never less than 0.012.

Errors say which step failed. "closed on nothing" means the jaws shut completely at the grasp
point: the object is not where x, y says, so look again before retrying. A success here means
the jaws are held apart after the lift; confirm with a camera that it is the object you meant.
"""

import asyncio

import numpy as np

from elrobot import vision
from elrobot.robot import RobotError
from util import capture
from util.motion import approach_for, require_ready, tcp_report


async def refine_target(ctx, target: np.ndarray, max_shift: float = 0.05) -> np.ndarray:
    """From the hover pose, find the outline nearest the expected pixel and return its table point."""
    img = await ctx.frame("wrist")
    cam = ctx.skills.cam.scaled(img.shape[1], img.shape[0])
    T = ctx.robot.tcp()
    expected = cam.project([target[0], target[1], ctx.cfg.table_z], T)
    if expected is None:
        return target
    blob = vision.nearest_blob(vision.segment_objects(img, ignore=capture.jaw_mask(img), min_area=400), *expected, max_dist=0.3 * img.shape[1])
    if blob is None:
        return target
    p = cam.pixel_to_plane(*blob.centroid, T, ctx.cfg.table_z)
    if p is None or np.linalg.norm(p[:2] - target[:2]) > max_shift:
        return target
    return np.array([p[0], p[1], ctx.cfg.table_z])


async def pick_at(ctx, x: float, y: float, yaw: float | None = None, grasp_height: float = 0.018, refine: bool = True) -> dict:
    robot = ctx.robot
    require_ready(ctx)
    if robot.holding:
        raise RobotError("already holding something; place it first")
    target = np.array([x, y, ctx.cfg.table_z], dtype=float)
    corrections = []

    await robot.set_gripper(1.0, label="opening")
    approach, pitch, q_hover, hover_pos, grasp = approach_for(ctx, target, grasp_height, yaw)
    await robot.move_joints(q_hover, speed=0.9, label="reaching")
    if refine:
        for _ in range(2):
            better = await refine_target(ctx, target)
            shift = float(np.linalg.norm(better[:2] - target[:2]))
            if shift < 0.004:
                break
            corrections.append(round(shift * 1000, 1))
            target = better
            approach, pitch, q_hover, hover_pos, grasp = approach_for(ctx, target, grasp_height, yaw)
            await robot.move_joints(q_hover, speed=0.6, label="lining up")
    ctx.emit("markers", markers=[{"id": "target", "label": "pick", "pos": target.tolist(), "color": "#3ddc84", "radius": 0.014}])

    path = robot.plan_linear(robot._plan_start(), grasp, approach=approach, allow_low=True)
    await robot.move_path(path, speed=0.5, label="descending")
    grip = await robot.grasp()
    if not grip["holding"]:
        await robot.set_gripper(1.0, label="opening")
        await robot.move_path(path[::-1], speed=0.7, label="backing off")
        raise RobotError(
            f"closed on nothing at x={target[0]:+.3f} y={target[1]:+.3f}: the jaws shut completely. "
            "The object is not there; look again (look_at_table, find_objects) before retrying."
        )
    # back out to the hover point: known reachable, unlike a taller vertical lift at the edge of reach
    await robot.move_path(path[::-1], speed=0.6, label="lifting")
    await asyncio.sleep(0.25)
    if not robot.still_holding():
        raise RobotError("had it, but it slipped out while lifting; look again to see where it ended up")
    return {
        "holding": True,
        "width_mm": grip["width_mm"],
        "picked_from": [round(float(target[0]), 3), round(float(target[1]), 3)],
        "refine_corrections_mm": corrections,
        "approach_pitch_deg": pitch,
        "tcp": tcp_report(ctx),
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(pick_at)
