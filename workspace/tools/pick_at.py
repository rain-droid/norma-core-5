"""Pick up the object at table position x, y: the whole grasp, built from joint commands.

Steps, each of which you can also do yourself with the tool named:
 1. open the jaws (open_gripper);
 2. choose an approach: straight down if the spot is within about 27 cm, otherwise leaning
    forward in 10 degree steps (util/planning.py approach_for), and go to a hover point 7 cm
    back along it (joint_command);
 3. descend in a straight line to `grasp_height` above the table (move_tcp straight);
 4. close until the jaws stall (close_gripper);
 5. retreat up the same line to the hover point, lifting the object 7 cm, and check the jaws
    are still held apart.

x, y is the centre of the object where it stands on the table: measure it with the top camera
(find_objects, pixel_to_table). The jaws only centre an object along the direction they close
in; along the finger pads it stays wherever x, y put it.

`yaw_deg` rotates the jaws about the vertical, in degrees; 0 closes left-right. Pass the
`yaw_deg` find_objects reported so the jaws close across a flat side. The jaws open 53 mm.
`grasp_height` is where the tool point ends up above the table: about half the object's height.
The fingertips are 12 mm beyond the tool point, so
below 0.020 they press into the mat, so anything lower is raised to 0.020 (MIN_GRASP_HEIGHT; the
result says so). On the real arm the planner adds the 10 mm model correction plus the
extra_descent_clearance_m setting (currently 15 mm), keeping the old hover height so the
descent is shorter. Do not subtract these offsets from grasp_height after an empty grasp.
Use fresh top-camera x/y and depth_scan(at=[[x,y]]) for object height before descending;
scan_obstacles installs the separate obstacle map. A short object is gripped by its upper part.
Beyond about 27 cm the
gripper has to lean, and leaning jaws that are also turned (`yaw_deg` not 0) dip one fingertip
towards the table.

`refine: true` adds a look through the wrist camera from the hover point and shifts x, y onto
the outline nearest to where the target should appear (at most 5 cm, twice). Only use it once
the wrist camera is calibrated: with the default model it pulls the target centimetres off.

Errors say which step failed. "closed on nothing" means the jaws shut completely at the grasp
point: the object is not where x, y says, so look again before retrying. A success here means
the jaws are held apart after the lift; confirm with a camera that it is the object you meant.
"""

import time

import numpy as np

from util import capture, vision
from util.arm import RobotError
from util.motion import require_ready, tcp_report, carry

MIN_GRASP_HEIGHT = 0.020  # metres above the table; lower values are raised to this


def refine_target(ctx, target: np.ndarray, max_shift: float = 0.05) -> np.ndarray:
    """From the hover pose, find the outline nearest the expected pixel and return its table point."""
    img = ctx.frame("wrist")
    cam = capture.wrist_camera(ctx.root).scaled(img.shape[1], img.shape[0])
    T = ctx.plan.fk(ctx.arm.q())
    expected = cam.project(target, T)
    if expected is None:
        return target
    blob = vision.nearest_blob(vision.segment_objects(img, ignore=capture.jaw_mask(img), min_area=400), *expected, max_dist=0.3 * img.shape[1])
    if blob is None:
        return target
    p = cam.pixel_to_plane(*blob.centroid, T, target[2])
    if p is None or np.linalg.norm(p[:2] - target[:2]) > max_shift:
        return target
    return np.array([p[0], p[1], target[2]])


def pick_at(ctx, x: float, y: float, yaw_deg: float | None = None, grasp_height: float = 0.022, refine: bool = False) -> dict:
    arm, plan = ctx.arm, ctx.plan
    if require_ready(ctx, awake=True)["gripper"]["holding"]:
        raise RobotError("already holding something; place it first")
    target = np.array([x, y, plan.table_z], dtype=float)
    corrections = []
    # the fingertips reach 12 mm past the tool point and the arm sags: below MIN_GRASP_HEIGHT they dig into the mat
    asked_height = grasp_height
    grasp_height = max(grasp_height, MIN_GRASP_HEIGHT)

    # plan the whole approach before anything moves, so an impossible grasp is refused with the arm where it was
    approach, pitch, q_hover, hover_pos, grasp = plan.approach_for(target, grasp_height, yaw_deg)
    down = plan.line(q_hover, grasp, approach=approach, allow_low=True)
    arm.joints(gripper=1.0, label="opening")
    carry(ctx, q_hover, hover_pos, speed=0.9, label="reaching")  # over what stands on the way, when the table has been scanned
    if refine:
        for _ in range(2):
            better = refine_target(ctx, target)
            shift = float(np.linalg.norm(better[:2] - target[:2]))
            if shift < 0.004:
                break
            corrections.append(round(shift * 1000, 1))
            target = better
            approach, pitch, q_hover, hover_pos, grasp = plan.approach_for(target, grasp_height, yaw_deg)
            down = plan.line(q_hover, grasp, approach=approach, allow_low=True)
            arm.joints(q_hover, speed=0.6, label="lining up")
    ctx.emit("markers", markers=[{"id": "target", "label": "pick", "pos": target.tolist(), "color": "#3ddc84", "radius": 0.014}])

    arm.joints(down, speed=0.5, allow_low=True, label="descending")
    grip = arm.joints(gripper=0.0, label="grasping")["gripper"]
    if not grip["holding"]:
        arm.joints(gripper=1.0, label="opening")
        arm.joints(down[::-1], speed=0.7, allow_low=True, label="backing off")
        raise RobotError(
            f"closed on nothing at x={target[0]:+.3f} y={target[1]:+.3f}: the jaws shut completely. "
            "The object is not there; look again (capture_camera, find_objects) before retrying."
        )
    # back up the line it came down: known reachable, unlike a taller vertical lift at the edge of reach
    res = arm.joints(down[::-1], speed=0.6, allow_low=True, label="lifting")
    time.sleep(0.25)
    if not arm.state()["gripper"]["holding"]:
        raise RobotError("had it, but it slipped out while lifting; look again to see where it ended up")
    return {
        "holding": True,
        "width_mm": grip["width_mm"],
        "picked_from": [round(float(target[0]), 3), round(float(target[1]), 3)],
        "refine_corrections_mm": corrections,
        "grasp_height_m": round(grasp_height, 3),
        **({"note": f"grasp_height {asked_height} raised to {MIN_GRASP_HEIGHT}: lower digs into the mat"} if grasp_height != asked_height else {}),
        "approach_pitch_deg": pitch,
        "tcp": tcp_report(res["tcp"], res["gripper"]),
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(pick_at)
