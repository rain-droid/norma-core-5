"""Draw the joint manual: visuals/jointN.jpg shows what each joint does.

    pixi run python robot_config/build_visuals.py

One picture per joint: the arm with every other joint at zero (stretched out level, pointing
forward), and that joint at -90, 0 and +90 degrees, or at its limit where that comes first.
Orange is what the joint carries, the cyan arrow its axis: a positive angle turns
counter-clockwise seen from the arrow's tip. The view is the one that looks along the axis.
`gripper.jpg` shows the jaws closed, half open and open; `three_views.jpg` the ready pose.

These are pictures of the model in poses chosen to explain, not poses to send: some put the
arm through the table. Run it again after changing model.urdf.
"""

import math
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tools"))

from util import render  # noqa: E402
from util.motion import READY_DEG  # noqa: E402

NAMES = {1: "base rotation", 2: "shoulder", 3: "lower elbow", 4: "upper elbow", 5: "forearm roll", 6: "wrist bend", 7: "wrist roll"}


def joint_picture(model: render.Model, n: int) -> np.ndarray:
    joint = next(j for j in model.joints if j["name"] == render.ARM_JOINTS[n - 1])
    carried = model.moved_by(joint["name"])
    angles = [max(math.degrees(joint["lower"]), -90.0), 0.0, min(math.degrees(joint["upper"]), 90.0)]
    zero = model.values(np.zeros(7))
    frame = model.poses(zero)[1][joint["name"]]
    origin, axis = frame[:3, 3], frame[:3, :3] @ (joint["axis"] / np.linalg.norm(joint["axis"]))
    view = max(render.VIEWS, key=lambda v: abs(float(render.VIEWS[v][2] @ axis)))
    poses = [model.triangles({**zero, joint["name"]: math.radians(a)}, highlight=carried) for a in angles]
    centre, half = render.bounds_of(np.concatenate([t for t, _ in poses]))
    panels = []
    for angle, (tris, colors) in zip(angles, poses):
        panel = render.Panel(view, centre, half)
        panel.grid()
        panel.solids(tris, colors)
        panel.axis(origin, axis)
        panels.append(panel.title(f"joint {n} ({NAMES[n]}) at {angle:+.0f} deg"))
    return np.hstack(panels)


def gripper_picture(model: render.Model) -> np.ndarray:
    hand = model.moved_by(render.ARM_JOINTS[6])
    zero = np.zeros(7)
    centre, half = None, None
    panels = []
    for opening in (0.0, 0.5, 1.0):
        values = model.values(zero, opening)
        links, _ = model.poses(values)
        if centre is None:
            centre, half = render.bounds_of(np.concatenate([model.meshes[name] @ links[name][:3, :3].T + links[name][:3, 3] for name in hand]), margin=0.02)
        panel = render.Panel("from above", centre, half)
        panel.grid(step=0.02)
        panel.solids(*model.triangles(values, highlight={"Gripper_Jaw_01_v1_1", "Gripper_Jaw_02_v1_1"}), density=6.0)
        panels.append(panel.title(f"gripper at {opening:g} ({opening * 53.6:.0f} mm between the pads)"))
    return np.hstack(panels)


if __name__ == "__main__":
    out = HERE / "visuals"
    out.mkdir(exist_ok=True)
    model = render.Model(HERE / "model.urdf")
    for n in NAMES:
        cv2.imwrite(str(out / f"joint{n}.jpg"), joint_picture(model, n), [cv2.IMWRITE_JPEG_QUALITY, 88])
    cv2.imwrite(str(out / "gripper.jpg"), gripper_picture(model), [cv2.IMWRITE_JPEG_QUALITY, 88])
    ready = np.radians(READY_DEG)
    cv2.imwrite(str(out / "three_views.jpg"), render.three_views(model, ready, 1.0), [cv2.IMWRITE_JPEG_QUALITY, 88])
    print(f"wrote {len(NAMES) + 2} pictures to {out}")
