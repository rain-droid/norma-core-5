"""Fit the top camera to the mat from the four corner tags.

The mat has an AprilTag (family 25h9) at each corner. Their centres, measured with a ruler,
are in TAGS below in mat coordinates: metres, origin at tag 6 (the corner behind the robot on
its left), X towards tag 10 (the robot's right), Y towards tag 11 (forward). Four centres give
the homography from `top` pixels to the mat surface exactly, and that homography also says
where the camera is (util/capture.py, camera_from_homography), which is what lets
pixel_to_table handle points above the mat. The tags are small in the image and do not decode
in every frame, so each is searched for over `frames` frames and its median is used.

Writes robot_config/camera_top.json. Run it again whenever the camera or the mat may have
moved: `moved_px` says how far each tag is from the previous fit. `tag_side_mm` is each tag's
size as the fit measures it, a check on the ruler numbers (the tags are 19 mm), and
`camera_on_mat_m` should be where the camera actually stands.

Not fitted here: `base_in_mat`, where the robot's base sits on the mat (x, y in metres and
rotation in degrees). It was set by hovering the gripper over an object located with this
camera and correcting until it was centred; it stays as it is in the file.

The fit belongs to one frame size (`image_size` in the file). When the tracker lists an object
in the middle of the mat under `at_edge`, compare `image_size` with the frame: a fit made on
640x480 frames is 106 px off in u on the 853x480 frames the daemon sends now. After writing a
new fit, restart the tracker (track_objects stop, then read): it loads the file once. Tag 5
(near corner, left in the image) sits half below the frame on 853x480, so it does not decode
until the camera is tilted down a little.

The far tags only decode with a short exposure, which is set on the robot side (in the
repository that runs the daemon: scripts/camera_exposure.py 40, with the Station stopped).
"""

import json

import cv2
import numpy as np

from util import capture
from util.arm import RobotError

# shortcut: ruler distances between tag centres; re-measure if the tags are moved
TAGS = {6: (0.0, 0.0), 10: (0.585, 0.0), 11: (0.0, 0.315), 5: (0.585, 0.31)}


def detect(gray: np.ndarray) -> dict[int, np.ndarray]:
    """Tag id -> 4x2 corner pixels. Upscaled first: the tags are about 15 px wide."""
    aruco = cv2.aruco
    params = aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin, params.adaptiveThreshWinSizeMax, params.adaptiveThreshWinSizeStep = 3, 45, 4
    params.minMarkerPerimeterRate = 0.005
    detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_25h9), params)
    found = {}
    for scale in (3, 4, 5):
        corners, ids, _ = detector.detectMarkers(cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC))
        for i, c in zip(np.ravel(ids) if ids is not None else [], corners):
            found.setdefault(int(i), c[0] / scale)
    return found


def calibrate_top(ctx, frames: int = 30) -> dict:
    if ctx.arm.meta()["mode"] == "sim":
        raise RobotError("the simulated top camera has no tags; its exact calibration is written to robot_config/camera_top.sim.json when the daemon starts")
    seen: dict[int, list] = {i: [] for i in TAGS}
    for _ in range(frames):
        img = ctx.frame("top", settle=0.1)
        for i, c in detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)).items():
            if i in seen:
                seen[i].append(c)
    missing = [i for i, v in seen.items() if not v]
    if missing:
        raise RobotError(f"tag(s) {missing} never decoded in {frames} frames; check they are uncovered and inside the image "
                         "(the far ones need a short exposure, set on the robot side)")  # fmt: skip
    corners = {i: np.median(v, axis=0) for i, v in seen.items()}
    centres = {i: c.mean(axis=0) for i, c in corners.items()}
    H, _ = cv2.findHomography(np.array([centres[i] for i in TAGS]), np.array([TAGS[i] for i in TAGS]))
    size = [img.shape[1], img.shape[0]]
    P, eye = capture.camera_from_homography(H, size)

    def to_mat(px):
        return cv2.perspectiveTransform(np.asarray(px, dtype=np.float64).reshape(-1, 1, 2), H).reshape(-1, 2)

    sides = {i: round(float(np.mean(np.linalg.norm(np.roll(m, -1, axis=0) - m, axis=1))) * 1000, 1) for i, c in corners.items() for m in [to_mat(c)]}
    path = ctx.root / "robot_config" / "camera_top.json"
    old = json.loads(path.read_text()) if path.exists() else {}
    moved = {i: round(float(np.linalg.norm(centres[i] - old["tags_px"][str(i)])), 1) for i in TAGS} if old.get("tags_px") else None
    path.write_text(json.dumps({
        "H_mat_px": H.tolist(),
        "tags_px": {str(i): [round(float(v), 1) for v in centres[i]] for i in TAGS},
        "image_size": size,
        "base_in_mat": old.get("base_in_mat"),
        "P_px_mat": P.tolist(),
        "camera_mat": [round(float(v), 4) for v in eye],
    }, indent=1) + "\n")  # fmt: skip
    marked = img.copy()
    for c in corners.values():
        cv2.polylines(marked, [c.astype(np.int32)], True, (0, 255, 0), 2)
    return {
        "tags_px": {i: [round(float(v), 1) for v in centres[i]] for i in TAGS},
        "decoded_in_frames": {i: len(v) for i, v in seen.items()},
        "tag_side_mm": sides,
        "camera_on_mat_m": [round(float(v), 3) for v in eye],
        "moved_px": moved,
        "base_in_mat": old.get("base_in_mat"),
        "image": ctx.save_image("tags.jpg", marked, "top view, decoded tags outlined"),
        "saved": str(path.relative_to(ctx.root)),
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(calibrate_top)
