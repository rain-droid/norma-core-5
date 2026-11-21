"""Fit the overhead camera's pixel -> mat mapping from the four corner tags.

The mat has an AprilTag (family 25h9) at each corner. Their centres, measured with a ruler, are
in TAGS below in mat coordinates: metres, origin at tag 6 (the corner behind the robot on its
left), X towards tag 10 (the robot's right), Y towards tag 11 (forward). Four centres give the
homography from `top` pixels to that plane exactly. The tags are small in the image and do not
decode in every frame, so each is searched for over `frames` frames and its median is used.

Writes ../data/camera_top.json. Run it again whenever the camera or the mat may have moved:
`moved_px` says how far each tag is from the previous fit. `tag_side_mm` is each tag's size as
the fit measures it, a check on the ruler numbers (the tags are 20 mm). The mapping holds only
for points on the mat surface: the top of a 25 mm object reads 1-2 cm further from the camera.
"""

import json

import cv2
import numpy as np

from elrobot.robot import RobotError

MOTION = False

# shortcut: assumes the ruler distances (0.585 / 0.315 / 0.31 m) are tag centre to tag centre;
# if they were inner edge to inner edge, add 0.02 to each.
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


async def calibrate_top(ctx, frames: int = 30) -> dict:
    seen: dict[int, list] = {i: [] for i in TAGS}
    for _ in range(frames):
        img = await ctx.frame("top", settle=0.1)
        for i, c in detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)).items():
            if i in seen:
                seen[i].append(c)
    missing = [i for i, v in seen.items() if not v]
    if missing:
        raise RobotError(f"tag(s) {missing} never decoded in {frames} frames; check they are uncovered and inside the image "
                         "(the far ones need a short exposure, scripts/camera_exposure.py 40)")  # fmt: skip
    corners = {i: np.median(v, axis=0) for i, v in seen.items()}
    centres = {i: c.mean(axis=0) for i, c in corners.items()}
    H, _ = cv2.findHomography(np.array([centres[i] for i in TAGS]), np.array([TAGS[i] for i in TAGS]))

    def to_mat(px):
        return cv2.perspectiveTransform(np.asarray(px, dtype=np.float64).reshape(-1, 1, 2), H).reshape(-1, 2)

    sides = {i: round(float(np.mean(np.linalg.norm(np.roll(m, -1, axis=0) - m, axis=1))) * 1000, 1) for i, c in corners.items() for m in [to_mat(c)]}
    path = ctx.cfg.data_dir / "camera_top.json"
    old = json.loads(path.read_text()) if path.exists() else {}
    moved = {i: round(float(np.linalg.norm(centres[i] - old["tags_px"][str(i)])), 1) for i in TAGS} if old.get("tags_px") else None
    path.write_text(json.dumps({
        "H_mat_px": H.tolist(),
        "tags_px": {str(i): [round(float(v), 1) for v in centres[i]] for i in TAGS},
        "image_size": [img.shape[1], img.shape[0]],
        "base_in_mat": old.get("base_in_mat"),  # x, y, yaw of the robot base on the mat; fitted separately
    }, indent=1) + "\n")  # fmt: skip
    marked = img.copy()
    for i, c in corners.items():
        cv2.polylines(marked, [c.astype(np.int32)], True, (0, 255, 0), 2)
    return {
        "tags_px": {i: [round(float(v), 1) for v in centres[i]] for i in TAGS},
        "decoded_in_frames": {i: len(v) for i, v in seen.items()},
        "tag_side_mm": sides,
        "moved_px": moved,
        "image": ctx.save_image("tags.jpg", marked, "overhead view, decoded tags outlined"),
        "saved": str(path),
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(calibrate_top)
