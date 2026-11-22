"""Camera calibration helpers: chessboard detection, view selection and intrinsic calibration.

The lens model matches `vision.CameraModel`: pinhole (fx, fy, cx, cy) plus two radial
distortion terms k1, k2. Tangential distortion and k3 are fixed at zero, which is stable with
the 15-25 views a hand-held session produces and is all a webcam lens needs.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

import cv2
import numpy as np

CALIB_FLAGS = cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3


@dataclass
class Chessboard:
    """A flat chessboard. `cols` x `rows` counts INNER corners (where four squares meet)."""

    cols: int
    rows: int
    square_m: float

    def object_points(self) -> np.ndarray:
        """Corner positions on the board plane (z = 0), in metres, row-major like OpenCV returns them."""
        grid = np.zeros((self.rows * self.cols, 3), np.float32)
        grid[:, :2] = np.mgrid[0 : self.cols, 0 : self.rows].T.reshape(-1, 2) * self.square_m
        return grid


def gray(img: np.ndarray) -> np.ndarray:
    return img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def find_corners(img: np.ndarray, cols: int, rows: int) -> np.ndarray | None:
    """Sub-pixel inner corners of a `cols` x `rows` chessboard, shape (N, 1, 2), or None.

    Uses the sector-based detector, which is more accurate and more tolerant of blur and
    uneven light than the classic one.
    """
    g = gray(img)
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    ok, corners = cv2.findChessboardCornersSB(g, (cols, rows), flags=flags)
    if not ok or corners is None:
        return None
    return corners.reshape(-1, 1, 2).astype(np.float32)


def detect_pattern_size(img: np.ndarray, max_corners: int = 14) -> tuple[int, int] | None:
    """Guess the inner-corner count of the chessboard in `img` by trying sizes, largest first.

    A smaller pattern also matches inside a bigger board, so the first (largest) hit wins. This
    uses the classic quad-based detector on purpose: the sector-based one also accepts the edge
    between the board's white margin and the background as an extra row of corners.
    """
    g = gray(img)
    sizes = sorted(
        ((c, r) for c, r in itertools.product(range(3, max_corners + 1), repeat=2) if c >= r),
        key=lambda s: -s[0] * s[1],
    )
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    for c, r in sizes:
        ok, _ = cv2.findChessboardCorners(g, (c, r), flags=flags)
        if ok:
            return c, r
    return None


def sharpness(img: np.ndarray, corners: np.ndarray) -> float:
    """Variance of the Laplacian inside the board's bounding box: low means motion blur."""
    g = gray(img)
    x, y, w, h = cv2.boundingRect(corners.reshape(-1, 2).astype(np.float32))
    roi = g[max(y, 0) : y + h, max(x, 0) : x + w]
    if roi.size == 0:
        return 0.0
    return float(cv2.Laplacian(roi, cv2.CV_64F).var())


def view_signature(corners: np.ndarray, cols: int, rows: int, width: int, height: int) -> np.ndarray:
    """A small vector describing where the board is and how it is tilted in the image.

    [centre x, centre y, size, horizontal tilt, vertical tilt], all roughly in 0..1, so a plain
    Euclidean distance says how different two views are. Tilt is the length ratio of opposite
    board edges, which grows with the board's angle to the image plane.
    """
    p = corners.reshape(rows, cols, 2)
    tl, tr, bl, br = p[0, 0], p[0, -1], p[-1, 0], p[-1, -1]
    centre = p.reshape(-1, 2).mean(axis=0)
    area = cv2.contourArea(np.array([tl, tr, br, bl], np.float32))
    top, bottom = np.linalg.norm(tr - tl), np.linalg.norm(br - bl)
    left, right = np.linalg.norm(bl - tl), np.linalg.norm(br - tr)
    tilt_h = (top - bottom) / max(top + bottom, 1e-6)
    tilt_v = (left - right) / max(left + right, 1e-6)
    return np.array([centre[0] / width, centre[1] / height, math.sqrt(area / (width * height)), 2 * tilt_h, 2 * tilt_v])


def coverage(corner_sets: list[np.ndarray], width: int, height: int, nx: int = 4, ny: int = 3) -> np.ndarray:
    """Boolean nx-by-ny grid of image cells that some captured corner falls into."""
    grid = np.zeros((ny, nx), bool)
    for c in corner_sets:
        pts = c.reshape(-1, 2)
        ix = np.clip((pts[:, 0] / width * nx).astype(int), 0, nx - 1)
        iy = np.clip((pts[:, 1] / height * ny).astype(int), 0, ny - 1)
        grid[iy, ix] = True
    return grid


@dataclass
class IntrinsicsResult:
    width: int
    height: int
    K: np.ndarray
    dist: np.ndarray  # k1, k2, p1, p2, k3 (p and k3 are zero)
    rms_px: float
    per_view_px: list[float]
    used: list[int]  # indices of the views kept after outlier rejection

    @property
    def fx(self) -> float:
        return float(self.K[0, 0])

    @property
    def fy(self) -> float:
        return float(self.K[1, 1])

    @property
    def cx(self) -> float:
        return float(self.K[0, 2])

    @property
    def cy(self) -> float:
        return float(self.K[1, 2])

    @property
    def hfov_deg(self) -> float:
        return math.degrees(2 * math.atan(self.width / (2 * self.fx)))

    @property
    def vfov_deg(self) -> float:
        return math.degrees(2 * math.atan(self.height / (2 * self.fy)))


def _calibrate(obj: list[np.ndarray], img: list[np.ndarray], size: tuple[int, int]):
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj, img, size, None, None, flags=CALIB_FLAGS)
    per_view = []
    for o, i, r, t in zip(obj, img, rvecs, tvecs):
        proj, _ = cv2.projectPoints(o, r, t, K, dist)
        per_view.append(float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - i.reshape(-1, 2)) ** 2, axis=1)))))
    return float(rms), K, dist.reshape(-1), per_view


def calibrate_intrinsics(
    corner_sets: list[np.ndarray], board: Chessboard, width: int, height: int, min_views: int = 8
) -> IntrinsicsResult:
    """Calibrate from detected corners, then drop views far worse than the rest and redo it once.

    A view is an outlier if its error is above 2.5x the median and above 0.5 px (a blurred
    frame or a board that flexed). At least `min_views` views are always kept.
    """
    if len(corner_sets) < min_views:
        raise ValueError(f"need at least {min_views} views with the board, have {len(corner_sets)}")
    objp = board.object_points()
    idx = list(range(len(corner_sets)))
    rms, K, dist, per_view = _calibrate([objp] * len(idx), [corner_sets[i] for i in idx], (width, height))
    med = float(np.median(per_view))
    keep = [i for i, e in zip(idx, per_view) if e <= max(2.5 * med, 0.5)]
    if len(keep) < len(idx) and len(keep) >= min_views:
        idx = keep
        rms, K, dist, per_view = _calibrate([objp] * len(idx), [corner_sets[i] for i in idx], (width, height))
    return IntrinsicsResult(width=width, height=height, K=K, dist=dist, rms_px=rms, per_view_px=per_view, used=idx)


def draw_corners(img: np.ndarray, corners: np.ndarray | None, cols: int, rows: int) -> np.ndarray:
    out = img.copy()
    if corners is not None:
        cv2.drawChessboardCorners(out, (cols, rows), corners, True)
    return out


def draw_coverage(img: np.ndarray, grid: np.ndarray) -> np.ndarray:
    """Tint image cells that already have corners green, so the user sees where to move the board."""
    out = img.copy()
    h, w = out.shape[:2]
    ny, nx = grid.shape
    overlay = out.copy()
    for iy in range(ny):
        for ix in range(nx):
            if grid[iy, ix]:
                x0, y0 = ix * w // nx, iy * h // ny
                cv2.rectangle(overlay, (x0, y0), ((ix + 1) * w // nx, (iy + 1) * h // ny), (0, 200, 0), -1)
    return cv2.addWeighted(overlay, 0.18, out, 0.82, 0)


# ============================================================================= extrinsics


def make_T(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def invert(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def rotation_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    """Angle of the rotation that takes Ra to Rb, in degrees."""
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


# --------------------------------------------------------------------------- tags

TAG_FAMILIES = {
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "4x4_50": cv2.aruco.DICT_4X4_50,
    "6x6_250": cv2.aruco.DICT_6X6_250,
}


def tag_detector(family: str = "tag25h9") -> cv2.aruco.ArucoDetector:
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(TAG_FAMILIES[family]), params)


def tag_centre(corners: np.ndarray) -> np.ndarray:
    """Image of the tag's centre: where its diagonals cross (the corner mean is biased under perspective)."""
    p0, p1, p2, p3 = corners.reshape(4, 2).astype(float)
    d1, d2 = p2 - p0, p3 - p1
    A = np.array([d1, -d2]).T
    if abs(np.linalg.det(A)) < 1e-9:
        return corners.reshape(4, 2).mean(axis=0)
    s = np.linalg.solve(A, p1 - p0)[0]
    return p0 + s * d1


def detect_tags(img: np.ndarray, detector: cv2.aruco.ArucoDetector) -> dict[int, np.ndarray]:
    """Tag id -> image position of its centre."""
    corners, ids, _ = detector.detectMarkers(gray(img))
    if ids is None:
        return {}
    return {int(i): tag_centre(c) for i, c in zip(ids.reshape(-1), corners)}


# --------------------------------------------------------------------------- fixed camera from touched points


@dataclass
class FixedCameraResult:
    T_base_cam: np.ndarray
    reproj_px: list[float]  # per point
    plane_err_mm: list[float]  # per point: ray through the pixel, cut at the point's height, vs the point
    rms_px: float


def solve_fixed_camera(points_base: np.ndarray, pixels: np.ndarray, K: np.ndarray, dist: np.ndarray) -> FixedCameraResult:
    """Pose of a fixed camera from base-frame points and the pixels they appear at (>= 4 points,
    not all on one line). Works for points that all lie on the table."""
    obj = np.asarray(points_base, np.float64).reshape(-1, 1, 3)
    img = np.asarray(pixels, np.float64).reshape(-1, 1, 2)
    if len(obj) < 4:
        raise ValueError(f"need at least 4 points, have {len(obj)}")
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        raise ValueError("solvePnP failed: are the points spread out and not on a line?")
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)
    T_cam_base = make_T(cv2.Rodrigues(rvec)[0], tvec)
    T_base_cam = invert(T_cam_base)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    reproj = np.linalg.norm(proj.reshape(-1, 2) - img.reshape(-1, 2), axis=1)
    # metric check: cast each pixel's ray and cut it at the touched point's height
    plane_err = []
    for p, uv in zip(obj.reshape(-1, 3), img.reshape(-1, 2)):
        n = cv2.undistortPoints(uv.reshape(1, 1, 2), K, dist).reshape(2)
        d = T_base_cam[:3, :3] @ np.array([n[0], n[1], 1.0])
        o = T_base_cam[:3, 3]
        if abs(d[2]) < 1e-9:
            plane_err.append(float("inf"))
            continue
        hit = o + (p[2] - o[2]) / d[2] * d
        plane_err.append(float(np.linalg.norm(hit[:2] - p[:2]) * 1000))
    return FixedCameraResult(T_base_cam, reproj.tolist(), plane_err, float(np.sqrt(np.mean(reproj**2))))


def fit_plane(points: np.ndarray) -> tuple[np.ndarray, float, float]:
    """Least-squares plane through points: (unit normal pointing up, height at x=y=0, tilt in degrees)."""
    P = np.asarray(points, float).reshape(-1, 3)
    c = P.mean(axis=0)
    _, _, vt = np.linalg.svd(P - c)
    n = vt[-1]
    if n[2] < 0:
        n = -n
    z0 = float(c @ n / n[2])  # n . (x, y, z) = n . c  at x = y = 0
    return n, z0, math.degrees(math.acos(min(1.0, n[2])))


# --------------------------------------------------------------------------- wrist camera (eye-in-hand)


def board_pose(corners: np.ndarray, board: Chessboard, K: np.ndarray, dist: np.ndarray) -> tuple[np.ndarray, float]:
    """Pose of the chessboard in the camera frame (T_cam_board) and its reprojection error in px."""
    obj = board.object_points()
    ok, rvec, tvec = cv2.solvePnP(obj, corners, K, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        raise ValueError("board pose failed")
    rvec, tvec = cv2.solvePnPRefineLM(obj, corners, K, dist, rvec, tvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - corners.reshape(-1, 2)) ** 2, axis=1))))
    return make_T(cv2.Rodrigues(rvec)[0], tvec), err


@dataclass
class HandEyeResult:
    T_tool_cam: np.ndarray
    T_base_board: np.ndarray  # where the board is, averaged over all samples
    spread_mm: list[float]  # per sample: how far its board-origin estimate is from the average
    spread_deg: list[float]
    method: str


def unflip_boards(T_base_tool: list[np.ndarray], T_cam_board: list[np.ndarray], board: Chessboard) -> tuple[list[np.ndarray], list[int]]:
    """Board poses with every sample using the same corner as the board's origin.

    A chessboard with an even number of corners one way looks the same turned by 180 degrees, so
    the detector starts counting from either end depending on how the camera is turned. Such a
    sample puts the board half a turn round and a board's length away, and ruins the hand-eye
    solution. The board does not move, so the samples that disagree with most of the others about
    which way its x axis points are the turned ones. Returns the poses and which were turned back.
    """
    half_turn = make_T(np.diag([-1.0, -1.0, 1.0]), [(board.cols - 1) * board.square_m, (board.rows - 1) * board.square_m, 0.0])
    boards, turned = [np.array(T, dtype=float) for T in T_cam_board], set()
    for _ in range(4):
        res = solve_hand_eye(T_base_tool, boards)
        wrong = [i for i, (Tg, Tc) in enumerate(zip(T_base_tool, boards)) if (Tg @ res.T_tool_cam @ Tc)[:3, 0] @ res.T_base_board[:3, 0] < 0]
        if not wrong or len(wrong) * 2 >= len(boards):
            break
        for i in wrong:
            boards[i] = boards[i] @ half_turn
            turned ^= {i}
    return boards, sorted(turned)


def solve_hand_eye(T_base_tool: list[np.ndarray], T_cam_board: list[np.ndarray]) -> HandEyeResult:
    """Camera pose on the tool from >= 3 (better 12+) samples with clearly different orientations.

    Runs the Park and Tsai solvers and keeps whichever predicts the (fixed) board most
    consistently across samples. That consistency is also the quality figure to look at.
    """
    if len(T_base_tool) < 3:
        raise ValueError("need at least 3 samples")
    Rg = [T[:3, :3] for T in T_base_tool]
    tg = [T[:3, 3] for T in T_base_tool]
    Rt = [T[:3, :3] for T in T_cam_board]
    tt = [T[:3, 3] for T in T_cam_board]
    best = None
    for name, method in (("park", cv2.CALIB_HAND_EYE_PARK), ("tsai", cv2.CALIB_HAND_EYE_TSAI), ("daniilidis", cv2.CALIB_HAND_EYE_DANIILIDIS)):
        try:
            R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=method)
        except cv2.error:
            continue
        X = make_T(R, t)
        boards = [Tg @ X @ Tc for Tg, Tc in zip(T_base_tool, T_cam_board)]
        pos = np.array([B[:3, 3] for B in boards])
        mean_pos = pos.mean(axis=0)
        # average rotation: project the mean matrix back onto SO(3)
        U, _, Vt = np.linalg.svd(sum(B[:3, :3] for B in boards))
        R_mean = U @ Vt
        if np.linalg.det(R_mean) < 0:
            U[:, -1] *= -1
            R_mean = U @ Vt
        spread = (np.linalg.norm(pos - mean_pos, axis=1) * 1000).tolist()
        spread_deg = [rotation_deg(R_mean, B[:3, :3]) for B in boards]
        score = float(np.mean(spread))
        if best is None or score < best[0]:
            best = (score, HandEyeResult(X, make_T(R_mean, mean_pos), spread, spread_deg, name))
    if best is None:
        raise ValueError("hand-eye calibration failed")
    return best[1]
