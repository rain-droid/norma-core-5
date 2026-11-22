"""Intrinsic calibration on synthetic views of a chessboard seen by a known camera."""

import cv2
import numpy as np

from elrobot import calib

W, H = 853, 480
K_TRUE = np.array([[560.0, 0, 430.0], [0, 560.0, 236.0], [0, 0, 1]])
BOARD = calib.Chessboard(cols=9, rows=6, square_m=0.025)
SQ_PX, MARGIN_PX = 40, 40


def board_image() -> np.ndarray:
    nx, ny = BOARD.cols + 1, BOARD.rows + 1
    img = np.full((ny * SQ_PX + 2 * MARGIN_PX, nx * SQ_PX + 2 * MARGIN_PX), 255, np.uint8)
    for j in range(ny):
        for i in range(nx):
            if (i + j) % 2 == 0:
                x, y = MARGIN_PX + i * SQ_PX, MARGIN_PX + j * SQ_PX
                img[y : y + SQ_PX, x : x + SQ_PX] = 0
    return img


def render(rvec, tvec) -> np.ndarray:
    """The board (z = 0 plane, origin at its first inner corner) as the true camera sees it."""
    R = cv2.Rodrigues(np.asarray(rvec, float))[0]
    q = BOARD.square_m / SQ_PX
    o = MARGIN_PX + SQ_PX
    board_px_to_m = np.array([[q, 0, -o * q], [0, q, -o * q], [0, 0, 1]])
    H_img = K_TRUE @ np.column_stack([R[:, 0], R[:, 1], tvec]) @ board_px_to_m
    img = cv2.warpPerspective(board_image(), H_img, (W, H), flags=cv2.INTER_LINEAR, borderValue=128)
    return cv2.GaussianBlur(img, (3, 3), 0.6)


def views():
    rng = np.random.default_rng(0)
    centre = np.array([BOARD.cols * BOARD.square_m / 2, BOARD.rows * BOARD.square_m / 2, 0])
    for _ in range(16):
        rvec = np.array([np.pi, 0, 0]) + rng.uniform(-0.45, 0.45, 3) * [1, 1, 0.3]
        R = cv2.Rodrigues(rvec)[0]
        # place the board centre somewhere in view at 0.30-0.45 m
        p = np.array([rng.uniform(-0.08, 0.08), rng.uniform(-0.05, 0.05), rng.uniform(0.30, 0.45)])
        yield render(rvec, p - R @ centre)


def test_recovers_known_intrinsics():
    corners = [c for c in (calib.find_corners(v, BOARD.cols, BOARD.rows) for v in views()) if c is not None]
    assert len(corners) >= 12
    res = calib.calibrate_intrinsics(corners, BOARD, W, H)
    assert res.rms_px < 0.3, res.rms_px
    assert abs(res.fx - 560) / 560 < 0.01, res.fx
    assert abs(res.fy - 560) / 560 < 0.01, res.fy
    assert abs(res.cx - 430) < 5 and abs(res.cy - 236) < 5, (res.cx, res.cy)
    assert abs(res.dist[0]) < 0.05


def test_detect_pattern_size():
    img = next(iter(views()))
    assert calib.detect_pattern_size(img) == (9, 6)


def test_view_signature_separates_positions():
    rng_views = list(views())[:2]
    sigs = [calib.view_signature(calib.find_corners(v, 9, 6), 9, 6, W, H) for v in rng_views]
    assert np.linalg.norm(sigs[0] - sigs[1]) > 0.05


# ----------------------------------------------------------------------------- extrinsics


def _look_at(eye, target, up=(0, 0, 1)) -> np.ndarray:
    """Camera pose in the base frame (OpenCV axes: z forward, y down) looking from eye at target."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return calib.make_T(np.column_stack([x, y, z]), eye)


def _project(T_base_cam, pts):
    T = calib.invert(T_base_cam)
    rvec = cv2.Rodrigues(T[:3, :3])[0]
    uv, _ = cv2.projectPoints(np.asarray(pts, float).reshape(-1, 1, 3), rvec, T[:3, 3], K_TRUE, np.zeros(5))
    return uv.reshape(-1, 2)


def test_fixed_camera_from_touched_table_points():
    rng = np.random.default_rng(3)
    T_true = _look_at([0.05, -0.15, 0.35], [0.0, 0.25, 0.0])  # 35 cm up, behind-ish, looking at the table
    pts = np.array([[x, y, 0.0] for x, y in [(-0.15, 0.15), (0.15, 0.15), (-0.12, 0.32), (0.12, 0.33), (0.0, 0.22), (0.18, 0.25), (-0.18, 0.26), (0.05, 0.38)]])
    touched = pts + rng.normal(0, 0.001, pts.shape)  # 1 mm touch error
    pix = _project(T_true, pts) + rng.normal(0, 0.3, (len(pts), 2))
    res = calib.solve_fixed_camera(touched, pix, K_TRUE, np.zeros(5))
    assert np.linalg.norm(res.T_base_cam[:3, 3] - T_true[:3, 3]) < 0.01
    assert calib.rotation_deg(res.T_base_cam[:3, :3], T_true[:3, :3]) < 1.0
    assert max(res.plane_err_mm) < 4


def test_tag_centre_is_diagonal_intersection():
    sq = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], float)
    assert np.allclose(calib.tag_centre(sq), [5, 5])


def test_fit_plane_tilt():
    pts = np.array([[x, y, 0.01 + 0.0 * x] for x in (-0.1, 0, 0.1) for y in (0.1, 0.2, 0.3)])
    n, z0, tilt = calib.fit_plane(pts)
    assert abs(z0 - 0.01) < 1e-9 and tilt < 1e-6


def test_hand_eye_recovers_wrist_mount():
    rng = np.random.default_rng(5)
    from scipy.spatial.transform import Rotation as Rot

    X_true = calib.make_T(Rot.from_euler("xyz", [3, -2, 1], degrees=True).as_matrix(), [0.002, -0.045, -0.10])
    T_base_board = calib.make_T(np.diag([1.0, -1.0, -1.0]), [-0.1, 0.2, 0.0])  # board flat on the table
    tools, boards = [], []
    for _ in range(15):
        # camera somewhere 20-35 cm above the board, looking at it from varied angles
        eye = np.array([rng.uniform(-0.15, 0.05), rng.uniform(0.1, 0.3), rng.uniform(0.2, 0.35)])
        target = np.array([-0.04, 0.26, 0.0]) + rng.normal(0, 0.02, 3)
        T_cam = _look_at(eye, target, up=rng.normal(0, 1, 3))
        T_tool = T_cam @ calib.invert(X_true)
        noise = calib.make_T(Rot.from_rotvec(rng.normal(0, 0.002, 3)).as_matrix(), rng.normal(0, 0.0005, 3))
        tools.append(T_tool @ noise)
        boards.append(calib.invert(T_cam) @ T_base_board)
    res = calib.solve_hand_eye(tools, boards)
    assert np.linalg.norm(res.T_tool_cam[:3, 3] - X_true[:3, 3]) < 0.004, res.T_tool_cam[:3, 3]
    assert calib.rotation_deg(res.T_tool_cam[:3, :3], X_true[:3, :3]) < 1.0
    assert np.mean(res.spread_mm) < 3
