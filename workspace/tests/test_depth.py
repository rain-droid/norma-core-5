"""The depth camera's geometry, on a made-up table: a mat, a block on it, a camera looking down at
them from across the table. No camera and no SDK needed: the depth picture is computed."""

import math

import numpy as np

from util import depth

TABLE_Z = 0.010
BASE_IN_MAT = (0.30, -0.12, 5.0)  # where the robot stands on the mat, as camera_top.json has it
TAGS = {6: (0.0, 0.0), 10: (0.585, 0.0), 11: (0.0, 0.315), 5: (0.585, 0.31)}
BLOCK = {"x": 0.06, "y": 0.26, "side": 0.04, "height": 0.03}  # base frame


def _mat_to_base(m):
    bx, by, yaw = BASE_IN_MAT
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    R = np.array([[c, -s], [s, c]])
    return R.T @ (np.asarray(m, dtype=float) - [bx, by])


def _camera():
    """T_base_depth of a camera 40 cm up at the far side of the mat, looking back and down at it."""
    eye, target = np.array([0.03, 0.62, 0.40]), np.array([0.0, 0.25, TABLE_Z])
    z = (target - eye) / np.linalg.norm(target - eye)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, np.cross(z, x), z])  # the camera's x right, y down, z out
    T[:3, 3] = eye
    return T


def _scene(T):
    """The depth picture of the mat and the block, and the camera's own calibration."""
    K = np.array([[306.0, 0, 320.0], [0, 306.0, 200.0], [0, 0, 1]])
    v, u = np.mgrid[0:400, 0:640]
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones(u.shape)], axis=-1) @ T[:3, :3].T
    eye = T[:3, 3]
    with np.errstate(divide="ignore", invalid="ignore"):
        best = (TABLE_Z - eye[2]) / rays[..., 2]  # the mat
        lo = np.array([BLOCK["x"] - BLOCK["side"] / 2, BLOCK["y"] - BLOCK["side"] / 2, TABLE_Z])
        hi = np.array([BLOCK["x"] + BLOCK["side"] / 2, BLOCK["y"] + BLOCK["side"] / 2, TABLE_Z + BLOCK["height"]])
        t1, t2 = (lo - eye) / rays, (hi - eye) / rays
        enter, leave = np.minimum(t1, t2).max(axis=-1), np.maximum(t1, t2).min(axis=-1)
    hit = (enter < leave) & (enter > 0)
    best = np.where(hit, enter, best)
    depth_m = np.where(best > 0, best, np.nan).astype(np.float32)  # the rays have z = 1 in the camera frame, so this is the depth
    # a colour camera 25 mm to the depth sensor's right, turned a degree: a depth-frame point p is at R p + t in it
    a = math.radians(1.0)
    R = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    cal = {"depth_K": K.tolist(), "colour_K": [[720.0, 0, 640.0], [0, 720.0, 360.0], [0, 0, 1]], "colour_dist": [0.0] * 5,
           "R_colour_depth": R.tolist(), "t_colour_depth": [-0.025, 0.0, 0.0]}  # fmt: skip
    return depth_m, cal


def _tags_px(T, cal, size=(853, 480)):
    """Where the colour camera sees the two tags on the robot's side of the mat."""
    K = np.array(cal["colour_K"]) * [[size[0] / 1280], [size[1] / 720], [1]]
    out = {}
    for i in (6, 10):
        p_base = np.array([*_mat_to_base(TAGS[i]), TABLE_Z])
        p_depth = T[:3, :3].T @ (p_base - T[:3, 3])
        p = K @ (np.array(cal["R_colour_depth"]) @ p_depth + cal["t_colour_depth"])
        out[i] = p[:2] / p[2]
    return out


def test_the_camera_is_placed_from_the_mat_plane_and_two_tags():
    T = _camera()
    depth_m, cal = _scene(T)
    n, d, flatness = depth.table_plane(depth.points(depth_m, cal["depth_K"]))
    fitted, fit = depth.register(n, d, cal, _tags_px(T, cal), (853, 480), TAGS, BASE_IN_MAT, TABLE_Z)
    assert flatness < 0.0005
    assert max(fit["tag_fit_mm"]) < 1.0
    assert np.allclose(fitted, T, atol=2e-3), f"fitted\n{fitted.round(3)}\nwanted\n{T.round(3)}"


def test_a_block_on_the_mat_is_found_with_its_height():
    T = _camera()
    depth_m, cal = _scene(T)
    grid = depth.height_map(depth.to_base(depth.points(depth_m, cal["depth_K"]), T), TABLE_Z)
    found = depth.bumps(grid)
    assert len(found) == 1
    block = found[0]
    assert abs(block["height_mm"] - 30) <= 2
    assert math.hypot(block["x"] - BLOCK["x"], block["y"] - BLOCK["y"]) < 0.008
    assert abs(depth.height_at(grid, BLOCK["x"], BLOCK["y"]) - 0.03) < 0.002
    assert abs(depth.height_at(grid, -0.15, 0.20)) < 0.002  # bare mat
    assert depth.picture(grid, [{"x": block["x"], "y": block["y"], "label": "30 mm"}]).shape[2] == 3
