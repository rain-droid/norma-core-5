import copy
import numpy as np
import pytest

from so101_station.cleanup.backend import mock_workspace, MockBackend
from so101_station.cleanup.geometry import ArmModel, CleanupError, TableMap


def test_arbitrary_pixel_projection_is_continuous_not_pickup_slots():
    table = TableMap(mock_workspace())
    for x, y in [[.015, -.195], [.035, -.18], [.065, -.20]]:
        xy = table.project([80 + x * 4000, 60 + (y + .22) * 3000])
        assert np.allclose(xy, [x, y], atol=1e-7)
    for pixel in ([-1, 50], [639, 479], [float("nan"), 20], [480, 300]):
        with pytest.raises(CleanupError):
            table.project(pixel)


def test_calibration_requires_measured_coverage_and_fixed_camera():
    for mutate in (lambda c: c.update(camera_fixed=False), lambda c: c.update(pixel_points=[[1,1]]*6),
                   lambda c: c.update(workspace_polygon_m=[[0,-.4],[.12,-.4],[.12,-.12],[0,-.12]]),
                   lambda c: c.update(travel_z_m=float("nan"))):
        config = copy.deepcopy(mock_workspace()); mutate(config)
        with pytest.raises(CleanupError):
            TableMap(config)


def test_real_model_ik_tracks_free_table_points_and_rejects_unreachable():
    model = ArmModel(); seed = MockBackend(model).q
    for target in [[.025,-.19,.008],[.06,-.185,.02],[.10,-.14,.04]]:
        q = model.solve(target, seed)
        T = model.fk(q)
        assert np.linalg.norm(T[:3,3] - target) < .004
        assert T[:3,2] @ [0,0,-1] > .98
        model.validate(q)
    with pytest.raises(CleanupError):
        model.solve([.8, 0, .1], seed)
    with pytest.raises(CleanupError):
        model.validate(seed + 4)


def test_linear_descent_has_continuous_checked_joint_waypoints():
    model = ArmModel(); seed = model.solve([.025,-.19,.075], np.zeros(5))
    path = model.line(seed, [.025,-.19,.008])
    assert len(path) > 1
    for q in path:
        assert np.max(np.abs(np.array(q) - seed)) <= .25
        assert np.linalg.norm(model.fk(q)[:2,3] - [.025,-.19]) < .004
        model.validate(q)
        seed = q
