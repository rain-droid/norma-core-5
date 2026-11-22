"""Both arms in one space: where the second arm's parts are, how close the arms are, and that a path into the other arm is refused."""

import json
import math
import shutil
from pathlib import Path

import numpy as np
import pytest

from util.arm import RobotError
from util.shared_space import SharedSpace

CONFIG = Path(__file__).resolve().parents[1] / "robot_config"
HOME_THIS = np.radians([0.0, -21.0, -29.0, 36.0, 0.0, 73.5, 0.0])
READY_SECOND = np.radians([0.0, -45.0, 45.0, 75.0, 0.0])


@pytest.fixture
def space(tmp_path):
    (tmp_path / "robot_config").mkdir()
    for name in ("model.urdf", "second_arm.urdf"):
        shutil.copy(CONFIG / name, tmp_path / "robot_config" / name)
    # the second arm 29 cm to this arm's right, turned 12 degrees, as fitted on the real table (with the fit's tilt, which is dropped)
    yaw, tilt = math.radians(-12.0), math.radians(8.0)
    c, s, ct, st = math.cos(yaw), math.sin(yaw), math.cos(tilt), math.sin(tilt)
    T = [[c, -s * ct, s * st, 0.29], [s, c * ct, -c * st, 0.01], [0.0, st, ct, 0.04], [0, 0, 0, 1]]
    (tmp_path / "robot_config" / "second_arm.json").write_text(json.dumps({"T_base_second": T}))
    return SharedSpace(tmp_path)


def test_the_second_arm_stands_upright_where_it_was_fitted(space):
    assert np.allclose(space.T[:3, 3], [0.29, 0.01, 0.0])
    assert np.allclose(space.T[2, :3], [0, 0, 1])
    own = space.second.tool_point(READY_SECOND)  # its daemon reports x=+0.001 y=+0.184 z=+0.074 for this pose
    assert np.allclose(own, [0.001, 0.184, 0.074], atol=0.004)


def test_arms_at_their_ready_poses_are_well_apart_and_a_meeting_of_grippers_is_seen(space):
    gap, _ = space.between(HOME_THIS, READY_SECOND)
    assert gap > 0.10
    # the second arm reaching for this arm's tool point: the grippers overlap, and that pair can be left out
    q = space.second_ik(space.this.fk(HOME_THIS)[:3, 3], pitch_deg=30)
    gap, where = space.between(HOME_THIS, q)
    assert gap < 0 and "gripper" in where
    assert space.between(HOME_THIS, q, grippers_may_meet=True)[0] > gap


def test_a_way_into_the_other_arm_is_refused_and_the_way_out_is_not(space):
    into = space.second_ik(space.this.fk(HOME_THIS)[:3, 3], pitch_deg=30)
    why = space.path_clear("second", [READY_SECOND, into], HOME_THIS)
    assert why and "would come within" in why
    assert space.path_clear("second", [into, READY_SECOND], HOME_THIS) is None  # already close: leaving is allowed
    assert space.path_clear("second", [READY_SECOND, np.radians([30.0, -45, 45, 75, 0])], HOME_THIS) is None  # turning away


def test_the_second_arms_inverse_kinematics_puts_its_tool_point_where_asked(space):
    for place, pitch in (([0.14, 0.22, 0.05], 90), ([0.10, 0.20, 0.15], 0), ([0.20, 0.25, 0.08], 60)):
        q = space.second_ik(place, pitch_deg=pitch)
        assert np.linalg.norm(space.second_tool(q) - place) < 0.004
        assert abs(math.degrees(math.asin(-space.second.points_along(q)[2])) - pitch) < 8
    with pytest.raises(RobotError):
        space.second_ik([-0.30, 0.30, 0.05])  # half a metre from its base
