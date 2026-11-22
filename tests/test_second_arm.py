"""The second arm in the digital twin: where it is drawn, and that the daemon carries it in its state."""

import json
import math

from elrobot.config import Config
from elrobot.daemon import Hub
from elrobot.second_arm import SecondArm


def test_the_second_arm_is_drawn_upright_on_the_table_where_the_fit_puts_it(tmp_path):
    (tmp_path / "robot_config").mkdir()
    yaw, tilt = math.radians(-12.0), math.radians(8.0)  # the fit of points in the air comes out tilted
    c, s, ct, st = math.cos(yaw), math.sin(yaw), math.cos(tilt), math.sin(tilt)
    T = [[c, -s * ct, s * st, 0.286], [s, c * ct, -c * st, 0.013], [0.0, st, ct, 0.039], [0, 0, 0, 1]]
    arm = SecondArm("http://127.0.0.1:1", tmp_path)
    assert arm.placement() is None  # not fitted yet
    (tmp_path / "robot_config" / "second_arm.json").write_text(json.dumps({"T_base_second": T}))
    placed = arm.placement()
    assert [round(placed[0][3], 3), round(placed[1][3], 3), placed[2][3]] == [0.286, 0.013, 0.0]
    assert abs(math.degrees(math.atan2(placed[1][0], placed[0][0])) + 12.0) < 0.01
    assert placed[2][:3] == [0.0, 0.0, 1.0]


def test_the_state_says_null_while_the_second_arms_daemon_is_not_answering(tmp_path):
    hub = Hub(Config(sim=True, voice=False, workspace_dir=tmp_path, second_arm_url=""))
    assert "second_arm" not in hub.snapshot()  # none configured
    hub.second = SecondArm("http://127.0.0.1:1", tmp_path)
    assert hub.snapshot()["second_arm"] is None
