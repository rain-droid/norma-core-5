"""Descent protection without moving hardware: commands go to a fake transport."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from util import arm as arm_module
from util.arm import Arm, RobotError, extra_descent_clearance
from util.kinematics import Kinematics
from util.planning import Planner

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def setup(monkeypatch, tmp_path):
    config = tmp_path / "motion.json"
    config.write_text(json.dumps({"extra_descent_clearance_m": 0.015}))
    monkeypatch.setattr(arm_module, "MOTION_CONFIG", config)
    sent = []
    state = {"joints": [{"q": v} for v in [0.1, 0, 0, 0, 0, 0, 0]]}
    meta = {"mode": "real", "table_z": 0.0,
            "clearance": {"floor": 0.008, "body": 0.03, "min_y": -0.1, "limit_margin": 0.06}}

    def request(path, body=None, **kw):
        if path == "/api/state":
            return state
        if path == "/api/meta":
            return meta
        assert path == "/api/joints"
        sent.append(body)
        return {"ok": True}

    monkeypatch.setattr(arm_module, "_request", request)
    monkeypatch.setattr(Arm, "_mind_the_other_arm", lambda *args: None)
    arm = Arm()
    arm._mat_kin = SimpleNamespace(geometry=lambda q: SimpleNamespace(
        tcp=np.array([0, 0, q[0]]), tips=np.array([[0, 0, q[0]], [0, 0, q[0]]])))
    return arm, sent, config, state, meta


@pytest.mark.parametrize("check,allow_low", [(True, False), (True, True), (False, False), (False, True)])
def test_all_joint_commands_refuse_low_descents_before_sending(setup, check, allow_low):
    arm, sent, _, _, _ = setup
    with pytest.raises(RobotError, match="extra mat clearance"):
        arm.joints([0.020, 0, 0, 0, 0, 0, 0], check=check, allow_low=allow_low)
    assert sent == []


def test_sweep_is_checked_even_when_endpoints_are_high(setup):
    arm, sent, _, _, _ = setup
    def geometry(q):
        z = 0.010 + 40 * (q[0] - 0.15) ** 2
        return SimpleNamespace(tcp=np.array([0, 0, z]), tips=np.array([[0, 0, z]]))
    arm._mat_kin = SimpleNamespace(geometry=geometry)
    with pytest.raises(RobotError, match="extra mat clearance"):
        arm.joints([0.2, 0, 0, 0, 0, 0, 0], check=False)
    assert sent == []


def test_upward_recovery_from_below_new_floor_is_allowed(setup):
    arm, sent, _, state, _ = setup
    state["joints"][0]["q"] = 0.015
    arm.joints([0.05, 0, 0, 0, 0, 0, 0], check=False)
    assert len(sent) == 1


def test_descent_from_below_new_floor_is_refused(setup):
    arm, sent, _, state, _ = setup
    state["joints"][0]["q"] = 0.015
    with pytest.raises(RobotError, match="extra mat clearance"):
        arm.joints([0.01, 0, 0, 0, 0, 0, 0], check=False)
    assert sent == []


def test_gripper_only_commands_remain_available_with_bad_config(setup):
    arm, sent, config, _, _ = setup
    config.write_text("broken")
    arm.joints(gripper=1.0)
    assert len(sent) == 1


@pytest.mark.parametrize("value", [-0.01, 0.11, True, None, "15mm", float("nan"), float("inf")])
def test_invalid_clearance_fails_closed(setup, value):
    arm, sent, config, _, _ = setup
    config.write_text(json.dumps({"extra_descent_clearance_m": value}))
    with pytest.raises(RobotError, match="extra_descent_clearance_m"):
        arm.joints([0.08, 0, 0, 0, 0, 0, 0], check=False)
    assert sent == []


def test_missing_config_retains_clearance(setup):
    _, _, config, _, _ = setup
    config.unlink()
    assert extra_descent_clearance() == 0.015


def test_malformed_config_fails_closed(setup):
    arm, sent, config, _, _ = setup
    config.write_text("broken")
    with pytest.raises(RobotError, match="cannot read descent clearance"):
        arm.joints([0.08, 0, 0, 0, 0, 0, 0])
    assert sent == []


def test_changed_clearance_is_checked_on_retry(setup, monkeypatch):
    arm, sent, config, state, _ = setup
    original = arm_module._request
    def request(path, body=None, **kw):
        result = original(path, body, **kw)
        if path == "/api/state":
            result.update(connected=True, calibrated=True)
            for j in result["joints"]:
                j.update(id=1, error=0)
        if path == "/api/joints":
            config.write_text(json.dumps({"extra_descent_clearance_m": 0.050}))
            return {"ok": False, "kind": "blocked", "error": "blocked: test"}
        return result
    monkeypatch.setattr(arm_module, "_request", request)
    monkeypatch.setattr(arm_module, "BLOCKED_PAUSE_S", 0)
    with pytest.raises(RobotError, match="extra mat clearance"):
        arm.joints([0.04, 0, 0, 0, 0, 0, 0])
    assert len(sent) == 1  # no unchecked retry was sent


def test_the_descent_that_contacted_the_mat_is_now_refused(setup):
    arm, sent, _, state, meta = setup
    meta["table_z"] = 0.01
    arm._mat_kin = Kinematics(ROOT / "robot_config/model.urdf")
    start = [-0.84510, 0.40252, -0.66811, 0.05777, -0.02262, 1.42076, 0.14537]
    end = [-0.83740, 0.54400, -0.51646, 0.16967, -0.03104, 1.01557, 0.13953]
    state["joints"] = [{"q": q} for q in start]
    with pytest.raises(RobotError, match="extra mat clearance"):
        arm.joints(end, check=False, allow_low=True)
    assert sent == []


def test_approach_is_shorter_and_generic_planning_obeys_floor(setup):
    arm, sent, _, _, meta = setup
    meta["table_z"] = 0.01
    kin = Kinematics(ROOT / "robot_config/model.urdf")
    arm.limits = lambda: (kin.lower, kin.upper)
    plan = Planner(arm)
    target = np.array([0.0, 0.22, 0.01])
    a, _, q_hover, hover, grasp = plan.approach_for(target, 0.022, 0)
    assert grasp[2] == pytest.approx(0.01 + 0.022 + 0.010 + 0.015)
    assert hover[2] == pytest.approx(0.01 + 0.022 + 0.010 - a[2] * 0.07)
    assert hover[2] - grasp[2] == pytest.approx(-a[2] * 0.07 - 0.015)
    path = plan.line(q_hover, grasp, approach=a, allow_low=True)
    assert all(plan.blocked(q, allow_low=True) is None for q in path)
    with pytest.raises(RobotError, match="rejected"):
        plan.line(q_hover, [0.0, 0.22, 0.025], approach=a, allow_low=True)
    assert not plan.solve([0.0, 0.22, 0.025], pitch=90, q_seed=q_hover, allow_low=True).ok
    assert sent == []
