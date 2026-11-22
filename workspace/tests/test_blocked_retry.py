"""What `Arm.joints` does when the robot side stops a move as blocked, without a robot: `Arm`
talks to a fake daemon, and robot_config/motion.json is a file in a temporary directory."""

import json

import pytest

from util import arm as arm_module
from util.arm import Arm, RobotError, blocked_retry_speed

BLOCKED = "blocked: joint 2 is 15 deg behind where it should be. Something is in the way or the load is too heavy."


class Daemon:
    """Stands in for the robot side: answers /api/joints from a list and remembers what was sent."""

    def __init__(self, answers, q=None, fault=False, connected=True):
        self.answers = list(answers)
        self.sent = []
        self.q = [0.0] * 7 if q is None else q
        self.fault, self.connected = fault, connected

    def __call__(self, path, body=None, timeout=10.0, raw=False):
        if path == "/api/state":
            return {"connected": self.connected, "calibrated": True, "joints": [{"id": i + 1, "q": v, "error": 32 if self.fault and i == 1 else 0} for i, v in enumerate(self.q)]}
        assert path == "/api/joints"
        self.sent.append(body)
        answer = self.answers.pop(0)
        return {"ok": True, "joints": body.get("path", [self.q])[-1]} if answer is None else {"ok": False, "error": answer}


@pytest.fixture
def config(monkeypatch, tmp_path):
    """Write motion.json: config(0.25), or config(text="...") for a file as it stands. None yet: the default."""
    path = tmp_path / "motion.json"
    monkeypatch.setattr(arm_module, "MOTION_CONFIG", path)

    def write(value=None, text=None):
        path.write_text(json.dumps({"blocked_retry_speed": value}) if text is None else text)

    return write


@pytest.fixture
def daemon(monkeypatch, config):
    # These tests isolate retry behaviour; mat clearance has its own swept-path tests.
    monkeypatch.setattr(Arm, "_mind_the_mat", lambda self, path: None)
    def make(*answers, **kw):
        d = Daemon(answers, **kw)
        monkeypatch.setattr(arm_module, "_request", d)
        monkeypatch.setattr(arm_module, "BLOCKED_PAUSE_S", 0.0)
        return d

    return make


# ---- a blocked move is tried once more, slower


def test_a_move_that_goes_through_is_sent_once(daemon):
    d = daemon(None)
    res = Arm().joints([0.1] * 7, speed=0.8)
    assert len(d.sent) == 1 and "retried_blocked" not in res


def test_blocked_is_sent_again_at_half_speed_by_default(daemon, tmp_path):
    d = daemon(BLOCKED, None)
    log = tmp_path / "joint_commands.jsonl"
    res = Arm(log=log).joints([0.1] * 7, speed=0.8, check=False, allow_low=True, label="reaching")
    assert [b["speed"] for b in d.sent] == [0.8, 0.4]
    assert d.sent[1]["label"] == "reaching (again, slower)"
    assert (d.sent[1]["check"], d.sent[1]["allow_low"]) == (False, True)  # the same command otherwise
    assert res["retried_blocked"] == BLOCKED
    lines = [json.loads(line) for line in log.read_text().splitlines()]
    assert [line["outcome"] for line in lines] == [BLOCKED, "done"]  # both tries are on record


def test_blocked_twice_is_an_error_after_two_tries(daemon):
    d = daemon(BLOCKED, BLOCKED.replace("joint 2", "joint 4"))
    with pytest.raises(RobotError, match="joint 4"):  # the second refusal is the one reported
        Arm().joints([0.1] * 7)
    assert len(d.sent) == 2


def test_second_try_continues_from_where_the_arm_stopped(daemon):
    path = [[0.0] * 7, [0.2] * 7, [0.4] * 7, [0.6] * 7]
    d = daemon(BLOCKED, None, q=[0.37] * 7)  # stopped just short of the third waypoint
    Arm().joints(path)
    assert d.sent[1]["path"] == path[2:]


def test_second_try_is_never_slower_than_the_slowest_speed(daemon):
    d = daemon(BLOCKED, None)
    Arm().joints([0.1] * 7, speed=0.06)
    assert d.sent[1]["speed"] == 0.05


@pytest.mark.parametrize("error", [
    "move rejected: fingertips would be 2 mm into the table",
    "stopped before it finished; the arm is holding position",
    "servo fault on joint(s) [2]",
    "lost servo state for 1.2s; holding position",
])  # fmt: skip
def test_other_refusals_are_not_tried_again(daemon, error):
    d = daemon(error)
    with pytest.raises(RobotError, match=error[:20]):
        Arm().joints([0.1] * 7)
    assert len(d.sent) == 1


def test_not_tried_again_when_asked_not_to(daemon):
    d = daemon(BLOCKED)
    with pytest.raises(RobotError, match="blocked"):
        Arm().joints([0.1] * 7, retry_blocked=False)
    assert len(d.sent) == 1


@pytest.mark.parametrize("kw", [{"fault": True}, {"connected": False}])
def test_not_tried_again_on_an_arm_that_cannot_be_used(daemon, kw):
    d = daemon(BLOCKED, **kw)
    with pytest.raises(RobotError, match="blocked"):
        Arm().joints([0.1] * 7)
    assert len(d.sent) == 1


def test_a_blocked_gripper_command_is_not_tried_again(daemon):
    d = daemon(BLOCKED)
    with pytest.raises(RobotError):
        Arm().joints(gripper=0.5)
    assert len(d.sent) == 1


# ---- how much slower is in robot_config/motion.json


@pytest.mark.parametrize("factor, second", [(0.25, 0.2), (0.5, 0.4), (1, 0.8), (0.01, 0.05)])
def test_the_config_sets_how_much_slower(daemon, config, factor, second):
    config(factor)
    d = daemon(BLOCKED, None)
    Arm().joints([0.1] * 7, speed=0.8)
    assert [b["speed"] for b in d.sent] == [0.8, second]


def test_zero_in_the_config_means_no_second_try(daemon, config):
    config(0)
    d = daemon(BLOCKED)
    with pytest.raises(RobotError, match="blocked"):
        Arm().joints([0.1] * 7)
    assert len(d.sent) == 1


def test_the_config_is_read_when_a_move_is_blocked_not_before(daemon, config):
    d = daemon(BLOCKED, None)
    arm = Arm()
    config(0.25)  # edited after the arm was made
    arm.joints([0.1] * 7, speed=0.8)
    assert d.sent[1]["speed"] == 0.2


def test_a_config_without_the_key_is_the_default(config):
    config(text="{}")
    assert blocked_retry_speed() == 0.5


@pytest.mark.parametrize("text", ['{"blocked_retry_speed": 1.5}', '{"blocked_retry_speed": -0.1}', '{"blocked_retry_speed": "half"}',
                                  '{"blocked_retry_speed": true}', '{"blocked_retry_speed": null}', "not json", "[0.5]"])  # fmt: skip
def test_a_bad_config_means_no_second_try_and_the_error_says_so(daemon, config, text):
    config(text=text)
    d = daemon(BLOCKED)
    with pytest.raises(RobotError, match=r"blocked: joint 2 .* Not tried again slower: robot_config/motion\.json"):
        Arm().joints([0.1] * 7)
    assert len(d.sent) == 1


def test_a_bad_config_does_not_touch_a_move_that_goes_through(daemon, config):
    config(text="not json")
    d = daemon(None)
    Arm().joints([0.1] * 7)
    assert len(d.sent) == 1


def test_the_file_in_robot_config_is_valid_and_half_speed():
    assert arm_module.MOTION_CONFIG.name == "motion.json" and arm_module.MOTION_CONFIG.parent.name == "robot_config"
    assert json.loads(arm_module.MOTION_CONFIG.read_text())["blocked_retry_speed"] == 0.5
