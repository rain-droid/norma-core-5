"""What the tools do when the robot side refuses a move, without a robot: `Arm` talks to a fake
daemon, `rest` to a fake arm, and the tracker is fed outlines instead of camera frames."""

import json
from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest

from util import arm as arm_module
from util import tracker
from util.arm import Arm, RobotError

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
def daemon(monkeypatch):
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


def test_blocked_is_sent_again_at_half_speed(daemon, tmp_path):
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


# ---- rest


class FakeArm:
    def __init__(self, open_=0.98, holding=False):
        self.gripper = {"open": open_, "holding": holding}
        self.commands, self.torque_on = [], True

    def state(self):
        return {"connected": True, "calibrated": True, "torque": self.torque_on, "gripper": dict(self.gripper),
                "joints": [{"id": i + 1, "q": 0.0, "error": 0, "lower": -2.0, "upper": 2.0} for i in range(7)]}  # fmt: skip

    def why_not_ready(self, state=None):
        return None

    def limits(self):
        return np.full(7, -2.0), np.full(7, 2.0)

    def joints(self, path=None, **kw):
        self.commands.append({"path": path, **kw})
        if kw.get("gripper") is not None:
            self.gripper["open"] = kw["gripper"]
        return {"ok": True}

    def torque(self, enable):
        self.torque_on = enable


def test_rest_brings_the_jaws_together_before_folding_and_folds_slowly():
    from rest import rest
    from util.motion import seat

    arm = FakeArm(open_=0.98)
    out = rest(SimpleNamespace(arm=arm))
    first, above, down = arm.commands
    assert first["path"] is None and 0 < first["gripper"] <= 0.05  # together, but not 0: that is a grasp
    sit, over = seat(arm)
    assert np.allclose(above["path"], over) and np.allclose(down["path"], sit)
    assert above["speed"] <= 0.3 and down["speed"] <= 0.3
    assert out == {"pose": "sitting", "torque": False}


def test_rest_leaves_closed_jaws_alone():
    from rest import rest

    arm = FakeArm(open_=0.014)
    rest(SimpleNamespace(arm=arm))
    assert len(arm.commands) == 2 and all(c["path"] is not None for c in arm.commands)


def test_rest_refuses_while_holding():
    from rest import rest

    arm = FakeArm(open_=0.5, holding=True)
    with pytest.raises(RobotError, match="holding"):
        rest(SimpleNamespace(arm=arm))
    assert arm.commands == []


# ---- the tracker: what the arm put down is not an intrusion


class FlatCamera:
    """A camera for which a millimetre on the table is a pixel."""

    def to_pixel(self, p):
        return np.array([400 + 1000 * p[0], 1000 * p[1]])


def watcher() -> tracker.Watcher:
    w = tracker.Watcher.__new__(tracker.Watcher)  # without the camera, the planner and the daemon
    w.cam, w.table_z = FlatCamera(), 0.0
    w.tracks, w.maybe, w.numbers, w.put_down, w.news = [], [], {}, [], []
    w.holding = None
    w.by_arm = np.zeros((480, 853), bool)
    return w


def outline(x, y, colour="pink"):
    u, v = FlatCamera().to_pixel([x, y, 0])
    return {"x": x, "y": y, "u": float(u), "v": float(v), "color": colour, "cut": False, "yaw_deg": 0, "width_mm": 40, "footprint_mm": [40, 40],
            "height_mm": None, "area_px": 1600, "polygon": [], "box": [u - 20, v - 20, u + 20, v + 20]}  # fmt: skip


def look(w, seen, now):
    w.news = []
    w._match(seen, np.zeros((480, 853), bool), [], now)
    return [what for what, _, _ in w.news]


def test_something_new_on_the_table_is_news():
    assert look(watcher(), [outline(0.0, 0.27)], now=100.0) == ["something new is on the table"]


def test_what_the_jaws_put_down_untracked_is_not_news_and_becomes_a_track():
    w = watcher()
    spot = np.array([0.016, 0.267, 0.03])
    w._gripper(True, spot, 100.0)  # the jaws closed on something that was never a track (pick_at)
    assert w.holding == ""
    w._gripper(False, spot, 110.0)  # and let it go here
    for i in range(tracker.CONFIRM):
        assert look(w, [outline(0.013, 0.269)], now=112.0 + i / 10) == []
    assert [t["id"] for t in w.tracks] == ["pink-1"]


def test_something_new_elsewhere_is_still_news_after_a_put_down():
    w = watcher()
    w._gripper(True, np.array([0.016, 0.267, 0.03]), 100.0)
    w._gripper(False, np.array([0.016, 0.267, 0.03]), 110.0)
    assert look(w, [outline(0.013, 0.269), outline(-0.15, 0.20, "blue")], now=112.0) == ["something new is on the table"]


def test_a_put_down_is_forgotten_after_a_minute():
    w = watcher()
    w._gripper(True, np.array([0.016, 0.267, 0.03]), 100.0)
    w._gripper(False, np.array([0.016, 0.267, 0.03]), 110.0)
    assert look(w, [outline(0.013, 0.269)], now=110.0 + tracker.PUT_DOWN_S + 1) == ["something new is on the table"]
