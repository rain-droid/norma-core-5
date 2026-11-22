"""Collaboration mode on the simulated arm: slower, softer, and it backs off when touched."""

import asyncio
import os
import shutil
import socket
import sys
import threading
import time

import numpy as np
import pytest
import uvicorn

from elrobot.config import REPO_ROOT, Config
from elrobot.daemon import create_app
from elrobot.kinematics import Kinematics
from elrobot.link import SimLink
from elrobot.robot import Contact, Reflex, Robot
from elrobot.servo import ARM_IDS, REG_TORQUE_LIMIT


def test_a_step_in_load_is_a_touch_and_a_slow_drift_is_not():
    reflex = Reflex(load_rise=150.0, lag_rise=0.09)
    load, lag = np.zeros(7), np.zeros(7)
    hit = None
    for i in range(60):  # 1.2 s of a move whose load drifts by 200 units a second, as gravity does
        load[1] += 4.0
        hit = hit or reflex.update(load.copy(), lag, ending=False)
        time.sleep(0.02)
    assert hit is None
    for i in range(12):  # then a hand: 400 units at once
        hit = hit or reflex.update(load + np.array([0, 400, 0, 0, 0, 0, 0.0]), lag, ending=False)
        time.sleep(0.02)
    assert hit and hit.startswith("joint 2")


async def _robot(collab: bool) -> Robot:
    cfg = Config(collab=collab)
    kin = Kinematics(cfg.urdf)
    limits = {j.id: (j.lower, j.upper) for j in kin.joints}
    limits[8] = (kin.gripper_lower, kin.gripper_upper)
    link = SimLink(limits)
    await link.start()
    assert await link.wait_connected(2)
    robot = Robot(link, cfg)
    await robot.enable()
    return robot


def test_touched_in_mid_move_it_stops_backs_off_and_says_contact():
    async def scenario():
        robot = await _robot(collab=True)
        assert robot.pace == pytest.approx(robot.cfg.collab_speed)  # slower throughout
        start = robot.q()
        target = start.copy()
        target[0] += 0.9

        async def push():  # well into the move a hand leans on the base joint
            await asyncio.sleep(1.3)
            robot.link.push[ARM_IDS[0]] = 450

        pusher = asyncio.create_task(push())
        with pytest.raises(Contact) as stopped:
            await robot.move_joints(target)
        pusher.cancel()
        robot.link.push.clear()
        assert "backed off" in str(stopped.value) and str(stopped.value).startswith("contact: joint 1")
        travelled = abs(robot.q()[0] - start[0])
        assert 0.02 < travelled < 0.8, travelled  # it had set off, and it did not arrive
        await robot.link.stop()

    asyncio.run(scenario())


def test_switching_it_on_lowers_the_torque_limit_and_off_restores_it():
    async def scenario():
        robot = await _robot(collab=False)
        written = []
        sync_write = robot.link.sync_write

        async def spy(register, values):
            if register == REG_TORQUE_LIMIT:
                written.append(int.from_bytes(values[ARM_IDS[0]], "little"))
            return await sync_write(register, values)

        robot.link.sync_write = spy
        assert await robot.set_collab(True) is True and robot.snapshot()["collab"] is True
        assert await robot.set_collab(False) is False
        assert written == [robot.cfg.collab_torque_limit, robot.cfg.arm_torque_limit]
        assert robot.pace == 1.0
        await robot.link.stop()

    asyncio.run(scenario())


def test_led_by_hand_it_follows_the_push_stays_there_and_reports_the_path():
    async def scenario():
        robot = await _robot(collab=False)
        start = robot.q()

        async def hand():  # a hand holds the base joint 0.12 rad off its command for a second and a half, then lets go
            await asyncio.sleep(0.6)
            robot.link.shove[ARM_IDS[0]] = 0.12 * 4096 / (2 * np.pi)
            await asyncio.sleep(1.5)
            robot.link.shove.clear()

        pushing = asyncio.create_task(hand())
        led = await robot.guide(seconds=12.0)
        await pushing
        assert led["ended"] == "left alone" and led["moved"]
        went = robot.q()[0] - start[0]
        assert 0.25 < went < 1.0, went  # it followed the hand
        assert np.max(np.abs(robot.q()[1:] - start[1:])) < 0.02  # and only where it was pushed
        assert len(led["path"]) >= 3 and abs(led["path"][-1][0] - robot.q()[0]) < 0.05
        await asyncio.sleep(0.5)
        assert abs(robot.q()[0] - start[0] - went) < 0.02  # let go, it stays
        await robot.link.stop()

    asyncio.run(scenario())


def test_left_untouched_the_guiding_ends_on_time_without_having_moved():
    async def scenario():
        robot = await _robot(collab=False)
        start = robot.q()
        led = await robot.guide(seconds=1.5)
        assert led == {"path": led["path"], "moved": False, "ended": "time", "seconds": led["seconds"]} and len(led["path"]) == 1
        assert np.max(np.abs(robot.q() - start)) < 0.01
        await robot.link.stop()

    asyncio.run(scenario())


# ---------------------------------------------------------------- through the workspace's tools


def _forget_tools() -> None:
    for name, module in list(sys.modules.items()):
        if "/workspace/tools/" in (getattr(module, "__file__", None) or "") or name == "util":
            del sys.modules[name]


@pytest.fixture(scope="module")
def workspace(tmp_path_factory):
    root = tmp_path_factory.mktemp("agent") / "workspace"
    shutil.copytree(REPO_ROOT / "workspace", root, ignore=shutil.ignore_patterns("perception", "tool_call_history", "state", "__pycache__", "assets", "*.sim.json", ".*"))
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    app = create_app(Config(sim=True, voice=False, workspace_dir=root, table_z=0.010))
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning"))
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    while not server.started:
        time.sleep(0.05)
    os.environ["ELROBOT_DAEMON"] = f"http://127.0.0.1:{port}"
    _forget_tools()
    sys.path.insert(0, str(root / "tools"))
    yield root, app.state.hub.link
    sys.path.remove(str(root / "tools"))
    _forget_tools()
    server.should_exit = True
    thread.join(5)


def joints_now() -> np.ndarray:
    from util.arm import Arm

    return np.array([j["q"] for j in Arm().state()["joints"][:7]])


def run(tool: str, **arguments) -> dict:
    from util.cli import call

    sys.modules.pop(tool, None)
    return call(getattr(__import__(tool), tool), arguments)


def test_the_agent_can_switch_collaboration_on_and_a_touch_ends_the_move_as_contact(workspace):
    root, link = workspace
    assert "error" not in run("home")
    assert run("collaboration", enable=True)["collaboration"] is True
    q = joints_now().tolist()
    q[0] += 0.9
    threading.Timer(1.3, lambda: link.push.__setitem__(ARM_IDS[0], 450)).start()
    touched = run("joint_command", joints=q)
    link.push.clear()
    assert touched["error"].startswith("contact: joint 1"), touched
    assert "retried_blocked" not in touched  # a touch is not answered by pushing again
    assert run("collaboration", enable=False)["collaboration"] is False


def test_a_place_shown_by_hand_is_saved_and_replayed(workspace):
    root, link = workspace
    assert "error" not in run("home")
    start = joints_now()

    def hand():
        time.sleep(1.0)
        link.shove[ARM_IDS[0]] = 0.12 * 4096 / (2 * np.pi)
        time.sleep(1.5)
        link.shove.clear()

    threading.Thread(target=hand, daemon=True).start()
    led = run("hand_guide", seconds=12, save_as="Over there", then_gripper=0.0, new=True)
    assert led["moved"] and led["ended"] == "left alone" and led["saved"] == "state/demos/over-there.json", led
    shown = joints_now()
    assert abs(shown[0] - start[0]) > 0.25

    assert "error" not in run("home")
    assert run("replay_demo")["demos"] == ["over-there"]
    again = run("replay_demo", name="over there")
    assert again["completed"] and [sorted(s) for s in again["steps"]] == [["path"], ["gripper", "holding"]], again
    now = joints_now()
    assert np.max(np.abs(now - shown)) < 0.03, (now, shown)
