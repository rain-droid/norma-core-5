"""Leading the simulated arm by hand: it follows a push, stays where it is left, and what it was shown is replayed."""

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
from elrobot.robot import Robot
from elrobot.servo import ARM_IDS


async def _robot() -> Robot:
    cfg = Config()
    kin = Kinematics(cfg.urdf)
    limits = {j.id: (j.lower, j.upper) for j in kin.joints}
    limits[8] = (kin.gripper_lower, kin.gripper_upper)
    link = SimLink(limits)
    await link.start()
    assert await link.wait_connected(2)
    robot = Robot(link, cfg)
    await robot.enable()
    return robot


def test_led_by_hand_it_follows_the_push_stays_there_and_reports_the_path():
    async def scenario():
        robot = await _robot()
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
        robot = await _robot()
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
