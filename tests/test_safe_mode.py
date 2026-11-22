"""Safe mode against the simulated robot: the gripper points straight down and travels in
straight lines, and what cannot be reached that way is refused."""

import json
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
from elrobot.simworld import Block

TABLE_Z = 0.010


def _forget_tools() -> None:
    for name, module in list(sys.modules.items()):
        if "/workspace/tools/" in (getattr(module, "__file__", None) or "") or name == "util":
            del sys.modules[name]


@pytest.fixture(scope="module")
def robot(tmp_path_factory):
    root = tmp_path_factory.mktemp("agent") / "workspace"
    shutil.copytree(REPO_ROOT / "workspace", root, ignore=shutil.ignore_patterns("perception", "tool_call_history", "state", "__pycache__", "assets", "*.sim.json", ".*"))
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    app = create_app(Config(sim=True, voice=False, workspace_dir=root, table_z=TABLE_Z))
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning"))
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    while not server.started:
        time.sleep(0.05)
    os.environ["ELROBOT_DAEMON"] = f"http://127.0.0.1:{port}"
    os.environ["ELROBOT_SAFE_MODE"] = "1"
    _forget_tools()
    sys.path.insert(0, str(root / "tools"))
    yield root, app.state.hub.link.frame_provider.__self__
    sys.path.remove(str(root / "tools"))
    _forget_tools()
    os.environ["ELROBOT_SAFE_MODE"] = "0"
    server.should_exit = True
    thread.join(5)


def run(tool: str, **arguments) -> dict:
    from util.cli import call

    sys.modules.pop(tool, None)
    return call(getattr(__import__(tool), tool), arguments)


def tool_points(root, result) -> list[tuple[np.ndarray, float]]:
    """Every waypoint a call sent, as (tool point, degrees from straight down)."""
    from util import safe
    from util.arm import Arm
    from util.planning import Planner

    plan = Planner(Arm())
    out = []
    for line in (root / result["history"] / "joint_commands.jsonl").read_text().splitlines():
        for q in json.loads(line).get("path", []):
            T = plan.fk(q)
            out.append((T[:3, 3], safe.tilt_deg(T)))
    return out


def assert_straight_and_down(points, what: str) -> None:
    assert points, f"{what}: nothing was sent"
    assert max(t for _, t in points) < 16.0, f"{what}: the gripper tilted {max(t for _, t in points):.0f} degrees"
    for (a, _), (b, _) in zip(points, points[1:]):
        across, up = float(np.linalg.norm(b[:2] - a[:2])), abs(float(b[2] - a[2]))
        assert across < 0.004 or up < 0.004, f"{what}: a step went {across * 1000:.0f} mm across and {up * 1000:.0f} mm up at once"


def test_everything_is_straight_lines_with_the_gripper_straight_down(robot):
    root, world = robot
    world.blocks = [
        Block("red cube", np.array([0.07, 0.20]), (0.025, 0.025), 0.025, (60, 60, 220)),
        Block("green bar", np.array([-0.08, 0.17]), (0.045, 0.022), 0.022, (80, 180, 70)),
    ]
    home = run("home")
    assert home["pose"] == "ready" and abs(home["tcp"]["pitch_deg"] - 90) < 3, home

    picked = run("pick_at", x=0.07, y=0.20, yaw_deg=0)
    assert picked.get("holding") and picked["approach_pitch_deg"] == 90.0, picked
    assert_straight_and_down(tool_points(root, picked), "pick")

    placed = run("place_at", x=-0.06, y=0.22, yaw_deg=40)  # the jaws turned: the gripper still points straight down
    assert "error" not in placed and placed["approach_pitch_deg"] == 90.0, placed
    assert_straight_and_down(tool_points(root, placed), "place")
    red = next(b for b in world.blocks if b.name == "red cube")
    assert np.linalg.norm(red.xy - [-0.06, 0.22]) < 0.012 and red.z < 0.005

    back = run("home")
    assert_straight_and_down(tool_points(root, back), "home from a straight-down pose")


def test_what_is_too_far_is_refused_before_anything_moves(robot):
    root, world = robot
    world.blocks = [Block("blue cube", np.array([0.0, 0.33]), (0.025, 0.025), 0.025, (210, 120, 40))]
    run("home")
    far = run("pick_at", x=0.0, y=0.33)
    assert "too far for safe mode" in far["error"] and "moved closer" in far["error"], far
    assert far["joint_command_count"] == 0
    assert np.allclose(world.blocks[0].xy, [0.0, 0.33])

    leaning = run("move_tcp", x=0.0, y=0.20, z=0.08, pitch_deg=45)
    assert "only points straight down" in leaning["error"], leaning
    high = run("move_tcp", x=0.0, y=0.20, z=0.20)
    assert "error" in high  # straight down the arm does not reach that high


def test_with_safe_mode_off_the_arm_leans_again(robot):
    os.environ["ELROBOT_SAFE_MODE"] = "0"
    try:
        _, world = robot
        world.blocks = [Block("blue cube", np.array([0.0, 0.33]), (0.025, 0.025), 0.025, (210, 120, 40))]
        run("home")
        got = run("pick_at", x=0.0, y=0.33)
        assert got.get("holding") and got["approach_pitch_deg"] < 90, got
        run("place_at", x=0.0, y=0.20)
    finally:
        os.environ["ELROBOT_SAFE_MODE"] = "1"
