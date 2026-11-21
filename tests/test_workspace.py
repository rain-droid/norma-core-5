"""The agent workspace against a simulated robot: its tools run in this process, as they do in
an agent's shell, and reach the arm only through joint commands over HTTP."""

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

TABLE_Z = 0.010  # not zero, so a tool that forgets the table height shows


@pytest.fixture(scope="module")
def robot(tmp_path_factory):
    root = tmp_path_factory.mktemp("agent") / "workspace"
    shutil.copytree(REPO_ROOT / "workspace", root, ignore=shutil.ignore_patterns("perception", "tool_call_history", "__pycache__", "assets", "*.sim.json", ".*"))
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    app = create_app(Config(sim=True, voice=False, workspace_dir=root, table_z=TABLE_Z))
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning"))
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    while not server.started:
        time.sleep(0.05)
    os.environ["ELROBOT_DAEMON"] = f"http://127.0.0.1:{port}"  # read by util/arm.py when first imported
    sys.path.insert(0, str(root / "tools"))
    yield root, app.state.hub.link.frame_provider.__self__  # the simulated world: where the blocks really are
    server.should_exit = True
    thread.join(5)


def run(tool: str, **arguments) -> dict:
    from util.cli import call

    return call(getattr(__import__(tool), tool), arguments)


def test_pick_and_place_from_the_top_camera(robot):
    root, world = robot
    red = next(b for b in world.blocks if b.name == "red cube")
    assert run("home")["pose"] == "ready"

    seen = {o["color"]: o for o in run("find_objects")["objects"]}
    assert np.linalg.norm(np.array([seen["red"]["x"], seen["red"]["y"]]) - red.xy) < 0.006, seen["red"]
    assert abs(seen["red"]["width_mm"] - 25) <= 3

    picked = run("pick_at", x=seen["red"]["x"], y=seen["red"]["y"], yaw=seen["red"]["yaw_deg"])
    assert picked.get("holding") and 20 < picked["width_mm"] < 30, picked
    placed = run("place_at", x=0.10, y=0.28)
    assert placed.get("released_something"), placed
    assert np.linalg.norm(red.xy - [0.10, 0.28]) < 0.01 and red.z == 0.0, "the block is where it was put, on the table"

    # the record of what crossed to the robot: joint angles and jaw openings, nothing else
    commands = [json.loads(line) for line in (root / picked["history"] / "joint_commands.jsonl").read_text().splitlines()]
    assert len(commands) == picked["joint_command_count"] == 5
    assert all(set(c) <= {"t", "label", "path", "gripper", "speed", "check", "allow_low", "outcome"} for c in commands)
    assert all(len(q) == 7 for c in commands for q in c.get("path", []))
    assert json.loads((root / "perception/proprioception/state.json").read_text())["connected"]


def test_the_robot_side_refuses_and_a_broken_tool_says_so(robot):
    from util.arm import Arm
    from util.planning import Planner

    run("home")
    before = Arm().q()
    over = run("joint_command", joints=[0, 0, 0, 0, 0, 0, 9])
    assert over["error_kind"] == "robot" and "outside its limits" in over["error"]

    # a pose with the fingertips 2 cm under the table, planned without asking whether it is allowed
    plan = Planner(Arm())
    under = plan.kin.ik([0.0, 0.20, TABLE_Z - 0.02], approach=[0, 0, -1], q_seed=before)
    assert under.ok
    refused = run("joint_command", joints=under.q.tolist())
    assert "into the table" in refused["error"], refused
    assert np.allclose(Arm().q(), before, atol=0.02), "nothing moved"

    assert "bad arguments" in run("move_tcp", x=0.0, y=0.2, zz=0.1)["error"]

    def broken(ctx):
        return ctx.arm.q()[99]

    from util.cli import call

    bug = call(broken)
    assert bug["error_kind"] == "tool_bug" and "IndexError" in bug["traceback"]


def test_top_camera_measures_at_a_height(robot):
    from util.capture import camera_from_homography

    _, world = robot
    point = np.array([0.05, 0.25, TABLE_Z + 0.03])
    u, v = world.top_cam.project(point, world.top_pose)
    shot = run("capture_camera", camera="top")
    read = run("pixel_to_table", image_id=shot["image_id"], u=u, v=v, height=0.03)
    assert np.allclose([read["x"], read["y"], read["z"]], point, atol=0.002), read

    # and the camera's position is recoverable from the mat homography alone, as calibrate_top relies on
    cal = world.calibration()
    _, eye = camera_from_homography(cal["H_mat_px"], cal["image_size"])
    assert np.allclose(eye, cal["camera_mat"], atol=1e-4)


def test_workspace_plans_with_the_model_the_robot_side_checks_against():
    """The workspace carries its own copy of the robot model. If the two drifted apart, plans made
    here would be judged against a different arm."""
    assert (REPO_ROOT / "workspace/robot_config/model.urdf").read_bytes() == Config().urdf.read_bytes()
