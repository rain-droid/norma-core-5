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
    if "util.tracker" in sys.modules:
        sys.modules["util.tracker"].stop()  # the one process a tool leaves behind
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

    picked = run("pick_at", x=seen["red"]["x"], y=seen["red"]["y"], yaw_deg=seen["red"]["yaw_deg"])
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


def test_maroon_ring_is_red_across_the_hue_wrap(robot):
    import cv2
    from find_objects import color_name
    from util.vision import blobs_from_mask

    # Dark table visible through the hole, red hues on both sides of HSV's wrap.
    # A linear hue median calls this blue; the saturated ring must decide its colour.
    y, x = np.indices((80, 80))
    radius2 = (x - 39.5) ** 2 + (y - 39.5) ** 2
    ring = (radius2 < 30**2) & (radius2 > 16**2)
    hsv = np.full((80, 80, 3), (105, 65, 70), dtype=np.uint8)
    hsv[ring] = (1, 210, 95)
    hsv[ring & (x >= 40)] = (179, 210, 95)
    blob = blobs_from_mask(ring.astype(np.uint8) * 255, 120)[0]
    assert color_name(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), blob) == "red"

    hsv[ring] = (110, 210, 95)
    assert color_name(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), blob) == "blue"


def test_workspace_plans_with_the_model_the_robot_side_checks_against():
    """The workspace carries its own copy of the robot model. If the two drifted apart, plans made
    here would be judged against a different arm."""
    assert (REPO_ROOT / "workspace/robot_config/model.urdf").read_bytes() == Config().urdf.read_bytes()


# ---- live tracking: positions that stay current, and what the tools do when the table changes


def arrange(world) -> dict:
    """A fresh table and a fresh tracker; returns the blocks by colour."""
    from util import tracker

    tracker.stop()
    run("home")
    run("open_gripper")
    world.held = world.hand = None
    blocks = {b.name.split()[0]: b for b in world.blocks}
    for colour, xy in (("red", (0.07, 0.20)), ("blue", (-0.08, 0.22)), ("green", (0.0, 0.28))):
        blocks[colour].xy, blocks[colour].z = np.array(xy), 0.0
    return blocks


def when(condition, then) -> threading.Thread:
    """Do `then` once, from the side, as soon as `condition()` holds: the world changing while a tool runs."""

    def watch():
        deadline = time.time() + 30
        while time.time() < deadline:
            if condition():
                return then()
            time.sleep(0.02)

    thread = threading.Thread(target=watch, daemon=True)
    thread.start()
    return thread


def doing(label: str):
    from util.arm import Arm

    return lambda: Arm().state()["activity"] == label


def test_tracks_keep_their_names_as_objects_move(robot):
    _, world = robot
    blocks = arrange(world)
    first = {t["id"]: t for t in run("track_objects")["tracks"]}
    assert set(first) == {"red-1", "blue-1", "green-1"}, first
    for name, t in first.items():
        assert t["state"] == "visible" and np.linalg.norm([t["x"], t["y"]] - blocks[t["color"]].xy) < 0.008, t

    blocks["red"].xy = np.array([0.15, 0.30])  # further than any one step of a sliding object
    time.sleep(1.0)
    after = {t["id"]: t for t in run("track_objects")["tracks"]}
    assert set(after) == set(first), "the same object under the same name, not a new one"
    assert np.linalg.norm([after["red-1"]["x"], after["red-1"]["y"]] - blocks["red"].xy) < 0.008 and after["red-1"]["still_s"] < 2.0


def test_pick_goes_to_where_the_object_is_by_then(robot):
    _, world = robot
    blocks = arrange(world)
    run("track_objects")

    def nudge():
        time.sleep(0.1)
        blocks["red"].xy = blocks["red"].xy + [0.03, 0.03]

    when(doing("reaching"), nudge)
    picked = run("pick_object", object="red-1")
    assert picked.get("holding") and world.held is blocks["red"], picked
    assert picked["tried"], "it noticed: either aimed again from the hover point, or closed on nothing and looked again"

    placed = run("place_object", x=0.12, y=0.26)
    assert placed.get("checked") and placed["off_target_mm"] < 12, placed
    assert np.linalg.norm(blocks["red"].xy - [0.12, 0.26]) < 0.01


def test_a_hand_in_the_work_area_stops_the_arm_and_the_pick_resumes(robot):
    from elrobot.simworld import Block
    from util import tracker

    _, world = robot
    blocks = arrange(world)
    run("track_objects")

    def reach_in():
        world.hand = Block("hand", np.array([0.27, 0.33]), (0.26, 0.10), 0.03, (120, 60, 30))
        time.sleep(1.5)
        blocks["red"].xy = np.array([0.13, 0.24])  # and it moves the object while it is there
        world.hand = None

    when(doing("reaching"), reach_in)
    picked = run("pick_object", object="red-1")
    assert picked.get("holding") and world.held is blocks["red"], picked
    assert any("stopped" in step for step in picked["tried"]), picked
    assert tracker.read()["stopped_arm_t"], "the tracker is what stopped it"


def test_a_stack_is_checked_and_an_object_that_fell_off_is_put_back(robot):
    _, world = robot
    blocks = arrange(world)
    run("track_objects")
    assert run("pick_object", object="red-1").get("holding")

    def fall_off():
        time.sleep(0.3)
        blocks["red"].xy, blocks["red"].z = np.array([-0.15, 0.30]), 0.0

    when(doing("backing off"), fall_off)
    placed = run("place_object", on="blue-1")
    assert placed.get("checked") and any("placed it again" in step for step in placed["tried"]), placed
    assert blocks["red"].z == blocks["blue"].height and np.linalg.norm(blocks["red"].xy - blocks["blue"].xy) < 0.012
    tracks = {t["id"]: t for t in run("track_objects")["tracks"]}
    assert tracks["red-1"]["on"] == "blue-1"

    # and it comes off again from on top, not from the table
    assert run("pick_object", object="red-1").get("holding") and world.held is blocks["red"]
