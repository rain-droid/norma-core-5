"""Chess and drawing in the agent workspace, against the simulated robot: a board of blocks seen
by the simulated top camera, moved by a "human" (the test) and by the arm."""

import json
import os
import shutil
import socket
import sys
import threading
import time

import chess
import numpy as np
import pytest
import uvicorn

from elrobot.config import REPO_ROOT, Config
from elrobot.daemon import create_app
from elrobot.simworld import Block

TABLE_Z = 0.010
SQUARE = 0.028
KING = 0.045
# The arm plays black, so black's back rank is the one nearest the base. Outer corners a1, h1, h8, a8.
CORNERS = [[0.112, 0.344], [-0.112, 0.344], [-0.112, 0.120], [0.112, 0.120]]
RATIO = {"k": 1.0, "q": 0.9, "b": 0.78, "n": 0.68, "r": 0.6, "p": 0.52}


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
    _forget_tools()  # those of an earlier test module's workspace, bound to its daemon
    sys.path.insert(0, str(root / "tools"))
    yield root, app.state.hub.link.frame_provider.__self__
    sys.path.remove(str(root / "tools"))
    _forget_tools()  # and ours, so the next module's workspace imports its own
    server.should_exit = True
    thread.join(5)


def _forget_tools() -> None:
    for name, module in list(sys.modules.items()):
        if "/workspace/tools/" in (getattr(module, "__file__", None) or "") or name == "util":
            del sys.modules[name]


def run(tool: str, **arguments) -> dict:
    from util.cli import call

    sys.modules.pop(tool, None)
    return call(getattr(__import__(tool), tool), arguments)


def square_xy(name: str) -> np.ndarray:
    sq = chess.parse_square(name)
    a1, h1, h8, a8 = (np.array(c) for c in CORNERS)
    s, t = (chess.square_file(sq) + 0.5) / 8, (chess.square_rank(sq) + 0.5) / 8
    return (1 - s) * (1 - t) * a1 + s * (1 - t) * h1 + s * t * h8 + (1 - s) * t * a8


def set_pieces(world, board: chess.Board) -> None:
    world.blocks = [
        Block(chess.square_name(sq), square_xy(chess.square_name(sq)), (0.016, 0.016), KING * RATIO[p.symbol().lower()],
              (235, 235, 235) if p.color else (45, 45, 45))
        for sq, p in board.piece_map().items()
    ]  # fmt: skip


def piece_at(world, name: str):
    """The block standing on a square, if any."""
    near = [b for b in world.blocks if np.linalg.norm(b.xy - square_xy(name)) < 0.012 and b.z < 0.005]
    return near[0] if near else None


def human(world, board: chess.Board, uci: str) -> None:
    """The person at the table plays a move: blocks jump."""
    mv = chess.Move.from_uci(uci)
    if board.is_capture(mv):
        world.blocks.remove(piece_at(world, chess.square_name(mv.to_square)))
    piece_at(world, chess.square_name(mv.from_square)).xy = square_xy(chess.square_name(mv.to_square))
    if board.is_castling(mv):
        rank = chess.square_name(mv.from_square)[1]
        a, b = ("h", "f") if board.is_kingside_castling(mv) else ("a", "d")
        piece_at(world, a + rank).xy = square_xy(b + rank)
    board.push(mv)


def test_a_game_of_chess(robot):
    root, world = robot
    board = chess.Board()
    set_pieces(world, board)
    assert run("home")["pose"] == "ready"

    setup = run("chess_setup", king_height=KING, corners_xy=CORNERS, robot_color="black")
    assert "error" not in setup, setup
    assert abs(setup["square_mm"] - 28) < 0.5
    assert setup["unreachable_squares"] == [] and setup["squares_that_disagree"] == []
    assert (root / setup["board_image"]).is_file()

    nothing = run("chess_see")  # the human has not moved yet
    assert nothing["move_seen"] is False and "recorded" not in nothing

    def their(uci):
        human(world, board, uci)
        seen = run("chess_see")
        assert seen.get("move") == uci and seen["certain"] and seen["recorded"], seen
        assert seen["fen"] == board.fen()

    def mine(uci, steps=1):
        mv = chess.Move.from_uci(uci)
        played = run("chess_move", move=uci)
        assert "error" not in played, played
        assert len(played["steps"]) == steps, played
        board.push(mv)
        assert piece_at(world, chess.square_name(mv.to_square)) is not None, "the piece is not standing on its square"
        assert piece_at(world, chess.square_name(mv.from_square)) is None
        assert played["verified_by_camera"], played
        assert played["fen"] == board.fen()
        return played

    assert "human's turn" in run("chess_move", move="e7e5")["error"]
    their("e2e4")
    assert "not legal" in run("chess_move", move="e7e4")["error"]
    mine("d7d5")
    their("e4d5")  # the human captures
    n = len(world.blocks)
    mine("d8d5", steps=2)  # the arm captures: the pawn goes off the board, the queen takes its square
    assert len(world.blocks) == n and sum(1 for b in world.blocks if b.z < 0.005 and abs(b.xy[0]) > 0.118) == 1
    their("g1f3")
    mine("c8g4")
    their("f1e2")
    mine("b8c6")
    their("e1g1")  # castles
    mine("e8c8", steps=2)  # so does the arm: king, then rook

    # a move the camera cannot explain is reported, not recorded
    piece_at(world, "a2").xy = square_xy("a5")
    odd = run("chess_see")
    assert not odd.get("certain") and "recorded" not in odd and odd["fen"] == board.fen()
    piece_at(world, "a5").xy = square_xy("a2")
    assert run("chess_record", move="a2a3")["squares_that_disagree"]  # recorded by hand, but the board does not show it
    assert run("chess_record", undo=True)["squares_that_disagree"] == []


def test_copy_a_drawing(robot):
    import cv2

    root, world = robot
    world.blocks = []
    run("home")

    # the picture to copy: a house and a sun
    picture = np.full((500, 640, 3), 255, np.uint8)
    cv2.rectangle(picture, (150, 250), (400, 440), (0, 0, 0), 5)
    cv2.line(picture, (130, 250), (275, 120), (0, 0, 0), 5)
    cv2.line(picture, (275, 120), (420, 250), (0, 0, 0), 5)
    cv2.circle(picture, (520, 110), 45, (0, 0, 0), 5)
    cv2.imwrite(str(root / "house.png"), picture)
    planned = run("draw_plan", image_path="house.png")
    assert "error" not in planned and 2 <= planned["strokes"] <= 8, planned
    assert (root / planned["preview_image"]).is_file()

    corners = [[0.08, 0.14], [-0.08, 0.14], [-0.08, 0.26], [0.08, 0.26]]  # the drawing reads from the far side of the table
    assert "no pen in hand" in run("draw_run", dry_run=True)["error"]
    offered = run("draw_pen", corners=corners)
    assert "error" not in offered and offered["sheet_mm"] == [160, 120], offered
    assert run("draw_pen", grip=True)["holding"] is False  # nobody has put a pen in yet

    # a person stands the pen in the open jaws, its tip on the paper
    from util.arm import Arm
    tcp = np.array(Arm().state()["ee"]["pos"])
    run("open_gripper")
    world.blocks = [Block("pen", tcp[:2].copy(), (0.012, 0.012), 0.13, (40, 40, 200))]
    held = run("draw_pen", grip=True)
    assert held["holding"] and 40 < held["pen_tip_beyond_tool_point_mm"] < 80, held

    dry = run("draw_run", dry_run=True)
    assert dry.get("planned") and dry["strokes"] == planned["strokes"], dry
    drawn = run("draw_run", speed=0.8)
    assert drawn.get("drawn"), drawn

    # where did the pen tip touch the paper? from every joint command that was sent
    from util import strokes as S
    from util.planning import Planner
    plan, st = Planner(Arm()), S.load()
    a, reach = np.array(st["pen"]["approach"]), st["pen"]["tip_to_tool"]
    ink = []
    for line in (root / drawn["history"] / "joint_commands.jsonl").read_text().splitlines():
        for q in json.loads(line).get("path", []):
            T = plan.fk(q)
            tip = T[:3, 3] + a * reach
            assert np.linalg.norm(T[:3, 2] - a) < 0.08, "the pen tilted"
            if tip[2] < plan.table_z + 0.0005:
                ink.append(tip[:2])
    ink = np.array(ink)
    want = np.vstack([S.dense(s, 0.002) for s in S.on_sheet(st["drawing"], corners, 0.015)[0]])

    def nearest(a_pts, b_pts):
        return np.array([np.linalg.norm(b_pts - p, axis=1).min() for p in a_pts])

    assert nearest(want, ink).max() < 0.005, "part of the picture was not drawn"
    assert nearest(ink, want).max() < 0.003, "the pen touched the paper where there is no line"
    lo, hi = np.min(corners, axis=0), np.max(corners, axis=0)
    assert np.all(ink > lo + 0.01) and np.all(ink < hi - 0.01), "ink outside the margin"

    # strokes composed by the agent itself: a triangle
    tri = run("draw_plan", strokes=[[[0, 0], [10, 0], [5, 8], [0, 0]]])
    assert tri["strokes"] == 1 and abs(tri["height_over_width"] - 0.8) < 1e-6
    assert run("draw_run", dry_run=True)["planned"]
    assert run("draw_pen", release=True)["holding"] is False  # the pen is handed back
    assert "no pen in hand" in run("draw_run", dry_run=True)["error"]


def test_grasp_planner_avoids_a_neighbour(robot):
    root, world = robot
    world.blocks = [
        Block("red cube", np.array([0.07, 0.20]), (0.025, 0.025), 0.025, (60, 60, 220)),
        Block("blue cube", np.array([0.105, 0.20]), (0.025, 0.025), 0.025, (210, 120, 40)),  # 10 mm to one side of it
        Block("green bar", np.array([-0.08, 0.24]), (0.070, 0.022), 0.022, (80, 180, 70)),
    ]
    run("home")
    neighbour = {"x": 0.105, "y": 0.20, "yaw_deg": 0, "footprint_mm": [25, 25], "height_mm": 25}

    alone = run("plan_grasp", x=0.07, y=0.20, footprint_mm=[25, 25], height_mm=25)
    assert "error" not in alone and alone["rejected"] == [], alone
    assert (root / alone["grasp_image"]).is_file()

    crowded = run("plan_grasp", x=0.07, y=0.20, footprint_mm=[25, 25], height_mm=25, obstacles=[neighbour])
    best = crowded["candidates"][0]
    assert abs(best["yaw_deg"]) == 90, best  # jaws close front-to-back, fingers clear of the cube beside it
    assert best["finger_clearance_mm"] >= 2
    assert any("finger would come down" in r["why"] for r in crowded["rejected"])
    assert all(abs(c["yaw_deg"]) == 90 for c in crowded["candidates"])

    # a low neighbour is not in the way of fingertips that stay above it
    flat = run("plan_grasp", x=0.07, y=0.20, footprint_mm=[25, 25], height_mm=60, obstacles=[{**neighbour, "height_mm": 5}])
    assert not any("finger" in r["why"] for r in flat["rejected"])

    # a bar wider than the jaws one way is gripped across its narrow side; too wide both ways is refused with a reason
    bar = run("plan_grasp", x=-0.08, y=0.24, footprint_mm=[70, 22], yaw_deg=0, height_mm=22)
    assert bar["candidates"][0]["closes_on_mm"] == 22 and bar["candidates"][0]["cost_parts"]["offset"] == 0
    assert any("wider than the jaws" in r["why"] for r in bar["rejected"])
    slab = run("plan_grasp", x=0.0, y=0.22, footprint_mm=[80, 70], height_mm=20)
    assert slab["candidates"] == [] and "no grasp survives" in slab["note"]

    picked = run("plan_grasp", x=0.07, y=0.20, footprint_mm=[25, 25], height_mm=25, obstacles=[neighbour], pick=True)
    assert picked.get("holding") and 20 < picked["width_mm"] < 30, picked
    assert world.held is not None and world.held.name == "red cube"
    blue = next(b for b in world.blocks if b.name == "blue cube")
    assert np.allclose(blue.xy, [0.105, 0.20])
