"""Chess on a physical board: where the squares are, what the top camera sees on them, which
move was played, and how to carry a piece.

The game itself is tracked with python-chess in `state/chess.json`. The camera is not asked to
recognise piece shapes: at 640x480 from across the table a knight and a bishop are a dozen
pixels each. It only has to tell, per square, whether it is empty, holds a light piece or holds
a dark piece. What each of those looks like is learned from the board itself every time the
position is known (`fit_model`), so it follows the lighting, the set and the camera angle.
The human's move is then the legal move whose resulting position best explains the picture
(`detect_move`); an illegal or ambiguous picture is reported, not guessed.

The board is fixed by the four outer corners of its 8x8 grid in the base frame (a1, h1, h8, a8).
From across the table a standing piece hides part of the squares behind it. So what a square
should look like is worked out for each candidate position from the top camera's calibrated
position: a line of sight to an empty square that passes through a piece nearer the camera is
expected to show that piece (`expected_labels`).
"""

import json
import math
import shutil
from pathlib import Path

import chess
import chess.engine
import cv2
import numpy as np

from util import vision
from util.arm import RobotError
from util.motion import READY_DEG

ROOT = Path(__file__).resolve().parents[2]
STATE = ROOT / "state" / "chess.json"
# Height of each piece as a fraction of the king's (a usual Staunton set).
HEIGHT_RATIO = {"k": 1.0, "q": 0.9, "b": 0.78, "n": 0.68, "r": 0.6, "p": 0.52}
MAX_SAMPLES = 48  # remembered examples per (class, square colour)
PATCH = 10  # pixels per side of a sampled patch


def load() -> dict:
    if not STATE.exists():
        raise RobotError("no chess board is set up: run tools/chess_setup.py first (see skills/play-chess)")
    return json.loads(STATE.read_text())


def save(st: dict) -> None:
    STATE.parent.mkdir(exist_ok=True)
    tmp = STATE.with_suffix(".tmp")
    tmp.write_text(json.dumps(st, indent=1) + "\n")
    tmp.replace(STATE)


def board_of(st: dict) -> chess.Board:
    b = chess.Board(st["start_fen"])
    for uci in st["moves"]:
        b.push_uci(uci)
    return b


# --------------------------------------------------------------------------- geometry


def board_xy(st: dict, u: float, v: float) -> np.ndarray:
    """Base-frame x, y of board coordinates in squares: (0, 0) is the outer corner of a1,
    u runs along the files towards h, v along the ranks towards 8. Off-board values extrapolate."""
    a1, h1, h8, a8 = (np.array(c) for c in st["corners_xy"])
    s, t = u / 8.0, v / 8.0
    return (1 - s) * (1 - t) * a1 + s * (1 - t) * h1 + s * t * h8 + (1 - s) * t * a8


def square_xy(st: dict, sq: int) -> np.ndarray:
    return board_xy(st, chess.square_file(sq) + 0.5, chess.square_rank(sq) + 0.5)


def square_size(st: dict) -> float:
    c = [np.array(p) for p in st["corners_xy"]]
    return float(np.mean([np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]) / 8.0)


def file_axis_deg(st: dict) -> float:
    """Direction of the files (a towards h) in the base frame, degrees from +x."""
    d = np.array(st["corners_xy"][1]) - np.array(st["corners_xy"][0])
    return math.degrees(math.atan2(d[1], d[0]))


def piece_height(st: dict, piece: chess.Piece) -> float:
    return st["king_height"] * HEIGHT_RATIO[piece.symbol().lower()]


def captured_slot(st: dict, k: int) -> np.ndarray:
    """Where the k-th captured piece is put: a column one square clear of the board's side,
    filled from the end nearest the robot's base, then a second column further out."""
    sign = 1.0 if st["captured_side"] == "h" else -1.0
    u = (9.2 if sign > 0 else -1.2) + sign * 1.1 * (k // 8)
    column = sorted((board_xy(st, u, 0.5 + i) for i in range(8)), key=lambda p: float(np.linalg.norm(p - [0.0, 0.04])))
    return column[k % 8]


def turn_aside(ctx) -> None:
    """Swing the arm to the side of the table away from the board, so the top camera sees every
    square. The ready pose with the base turned; does nothing if the arm is already there or limp."""
    state = ctx.arm.state()
    if not state["torque"] or not state["connected"]:
        return
    lo, hi = ctx.arm.limits()
    aside = np.radians(READY_DEG)
    st = json.loads(STATE.read_text()) if STATE.exists() else None
    centre_x = float(np.mean([c[0] for c in st["corners_xy"]])) if st else 0.0
    turn = float(np.clip(math.radians(75), 0.0, min(hi[0], -lo[0]) - 0.05))
    sides = []
    for sign in (1, -1):  # the side whose tool point ends up furthest from the board's centre line
        trial = aside.copy()
        trial[0] = sign * turn
        sides.append((abs(ctx.plan.fk(trial)[0, 3] - centre_x), sign))
    aside[0] = max(sides)[1] * turn
    q = np.array([j["q"] for j in state["joints"][:7]])
    if np.max(np.abs(q - aside)) < 0.05:
        return
    if state["ee"]["pos"][2] < ctx.plan.table_z + 0.06:  # near the board: straight up first, so nothing is swept over
        x, y, _ = state["ee"]["pos"]
        try:
            ctx.arm.joints(ctx.plan.line(q, [x, y, ctx.plan.table_z + 0.10], allow_low=True), speed=0.5, allow_low=True, label="lifting clear")
        except RobotError:
            pass
    ctx.arm.joints(aside, speed=0.7, label="turning aside")


# --------------------------------------------------------------------------- seeing


# Every square is looked at along two lines of sight from the camera: one to the middle of the
# square on the board, one to a point a little above it, inside any piece standing there.
BODY = 0.5  # that point's height, as a fraction of a pawn's
RADIUS = 0.27  # a piece's body, as a fraction of a square, for working out what hides what


def sample_points(st: dict) -> np.ndarray:
    """The 128 points looked at, base frame: for each square its centre on the board, then its body point."""
    z0 = st["board_height"]
    zb = z0 + BODY * st["king_height"] * HEIGHT_RATIO["p"]
    return np.array([[*square_xy(st, sq), z] for sq in chess.SQUARES for z in (z0, zb)])


def square_features(st: dict, cam, img: np.ndarray) -> np.ndarray:
    """Colour and contrast of the image round each sample point (two rows per square)."""
    lab = cv2.cvtColor(cv2.GaussianBlur(img, (3, 3), 0), cv2.COLOR_BGR2LAB)
    half = 0.15  # the patch covers the middle 30% of the square
    feats = np.zeros((128, 4), np.float32)
    for i, sq in enumerate(chess.SQUARES):
        f, r = chess.square_file(sq), chess.square_rank(sq)
        for k, z in enumerate((st["board_height"], st["board_height"] + BODY * st["king_height"] * HEIGHT_RATIO["p"])):
            quad = [cam.to_pixel([*board_xy(st, f + 0.5 + du, r + 0.5 + dv), cam.table_z + z]) for du, dv in ((-half, -half), (half, -half), (half, half), (-half, half))]
            M = cv2.getPerspectiveTransform(np.float32(quad), np.float32([[0, 0], [PATCH, 0], [PATCH, PATCH], [0, PATCH]]))
            patch = cv2.warpPerspective(lab, M, (PATCH, PATCH), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE).reshape(-1, 3).astype(np.float32)
            feats[2 * i + k] = [*patch.mean(axis=0), patch[:, 0].std()]
    return feats


def expected_labels(st: dict, cam, board: chess.Board) -> list[str]:
    """What each line of sight should show in a given position: "w" or "b" for the first piece
    it meets (which may stand on a square nearer the camera), else the board square it lands
    on ("e0" light, "e1" dark) or "x" for the table beyond the board. "?" where the line grazes
    the edge of a piece, so the patch round it shows a mixture and says nothing either way."""
    thin, fat = _labels(st, cam, board, 0.55, -0.2), _labels(st, cam, board, 1.5, 0.2)
    return [a if a == b else "?" for a, b in zip(thin, fat)]


def _labels(st: dict, cam, board: chess.Board, radius_scale: float, taller: float) -> list[str]:
    """expected_labels for pieces drawn thinner and shorter, or fatter and taller, than they are
    (`taller` in pawn heights)."""
    eye = cam.eye_base() - [0.0, 0.0, cam.table_z]  # heights from the table
    pts = sample_points(st)
    z0 = st["board_height"]
    # carry each line of sight on to the board surface: what is seen if nothing is in the way
    t_land = (eye[2] - z0) / (eye[2] - pts[:, 2])
    land = eye[None, :2] + t_land[:, None] * (pts[:, :2] - eye[None, :2])
    d = np.column_stack([land - eye[None, :2], np.full(len(pts), z0 - eye[2])])  # eye -> landing point, t in 0..1
    first = np.full(len(pts), np.inf)
    label = np.empty(len(pts), dtype=object)
    r = RADIUS * radius_scale * square_size(st)
    extra = taller * st["king_height"] * HEIGHT_RATIO["p"]
    for sq, piece in board.piece_map().items():
        c = square_xy(st, sq)
        top = z0 + piece_height(st, piece) + extra
        # where the line is within r of the piece's axis (a quadratic in t) and between its foot and its top
        ox, oy = eye[0] - c[0], eye[1] - c[1]
        A = d[:, 0] ** 2 + d[:, 1] ** 2
        B = 2 * (ox * d[:, 0] + oy * d[:, 1])
        disc = B * B - 4 * A * (ox * ox + oy * oy - r * r)
        ok = disc > 0
        root = np.sqrt(np.where(ok, disc, 0.0))
        t_in, t_out = (-B - root) / (2 * A), (-B + root) / (2 * A)
        t_top = (top - eye[2]) / d[:, 2]  # the line comes down through the piece's top height here
        enter = np.maximum(t_in, t_top)
        hit = ok & (enter < np.minimum(t_out, 1.0)) & (enter < first)
        first[hit] = enter[hit]
        label[hit] = "w" if piece.color else "b"
    a1 = np.array(st["corners_xy"][0])
    uf = (np.array(st["corners_xy"][1]) - a1) / 8.0
    ur = (np.array(st["corners_xy"][3]) - a1) / 8.0
    inv = np.linalg.inv(np.column_stack([uf, ur]))
    for i in np.flatnonzero(first == np.inf):
        f, rk = np.floor(inv @ (land[i] - a1)).astype(int)
        label[i] = f"e{int((f + rk) % 2 == 0)}" if 0 <= f < 8 and 0 <= rk < 8 else "x"
    return list(label)


def fit_model(st: dict, cam, feats: np.ndarray, board: chess.Board) -> None:
    """Remember what a white piece, a black piece and an empty square look like, from a position
    that is certain. Examples accumulate over the game, so every piece type and the present
    lighting are represented."""
    model = st.get("model") or {"samples": {}}
    for feat, key in zip(feats, expected_labels(st, cam, board)):
        if key == "?":
            continue
        model["samples"].setdefault(key, []).append([round(float(x), 2) for x in feat])
        model["samples"][key] = model["samples"][key][-MAX_SAMPLES:]
    model["scale"] = np.maximum(feats.std(axis=0), 1.0).tolist()
    st["model"] = model


def _dist(model: dict, feat, key: str) -> float:
    """Distance to the most similar remembered example of that kind."""
    if key == "?":
        return 0.5  # a grazing line of sight: a small fixed cost whatever it shows
    samples = model["samples"].get(key) or (model["samples"].get("e1" if key == "e0" else "e0") if key[0] == "e" else None)
    if not samples:
        return 0.0
    d = (np.asarray(samples) - feat) / np.asarray(model["scale"])
    return float(np.sqrt((d * d).sum(axis=1)).min())


_NAMES = {"e": "empty board", "x": "the table", "w": "a white piece", "b": "a black piece"}


def disagreements(st: dict, cam, feats: np.ndarray, board: chess.Board) -> list[dict]:
    """Squares where what the camera sees differs from what the tracked position should show."""
    m, out, seen_squares = st["model"], [], set()
    for i, want in enumerate(expected_labels(st, cam, board)):
        if want == "?":
            continue
        seen = min(m["samples"], key=lambda k: _dist(m, feats[i], k))
        clearly = _dist(m, feats[i], want) - _dist(m, feats[i], seen) > 0.75
        if seen[0] != want[0] and clearly and (i // 2) not in seen_squares:
            seen_squares.add(i // 2)
            out.append({"square": chess.square_name(i // 2), "expected_to_see": _NAMES[want[0]], "camera_sees": _NAMES[seen[0]]})
    return out


def detect_move(st: dict, cam, feats: np.ndarray) -> dict:
    """Which legal move best explains what the camera sees now."""
    board, m = board_of(st), st["model"]

    def cost(b: chess.Board) -> float:
        return sum(_dist(m, feats[i], key) for i, key in enumerate(expected_labels(st, cam, b)))

    stay = cost(board)
    scored = []
    for mv in board.legal_moves:
        if mv.promotion and mv.promotion != chess.QUEEN:
            continue  # under-promotions look the same from here; the note below says so
        board.push(mv)
        scored.append((cost(board), mv))
        board.pop()
    scored.sort(key=lambda t: t[0])
    if not scored:
        return {"move_seen": False, "note": "there are no legal moves: the game is over", **game_status(board)}
    best_cost, best = scored[0]
    second = scored[1][0] if len(scored) > 1 else stay
    board.push(best)
    wrong = disagreements(st, cam, feats, board)
    board.pop()
    moved = stay - best_cost > 1.0
    certain = moved and not wrong and second - best_cost > 1.0
    out = {
        "move_seen": bool(moved),
        "certain": bool(certain),
        "move": best.uci(),
        "san": board.san(best),
        "margin_over_next_best": round(second - best_cost, 2),
        "margin_over_no_move": round(stay - best_cost, 2),
        "squares_that_disagree": wrong,
        "alternatives": [{"move": mv.uci(), "san": board.san(mv), "worse_by": round(c - best_cost, 2)} for c, mv in scored[1:4]],
    }
    if not moved:
        out["note"] = "the board looks unchanged: has the human moved, and is nothing (a hand, the arm) over the board?"
    elif not certain:
        out["note"] = "not certain: open the image, ask the human if need be, then record the move with tools/chess_record.py"
    elif best.promotion:
        out["note"] = "a promotion: a queen is assumed; ask the human if they took another piece"
    return out


def overlay(st: dict, cam, img: np.ndarray, board: chess.Board | None = None) -> np.ndarray:
    """The capture with the board's grid and square names drawn where the tools believe they are."""
    out = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
    z = cam.table_z + st["board_height"]

    def px(u, v):
        return tuple(int(round(2 * c)) for c in cam.to_pixel([*board_xy(st, u, v), z]))

    for i in range(9):
        cv2.line(out, px(i, 0), px(i, 8), (0, 255, 255), 1, cv2.LINE_AA)
        cv2.line(out, px(0, i), px(8, i), (0, 255, 255), 1, cv2.LINE_AA)
    for sq in chess.SQUARES:
        f, r = chess.square_file(sq), chess.square_rank(sq)
        if r in (0, 7) or f in (0, 7):
            x, y = px(f + 0.5, r + 0.5)
            vision._label(out, chess.square_name(sq), (x - 9, y + 4), scale=0.38, color=(0, 255, 255))
    return out


# --------------------------------------------------------------------------- the game


def game_status(board: chess.Board) -> dict:
    out = {"turn": "white" if board.turn else "black", "check": board.is_check()}
    if board.is_game_over():
        out["game_over"] = board.result()
        out["outcome"] = board.outcome().termination.name.lower()
    return out


def transfers(st: dict, board: chess.Board, move: chess.Move) -> list[dict]:
    """The pieces to carry, in order, for one chess move: a captured piece off the board first,
    then the piece itself, then the rook when castling."""
    steps = []
    if board.is_capture(move):
        sq = move.to_square
        if board.is_en_passant(move):
            sq = chess.square(chess.square_file(move.to_square), chess.square_rank(move.from_square))
        steps.append({"what": f"take the captured piece off {chess.square_name(sq)}", "piece": board.piece_at(sq),
                      "src": square_xy(st, sq), "dst": captured_slot(st, st["captured"]), "squares": [sq], "to_table": True})  # fmt: skip
    steps.append({"what": f"move {chess.square_name(move.from_square)} to {chess.square_name(move.to_square)}", "piece": board.piece_at(move.from_square),
                  "src": square_xy(st, move.from_square), "dst": square_xy(st, move.to_square), "squares": [move.from_square, move.to_square]})  # fmt: skip
    if board.is_castling(move):
        rank = chess.square_rank(move.from_square)
        a, b = (chess.square(7, rank), chess.square(5, rank)) if board.is_kingside_castling(move) else (chess.square(0, rank), chess.square(3, rank))
        steps.append({"what": f"move the rook {chess.square_name(a)} to {chess.square_name(b)}", "piece": board.piece_at(a),
                      "src": square_xy(st, a), "dst": square_xy(st, b), "squares": [a, b]})  # fmt: skip
    return steps


def jaw_yaw_deg(st: dict, board: chess.Board, squares: list[int]) -> float:
    """Jaw direction (base frame, degrees; 0 closes along x) that puts the fingers beside the
    fewest neighbouring pieces: along a file, along a rank, or along a diagonal."""
    best = None
    for along, (df, dr) in ((0, (1, 0)), (90, (0, 1)), (45, (1, 1)), (135, (-1, 1))):
        clash = 0
        for sq in squares:
            f, r = chess.square_file(sq), chess.square_rank(sq)
            for sign in (1, -1):
                nf, nr = f + sign * df, r + sign * dr
                clash += 0 <= nf < 8 and 0 <= nr < 8 and board.piece_at(chess.square(nf, nr)) is not None
        if best is None or clash < best[0]:
            best = (clash, along)
    yaw = (best[1] + file_axis_deg(st)) % 180.0
    return yaw - 180.0 if yaw > 90.0 else yaw


def engine_move(board: chess.Board, seconds: float = 0.5) -> dict | None:
    """Stockfish's choice, if Stockfish is installed."""
    path = shutil.which("stockfish")
    if not path:
        return None
    with chess.engine.SimpleEngine.popen_uci(path) as eng:
        info = eng.analyse(board, chess.engine.Limit(time=seconds))
    mv = info["pv"][0]
    return {"move": mv.uci(), "san": board.san(mv), "score": str(info["score"].pov(board.turn))}
