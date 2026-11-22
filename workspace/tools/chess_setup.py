"""Tell the workspace where the chessboard is and start a game from the position on it.

Do this once per game, with every piece on its square, the arm out of the camera's view
(`home`, or it is turned aside for you) and nothing else on the board.

The board is given by the four OUTER corners of its 8x8 grid, in this order: the corner of a1,
of h1, of h8, of a8 (a1 is the dark square at white's left hand). Either as pixels of a `top`
capture (`image_id` + `corners_px`, read off its `grid_image`), or as base-frame positions
(`corners_xy`, metres). Pixels are sharpened onto the nearest printed corner (`snap`, at most
5 px) when the image has one there.

`king_height` is the height of the king in metres; the other pieces are taken as the usual
fractions of it. `board_height` is the thickness of the board: how far its playing surface is
above the mat. `robot_color` is the side the arm plays. `fen` starts from another position
than the usual one. `captured_side` is the side of the board ("h" or "a" file) beside which
captured pieces are put down; left out, the side the arm reaches better.

Returns `board_image`: the capture with the grid drawn where the tools now believe the squares
are. Open it. If the grid does not sit on the squares, the corners are wrong: run this again
with better ones before any move. Also returns the measured square size, the squares the arm
cannot reach (move the board closer to the base if there are any) and `squares_that_disagree`:
squares where the camera cannot tell a piece from an empty square in this very picture, which
means a piece is missing or misplaced, or something is over the board.
"""

import chess
import cv2
import numpy as np

from util import capture, chessboard
from util.arm import RobotError
from util.motion import require_ready


def _snap(img: np.ndarray, corners: np.ndarray, radius: int = 5) -> np.ndarray:
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    fine = cv2.cornerSubPix(gray, corners.astype(np.float32).reshape(-1, 1, 2).copy(), (radius, radius), (-1, -1),
                            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.05)).reshape(-1, 2)  # fmt: skip
    moved = np.linalg.norm(fine - corners, axis=1)
    return np.where((moved < radius)[:, None], fine, corners)


def reach_report(ctx, st: dict) -> tuple[list[str], list[str]]:
    """Squares, drawn as the board seen from white's side, and the names of those out of reach."""
    low = st["board_height"] + max(0.6 * st["king_height"] * chessboard.HEIGHT_RATIO["p"], 0.020)
    rows, bad = [], []
    for rank in range(7, -1, -1):
        line = f"{rank + 1} "
        for file in range(8):
            sq = chess.square(file, rank)
            try:
                ctx.plan.approach_for(chessboard.square_xy(st, sq), low, 0.0)
                line += "#"
            except RobotError:
                line += "."
                bad.append(chess.square_name(sq))
        rows.append(line)
    return [*rows, "  abcdefgh"], bad


def chess_setup(ctx, king_height: float, image_id: str | None = None, corners_px: list | None = None, corners_xy: list | None = None,
                board_height: float = 0.0, robot_color: str = "black", fen: str | None = None, captured_side: str | None = None,
                snap: bool = True) -> dict:  # fmt: skip
    require_ready(ctx)
    if robot_color not in ("white", "black"):
        raise RobotError("robot_color is 'white' or 'black'")
    meta = ctx.arm.meta()
    cam = capture.TopCamera(ctx.root, meta["mode"], meta["table_z"])
    if corners_xy is None:
        if image_id is None or corners_px is None or len(corners_px) != 4:
            raise RobotError("give the board's four outer corners (a1, h1, h8, a8): image_id with corners_px, or corners_xy")
        img0, cap = capture.load(ctx, image_id)
        if cap["camera"] != "top":
            raise RobotError("the board is measured with the top camera: pass a top capture")
        px = np.array(corners_px, dtype=float)
        if snap:
            px = _snap(img0, px)
        corners_xy = [cam.to_base(u, v, board_height)[:2].tolist() for u, v in px]
    if len(corners_xy) != 4:
        raise RobotError("corners_xy is four [x, y] positions: the outer corners of a1, h1, h8, a8")
    c = np.array(corners_xy, dtype=float)
    sides = [float(np.linalg.norm(c[(i + 1) % 4] - c[i])) for i in range(4)]
    if max(sides) > 1.15 * min(sides):
        raise RobotError(f"those corners do not make a square (sides {[round(s * 1000) for s in sides]} mm): check their order (a1, h1, h8, a8) and position")
    (ax, ay), (bx, by) = c[1] - c[0], c[3] - c[0]
    turn = ax * by - ay * bx
    if turn < 0:
        raise RobotError("those corners describe a mirrored board: going a1, h1, h8, a8 must turn anticlockwise seen from above")

    start = fen or chess.STARTING_FEN
    try:
        board = chess.Board(start)
    except ValueError as e:
        raise RobotError(f"not a valid FEN: {e}") from None
    st = {"corners_xy": c.round(4).tolist(), "king_height": king_height, "board_height": board_height, "robot_color": robot_color,
          "start_fen": start, "moves": [], "captured": 0, "captured_side": captured_side or "h", "model": None}  # fmt: skip
    if captured_side is None:  # the side whose first slots the arm reaches
        def reachable(side):
            trial = {**st, "captured_side": side}
            try:
                for k in (0, 7):
                    ctx.plan.approach_for(chessboard.captured_slot(trial, k), 0.03, 0.0)
                return True
            except RobotError:
                return False
        st["captured_side"] = "h" if reachable("h") else "a" if reachable("a") else "h"

    chessboard.turn_aside(ctx)
    img = ctx.frame("top")
    capture.freeze(ctx, "top", img, grid=False)
    feats = chessboard.square_features(st, cam, img)
    chessboard.fit_model(st, cam, feats, board)
    wrong = chessboard.disagreements(st, cam, feats, board)
    chessboard.save(st)
    rows, bad = reach_report(ctx, st)
    out = {
        "square_mm": round(chessboard.square_size(st) * 1000, 1),
        "corners_xy": st["corners_xy"],
        "robot_plays": robot_color,
        "captured_pieces_go_beside_file": st["captured_side"],
        "board_image": ctx.save_image("board.jpg", chessboard.overlay(st, cam, img), "the board with the grid the tools will use; check it sits on the squares"),
        "reach": rows,
        "unreachable_squares": bad,
        "squares_that_disagree": wrong,
        "fen": board.fen(),
        **chessboard.game_status(board),
    }
    return out


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(chess_setup)
