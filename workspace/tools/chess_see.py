"""Look at the chessboard: find the move the human just played, and show the board.

Takes a `top` picture with the arm turned aside, and asks which legal move best explains it.

`certain: true` means exactly one legal move fits and every square then looks as it should; the
move is recorded (`recorded: true`) and it is the arm's turn. Otherwise nothing is recorded:
open `board_image`, read `squares_that_disagree` and `alternatives`, ask the human if need be,
and record the move yourself with chess_record. `move_seen: false` means the board looks as it
did: the human has not moved yet, or a hand was over the board.

`record: false` only looks. `close_up` (a square name such as "e4") adds `close_up_image`, an
enlarged cut-out round that square, to tell which piece stands there.

Always returns the tracked position (`board`, white at the bottom, capitals white), `fen`, the
`legal_moves` for the side to move, and `engine` (Stockfish's suggestion) when Stockfish is installed.
"""

import chess
import cv2

from util import capture, chessboard
from util.motion import require_ready


def chess_see(ctx, record: bool = True, close_up: str | None = None) -> dict:
    require_ready(ctx)
    st = chessboard.load()
    meta = ctx.arm.meta()
    cam = capture.TopCamera(ctx.root, meta["mode"], meta["table_z"])
    chessboard.turn_aside(ctx)
    img = ctx.frame("top")
    capture.freeze(ctx, "top", img, grid=False)
    board = chessboard.board_of(st)
    feats = chessboard.square_features(st, cam, img)

    mine = chess.WHITE if st["robot_color"] == "white" else chess.BLACK
    out: dict = {}
    if board.turn == mine or board.is_game_over():
        out["squares_that_disagree"] = chessboard.disagreements(st, cam, feats, board)
        out["note"] = "the game is over" if board.is_game_over() else "it is the arm's turn: nothing to detect; choose a move and play it with chess_move"
    else:
        out = chessboard.detect_move(st, cam, feats)
        if out.get("certain") and record:
            board.push_uci(out["move"])
            st["moves"].append(out["move"])
            chessboard.fit_model(st, cam, feats, board)
            chessboard.save(st)
            out["recorded"] = True
    out["board_image"] = ctx.save_image("board.jpg", chessboard.overlay(st, cam, img), "the board now, with the grid the tools use")
    if close_up:
        sq = chess.parse_square(close_up)
        u, v = cam.to_pixel([*chessboard.square_xy(st, sq), cam.table_z + st["board_height"] + 0.5 * st["king_height"]])
        r = int(2.2 * abs(cam.to_pixel([*chessboard.board_xy(st, 1, 0), cam.table_z])[0] - cam.to_pixel([*chessboard.board_xy(st, 0, 0), cam.table_z])[0]) + 12)
        x0, y0 = max(int(u) - r, 0), max(int(v) - r, 0)
        cut = img[y0 : int(v) + r, x0 : int(u) + r]
        out["close_up_image"] = ctx.save_image("close_up.jpg", cv2.resize(cut, None, fx=6, fy=6, interpolation=cv2.INTER_CUBIC), f"enlarged view round {close_up}")
        out["close_up_tracked_piece"] = str(board.piece_at(sq) or "empty")
    out.update({"board": str(board).split("\n"), "fen": board.fen(), "robot_plays": st["robot_color"],
                "legal_moves": [m.uci() for m in board.legal_moves], **chessboard.game_status(board)})  # fmt: skip
    if board.turn == mine and not board.is_game_over():
        hint = chessboard.engine_move(board)
        if hint:
            out["engine"] = hint
    return out


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(chess_see)
