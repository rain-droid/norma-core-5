"""Record a chess move in the tracked game without moving anything, or take the last one back.

Use it when chess_see was not certain and you (or the human) know which move was played, and
after finishing or correcting a move on the board by hand. `move` is in UCI form: e2e4, e1g1 to
castle, e7e8q to promote. `undo: true` removes the last recorded move instead; the pieces are
not put back for you.

Afterwards it looks at the board with the top camera and reports `squares_that_disagree` with
the position as now tracked. An empty list means the board and the record agree.
"""

import chess

from util import capture, chessboard
from util.arm import RobotError


def chess_record(ctx, move: str | None = None, undo: bool = False) -> dict:
    st = chessboard.load()
    board = chessboard.board_of(st)
    if undo:
        if not st["moves"]:
            raise RobotError("there is no move to take back")
        board.pop()
        done = {"taken_back": st["moves"].pop()}
    else:
        try:
            mv = chess.Move.from_uci(move or "")
        except ValueError:
            raise RobotError(f"'{move}' is not a move in UCI form such as e2e4") from None
        if mv not in board.legal_moves:
            raise RobotError(f"{move} is not legal here; legal moves: {' '.join(m.uci() for m in board.legal_moves)}")
        done = {"recorded": mv.uci(), "san": board.san(mv)}
        board.push(mv)
        st["moves"].append(mv.uci())
    try:
        meta = ctx.arm.meta()
        cam = capture.TopCamera(ctx.root, meta["mode"], meta["table_z"])
        feats = chessboard.square_features(st, cam, ctx.frame("top"))
        wrong = chessboard.disagreements(st, cam, feats, board)
        if not wrong:
            chessboard.fit_model(st, cam, feats, board)
    except RobotError as e:
        wrong = f"could not look: {e}"
    chessboard.save(st)
    return {**done, "squares_that_disagree": wrong, "board": str(board).split("\n"), "fen": board.fen(), **chessboard.game_status(board)}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(chess_record)
