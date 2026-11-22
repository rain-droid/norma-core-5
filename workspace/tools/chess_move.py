"""Play a chess move on the board with the arm, check it with the camera, and record it.

`move` is in UCI form: e7e5, e8g8 to castle, e7e8q to promote. It must be legal and it must be
the arm's turn (see chess_see).

What it does, each step built from joint commands as pick_at and place_at are:
 1. a captured piece is carried off the board first, to the next free spot beside it;
 2. the piece is gripped a little above its middle, lifted straight up until its foot clears a
    standing king, carried over and set down on its square; when castling the rook follows;
 3. the jaws open only as wide as a square, and are turned along the file, the rank or a
    diagonal, whichever has the fewest neighbouring pieces beside the fingers;
 4. the arm turns aside, the top camera looks, and the move is recorded.

`verified_by_camera: true` means every square then looked as the new position says. If it is
false, read `squares_that_disagree` and open the image from chess_see: a piece may have been
knocked over or set down between squares. Straighten it (pick_at / place_at, or ask the human)
before the game goes on.

A promotion leaves the pawn on the last rank and says so in `needs_human`: ask the human to
swap it for the piece.

If a grasp or a move fails part way, the error says which step, nothing is recorded, and the
arm lets go and turns aside. Look at the board, finish the move by hand or with pick_at and
place_at, then record it with chess_record.
"""

import time

import chess
import numpy as np

from util import capture, chessboard
from util.arm import RobotError
from util.motion import require_ready

JAW_MAX = 0.053  # metres between the pads, fully open


def _column(ctx, xy, low: float, lift: float, yaw_deg: float):
    """Joint path straight down onto a point: from `lift` metres above it to tool height `low`
    over the table. Tries the most vertical gripper first. Returns (waypoints top to bottom, pitch)."""
    plan = ctx.plan
    bottom = np.array([xy[0], xy[1], plan.table_z + low])
    why = "no approach found"
    for yaw in dict.fromkeys((yaw_deg, 0.0)):  # leaning jaws that are also turned can dip a fingertip: fall back to yaw 0
        for pitch in (90, 80, 70, 60, 50, 40):
            r = plan.solve(bottom, pitch=pitch, yaw=yaw, q_seed=np.radians(chessboard.READY_DEG), allow_low=True)
            if not r.ok:
                why = r.describe()
                continue
            try:
                up = plan.line(r.q, bottom + [0.0, 0.0, lift], allow_low=True)
            except RobotError as e:
                why = str(e)
                continue
            return up[::-1], float(pitch)
    raise RobotError(f"can't reach x={xy[0]:+.3f} y={xy[1]:+.3f} from above: {why}")


def carry(ctx, st: dict, piece: chess.Piece, src, dst, yaw_deg: float, dst_surface: float) -> dict:
    """Pick the piece standing at `src` and stand it at `dst` (base x, y)."""
    arm = ctx.arm
    h = chessboard.piece_height(st, piece)
    grip = max(0.6 * h, 0.020)  # tool point above the piece's foot: below 20 mm the fingertips press into the board
    lift = st["king_height"] + 0.025  # the carried piece's foot clears a standing king
    opening = float(np.clip((0.8 * chessboard.square_size(st) + 0.012) / JAW_MAX, 0.55, 1.0))
    # everything is planned before anything moves, so an impossible move leaves the board untouched
    down_src, pitch = _column(ctx, src, st["board_height"] + grip, lift, yaw_deg)
    down_dst, _ = _column(ctx, dst, dst_surface + grip + 0.002, lift, yaw_deg)

    arm.joints(gripper=opening, label="opening")
    arm.joints(down_src[0], speed=0.8, label="reaching")
    arm.joints(down_src[1:], speed=0.4, allow_low=True, label="descending")
    grasp = arm.joints(gripper=0.0, label="grasping")["gripper"]
    if not grasp["holding"]:
        arm.joints(gripper=opening, label="opening")
        arm.joints(down_src[::-1][1:], speed=0.5, allow_low=True, label="backing off")
        raise RobotError(f"closed on nothing at x={src[0]:+.3f} y={src[1]:+.3f}: the piece is not where the board says")
    arm.joints(down_src[::-1][1:], speed=0.4, allow_low=True, label="lifting")
    time.sleep(0.2)
    if not arm.state()["gripper"]["holding"]:
        raise RobotError("had the piece, but it slipped out while lifting")
    arm.joints(down_dst[0], speed=0.6, label="carrying")
    arm.joints(down_dst[1:], speed=0.4, allow_low=True, label="lowering")
    arm.joints(gripper=opening, label="releasing")
    arm.joints(down_dst[::-1][1:], speed=0.5, allow_low=True, label="backing off")
    return {"width_mm": grasp["width_mm"], "approach_pitch_deg": pitch}


def chess_move(ctx, move: str, even_if_not_my_turn: bool = False) -> dict:
    require_ready(ctx, awake=True)
    st = chessboard.load()
    board = chessboard.board_of(st)
    mine = chess.WHITE if st["robot_color"] == "white" else chess.BLACK
    if board.turn != mine and not even_if_not_my_turn:
        raise RobotError("it is the human's turn: run tools/chess_see.py after they have moved")
    try:
        mv = chess.Move.from_uci(move)
    except ValueError:
        raise RobotError(f"'{move}' is not a move in UCI form such as e7e5") from None
    if mv not in board.legal_moves:
        raise RobotError(f"{move} is not legal here; legal moves: {' '.join(m.uci() for m in board.legal_moves)}")

    san, done = board.san(mv), []
    for step in chessboard.transfers(st, board, mv):
        yaw = chessboard.jaw_yaw_deg(st, board, step["squares"])
        try:
            got = carry(ctx, st, step["piece"], step["src"], step["dst"], yaw, 0.0 if step.get("to_table") else st["board_height"])
        except RobotError as e:
            try:
                ctx.arm.joints(gripper=1.0, label="letting go")
                chessboard.turn_aside(ctx)
            except RobotError:
                pass
            raise RobotError(f"failed to {step['what']}: {e}. Done before that: {done or 'nothing'}. The move is not recorded: "
                             "look at the board (chess_see with record false), finish it, then chess_record.") from None  # fmt: skip
        done.append({"step": step["what"], "piece": chess.piece_name(step["piece"].piece_type), "jaw_yaw_deg": round(yaw), **got})
        if step.get("to_table"):
            st["captured"] += 1
    board.push(mv)
    st["moves"].append(mv.uci())
    out = {"played": mv.uci(), "san": san, "steps": done}
    if mv.promotion:
        out["needs_human"] = f"ask the human to swap the pawn on {chess.square_name(mv.to_square)} for a {chess.piece_name(mv.promotion)}"
    try:
        chessboard.turn_aside(ctx)
        meta = ctx.arm.meta()
        cam = capture.TopCamera(ctx.root, meta["mode"], meta["table_z"])
        img = ctx.frame("top")
        feats = chessboard.square_features(st, cam, img)
        wrong = chessboard.disagreements(st, cam, feats, board)
        out["verified_by_camera"] = not wrong
        out["squares_that_disagree"] = wrong
        out["board_image"] = ctx.save_image("board.jpg", chessboard.overlay(st, cam, img), "the board after the move")
        if not wrong:
            chessboard.fit_model(st, cam, feats, board)
    except RobotError as e:
        out["verified_by_camera"] = False
        out["note"] = f"could not look afterwards: {e}"
    chessboard.save(st)
    out.update({"board": str(board).split("\n"), "fen": board.fen(), **chessboard.game_status(board)})
    return out


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(chess_move)
