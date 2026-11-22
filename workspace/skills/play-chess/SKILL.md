---
name: play-chess
description: Play a game of chess against a person on a physical board, seeing their moves with the top camera and moving the pieces with the arm.
---

The game is tracked in `state/chess.json`; the camera only has to tell empty squares from light
and dark pieces, and the human's move is the legal move that explains what it sees. You choose
the arm's moves yourself (or take `engine` from chess_see when Stockfish is installed).

Before the game, with the person:

1. The board has to fit the arm: squares of 30 mm or less (a 24 cm board), its near edge
   11 to 12 cm in front of the base, centred left to right. The arm reaches from about 9 cm to
   36 cm from its base, and a board any bigger has corners out of reach. Pieces must be
   narrower than the jaws (53 mm) and it helps if the two sides differ clearly from the board
   and from each other.
2. `home`, then `capture_camera`, and read the four OUTER corners of the 8x8 grid off
   `grid_image`, in the order a1, h1, h8, a8 (a1 is the dark corner square at white's left
   hand). Ask which side is white if the picture does not show it.
3. `chess_setup` with that `image_id`, `corners_px`, the king's height, the board's thickness
   and the side the arm plays. Open `board_image`: the yellow grid must lie on the squares all
   the way round. If it is off anywhere, correct the corners and run it again; nothing later
   can make up for a misplaced grid. `unreachable_squares` must be empty (else have the board
   moved and start again) and so must `squares_that_disagree` (else a piece is missing,
   misplaced, or something is lying on the board).

Each turn:

4. When the person says they have moved (or after a pause), `chess_see`. `certain: true` and
   `recorded: true`: it is the arm's turn. `move_seen: false`: they have not moved; wait and
   look again. Anything else: open `board_image`, read `squares_that_disagree` and
   `alternatives`, ask the person which move they played, and `chess_record` it.
5. Choose a move from `legal_moves`. Say it aloud in words before playing it.
6. `chess_move`. Read `verified_by_camera`. If false, look (`chess_see` with `record: false`)
   and put right whatever was knocked or left between squares with `pick_at` / `place_at`, or
   ask the person to. A promotion needs the person to swap the pawn: `needs_human` says so.
7. Tell the person it is their turn. Between turns the arm is turned aside; `rest` if the
   wait is long.

What goes wrong and what to do:

- "closed on nothing": the piece is not where the board says. The grid is probably off:
  compare `board_image` with the squares, and run `chess_setup` again with `fen` set to the
  current position (from chess_see) so the game continues.
- A piece falls over or is set down astride two squares: the far squares need a leaning
  gripper and the arm is a centimetre off out there. Have the person straighten it; the game
  record is unaffected. If it keeps happening, move the board 1 to 2 cm closer.
- `squares_that_disagree` after the person's move, all around one spot: a hand or sleeve was
  still over the board. Look again.
- The person takes a move back or moves twice: `chess_record` with `undo: true`, or record the
  extra move, until `squares_that_disagree` is empty.
- Castling, en passant and captures are each done as the transfers they need. Captured pieces
  go beside the board (`captured_pieces_go_beside_file`); keep that strip clear.

Leave the skill with the game over (`game_over` in any chess result) or stopped by the
person, the arm at `rest`, and the last position told to the person.
