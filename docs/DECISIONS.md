# Decisions

Why things are the way they are. Newest first. Each entry: what was decided, why, and what it costs.
Entries marked "(from the history)" were made before the 2026-11-22 sessions; their reasons are taken
from commit messages, `CLAUDE.md` and `workspace/AGENTS.md`, not witnessed.

## 2026-11-22

**Chess, drawing and the grasp planner were added to the existing workspace, not built as a second harness.**
A separate harness was started from scratch before this repo was found; it duplicated the robot layer and had
never touched hardware. The new capabilities were ported to the workspace's conventions instead. The prototype
was discarded.

**Chess reads squares as empty / light piece / dark piece, and takes the move from the rules.**
At this resolution a piece is a dozen pixels, so shapes are not recognised. The human's move is the legal move
that best explains the picture; anything ambiguous is reported, not guessed. Cost: a non-standard starting
position must be given as a FEN.

**Chess perception models what hides what.** From an oblique camera a tall piece covers the squares behind it.
For each candidate position the expected appearance of every line of sight is worked out from the camera's
position (`util/chessboard.py`, `expected_labels`). Without this a vacated square behind the king read as occupied.

**The pen is seated by a person, tip on the paper, before the jaws close.** That fixes the tip position with no
measuring and no pressing the pen down to find the paper, which risks a servo overload on a 7.5 V arm.

**The UGREEN webcam above the mat replaced the KD camera across the table as `top`.** Looking down hides far less
behind objects. Cost: fewer pixels per millimetre on the mat than the old camera had on its near side, and
`calibrate_top` cannot locate a camera that looks exactly straight down. The old camera is now `across`; its
calibration is `workspace/robot_config/camera_top.kd-backup.json`.

**`base_in_mat` was refitted: [0.2985, -0.1256, 4.77 deg], was [0.2765, -0.109, 0].** The old value had been tuned
by eye for the old camera. Two independent measurements agreed: the arm set a cube down at five known positions
and the top camera measured them (residual 3 to 8 mm after the fit), and the wrist camera looked at the mat tags
from four poses. Redo this whenever the robot or the mat moves.

**All cameras stream 1280x720 (stored 853x480).** The wrist calibration from `main` was made at that size; 640x480
is a different crop of the sensor, so the calibration would be wrong there. Cost: the Station refuses 1280x720
on a camera it has held before (libuvc error -51) and silently falls back, so `scripts/cameras_ready.sh` must
run before each Station start.

**The top camera has a fixed 6 ms exposure.** On auto exposure the mat's white corner tags wash out and do not
decode. It does not change the frame rate: the 15 fps comes from the Station keeping every second frame
(`frame-skip: 1`), which was left as it was.

**`pick_verified` is a new tool; `pick_at` is unchanged.** `pick_at`'s own `refine` option projects an outline's
centre onto the table plane, which overshoots by centimetres for an object with height seen at an angle, and
its jaw mask hides the object between the fingers. Rather than change a tested tool, the wrist checks went into
a separate one that uses the object's height and separates neighbours by colour.

**`origin/main` was merged, not cherry-picked,** to bring in the wrist-camera calibration with its code and history.

**python-chess (GPL-3.0) is a dependency** of the workspace and the root project, for legal moves and game state.

## Earlier (from the history)

**The robot takes joint commands only.** No poses, grasps or skills cross to the daemon; IK, planning and
perception are Python in `workspace/tools/` that the agent can read and change. This is the RoboHarness V2 idea:
the workspace is the harness.

**Workspace tools run in the agent's shell, not in the daemon.** A tool is a file the agent can edit and rerun;
the daemon keeps its own guards (limits, table, self-collision, blocked joints, servo faults) whatever tool code does.

**The workspace stands on its own:** its own interpreter, robot model and kinematics, so the directory can be copied whole.

**The arm runs on 7.5 V, not 12 V.** Station is built from NormaCore `6b494f77` with
`patches/norma-core-calibration-voltage.patch`; joint ranges are calibrated with `scripts/calibrate.py --manual`, not `--auto`.

**Tick-to-radian mapping uses the true 4096 ticks per turn,** anchored at the centre of the calibrated arc, not the
proportional mapping NormaCore's viewer uses.

**A "droop" loop trims servo sag** by integrating the settling error into a per-joint bias, because the servos run a soft P-only gain.

**Motion is refused while a servo reports a fault;** an overload needs a person to hold the arm while torque is cycled.

**Angles in tool arguments carry their unit** (`yaw_deg`, `pitch_deg`); everything else is metres and radians.

**The tracker's intrusion stop is a motion heuristic, not a safety interlock.** It is never to be disabled automatically.
