# Decisions

Why things are the way they are. Newest first. Each entry: what was decided, why, and what it costs.
Entries marked "(from the history)" were made before the 2026-11-22 sessions; their reasons are taken
from commit messages, `CLAUDE.md` and `workspace/AGENTS.md`, not witnessed.

## 2026-11-22

**Safe mode trades reach for predictability.** With it on, the gripper only points straight down and only
travels in straight lines, so nothing swings through the space above the table and a person can see where it
will go. It is enforced where every tool ends, in `Arm.joints` (`util/arm.py`): a single target is replaced by
an up, across, down route or refused, so tools written later, and scripts the agent writes, obey it without
knowing about it. The cost is large and worth stating: straight down, this arm works in an area of about
32 by 12 cm and no higher than 7 cm, against 36 cm of reach when it may lean; objects beyond that are refused,
not attempted, and a carried object clears only about 4 cm. Two moves are not lines, both in the air: getting
up from and sitting down on the resting seat, and reaching the straight-down posture from a pose that is not.
Chess (`turn_aside`, leaning grasps at the far ranks) does not work with it on.

**The chat box and the microphone drive the workspace agent, not a second tool set.** There were two agents: the
workspace one (top camera, calibration, names, checks) and the voice one, which only saw the daemon's older
wrist-camera tools. A person typing "pick up the ring" in the web UI should get the same harness as the terminal
does, so the voice agent now runs in `workspace/` with a shell, `AGENTS.md` and the skills. Cost: a turn is
slower than a single MCP call, and it spends more tokens.

**Three languages, decided per utterance and per sentence.** Speech to text chooses among the configured
languages only, because unrestricted detection on a two-word command wanders off into languages nobody here
speaks. The reply's voice is chosen sentence by sentence from the words themselves, falling back to the language
the person last used, since the voice model has to be told its language up front. The multilingual voice model
is licensed for non-commercial use only (CC BY-NC): fine for the hackathon, to be replaced or licensed before
anything is sold. Cost: multilingual recognition takes about 1.4 s an utterance on this laptop against 0.8 s
for English alone; `ELROBOT_LANGUAGES=en` goes back.

**That agent's commands are judged by a fixed rule, because nobody is there to approve them.** One Python file
from the workspace per command, through the workspace's interpreter; no pipes, chaining, redirection or
substitution; writing only inside the workspace and not its calibration; reading only inside the repository.
Scripts it writes itself can be run: that is how it does what no tool does (code as policy), and it means the
rule limits accidents, not a determined misuse. Do not expose the daemon beyond this machine.

**The web UI's live vision is worked out by the daemon, on request, and is for display only.** The camera
panel asks `GET /api/camera/{name}/vision` a few times a second while a live layer is switched on; nothing is
computed when nobody is looking, and the result is cached per frame. It draws object outlines, AprilTags, the
robot's frame and reach, and the arm as the kinematics place it. It deliberately does not feed the tools: they
keep their own, slower and more careful measurements (`find_objects`, the tracker). The "Arm" layer is the
quickest check that calibration still holds: the line should lie on the real arm.

**The camera panel shows every camera, not only the wrist.** It opens on `top`, which sees the whole table;
the choice and the layers are remembered in the browser, and `?camera=...&layers=...` in the address sets them.

**A move stopped as blocked is sent once more, slower, by the workspace, not by the daemon.** Most "blocked"
stops on this arm are a joint trailing a fast setpoint under load with nothing in the way, and the same move
slower goes through (seen on joints 1, 2, 4 and 6, and on `home` from rest above speed 0.5). It lives in `Arm.joints` (`util/arm.py`), which
every tool ends in, because the daemon takes joint commands and decides nothing: a second try is a second joint
command, and it is in `joint_commands.jsonl` like the first. Cost: when something really is in the way the arm
pushes against it a second time, slower, before the error comes back.

**Once, and only for "blocked".** A second refusal is raised as it is: two failures mean look, not try a third
time. A servo fault needs a person, a lost connection or a stop is not cured by going slower, and an envelope
refusal is the same at any speed, so none of those is tried again. Nor is a gripper-only command. The second try
starts from the waypoint nearest to where the arm stopped, not from the start of the path.

**How much slower is one number in a file: `blocked_retry_speed` in `workspace/robot_config/motion.json`, 0 to 1,
default 0.5.** A fraction of the first try's speed, so it scales with whatever speed the tool chose. 0 switches
the second try off, which is what a contact-seeking setup wants. It is read when a move is blocked, not at
start, so it can be tuned between two commands. A value that cannot be read means no second try and the error
says so: a typo in the file must not stop the arm from working, and must not pass unnoticed either.

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

**The tracker's reaction to an intrusion is a motion heuristic, not a safety interlock.** By default it slows the arm to a quarter of its speed while something else moves on the table and lets it return to full speed afterwards (the operator's choice, 2026-11-22: a stop for every hand at the edge of the mat made tasks fail); `on_intrusion: "stop"` halts it as before. It is never to be switched to `ignore` automatically.
