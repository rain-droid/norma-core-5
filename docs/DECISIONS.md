# Decisions

Why things are the way they are. Newest first. Each entry: what was decided, why, and what it costs.
Entries marked "(from the history)" were made before the 2026-11-22 sessions; their reasons are taken
from commit messages, `CLAUDE.md` and `workspace/AGENTS.md`, not witnessed.

## 2026-11-22

**The shared space checks paths; it does not search for them.** Both arms are capsules in one frame, and a
path is walked against the other arm and against the depth camera's height map before it is sent. A path that
fails is refused with the reason. The one planning rule is for carrying: up, across above the tallest thing on
the line, down. That covers a table top, where everything stands on one surface. Joining both arms into one
Pinocchio model and letting Pink's collision barriers bend the path (both are installed and support it), or a
planner such as cuRobo on the laptop's GPU, would be the next step up; neither was needed to stop the arms
hitting each other, and neither fixes what limits the hand-over, which is a centimetre of calibration.

**Obstacle maps expire after two minutes.** The arms move the very things the map shows. A stale map refuses
moves through places that are now empty and allows moves through things that were put there since, and the
second is the dangerous one. So a map is only used while fresh, and a tool that needs it scans first.

**The second arm runs the teammate's branch as its own process; it is not merged.** The `autopilot` branch
has what an SO-101 needs (profile, URDF with the parallel gripper, servo calibration, IK for five joints) but
forks from an early commit and rewrites `robot.py`, `kinematics.py` and `skills.py`, which have since changed
here beyond a merge an hour before submission. Run as a second daemon on the same Station it needed no change
at all; the workspace talks to it over its HTTP tools.

**The two arms are tied together by the depth camera, not by a ruler.** The SO-101 carries a coloured block
through a dozen places; the camera measures the block, the SO-101 reports its tool point, and the rigid fit
between the two sets is where it stands. The camera sees only the faces turned to it, so its "middle" is off
by a centimetre or two, but it is off the same way when the ElRobot later holds a thing out, so the error
largely cancels: measure and use with the same sensor.

**A small thing is handed over by its lower half, from below.** Both grippers are wider than a 3 cm cube.
Held by the ElRobot's fingertips at its top, the cube's lower half hangs free; the SO-101, which has no level
pose closer than about 24 cm to its own base and therefore no room for a run-in, comes up from 5 cm below
with its jaws level and open around it. Height is the tight direction (about 1 cm), and the SO-101 sags by a
varying amount when stretched out, so after arriving it is lifted until its own joint readings match.

**The depth camera is read with Orbbec's old SDK, built from source.** The DaBai DCW2's depth sensor speaks
Orbbec's OpenNI protocol. The current SDK (`pyorbbecsdk2` 2.1.3) lists the USB device and matches no driver to
it; the old one reads it, but its Linux wheel on PyPI (`pyorbbecsdk` 1.3.2) contains macOS libraries. So
`scripts/orbbec_setup.sh` builds the v1 bindings from Orbbec's repository into `bin/orbbec/` (not committed),
for the project's Python. The Station keeps the colour camera; the depth sensor is a separate USB device, so
the two do not compete.

**The depth sensor is placed from the mat's plane and two tags, not from a full pose fit.** The colour camera
sees only the two corner tags on the robot's side, small and at the same distance: a pose from those alone is
weak in tilt. The mat fills the depth picture, so its plane gives height and tilt to a millimetre, and the two
tag centres, followed down their sight lines to that plane, give the remaining turn and shift. The distance
between the two tags comes out 2 mm off the ruler's 585 mm, which checks the camera's own calibration too.

**RoboHarness's ARX_X5 workspace is the model for this one, and is not copied into it.** With a fixed depth
camera the rig now resembles theirs, and the layout here (one file per tool, `robot_config/`, `perception/`,
skills, frozen images, call history) already follows it. Its code does not carry over: about 39 000 lines of
`tools/util` built on the X5's link geometry and gripper (`x5_kinematics`, `x5_grasp_planner`, 4 300 lines),
on a 14-value two-arm command, and on the RoboDojo evaluator's loop (tools are generators that hand one
action per tick to a simulator, plans are signed per session), with simulator depth already aligned to the
colour picture. Porting the grasp planner to a seven-joint arm with soft servos and a noisy real depth
picture is days of work, not the two hours before submission. What its depth tools do is added here on this
arm's own motion code instead, starting with `depth_scan`.

**Hand-guiding keeps the motors on.** The usual way to teach a hobby arm by hand is to switch torque off and
record while a person carries it; on this arm that drops the gripper onto whatever is under it the moment the
person lets go. The Station leaves the servos on a soft proportional gain, so a hand already displaces a joint
by a few degrees: the daemon moves the command after the displacement (an admittance loop at 50 Hz) and the arm
goes where it is led and holds when released. The joint's slow error, which is gravity, is tracked and
subtracted so the arm does not creep; the price is that a push held still fades after a few seconds. What is
saved is joint angles: a replay repeats the motion exactly and knows nothing about where the objects are now.

**Safe mode and collaboration mode were removed the morning they were tried.** Safe mode (gripper straight
down only, straight-line travel, refuse what is too far) left the arm an area of about 32 by 12 cm and 7 cm of
height, against 36 cm of reach when it may lean: most of the table was "too far". Collaboration mode (lower
torque limit, speed capped at 0.6, stop and back off on a touch) made every task slower for a touch detection
nobody had needed yet. Both restricted the arm more than the demo could afford, so the code is gone, not
switched off: `git show 7027d7e` has both if they are wanted again. What remains for working near people is
what was there before: the stall guard (`blocked`), the tracker slowing the arm when something moves on the
table, and `stop`. Hand-guiding stayed; it restricts nothing.

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
