# Decisions

This document gives the reasons for the present state of the project. The newest entries are first. Each entry
gives the decision, the reason, and the cost.
The entries marked "(from the history)" were made before the 2026-11-22 sessions. Their reasons are from
commit messages, `CLAUDE.md` and `workspace/AGENTS.md`. Those reasons were not witnessed.

## 2026-11-22

**The shared space checks paths, and it does not search for paths.** Both arms are capsules in one frame. The
shared space checks a path step by step against the other arm before the path is sent. It also checks the path
against the height map of the depth camera. A path that fails is refused with the reason. The one planning
rule is for carrying: up, across above the tallest thing on the line, down. That rule covers a table top,
where everything stands on one surface. One next step up is to join both arms into one Pinocchio model and let
Pink's collision barriers bend the path. Pinocchio and Pink are installed, and they support this. The other
next step up is a planner such as cuRobo on the laptop's GPU. Neither step was necessary to stop the arms
from hitting each other. Neither step corrects the limit on the hand-over, which is a centimetre of
calibration.

**Obstacle maps expire after two minutes.** The arms move the same things that the map shows. An old map
refuses moves through places that are now empty. An old map also allows moves through things that were put
there after the scan. The second error is the dangerous one. Thus a map is used only while it is fresh. A tool
that needs a map scans first.

**The second arm runs the teammate's branch as its own process, and that branch is not merged.** The
`autopilot` branch has what an SO-101 needs: profile, URDF with the parallel gripper, servo calibration, IK
for five joints. But the branch forks from an early commit, and it rewrites `robot.py`, `kinematics.py` and
`skills.py`. Those files have changed here too much for a merge an hour before submission. As a second daemon
on the same Station, the branch needed no change at all. The workspace talks to that daemon over its HTTP
tools.

**The depth camera, not a ruler, relates the two arms to each other.** The SO-101 carries a coloured block
through a dozen places. The camera measures the block, and the SO-101 reports its tool point. The rigid fit
between the two sets gives the place where the SO-101 stands. The camera sees only the faces that are turned
to it. Thus its "middle" has an error of a centimetre or two. But the "middle" has the same error when the
ElRobot later holds a thing out. Thus the error largely cancels. Measure and use with the same sensor.

**A small thing is handed over by its lower half, from below.** Both grippers are wider than a 3 cm cube.
The ElRobot's fingertips hold the cube at its top, and the lower half of the cube hangs free. The SO-101 has
no level pose closer than about 24 cm to its own base. Thus it has no room for a run-in. It comes up from
5 cm below with its jaws level and open around the cube. Height is the tight direction (about 1 cm). The
SO-101 sags by a varying amount when it is stretched out. Thus, after it arrives, it is lifted until its own
joint readings match.

**The depth camera is read with Orbbec's old SDK, built from source.** The depth sensor of the DaBai DCW2
uses Orbbec's OpenNI protocol. The current SDK (`pyorbbecsdk2` 2.1.3) lists the USB device, but it matches no
driver to the device. The old SDK reads the device. But the Linux wheel of the old SDK on PyPI
(`pyorbbecsdk` 1.3.2) contains macOS libraries. Thus `scripts/orbbec_setup.sh` builds the v1 bindings from
Orbbec's repository into `bin/orbbec/` (not committed), for the project's Python. The Station keeps the
colour camera. The depth sensor is a separate USB device, thus the two do not compete.

**The plane of the mat and two tags locate the depth sensor, not a full pose fit.** The colour camera sees
only the two corner tags on the robot's side. Those tags are small and at the same distance. Thus a pose from
those tags alone is weak in tilt. The mat fills the depth picture. Thus the plane of the mat gives height and
tilt to a millimetre. The two tag centres, followed down their sight lines to that plane, give the remaining
turn and shift. The result for the distance between the two tags is 2 mm different from the ruler's 585 mm.
This also checks the camera's own calibration.

**The ARX_X5 workspace of RoboHarness is the model for this workspace, and it is not copied into this
workspace.** With a fixed depth camera, the rig now resembles their rig. The layout here already follows that
workspace: one file per tool, `robot_config/`, `perception/`, skills, frozen images, call history. The code
of that workspace does not transfer. About 39 000 lines of `tools/util` are built on the X5's link geometry
and gripper (`x5_kinematics`, `x5_grasp_planner`, 4 300 lines). Those lines are also built on a 14-value
two-arm command and on the loop of the RoboDojo evaluator. In that loop, tools are generators that hand one
action per tick to a simulator, and plans are signed per session. In that code, the simulator depth is
already aligned to the colour picture. To port the grasp planner to a seven-joint arm with soft servos and a
noisy real depth picture is days of work. That work does not fit in the two hours before submission. Instead,
the functions of its depth tools are added here on this arm's own motion code. The first is `depth_scan`.

**Hand-guiding keeps the motors on.** The usual method to teach a hobby arm by hand is to switch torque off
and record while a person carries the arm. On this arm, that method drops the gripper onto the thing below it
at the moment the person releases the arm. The Station leaves the servos on a soft proportional gain. Thus a
hand already displaces a joint by a few degrees. The daemon moves the command after the displacement (an
admittance loop at 50 Hz). The arm goes where a hand leads it, and the arm holds when the hand releases it.
The slow error of the joint, which is gravity, is tracked and subtracted. Thus the arm does not creep. The
cost is that a push that is held still fades after a few seconds. The saved data is joint angles. A replay
repeats the motion exactly, and it knows nothing about where the objects are now.

**Safe mode and collaboration mode were removed on the morning that they were tried.** Safe mode had these
rules: gripper straight down only, straight-line travel, refuse what is too far. It left the arm an area of
about 32 by 12 cm and 7 cm of height. Compare this with 36 cm of reach when the arm may lean. Most of the
table was "too far". Collaboration mode had these rules: lower torque limit, speed capped at 0.6, stop and
back off on a touch. It made every task slower for a touch detection that nobody needed yet. Both modes
restricted the arm more than the demo could accept. Thus the code is removed, not switched off.
`git show 7027d7e` has both modes if they are wanted again. For work near people, the things that were there
before remain. They are the stall guard (`blocked`), the tracker that slows the arm when something moves on
the table, and `stop`. Hand-guiding stayed. It restricts nothing.

**The chat box and the microphone drive the workspace agent, not a second tool set.** There were two agents.
The first was the workspace agent (top camera, calibration, names, checks). The second was the voice agent,
which saw only the daemon's older wrist-camera tools. A person who types "pick up the ring" in the web UI
must get the same harness as the terminal gets. Thus the voice agent now runs in `workspace/` with a shell,
`AGENTS.md` and the skills. Cost: a turn is slower than a single MCP call, and a turn uses more tokens.

**There are three languages, and the language is decided for each utterance and for each sentence.** Speech
to text chooses among the configured languages only. The reason is that unrestricted detection on a two-word
command goes to languages that nobody here speaks. The voice of the reply is chosen sentence by sentence from
the words themselves. The fallback is the language that the person last used. The reason is that the voice
model must be told its language in advance. The multilingual voice model is licensed for non-commercial use
only (CC BY-NC). That is acceptable for the hackathon. The model must be replaced or licensed before anything
is sold. Cost: multilingual recognition takes about 1.4 s an utterance on this laptop. Recognition for
English alone takes 0.8 s. `ELROBOT_LANGUAGES=en` returns to English alone.

**A fixed rule judges the commands of that agent, because nobody is there to approve them.** Each command is
one Python file from the workspace, through the workspace's interpreter. Pipes, chaining, redirection and
substitution are not permitted. A command can write only inside the workspace, and not to the calibration of
the workspace. A command can read only inside the repository. The agent can run scripts that it writes
itself. That is how the agent does what no tool does (code as policy). It also means that the rule limits
accidents, not a determined misuse. Do not expose the daemon beyond this machine.

**The daemon computes the live vision of the web UI on request, and the live vision is for display only.**
The camera panel calls `GET /api/camera/{name}/vision` a few times a second while a live layer is switched
on. Nothing is computed when nobody looks. The result is cached per frame. The live vision draws object
outlines, AprilTags, the robot's frame and reach, and the arm as the kinematics place it. By design, the live
vision does not supply data to the tools. The tools keep their own measurements, which are slower and more
careful (`find_objects`, the tracker). The "Arm" layer is the quickest check that the calibration is still
correct. If the calibration is correct, the line lies on the real arm.

**The camera panel shows every camera, not only the wrist camera.** The panel opens on `top`, which sees the
whole table. The browser remembers the choice and the layers. `?camera=...&layers=...` in the address sets
them.

**The workspace, not the daemon, sends a move that was stopped as blocked one more time, slower.** In most
"blocked" stops on this arm, a joint trails a fast setpoint under load, and nothing is in the way. The same
move, slower, succeeds (seen on joints 1, 2, 4 and 6, and on `home` from rest above speed 0.5). The second
try is in `Arm.joints` (`util/arm.py`), where every tool ends. The reason is that the daemon takes joint
commands and decides nothing. A second try is a second joint command. It is in `joint_commands.jsonl` like
the first command. Cost: when something really is in the way, the arm pushes against it a second time,
slower, before the error comes back.

**The second try occurs once, and only for "blocked".** A second refusal is raised as it is. Two failures
mean that it is necessary to look, not to try a third time. A servo fault needs a person. A lower speed does
not correct a lost connection or a stop. An envelope refusal is the same at any speed. Thus none of those is
tried again. A gripper-only command is also not tried again. The second try starts from the waypoint nearest
to the place where the arm stopped, not from the start of the path.

**One number in a file sets how much slower the second try is: `blocked_retry_speed` in
`workspace/robot_config/motion.json`, 0 to 1, default 0.5.** The number is a fraction of the speed of the
first try. Thus it scales with the speed that the tool chose. The value 0 switches the second try off, which
is what a contact-seeking setup needs. The number is read when a move is blocked, not at start. Thus it can
be tuned between two commands. If the value cannot be read, there is no second try, and the error says so. A
typo in the file must not stop the arm from working. A typo must also not pass unnoticed.

**Chess, drawing and the grasp planner were added to the existing workspace, not built as a second harness.**
A separate harness was started from nothing before this repo was found. It duplicated the robot layer, and it
was never used with hardware. Instead, the new capabilities were ported to the workspace's conventions. The
prototype was discarded.

**Chess reads squares as empty / light piece / dark piece, and takes the move from the rules.**
At this resolution, a piece is a dozen pixels. Thus shapes are not recognised. The human's move is the legal
move that best explains the picture. Anything ambiguous is reported, not guessed. Cost: a non-standard
starting position must be given as a FEN.

**Chess perception models what hides what.** From an oblique camera, a tall piece covers the squares behind
it. For each candidate position, the expected appearance of every line of sight is computed from the camera's
position (`util/chessboard.py`, `expected_labels`). Without this model, a vacated square behind the king was
read as occupied.

**A person seats the pen, with the tip on the paper, before the jaws close.** That fixes the tip position
without a measurement. It is also not necessary to press the pen down to find the paper. Such a press risks a
servo overload on a 7.5 V arm.

**The UGREEN webcam above the mat replaced the KD camera across the table as `top`.** With a view from above,
much less is hidden behind objects. Cost: there are fewer pixels per millimetre on the mat than the old
camera had on its near side. Also, `calibrate_top` cannot locate a camera that looks exactly straight down.
The old camera is now `across`. Its calibration is `workspace/robot_config/camera_top.kd-backup.json`.

**`base_in_mat` was refitted: it is [0.2985, -0.1256, 4.77 deg], and it was [0.2765, -0.109, 0].** The old
value was tuned by eye for the old camera. Two independent measurements agreed. In the first, the arm set a
cube down at five known positions, and the top camera measured them (residual 3 to 8 mm after the fit). In
the second, the wrist camera looked at the mat tags from four poses. Do this again each time the robot or the
mat moves.

**All cameras stream 1280x720 (stored 853x480).** The wrist calibration from `main` was made at that size.
640x480 is a different crop of the sensor, thus the calibration is wrong there. Cost: the Station refuses
1280x720 on a camera that it held before (libuvc error -51), and it silently falls back. Thus
`scripts/cameras_ready.sh` must run before each Station start.

**The top camera has a fixed 6 ms exposure.** With auto exposure, the mat's white corner tags are overexposed
and do not decode. The fixed exposure does not change the frame rate. The 15 fps occurs because the Station
keeps every second frame (`frame-skip: 1`). That setting stayed as it was.

**`pick_verified` is a new tool, and `pick_at` is unchanged.** The `refine` option of `pick_at` projects the
centre of an outline onto the table plane. For an object with height that is seen at an angle, this
overshoots by centimetres. Its jaw mask also hides the object between the fingers. A tested tool was not
changed. Instead, the wrist checks went into a separate tool. That tool uses the object's height and
separates neighbours by colour.

**`origin/main` was merged, not cherry-picked,** to include the wrist-camera calibration with its code and history.

**python-chess (GPL-3.0) is a dependency** of the workspace and the root project, for legal moves and game state.

## Earlier (from the history)

**The robot takes joint commands only.** No poses, grasps or skills go to the daemon. IK, planning and
perception are Python in `workspace/tools/`, and the agent can read and change that Python. This is the
RoboHarness V2 idea: the workspace is the harness.

**Workspace tools run in the agent's shell, not in the daemon.** A tool is a file that the agent can edit and
run again. The daemon keeps its own guards (limits, table, self-collision, blocked joints, servo faults),
whatever the tool code does.

**The workspace is independent:** it has its own interpreter, robot model and kinematics, thus the directory can be copied whole.

**The arm runs on 7.5 V, not 12 V.** Station is built from NormaCore `6b494f77` with
`patches/norma-core-calibration-voltage.patch`. Joint ranges are calibrated with `scripts/calibrate.py --manual`, not `--auto`.

**The tick-to-radian mapping uses the true 4096 ticks per turn,** with the anchor at the centre of the calibrated arc.
It is not the proportional mapping that NormaCore's viewer uses.

**A "droop" loop trims servo sag.** The loop integrates the settling error into a per-joint bias, because the servos run a soft P-only gain.

**Motion is refused while a servo reports a fault.** An overload needs a person to hold the arm while torque is cycled.

**Angles in tool arguments carry their unit** (`yaw_deg`, `pitch_deg`). Everything else is metres and radians.

**The tracker's reaction to an intrusion is a motion heuristic, not a safety interlock.** By default, the tracker slows the arm to a quarter of its speed while something else moves on the table. Afterwards, the tracker lets the arm return to full speed. This was the operator's choice, 2026-11-22: a stop for every hand at the edge of the mat made tasks fail. `on_intrusion: "stop"` halts the arm as before. The reaction must never be switched to `ignore` automatically.
