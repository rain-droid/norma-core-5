# Changelog

This file tells what changed, when it changed, and who made the change. The newest entry is first. The reasons
are in `docs/DECISIONS.md`. `docs/FEATURES.md` tells what works today. Add an entry with each push.

## 2026-11-22

### Pictures and measurements; an unplugged second arm no longer stops the first
Modified by: Atharva Bhorpe

- `docs/RESULTS.md` is new, with `docs/images/two_arm/`. It contains the web UI and the twin, the three camera
  views, the raw depth picture and the height maps. It contains the objects that each arm could pick, the
  obstacle map, and the refusals in the tools' own words. It contains the hand-over pictures, the stacking
  pictures and the numbers. It also contains the log of every calibration call, hand-over call and stacking
  call of the afternoon.
- Fixed a fault that occurred when the SO-101's daemon ran but the arm was unplugged. The collision check read
  an empty joint list. Every joint command of the ElRobot failed with an IndexError. Now
  `second_arm.joints_now` says that the arm is missing, and the check continues without the arm.
- `docs/HANDOFF.md` records the test state: all tests pass except one push-T case.

### Stacking by the second arm tried; documentation for the two-arm work
Modified by: Atharva Bhorpe

- `workspace/tools/second_arm_stack.py`: the SO-101 picks an object off the mat and sets it on another object.
  The top camera brings the jaws of the SO-101 over each place. On the real table, the SO-101 picked the cube
  2 times in 4. Both times, it set the cube beside the base: its joints put it 2.5 to 4.5 cm from where it is
  at the table. The top camera sees too little of the gripper under its own camera to correct that reliably.
- The SO-101's `reach` goes over the objects that are in the way, like the ElRobot's carry.
- `docs/SECOND_ARM.md` is new. It tells about the start, the calibration, the tools, the shared space, what is
  proven, and the known limits. README, FEATURES, INTERFACE, PLAN and HANDOFF now show the state of the
  afternoon.
- `workspace/.gitignore`: the repository keeps the poses shown by hand (`state/two_arm/`). The repository does
  not keep the Orbbec SDK's log folder.

### Things on the table as obstacles, carrying over them, wrist cameras in the models, all of it in the twin
Modified by: Atharva Bhorpe

- `workspace/tools/scan_obstacles.py`: `state/obstacles.npz` holds the depth camera's height map (4 mm cells),
  with both arms cut out. For two minutes, the shared space (`tools/util/shared_space.py`) also compares every
  checked move with the map. No part of an arm may go into an object on the table, except around the places
  where the move starts and ends. Seen on the real table: the shared space refused a carry of the sponge
  through the fallen pink block (47 mm).
- `util/motion.py` `carry`, which `pick_at` and `place_at` use: with a recent map, the move first goes up to
  where the gripper clears the tallest object along the straight line. Then the move goes across at that
  height. The arm leans during that part, because this arm cannot point straight down at that height. Then the
  move goes to the pose. Then the same carry went through, over the block.
- Both arms carry the wrist camera on the same mount on the same parallel gripper. The wrist camera is now in
  the ElRobot's URDF (`assets/elrobot/elrobot_follower.urdf`, `workspace/robot_config/model.urdf`). The links
  and the mesh come from the SO-101's URDF, whose gripper base frame is the same. The wrist camera is also in
  both copies of the kinematics as a body part (self-collision, table clearance). It is also in the shared
  space for both arms. The model puts the ElRobot's lens within 2 cm of where its camera calibration has the
  camera.
- Web UI: the ElRobot's twin shows its camera. The web UI draws the objects from the scan as columns coloured
  by height (new `obstacles` event), with their heights as labels. The web UI draws the SO-101's planned path,
  and any path that the shared space refused, like the ElRobot's own planned path (`plan` events from the
  workspace, both through `POST /api/event`).
- Start the SO-101's daemon and the main stack from your own terminal (`scripts/second_arm_up.sh`,
  `pixi run up`). If you start them from an agent session, they are ended after two hours.

### One space for both arms, with collision checks; hand-over from poses shown by hand
Modified by: Atharva Bhorpe

- `workspace/tools/util/shared_space.py`: this file has both arms in the ElRobot's base frame, each arm as
  five capsules. It reads the SO-101 from `robot_config/second_arm.urdf`. Its forward kinematics agree with its
  daemon to 1.8 mm over a recorded track. The SO-101 has its own inverse kinematics there. `between` gives the
  smallest gap. `path_clear` checks a joint path of one arm against the other arm, which stands still.
- The collision check refuses a command if the command brings any part within 2 cm of the other arm. This
  applies to every joint command of the ElRobot (`util/arm.py`, real arm only). It also applies to the SO-101's
  `home`, `move_to` and `move_joints` through `tools/second_arm.py`. A hand-over may let the grippers (and a
  gripper and the other arm's wrist) meet. `tools/arms_clearance.py` reads the gap. Tried on the real arms:
  the check refused a move of the ElRobot at the SO-101's gripper, and a full pick and hand-over cycle went
  through. Not checked: the SO-101 daemon's own tools (pick, place, gestures), objects on the table, a held
  object.
- `tools/record_poses.py`, `tools/replay_poses.py`: a person moves both limp arms through poses. The tools keep
  the poses in `state/two_arm/` and play them back. `hand_over` uses such a recording when there is one
  ("poses"). The arms go to where they were when the SO-101's jaws closed. The tool corrects the ElRobot onto
  the shown pose. If the jaws close on nothing, the ElRobot offers the object 1 cm further, up to 3 cm.
- `tools/teach_hand_over.py`: one hand-over, adjusted by a push on the ElRobot ("pose").
- Hand-over of the 3 cm cube, unattended runs that reached the cube: 0 of 3 before the SO-101's height was
  corrected from its own joints, 2 of 2 computed after, 1 of 2 from one adjusted pose, 4 of 6 from shown poses.

### Both arms in one digital twin
Modified by: Atharva Bhorpe

- `src/elrobot/second_arm.py`: the daemon reads the SO-101 daemon's state ten times a second. The daemon adds
  that state to its own state message and to `GET /api/state` as `second_arm`. `second_arm` has the joints, the
  gripper, and `T`. `T` is the SO-101's base in the ElRobot's frame, from
  `workspace/robot_config/second_arm.json`. `second_arm` is `null` while the SO-101's daemon does not answer.
  `second_arm` is absent when `second_arm_url` in the config is empty. `GET /api/second_arm/assets/...` passes
  the SO-101's URDF and meshes through.
- Web UI: the 3D view loads the second arm the first time the state carries a second arm that has been placed.
  The 3D view poses the second arm from its joint angles (dark parts, orange jaws). Seen on the real setup with
  both arms at home.
- The twin stands the SO-101 upright on the table at the fitted position and turn. The fit itself has a tilt of
  8.5 degrees. That tilt is the error of the fit, not of the arm.
- `tests/test_second_arm.py`.

### Second arm: an SO-101 next to the ElRobot, and a hand-over between them
Modified by: Atharva Bhorpe

- The SO-101 with the parallel gripper runs on its own daemon. That daemon is the `autopilot` branch (its
  SO-101 profile), checked out beside this repository. `scripts/second_arm_up.sh` starts the daemon on port
  8770 against the same Station. Nothing of that branch is merged here. `config.yaml` now names the ElRobot's
  servo bus (`bus_serial`), because the Station has two.
- Workspace: `tools/util/second_arm.py` (client and rigid fit), `tools/second_arm.py` (runs the SO-101's tools;
  the positions can be in the ElRobot's frame), `tools/calibrate_second_arm.py` (the SO-101 carries a coloured
  block through places that the depth camera measures; the tool writes `robot_config/second_arm.json`),
  `tools/hand_over.py`.
- `tools/util/depth.py`: `find_in_air` finds an object of one colour that is held above the mat, in base
  coordinates.
- On the real arms: the calibration fitted six places to 4 to 8 mm. The hand-over of the 3 cm blue cube from
  the ElRobot's fingertips to the SO-101 succeeded twice. Each time, it succeeded after one height correction
  made by hand. (The SO-101 sags 8 to 17 mm when it reaches out level.) That correction is now in the tool.
  The tool reads the correction from the SO-101's own joints. The correction has not yet had a run from start
  to finish. The arms could not hand over the pink foam block: the ElRobot's wedge fingers cover it.

### Depth camera across the table (Orbbec DaBai DCW2)
Modified by: Atharva Bhorpe

- An Orbbec DaBai DCW2 replaced the KD camera across the table. Its colour camera (USB `2bc5:0561`) is camera
  `across` through the Station (`config.yaml`). `scripts/cameras_ready.sh` hands it back to the kernel like the
  other cameras. Only one KD camera is left, so the match for `wrist` uses vendor:product alone.
- `workspace/tools/util/depth.py` reads its depth sensor (USB `2bc5:06a0`) with Orbbec's v1 SDK.
  `scripts/orbbec_setup.sh` builds the SDK into `bin/orbbec/` (the reason for a build is in
  `docs/DECISIONS.md`). A udev rule for vendor `2bc5` lets the user open both devices.
- New tools: `calibrate_depth` and `depth_scan`. `calibrate_depth` places the sensor in the base frame from the
  mat's plane and two corner tags. It writes `robot_config/camera_depth.json`. `depth_scan` gives the heights
  of everything on the mat, the heights at given spots, and a height map. On the real table: the tags fitted to
  1.0 mm, the mat was flat to 1.6 mm, and a 30 mm cube read 29 mm.
- `workspace/tests/test_depth.py`: tests the fit and the height map on a computed scene.
- The depth camera needs two machine settings. Both are in `scripts/orbbec_setup.sh`. The first is the udev
  rule. The second raises the kernel's USB transfer memory from 16 to 256 MB. At 16, the depth stream stops
  after three pictures once the Station has all three cameras at 1280x720.
- `scripts/cameras_ready.sh` now also streams a few 1280x720 pictures from the KD and Orbbec colour cameras
  through the kernel driver. Without that, the replugged wrist camera started at 640x480 (Station error -51).
  Its calibration does not fit 640x480.

### Safe mode and collaboration mode removed
Modified by: Atharva Bhorpe

- Both modes restricted the arm too much (the reasons are in `docs/DECISIONS.md`). These items are removed, not
  switched off:
  `workspace/tools/util/safe.py` and its hooks in `util/arm.py`, `util/planning.py` and `home`; the `safe_mode`
  and `carry_height` keys of `robot_config/motion.json`; the daemon's touch reflex, `POST /api/collab`, the
  `collab_*` settings, the `contact` error kind and `ELROBOT_TRACE`; the web UI's "Collaboration" switch;
  the `collaboration` tool; their sections in `workspace/AGENTS.md` and the recover skill; their tests.
- The arm leans again and reaches about 36 cm. `home` goes to the usual ready pose. Motions run at full speed.
- Hand-guiding (`hand_guide`, `replay_demo`, `POST /api/guide`) stays. Its tests are now `tests/test_hand_guide.py`.
- The two entries below describe what was removed. `git show 7027d7e` has the code.

### Hand-guiding: lead the arm by hand, replay what was shown
Modified by: Atharva Bhorpe

- `POST /api/guide` (`Robot.guide` in `src/elrobot/robot.py`): the arm holds itself up. When a hand pushes a
  joint off its command by more than `guide_deadband`, the command of that joint follows the hand. When the
  hand lets go, the arm stays. No torque is switched off, so nothing drops. The joint limits and the table
  still stop the arm.
- Workspace tools: `hand_guide` and `replay_demo`. `hand_guide`: lead the arm, optionally set the jaws after
  that, save the stretch under a name in `state/demos/`. `replay_demo`: the arm repeats the stretch alone, or
  backwards. The one-shot example is a pick and place taught by hand in three calls. `AGENTS.md` has a
  "Shown by hand" section.
- The simulator tests are in `tests/test_collab.py` (`SimLink.shove` holds a simulated joint off its command).
  The real arm held still for eight seconds untouched (no creep). Nobody has led the real arm by hand yet, so
  the deadband and the gain (`guide_*` in `config.yaml`) are untuned.

### Collaboration mode: softer, slower, backs off when touched
Modified by: Atharva Bhorpe

- The mode has a switch in the web UI's header ("Collaboration on/off"), `POST /api/collab`, and
  `collab: true` in `config.yaml`. It also has a workspace tool (`tools/collaboration.py`), so the agent can
  switch the mode on when it is asked to work with someone.
- While the mode is on, the torque limit of the arm servos drops (`collab_torque_limit`, 650 of 1000 against
  800). Motions run at no more than `collab_speed` (0.6) of their planned speed. A reflex in the daemon's
  control loop (`Reflex` in `src/elrobot/robot.py`) compares each joint's load and lag with its own level over
  the last quarter second. A step away from that level for four control ticks is a touch. Then the arm stops,
  retreats for half a second along the path it came by, and holds. The command ends with `kind: "contact"`.
  The workspace does not retry the command.
- The mode is not impedance control and not a safety function. These servos take position commands, so a
  person feels the arm before it yields. Guarded touch moves (grasp, pen down) keep their own contact
  detection.
- The simulator tests are in `tests/test_collab.py` (the simulated servos take an injected push,
  `SimLink.push`).
- The thresholds come from the real arm. `ELROBOT_TRACE=file` makes the daemon write every control tick of
  every ordinary move (command, measurement, load). Each move logs its largest lasting step in load and lag.
  Twelve free moves gave steps of up to 141 load units and 0.05 rad at this mode's speed. The thresholds are
  220 and 0.08 rad. Then six free moves with the mode on ran without a false stop (peaks 142 and 0.055).
  Not yet done: a person pushes the real arm.
- Safe mode ran on the real arm for the first time during this: `home` and 24 `move_tcp` moves, straight down
  within a degree, 0.4 to 4 mm from the target.
- `config.yaml`: the ids of the two KD cameras after a replug (wrist `11391:19040:5:35`, across `…:5:36`).

### Safe mode: straight down, straight lines, or not at all
Modified by: Atharva Bhorpe

- `workspace/robot_config/motion.json` has `"safe_mode": true` (`ELROBOT_SAFE_MODE=0` or `1` overrides it;
  `workspace/tools/util/safe.py`). While the mode is on, the gripper only points straight down, and the jaws
  turn to suit the object. Every move between two places is three straight lines: up to a carry height,
  across, down. `home` goes to a straight-down ready pose.
- Safe mode refuses what the arm cannot do that way, before the arm moves. Straight down, this arm reaches
  about 16 cm either side of the base and 12 to 24 cm in front of it, no higher than about 7 cm. An object
  further out is "too far". `describe_objects` marks that object `reachable: false`. The agent is told to ask
  for the object to be moved.
- Carried objects travel about 4 cm above the table (`carry_height` 0.065 less the grasp height).
- The simulator tests are in `tests/test_safe_mode.py`. The other simulator tests run with safe mode off. Not
  yet run on the real arm.

### The web UI's chat and voice drive the workspace agent; speech works on Linux
Modified by: Atharva Bhorpe

- The chat box and the microphone now connect to an agent that runs in `workspace/`. That agent uses the
  tools, the skills and the world model of `workspace/`. This is the same harness as a terminal session started
  there. Before, they reached a separate agent that only had the daemon's older wrist-camera tools.
  `ELROBOT_CHAT_AGENT=mcp` brings that separate agent back.
- Nobody approves that agent's commands, so a fixed rule approves them (`workspace_permission` in
  `src/elrobot/voice/agent.py`). The agent may run one Python file from the workspace per command. The agent
  may write inside the workspace except `robot_config/`. The agent may read inside the repository. The tests
  are in `tests/test_chat_agent.py`.
- Speech on Linux: `faster-whisper` does speech to text where MLX does not exist. Text to speech is Gradium's
  Phonon (`ptts`, the 31M English model, Apache 2.0, voice "Easton"). Kokoro is the fallback behind Phonon.
  `ELROBOT_PHONON_VOICE` picks another of Phonon's hundred or so voices. `ELROBOT_TTS_PROVIDER=kokoro` goes
  back to Kokoro. The `voice` extra installs the right one for each platform. The pixi environment now
  includes the `voice` extra.
- Speech to text off Apple silicon is now NVIDIA Parakeet TDT 0.6B v3 (`onnx-asr`, CC BY 4.0). Whisper `small`
  is behind it (`ELROBOT_STT_BACKEND=whisper`). Results on nine synthesised sentences in the three languages:
  Parakeet took 300 ms an utterance. It had 11% word errors clean and at 10 dB noise, and 33% at 3 dB. Whisper
  `small` took 1250 ms and had 15%, 19%, 22%. You cannot tell Parakeet which languages to expect. In the
  heaviest noise, Parakeet identified one French sentence as English. Synthetic voices, nine sentences: this
  is an indication, not a benchmark.
- The languages are English, French and German, in voice and in text (`ELROBOT_LANGUAGES`, default
  `en,fr,de`). Speech to text picks the most likely of those languages for each utterance (Whisper `small`).
  The agent answers in the language in which it was addressed. The voice of that language speaks each sentence
  of the reply (Easton, Baptiste, Mirko), from Phonon's 40M multilingual model. That model is CC BY-NC and
  gated. The machine must be logged in to a Hugging Face account that accepted the licence of the model. If
  not, the English model takes its place.
- `pixi run up` also starts the agent. `pixi run up-quiet` is the old behaviour (`--no-voice`). With the old
  behaviour, the chat box answers "Method Not Allowed".

### Objects by name, a world model, and skills the agent writes
Modified by: Atharva Bhorpe

- `describe_objects` looks with the top camera and returns a picture sheet with one cut-out per object.
  `name_objects` records what each object is. `state/world.json` keeps the names. A name stays with an object
  when the object moves.
- `world_model` reads what is on the table, where it is, what it stands on, and what was done with it.
- `grasp_object` and `put_object` work by name. A ring is recognised because the table shows through its hole.
  The arm grips a ring by the rim. On the real table, the ring, the pink box and the blue cube were found and
  named. The arm has not yet grasped by name.
- `create_skill` writes `skills/NAME/SKILL.md` from the agent's steps and appends the tool calls that the agent
  actually made.
- `workspace/AGENTS.md`: "Objects have names" and "Leave a skill behind".

### Tracker concludes piles it did not see being built
Modified by: Arian Malekzadeh

- Two objects, one on the other, are one outline from above. Until now, the tracker only knew a pile when the
  arm had built it. Now there is a new case: a track goes missing and does not appear elsewhere, while another
  outline has changed since then. The change can be in colour, area or height, or that outline stands in the
  place of the missing track. The tracker reports that the missing track lies on that outline: `on`, with
  `on_source: "inferred"` and `on_evidence`. A pile that the arm built has `on_source: "placed"`.
- The lower object keeps its name. (Before, it was renamed to the pile's colour after a second.) The tracker no
  longer forgets the upper object as `missing`. When the upper object is taken off and put elsewhere, the
  tracker finds it again and clears `on`.
- `docs/OVERLAPPING_OBJECTS.md`: the plan for this and for the second half. The second half is a close look
  with the wrist camera when the tracker is unsure (not built yet).
- The test is in `tests/test_workspace.py`, simulator only. Not run on the real arm.

### Web UI: camera switcher and live vision layers
Modified by: Atharva Bhorpe

- The camera panel has a tab for every camera that the daemon reports (`top`, `wrist`, `across`). A tab has a
  dot when its camera delivers frames. The panel opens on `top`.
- You can switch on five layers over the picture: **Objects** (live segmentation outlines), **Tags**
  (AprilTags), **Frame** (the robot's axes and reach, top camera), **Arm** (the arm as the kinematics place it,
  top camera), **Tools** (what the tools and the tracker last reported).
- New daemon endpoint: `GET /api/camera/{name}/vision` (`src/elrobot/vision_live.py`). The daemon computes it
  only while a browser asks. The test is in `tests/test_vision_live.py`.
- `?camera=top&layers=objects,frame,arm` in the address opens the panel in that state.
- Fixed: the enlarged camera view collapsed to a thin bar.
- `config.yaml`: the ids of the two KD cameras after the replug (wrist `5:17`, across `5:15`).

### Tracker slows the arm instead of stopping it
Modified by: Atharva Bhorpe

- While something else moves on the table, the tracker now slows the arm to a quarter of its speed. When the
  table is quiet, the tracker lets the arm return to full speed. `track_objects` takes `on_intrusion`
  (`"slow"` by default, `"stop"`, `"ignore"`) in place of `stop_arm_on_intrusion`.
- New: the daemon endpoint `POST /api/pace` and `state.pace`. `Robot.set_pace` scales a motion that is in
  progress.
- `pick_object`, `place_object`, `find_objects`, the pick-and-place and recover skills and `AGENTS.md` follow
  this change.

### Blocked moves retry once, slower
Modified by: Arian Malekzadeh

- When the robot reports a move as blocked, the move is tried once more at a lower speed.
  `workspace/robot_config/motion.json` tells how much lower. The tests are in `workspace/tests/`.

### Docs and plan
Modified by: Atharva Bhorpe

- `docs/FEATURES.md`, `docs/DECISIONS.md`, `docs/INTERFACE.md`, `docs/PLAN.md`, this changelog.
  `docs/HANDOFF.md` is now up to date. `CLAUDE.md` points to all of them.

### `pick_verified`: the wrist camera checks a pick before and after
Modified by: Atharva Bhorpe

- From the hover point, the tool measures how far the object is from the aim and lines up on the object. After
  the lift, the tool checks that the object is between the fingers. Run on the real arm with the blue cube.

### UGREEN webcam as the top camera; 1280x720 streams; calibration
Modified by: Atharva Bhorpe

- The UGREEN above the mat is `top`. The KD camera across the table is `across`.
- `base_in_mat` was refitted (it was 2.2 cm and 4.8 degrees off). The top camera was recalibrated from the mat
  tags.
- All cameras stream 1280x720, so the frames match the wrist calibration. `scripts/cameras_ready.sh` prepares
  them.

### Wrist-camera calibration
Modified by: Arian Malekzadeh (merged from `main`)

- Lens and hand-eye calibration of the wrist camera, `data/camera_wrist.json`, `scripts/calibrate_camera.py`.

### Chess, drawing and a grasp planner
Modified by: Atharva Bhorpe

- `chess_setup`, `chess_see`, `chess_move`, `chess_record`; `draw_plan`, `draw_pen`, `draw_run`; `plan_grasp`;
  skills `play-chess` and `draw`. Simulator only.
