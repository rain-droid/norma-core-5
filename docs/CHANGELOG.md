# Changelog

What changed, when, and who made the change. Newest first. The reasons are in `docs/DECISIONS.md`; what works
today is in `docs/FEATURES.md`. Add an entry with each push.

## 2026-11-22

### Pictures and measurements; an unplugged second arm no longer stops the first
Modified by: Atharva Bhorpe

- `docs/RESULTS.md` (new) with `docs/images/two_arm/`: the web UI and twin, the three camera views, the raw
  depth picture, height maps, which things each arm could pick, the obstacle map, the refusals in the tools'
  own words, hand-over and stacking pictures, the numbers, and the log of every calibration, hand-over and
  stacking call of the afternoon.
- Fixed: with the SO-101's daemon running but the arm unplugged, the collision check read an empty joint list
  and every joint command of the ElRobot failed with an IndexError. `second_arm.joints_now` now says the arm is
  missing and the check goes on without it.
- Test state recorded in `docs/HANDOFF.md`: all pass except one push-T case.

### Stacking by the second arm tried; documentation for the two-arm work
Modified by: Atharva Bhorpe

- `workspace/tools/second_arm_stack.py`: the SO-101 picks a thing off the mat and sets it on another, its
  jaws brought over each place by the top camera. On the real table it picked the cube 2 times in 4 and set it
  beside the base both times: its joints put it 2.5 to 4.5 cm from where it is at the table, and the top
  camera sees too little of the gripper under its own camera to correct that reliably.
- The SO-101's `reach` goes over what stands on the way, like the ElRobot's carry.
- `docs/SECOND_ARM.md` (new): start, calibration, tools, the shared space, what is proven, known limits.
  README, FEATURES, INTERFACE, PLAN and HANDOFF brought up to the state of the afternoon.
- `workspace/.gitignore`: the poses shown by hand (`state/two_arm/`) are kept in the repository; the Orbbec
  SDK's log folder is not.

### Things on the table as obstacles, carrying over them, wrist cameras in the models, all of it in the twin
Modified by: Atharva Bhorpe

- `workspace/tools/scan_obstacles.py`: the depth camera's height map (4 mm cells) is kept in
  `state/obstacles.npz` with both arms cut out. For two minutes the shared space
  (`tools/util/shared_space.py`) also walks every checked move against it: no part of an arm may dip into
  something on the table, except around where the move starts and ends. Seen on the real table: a carry of the
  sponge through the fallen pink block (47 mm) was refused.
- `util/motion.py` `carry`, used by `pick_at` and `place_at`: with a recent map, the move goes up to where the
  gripper clears the tallest thing along the straight line, across at that height (leaning, since this arm
  cannot point straight down up there), then to the pose. The same carry then went through, over the block.
- Both arms carry the wrist camera on the same mount on the same parallel gripper. It is now in the ElRobot's
  URDF (`assets/elrobot/elrobot_follower.urdf`, `workspace/robot_config/model.urdf`; the links and the mesh
  come from the SO-101's URDF, whose gripper base frame is the same), in both copies of the kinematics as a
  body part (self-collision, table clearance) and in the shared space for both arms. The model puts the
  ElRobot's lens within 2 cm of where its camera calibration has the camera.
- Web UI: the ElRobot's twin shows its camera; the scan's things are drawn as columns coloured by height
  (new `obstacles` event) with their heights as labels; the SO-101's planned way, and any way the shared space
  refused, is drawn like the ElRobot's own planned path (`plan` events from the workspace, both through
  `POST /api/event`).
- The SO-101's daemon and the main stack must be started from a terminal of one's own
  (`scripts/second_arm_up.sh`, `pixi run up`): started from an agent session they are ended after two hours.

### One space for both arms, with collision checks; hand-over from poses shown by hand
Modified by: Atharva Bhorpe

- `workspace/tools/util/shared_space.py`: both arms in the ElRobot's base frame, each as five capsules; the
  SO-101 read from `robot_config/second_arm.urdf` (its forward kinematics agree with its daemon to 1.8 mm over
  a recorded track) with its own inverse kinematics there. `between` gives the smallest gap, `path_clear` walks
  a joint path of one arm against the other standing still.
- Every joint command of the ElRobot (`util/arm.py`, real arm only) and the SO-101's `home`, `move_to` and
  `move_joints` through `tools/second_arm.py` are refused if any part would come within 2 cm of the other arm;
  a hand-over may let the grippers (and a gripper and the other's wrist) meet. `tools/arms_clearance.py` reads
  the gap. Tried on the real arms: a move of the ElRobot at the SO-101's gripper was refused, and a full
  pick and hand-over cycle went through. Not checked: the SO-101 daemon's own tools (pick, place, gestures),
  things on the table, a held thing.
- `tools/record_poses.py`, `tools/replay_poses.py`: a person moves both limp arms through poses; they are kept
  in `state/two_arm/` and played back. `hand_over` uses such a recording when there is one ("poses"): the
  arms go to where they were when the SO-101's jaws closed, the ElRobot is corrected onto the shown pose, and
  if the jaws close on nothing the ElRobot offers the thing 1 cm further, up to 3 cm.
- `tools/teach_hand_over.py`: one hand-over adjusted by pushing the ElRobot ("pose").
- Hand-over of the 3 cm cube, unattended runs that reached the cube: 0 of 3 before the SO-101's height was
  corrected from its own joints, 2 of 2 computed after, 1 of 2 from one adjusted pose, 4 of 6 from shown poses.

### Both arms in one digital twin
Modified by: Atharva Bhorpe

- `src/elrobot/second_arm.py`: the daemon reads the SO-101 daemon's state ten times a second and adds it to its
  own state message and `GET /api/state` as `second_arm` (joints, gripper, and `T`, its base in the ElRobot's
  frame from `workspace/robot_config/second_arm.json`); `null` while that daemon is not answering, absent when
  `second_arm_url` in the config is empty. `GET /api/second_arm/assets/...` passes its URDF and meshes through.
- Web UI: the 3D view loads the second arm the first time the state carries one that has been placed, and poses
  it from its joint angles (dark parts, orange jaws). Seen on the real setup with both arms at home.
- The twin stands the SO-101 upright on the table at the fitted position and turn; the fit itself is tilted
  8.5 degrees, which is the fit's error, not the arm's.
- `tests/test_second_arm.py`.

### Second arm: an SO-101 next to the ElRobot, and a hand-over between them
Modified by: Atharva Bhorpe

- The SO-101 with the parallel gripper runs on its own daemon, the `autopilot` branch (its SO-101 profile),
  checked out beside this repository and started by `scripts/second_arm_up.sh` on port 8770 against the same
  Station. Nothing of that branch is merged here. `config.yaml` now names the ElRobot's servo bus
  (`bus_serial`), since the Station has two.
- Workspace: `tools/util/second_arm.py` (client and rigid fit), `tools/second_arm.py` (runs the SO-101's tools,
  positions optionally in the ElRobot's frame), `tools/calibrate_second_arm.py` (the SO-101 carries a coloured
  block through places the depth camera measures; writes `robot_config/second_arm.json`), `tools/hand_over.py`.
- `tools/util/depth.py`: `find_in_air` finds a thing of one colour held above the mat, in base coordinates.
- On the real arms: calibration fitted six places to 4 to 8 mm. The hand-over of the 3 cm blue cube from the
  ElRobot's fingertips to the SO-101 succeeded twice, each time after one height correction made by hand
  (the SO-101 sags 8 to 17 mm reaching out level); that correction is now in the tool, read from the SO-101's
  own joints, and has not yet had a run from start to finish. The pink foam block could not be handed over:
  the ElRobot's wedge fingers cover it.

### Depth camera across the table (Orbbec DaBai DCW2)
Modified by: Atharva Bhorpe

- The KD camera across the table was replaced by an Orbbec DaBai DCW2. Its colour camera (USB `2bc5:0561`) is
  camera `across` through the Station (`config.yaml`; `scripts/cameras_ready.sh` hands it back to the kernel
  like the others). Only one KD camera is left, so `wrist` is matched by vendor:product alone.
- Its depth sensor (USB `2bc5:06a0`) is read by `workspace/tools/util/depth.py` with Orbbec's v1 SDK, built
  into `bin/orbbec/` by `scripts/orbbec_setup.sh` (why a build: `docs/DECISIONS.md`). A udev rule for vendor
  `2bc5` lets the user open both devices.
- New tools: `calibrate_depth` (places the sensor in the base frame from the mat's plane and two corner tags;
  writes `robot_config/camera_depth.json`) and `depth_scan` (heights of everything on the mat, heights at given
  spots, a height map). On the real table: tags fitted to 1.0 mm, mat flat to 1.6 mm, a 30 mm cube read 29 mm.
- `workspace/tests/test_depth.py`: the fit and the height map on a computed scene.
- Two machine settings the depth camera needs, both in `scripts/orbbec_setup.sh`: the udev rule, and the
  kernel's USB transfer memory raised from 16 to 256 MB (at 16 the depth stream stops after three pictures
  once the Station has all three cameras at 1280x720).
- `scripts/cameras_ready.sh` now also streams a few 1280x720 pictures from the KD and Orbbec colour cameras
  through the kernel driver. Without that the replugged wrist camera came up at 640x480 (Station error -51),
  which its calibration does not fit.

### Safe mode and collaboration mode removed
Modified by: Atharva Bhorpe

- Both restricted the arm too much (reasons in `docs/DECISIONS.md`). Removed, not switched off:
  `workspace/tools/util/safe.py` and its hooks in `util/arm.py`, `util/planning.py` and `home`; the `safe_mode`
  and `carry_height` keys of `robot_config/motion.json`; the daemon's touch reflex, `POST /api/collab`, the
  `collab_*` settings, the `contact` error kind and `ELROBOT_TRACE`; the web UI's "Collaboration" switch;
  the `collaboration` tool; their sections in `workspace/AGENTS.md` and the recover skill; their tests.
- The arm leans again and reaches about 36 cm; `home` goes to the usual ready pose; motions run at full speed.
- Hand-guiding (`hand_guide`, `replay_demo`, `POST /api/guide`) stays. Its tests are now `tests/test_hand_guide.py`.
- The two entries below describe what was removed; `git show 7027d7e` has the code.

### Hand-guiding: lead the arm by hand, replay what was shown
Modified by: Atharva Bhorpe

- `POST /api/guide` (`Robot.guide` in `src/elrobot/robot.py`): the arm holds itself up, and a joint that a hand
  pushes off its command by more than `guide_deadband` has the command follow the hand. Let go, it stays. No
  torque is switched off, so nothing drops; joint limits and the table still stop it.
- Workspace tools `hand_guide` (lead it, optionally set the jaws after, save the stretch under a name in
  `state/demos/`) and `replay_demo` (the arm repeats it alone, or backwards). A pick and place taught by hand
  in three calls is the one-shot example. `AGENTS.md` has a "Shown by hand" section.
- Simulator tests in `tests/test_collab.py` (`SimLink.shove` holds a simulated joint off its command).
  On the real arm it held still for eight seconds untouched (no creep); nobody has led it by hand yet, so
  deadband and gain (`guide_*` in `config.yaml`) are untuned.

### Collaboration mode: softer, slower, backs off when touched
Modified by: Atharva Bhorpe

- A switch in the web UI's header ("Collaboration on/off"), `POST /api/collab`, `collab: true` in `config.yaml`,
  and a workspace tool (`tools/collaboration.py`) so the agent can switch it on when asked to work with someone.
- While it is on: the arm servos' torque limit drops (`collab_torque_limit`, 650 of 1000 against 800), motions
  run at no more than `collab_speed` (0.6) of their planned speed, and a reflex in the daemon's control loop
  (`Reflex` in `src/elrobot/robot.py`) compares each joint's load and lag with its own level over the last
  quarter second. A step away from it for four control ticks is a touch: the arm stops, retreats half a second
  along the path it came by, holds, and the command ends with `kind: "contact"`. The workspace does not retry it.
- It is not impedance control and not a safety function: these servos take position commands, so the arm is
  felt before it yields. Guarded touch moves (grasp, pen down) keep their own contact detection.
- Simulator tests in `tests/test_collab.py` (the simulated servos take an injected push, `SimLink.push`).
- Thresholds come from the real arm: `ELROBOT_TRACE=file` makes the daemon write every control tick of every
  ordinary move (command, measurement, load), and each move logs its largest lasting step in load and lag.
  Twelve free moves gave steps of up to 141 load units and 0.05 rad at this mode's speed; the thresholds are
  220 and 0.08 rad. Six free moves with the mode on then ran without a false stop (peaks 142 and 0.055).
  Not yet done: a person pushing the real arm.
- Safe mode ran on the real arm for the first time during this: `home` and 24 `move_tcp` moves, straight down
  within a degree, 0.4 to 4 mm from the target.
- `config.yaml`: the two KD cameras' ids after a replug (wrist `11391:19040:5:35`, across `…:5:36`).

### Safe mode: straight down, straight lines, or not at all
Modified by: Atharva Bhorpe

- `workspace/robot_config/motion.json` has `"safe_mode": true` (`ELROBOT_SAFE_MODE=0` or `1` overrides it;
  `workspace/tools/util/safe.py`). While it is on, the gripper only points straight down, with the jaws turned
  to suit the object, and every move between two places is three straight lines: up to a carry height, across,
  down. `home` goes to a straight-down ready pose.
- What cannot be done that way is refused before the arm moves. Straight down, this arm reaches about 16 cm
  either side of the base and 12 to 24 cm in front of it, no higher than about 7 cm: an object further out is
  "too far", `describe_objects` marks it `reachable: false`, and the agent is told to ask for it to be moved.
- Carried objects travel about 4 cm above the table (`carry_height` 0.065 less the grasp height).
- Simulator tests in `tests/test_safe_mode.py`; the other simulator tests run with it off. Not yet run on the real arm.

### The web UI's chat and voice drive the workspace agent; speech works on Linux
Modified by: Atharva Bhorpe

- The chat box and the microphone now talk to an agent that runs in `workspace/` and uses its tools, skills and
  world model: the same harness as a terminal session started there. Before, they reached a separate agent that
  only had the daemon's older wrist-camera tools. `ELROBOT_CHAT_AGENT=mcp` brings that one back.
- Nobody approves that agent's commands, so a fixed rule does (`workspace_permission` in
  `src/elrobot/voice/agent.py`): it may run one Python file from the workspace per command, write inside the
  workspace except `robot_config/`, and read inside the repository. Tests in `tests/test_chat_agent.py`.
- Speech on Linux: `faster-whisper` for speech to text where MLX does not exist. Text to speech is Gradium's
  Phonon (`ptts`, the 31M English model, Apache 2.0, voice "Easton"), with Kokoro behind it as the fallback;
  `ELROBOT_PHONON_VOICE` picks another of its hundred or so voices, `ELROBOT_TTS_PROVIDER=kokoro` goes back.
  The `voice` extra installs the right one per platform, and the pixi environment now includes it.
- Speech to text off Apple silicon is now NVIDIA Parakeet TDT 0.6B v3 (`onnx-asr`, CC BY 4.0), with Whisper
  `small` behind it (`ELROBOT_STT_BACKEND=whisper`). On nine synthesised sentences in the three languages:
  Parakeet 300 ms an utterance and 11% word errors clean and at 10 dB noise, 33% at 3 dB; Whisper `small`
  1250 ms and 15%, 19%, 22%. Parakeet cannot be told which languages to expect, and in the heaviest noise it
  took one French sentence for English. Synthetic voices, nine sentences: an indication, not a benchmark.
- English, French and German, in voice and in text (`ELROBOT_LANGUAGES`, default `en,fr,de`). Speech to text
  picks the most likely of those languages for each utterance (Whisper `small`); the agent answers in the
  language it was addressed in; each sentence of the reply is voiced by that language's voice (Easton, Baptiste,
  Mirko) from Phonon's 40M multilingual model. That model is CC BY-NC and gated: the machine has to be logged in
  to a Hugging Face account that accepted its licence, otherwise the English model stands in.
- `pixi run up` starts the agent as well; `pixi run up-quiet` is the old behaviour (`--no-voice`), under which
  the chat box answers "Method Not Allowed".

### Objects by name, a world model, and skills the agent writes
Modified by: Atharva Bhorpe

- `describe_objects` looks with the top camera and returns a picture sheet, one cut-out per object;
  `name_objects` records what each is. Names are kept in `state/world.json` and stay with an object when it moves.
- `world_model` reads what is on the table, where, what it stands on and what was done with it.
- `grasp_object` and `put_object` work by name. A ring is recognised by the table showing through its hole and is
  gripped by the rim. On the real table the ring, the pink box and the blue cube were found and named; the arm
  has not yet grasped by name.
- `create_skill` writes `skills/NAME/SKILL.md` from the agent's steps and appends the tool calls it actually made.
- `workspace/AGENTS.md`: "Objects have names" and "Leave a skill behind".

### Tracker concludes piles it did not see being built
Modified by: Arian Malekzadeh

- Two objects on each other are one outline from above. Until now the tracker only knew a pile when the arm
  had built it. Now a track that goes missing without turning up elsewhere, while another outline has changed
  since (colour, area, height, or it stands in the missing one's place), is reported as lying on that one:
  `on`, with `on_source: "inferred"` and `on_evidence`. A pile the arm built has `on_source: "placed"`.
- The lower object keeps its name (it used to be renamed to the pile's colour after a second), and the upper
  one is no longer forgotten as `missing`. Taken off and put elsewhere, it is found again and `on` is cleared.
- `docs/OVERLAPPING_OBJECTS.md`: the plan for this and for the second half, a close look with the wrist camera
  when the tracker is unsure (not built yet).
- Test in `tests/test_workspace.py`, simulator only. Not run on the real arm.

### Web UI: camera switcher and live vision layers
Modified by: Atharva Bhorpe

- The camera panel has a tab for every camera the daemon reports (`top`, `wrist`, `across`), with a dot on the
  ones delivering frames. It opens on `top`.
- Five layers can be switched on over the picture: **Objects** (live segmentation outlines), **Tags**
  (AprilTags), **Frame** (the robot's axes and reach, top camera), **Arm** (the arm as the kinematics place it,
  top camera), **Tools** (what the tools and the tracker last reported).
- New daemon endpoint `GET /api/camera/{name}/vision` (`src/elrobot/vision_live.py`), computed only while a
  browser asks; test in `tests/test_vision_live.py`.
- `?camera=top&layers=objects,frame,arm` in the address opens the panel in that state.
- Fixed: the enlarged camera view collapsed to a thin bar.
- `config.yaml`: the two KD cameras' ids after the replug (wrist `5:17`, across `5:15`).

### Tracker slows the arm instead of stopping it
Modified by: Atharva Bhorpe

- While something else moves on the table, the tracker now slows the arm to a quarter of its speed and lets it
  return to full speed when the table is quiet. `track_objects` takes `on_intrusion` (`"slow"` by default,
  `"stop"`, `"ignore"`) in place of `stop_arm_on_intrusion`.
- New daemon endpoint `POST /api/pace` and `state.pace`; `Robot.set_pace` scales a motion under way.
- `pick_object`, `place_object`, `find_objects`, the pick-and-place and recover skills and `AGENTS.md` follow.

### Blocked moves retry once, slower
Modified by: Arian Malekzadeh

- A move the robot reports as blocked is tried once more at a lower speed; how much lower is in
  `workspace/robot_config/motion.json`. Tests in `workspace/tests/`.

### Docs and plan
Modified by: Atharva Bhorpe

- `docs/FEATURES.md`, `docs/DECISIONS.md`, `docs/INTERFACE.md`, `docs/PLAN.md`, this changelog; `docs/HANDOFF.md`
  brought up to date; `CLAUDE.md` points to all of them.

### `pick_verified`: the wrist camera checks a pick before and after
Modified by: Atharva Bhorpe

- From the hover point it measures how far the object is from the aim and lines up on it; after lifting it checks
  the object is between the fingers. Run on the real arm with the blue cube.

### UGREEN webcam as the top camera; 1280x720 streams; calibration
Modified by: Atharva Bhorpe

- The UGREEN above the mat is `top`; the KD camera across the table is `across`.
- `base_in_mat` refitted (it was 2.2 cm and 4.8 degrees off); top camera recalibrated from the mat tags.
- All cameras stream 1280x720 so frames match the wrist calibration; `scripts/cameras_ready.sh` prepares them.

### Wrist-camera calibration
Modified by: Arian Malekzadeh (merged from `main`)

- Lens and hand-eye calibration of the wrist camera, `data/camera_wrist.json`, `scripts/calibrate_camera.py`.

### Chess, drawing and a grasp planner
Modified by: Atharva Bhorpe

- `chess_setup`, `chess_see`, `chess_move`, `chess_record`; `draw_plan`, `draw_pen`, `draw_run`; `plan_grasp`;
  skills `play-chess` and `draw`. Simulator only.
