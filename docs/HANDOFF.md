# Handoff (2026-11-22, about 15:30)

**State.** The branch is `idea-1-arav`, and it is pushed. Two arms are on the table: the ElRobot and an SO-101
(`docs/SECOND_ARM.md`). An Orbbec depth camera stands across the table. Both arms are in one collision-checked
model and in the 3D view of the web UI. The things on the mat are obstacles there. The ElRobot hands a 3 cm
cube to the SO-101 in the air about two times in three. Stacking by the SO-101 was tried and did not work.
Both arms were left at home.

**To start:** Run `scripts/cameras_ready.sh && pixi run up` in one terminal. Run `scripts/second_arm_up.sh` in
another. Use terminals of your own for both. If you start them from an agent session, they are ended after two
hours. Run `scripts/orbbec_setup.sh` once per machine.

**Not yet seen working:**

- The obstacle columns and the SO-101's planned path in the 3D view. They were written after the last restart
  of the daemon.
- The daemon's own copy of the wrist-camera collision part. This is the same case: it loads at the next start.
- The wrist-turning step of `hand_over` for long things.
- `record_poses` played back in full by `replay_poses`. `hand_over` uses the hand-over part of it.

**Tests.** Workspace tests: 35 pass. The full simulator suite ran with both real daemons beside it. Everything
passes except `tests/test_push_t.py::test_the_t_is_pushed_onto_its_target[start0-target0]`. Its overlap came
to 0.732, where the test asks for more than 0.75. The push itself ended 2.5 mm and 4 degrees off. Not looked
into. The wrist camera became a body part in the kinematics the same afternoon, and that may or may not matter.

**Next:** refer to "Two arms" in `docs/PLAN.md`. The first item is to fit the SO-101 at table level. That item
unblocks the stacking.

---

# Earlier handoff (2026-11-22, about 04:30)

**State.** The branch is `idea-1-arav`, and it is pushed. The real arm works on Linux. The Station runs at
1280x720 with three cameras (`top` = UGREEN above the mat, `wrist`, `across`). The top camera is recalibrated.
`base_in_mat` is refitted. The wrist calibration from `main` is in use. The real arm picked a blue cube three
times, the last time with `pick_verified` from top-camera coordinates. Chess, drawing and `plan_grasp` exist
and pass in the simulator only. When the daemon stops a move as blocked, the harness now sends the move once
more at half speed (`blocked_retry_speed` in `workspace/robot_config/motion.json`). That is tested with a
faked daemon and has not yet happened on the real arm. The arm was left at `rest`. Refer to `docs/FEATURES.md`
for what is proven, and to `docs/DECISIONS.md` for why.

**Before you start the Station again:** Run `scripts/cameras_ready.sh` with the Station stopped. Then run
`pixi run up`. If the frames are 640x480, the script was skipped.

**Next (final submission is on 2026-11-22):**

- Record zero-shot and failure-recovery runs.
- Name objects by what they are. Keep a world model above the tracker.
- Make a `create_skill` tool.
- Record and replay a demonstrated motion.
- Draw on real paper.

Full chess on the real board is unlikely to work. The squares are 3 cm, and the arm is 1 to 2 cm off beyond
30 cm.

**Open problems:** The top camera's estimated position is probably incorrect (arm parts appear as grey objects
in `find_objects`). `pick_object` / `track_objects` have not been re-run since the camera change.

---

# Earlier handoff (2026-11-21)


The previous agent session stopped at its usage limit while it tested the `observe` perception tool.

**Done:** The Station is built from NormaCore `6b494f77fac366a878ae71222458b196f8400dee` plus
`patches/norma-core-calibration-voltage.patch`. The patch scales the calibration torque cap to supply voltage.
The arm runs on 7.5 V, not 12 V. The arm is calibrated (the offsets are in the servo EEPROM). The sitting pose
is in `data/sit_pose.json`. Kinematics/IK, motion, `Robot`, skills (pick/place/touch/gestures incl. fist bump),
tool registry, daemon and MCP server (`uv run elrobot-mcp`, 18 tools) work on the real arm. Droop correction
brings the arm to ~1 mm of target. Sim tests pass. Subagents built the web UI (`web/`, Vite) and the voice
agent (`src/elrobot/voice/`).

**Next:** Validate `observe` / vision. Then do an end-to-end pick of the white box or a blue cube via MCP.
Then connect the voice agent to the daemon. Hosted detection needs a Telekinesis API key
(platform.telekinesis.ai/api-keys) from the user.

**Rebuild ignored artifacts:** Clone norma-core into `refs/`. Check out the commit above. Apply the patch. Run
`cargo build --release`. Copy `target/release/station` to `bin/`. The models (`models/whisper-large-v3-turbo`,
`models/kokoro`) are downloaded, not committed.
