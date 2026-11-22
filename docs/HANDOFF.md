# Handoff (2026-11-22, about 15:30)

**State.** Branch `idea-1-arav`, pushed. Two arms on the table: the ElRobot and an SO-101 (`docs/SECOND_ARM.md`).
An Orbbec depth camera stands across the table. Both arms are in one collision-checked model and in the web
UI's 3D view, with the things on the mat as obstacles. The ElRobot hands a 3 cm cube to the SO-101 in the air
about two times in three. Stacking by the SO-101 was tried and did not work. Both arms were left at home.

**To start:** `scripts/cameras_ready.sh && pixi run up` in one terminal, `scripts/second_arm_up.sh` in
another. Both in terminals of your own: started from an agent session they are ended after two hours.
`scripts/orbbec_setup.sh` once per machine.

**Not yet seen working:** the obstacle columns and the SO-101's planned path in the 3D view (written after
the daemon was last restarted); the daemon's own copy of the wrist-camera collision part (the same: it loads
at the next start); the wrist-turning step of `hand_over` for long things; `record_poses` played back in full
by `replay_poses` (the hand-over part of it is what `hand_over` uses).

**Tests.** Workspace tests: 35 pass. Full simulator suite, run with both real daemons beside it: everything
passes except `tests/test_push_t.py::test_the_t_is_pushed_onto_its_target[start0-target0]`, whose overlap came
to 0.732 where the test asks for more than 0.75 (the push itself ended 2.5 mm and 4 degrees off). Not looked
into; the wrist camera became a body part in the kinematics the same afternoon and may or may not matter.

**Next:** see "Two arms" in `docs/PLAN.md`. The first item, fitting the SO-101 at table level, unblocks the
stacking.

---

# Earlier handoff (2026-11-22, about 04:30)

**State.** Branch `idea-1-arav`, pushed. Real arm working on Linux: Station at 1280x720 with three cameras
(`top` = UGREEN above the mat, `wrist`, `across`), top camera recalibrated, `base_in_mat` refitted, wrist
calibration from `main` in use. A blue cube was picked three times on the real arm, the last with
`pick_verified` from top-camera coordinates. Chess, drawing and `plan_grasp` exist and pass in the simulator
only. A move the daemon stops as blocked is now sent once more at half speed (`blocked_retry_speed` in
`workspace/robot_config/motion.json`); that is tested with a faked daemon and has not yet happened on the real
arm. The arm was left at `rest`. See `docs/FEATURES.md` for what is proven and `docs/DECISIONS.md` for why.

**Before starting the Station again:** `scripts/cameras_ready.sh` (Station stopped), then `pixi run up`.
If frames come out 640x480 the script was skipped.

**Next (final submission is on 2026-11-22):** record zero-shot and failure-recovery runs; name objects by what
they are and keep a world model above the tracker; a `create_skill` tool; record and replay a demonstrated
motion; drawing on real paper. Full chess on the real board is unlikely to work: squares are 3 cm and the arm
is 1 to 2 cm off beyond 30 cm.

**Open problems:** the top camera's estimated position is probably off (arm parts appear as grey objects in
`find_objects`); `pick_object` / `track_objects` have not been re-run since the camera change.

---

# Earlier handoff (2026-11-21)


The previous agent session stopped at its usage limit while testing the `observe` perception tool.

**Done:** Station built from NormaCore `6b494f77fac366a878ae71222458b196f8400dee` plus
`patches/norma-core-calibration-voltage.patch` (scales the calibration torque cap to supply voltage; the
arm runs on 7.5 V, not 12 V). Arm calibrated (offsets live in servo EEPROM), sitting pose in
`data/sit_pose.json`. Kinematics/IK, motion, `Robot`, skills (pick/place/touch/gestures incl. fist bump),
tool registry, daemon and MCP server (`uv run elrobot-mcp`, 18 tools) work on the real arm; droop
correction gets the arm to ~1 mm of target. Sim tests pass. Web UI (`web/`, Vite) and voice agent
(`src/elrobot/voice/`) were built by subagents.

**Next:** validate `observe` / vision, then do an end-to-end pick of the white box or a blue cube via MCP,
then wire the voice agent to the daemon. Hosted detection needs a Telekinesis API key
(platform.telekinesis.ai/api-keys) from the user.

**Rebuild ignored artifacts:** clone norma-core into `refs/`, check out the commit above, apply the patch,
`cargo build --release`, copy `target/release/station` to `bin/`. Models (`models/whisper-large-v3-turbo`,
`models/kokoro`) are downloaded, not committed.
