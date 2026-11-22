# Handoff (2026-11-22, about 04:30)

**State.** Branch `idea-1-arav`, pushed. Real arm working on Linux: Station at 1280x720 with three cameras
(`top` = UGREEN above the mat, `wrist`, `across`), top camera recalibrated, `base_in_mat` refitted, wrist
calibration from `main` in use. A blue cube was picked three times on the real arm, the last with
`pick_verified` from top-camera coordinates. Chess, drawing and `plan_grasp` exist and pass in the simulator
only. The arm was left at `rest`. See `docs/FEATURES.md` for what is proven and `docs/DECISIONS.md` for why.

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
