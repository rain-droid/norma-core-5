# Handoff (2026-11-21)

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
