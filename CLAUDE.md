# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Where to read first

| Question | File |
|---|---|
| What can it do, and how far is each part proven? | `docs/FEATURES.md` |
| Why is it built this way? | `docs/DECISIONS.md` |
| What are the contracts: tools, joint commands, files, frames, units? | `docs/INTERFACE.md` |
| The daemon's HTTP and WebSocket API | `docs/API.md` |
| How an agent should operate the arm | `workspace/AGENTS.md` and `workspace/skills/` |
| What is left to do before the submission, and in what order | `docs/PLAN.md` |
| Current state and next steps | `docs/HANDOFF.md` |

## What this is

`elrobot-agent`: agent control for the ElRobot arm (7 revolute joints + gripper, Feetech ST3215 servos, IDs 1–8) on top of the NormaCore Station. An AI talks to the robot through a tool registry exposed over HTTP, MCP and a voice agent. `docs/HANDOFF.md` records the current state and next steps; `docs/API.md` is the daemon's HTTP/WebSocket contract (keep it in sync when changing the daemon).

## Commands

Python 3.12, managed with `uv`:

```bash
uv sync                                   # install (add --extra voice for STT/TTS)
uv run --with pytest pytest tests -q      # all tests (pytest is not a declared dependency)
uv run --with pytest pytest tests/test_kinematics.py::test_zero_pose_matches_mesh_measurements -q

uv run elrobot up                         # Station + daemon (needs bin/station and hardware)
uv run elrobot up --sim                   # daemon only, simulated servos, no hardware
uv run elrobot call home                  # call one tool on the running daemon
uv run elrobot-mcp                        # MCP stdio shim (wired up in .mcp.json)
python -m elrobot.voice                   # voice-agent REPL against a stub robot

uv run python scripts/probe.py            # dump what the Station sees: servos, ranges, camera ids
uv run python scripts/snap.py             # save one frame per camera + servo state
uv run python scripts/calibrate.py --manual   # joint ranges (arm on 7.5 V: do not use --auto)
uv run python scripts/calibrate_camera.py --help  # camera lens/pose calibration (see below)
```

Web UI (`web/`, React + three.js + Vite): `npm install && npm run dev` (port 5173, proxies `/api`, `/ws`, `/assets` to the daemon on 8765); `npm run build` writes `web/dist`, which the daemon serves at `/`; `npm run typecheck`.

`ELROBOT_SIM`, `ELROBOT_VOICE`, `ELROBOT_PORT`, `ELROBOT_STATION` override `config.yaml`; every config key and its default lives in `src/elrobot/config.py` (unknown keys in `config.yaml` are an error).

## Station (Rust, not built here)

`bin/station` is gitignored and must be built: NormaCore at commit `6b494f77fac366a878ae71222458b196f8400dee`, apply `patches/norma-core-calibration-voltage.patch` (scales the calibration torque cap to the 7.5 V supply), `cargo build --release`, copy `target/release/station` to `bin/`. The norma-core clone belongs in `refs/` (gitignored). The Station must run from `station/` (`station/station.yaml`: servo bus, cameras at 1280x720 with `resize_target: 480`); it listens on TCP 8888 and serves its own viewer on 8889. The Python side never imports norma-core from the clone: `src/elrobot/norma.py` puts the vendored client + generated protobuf readers from `vendor/norma_core` on `sys.path`.

## Architecture

One process, the **daemon** (`daemon.py`, FastAPI on 8765), owns the arm. Everything else is a client of it, so there is a single source of commands and a single event stream:

```
Station (:8888) ⇄ Link ⇄ Robot ⇄ Skills ⇄ tools registry ⇄ daemon HTTP/WS
                                                            ↑     ↑      ↑
                                                       MCP shim  web UI  voice agent
```

- **`link.py`**: `StationLink` follows `st3215/inference` and the usbvideo camera queues (discovered via `inference-states`), keeping only the newest entry per queue and reconnecting when servo state goes quiet; writes are `sync_write`s of raw registers. `SimLink` is a drop-in simulated bus used by `--sim` and the tests.
- **`servo.py`**: register map and `JointRange`, the tick↔radian mapping. It uses the true 4096 ticks/turn scale, anchored so the centre of the Station's calibrated arc (`range_min`/`range_max`) is the centre of the URDF joint range; deliberately not the proportional 0–1 mapping NormaCore's viewer uses.
- **`kinematics.py`**: FK/IK built directly from `assets/elrobot/elrobot_follower.urdf`. Base frame: origin on the table under the base, x right, y forward, z up. TCP sits between the finger pads, 12 mm back from the fingertips (`TCP_BACKOFF`); tool z = approach, x = jaw closing axis.
- **`robot.py`**: `Robot` is the only thing that commands servos. One motion at a time, interruptible by `stop()`; paths are checked for table/self clearance; stall and stale-state detection. The servos run a soft P-only gain, so an outer "droop" loop integrates the settling error into a per-joint command bias (config `droop_*`).
- **`skills.py`**: behaviours on `Robot` (named poses, look/scan, pick/place/touch, gestures). Errors are `RobotError` messages written for an LLM: what failed and what to try.
- **`tools.py`**: the single tool registry (`@tool(name, description, schema)`). The daemon, the MCP server and the UI all use it. Motion tools are serialized (a second one gets `busy`); `motion=False` tools always run.
- **`mcp_server.py`**: stdio MCP shim; tool definitions come from `tools.py`, every call is forwarded to the daemon over HTTP (it never touches the arm itself).
- **`voice/`**: STT (mlx-whisper) → `agent.py` (Claude Agent SDK session that sees only the `elrobot` MCP tools) → TTS (kokoro). Models live in `models/` (gitignored, downloaded).
- **`vision.py`**: frame decode/overlays, simple table-top segmentation, and `CameraModel` (pinhole + k1/k2). A model is either tool-mounted (`rvec_tool_cam`/`t_tool_cam`, the wrist camera) or fixed in the room (`rvec_base_cam`/`t_base_cam`); `pixel_to_plane` intersects a pixel ray with a horizontal table plane, which is how 3D positions are obtained (cameras are RGB only, no depth).

## Calibration and data

- Joint ranges: Station calibration, stored in servo EEPROM / NormFS; done with `scripts/calibrate.py --manual`.
- Cameras: two identical "KD-USB Camera"s, `wrist` and `top`, told apart only by the USB-port-encoded unique id in `config.yaml` (re-run `probe.py` and swap if replugged). `scripts/calibrate_camera.py` (core in `calib.py`) does lens intrinsics with an 8x6-inner-corner, 25 mm chessboard, the fixed camera's pose by touching AprilTag (`tag25h9`) centres with the limp arm, and the wrist camera's mounting by hand-eye calibration. Results go to `data/camera_<name>.json`, which `Skills` loads (`camera_wrist.json`). The extrinsics/verify steps talk to the running daemon, not the Station.
- The wrist camera is calibrated (2026-11-22, `data/camera_wrist.json`): lens to 0.56 px; mounting `t_tool_cam` = [4.1, -73.4, -64.5] mm, tilted about 28 degrees, fitted to 49 hand-held poses from three runs with 8.7 px left over (about 7 mm at working distance). Each run alone stayed at 10 to 13 mm spread, above the script's 8 mm limit, so the saved mounting is a joint fit over all three and not what one run of the script writes. The default it replaces was about 4 cm and 28 degrees out.
- What the hand-eye runs taught:
  - The 8x6 board looks the same turned half round, and the detector then counts from the far corner: 11 of 49 samples came out that way and wreck the solution (49 mm spread instead of 13). `calib.unflip_boards` turns them back; the script calls it.
  - **The arm's accuracy falls off with reach.** A board lying on the table came out at the right height (14 mm) about 21 cm from the base and 2 to 2.5 cm too high at about 34 cm. That is a bias, not noise: the same place gives the same error, so averaging or a spread around the measurement does not remove it. A zero offset of a few degrees in shoulder or elbow would do this, or play in the gears; which has not been found out, and it was measured on the limp arm only. Joint calibration or a reach-dependent correction is what would remove it. Until then expect about 1 cm within 25 cm of the base and 2 cm or more beyond 30 cm, and the same for anything located through the wrist camera.
  - More poses do not help beyond about 15; different ones do. Lay the board about 20 cm in front of the base so the arm can look straight down and from both sides; further out it only gets slanted views from the robot's side.
  - The script waits for Enter twice. Started without a terminal it now carries on and leaves the arm limp at the end; each pose is written to `samples.json` as it is taken, and `--from-dir` solves again from them. `--guide` says which pose to take next (German).
- `data/sit_pose.json`: the folded resting pose. Runtime output goes to `logs/` (gitignored).
- Skills and tools currently use only the wrist camera; the top camera is not yet used for 3D.
