# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

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
uv run elrobot up --lan                   # also serve other machines (prints URL + access key); see docs/API.md "Control"
uv run elrobot call home                  # call one tool on the running daemon
uv run elrobot-mcp                        # MCP stdio shim (wired up in .mcp.json)
python -m elrobot.voice                   # voice-agent REPL against a stub robot

uv run python scripts/probe.py            # dump what the Station sees: servos, ranges, camera ids
uv run python scripts/snap.py             # save one frame per camera + servo state
uv run python scripts/calibrate.py --manual   # joint ranges (arm on 7.5 V: do not use --auto)
uv run python scripts/calibrate_camera.py --help  # camera lens/pose calibration (see below)
```

Web UI (`web/`, React + three.js + Vite): `npm install && npm run dev` (port 5173, proxies `/api`, `/ws`, `/assets` to the daemon on 8765; `ELROBOT_DAEMON=http://<host>:8765 ELROBOT_KEY=... npm run dev` for a daemon on another machine); `npm run build` writes `web/dist`, which the daemon serves at `/`; `npm run typecheck`.

`ELROBOT_SIM`, `ELROBOT_VOICE`, `ELROBOT_PORT`, `ELROBOT_HOST`, `ELROBOT_KEY`, `ELROBOT_STATION` override `config.yaml`; every config key and its default lives in `src/elrobot/config.py` (unknown keys in `config.yaml` are an error).

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
- `data/sit_pose.json`: the folded resting pose. Runtime output goes to `logs/` (gitignored).
- The daemon's skills (`skills.py`) use only the wrist camera. The agent workspace (`workspace/tools/`) measures with the top camera, calibrated from the mat's AprilTags into `workspace/robot_config/camera_top.json` (`tools/calibrate_top.py`).
- Servo calibration backup: `calibration/station-calib.zip` (restore into `station/` with the Station stopped). The arm servos' overload threshold is 60 % (`scripts/servo_overload.py`); Station auto-calibration resets it to 25 %.

## Station build notes (macOS)

Besides the steps above: `prost-build` needs `protoc` (a prebuilt one in `refs/protoc/` works, set `PROTOC` and `PROTOC_INCLUDE`); the Station embeds its viewer, so build `software/station/clients/station-viewer` (`npm ci && npm run build`) before `cargo build --release -p station`. Homebrew's `rustup` builds cmake from source on older macOS; the official rustup installer is faster. macOS asks for camera access on the first start; when the Station is launched from a background shell the request is denied silently ("Camera access denied" in `logs/station.log`): allow the terminal app under System Settings → Privacy & Security → Camera and restart.

## What needs improving

Found in a review on 2026-11-21, not yet fixed. Roughly in order of how much they hurt.

Correctness:
- **Jaw yaw is wrong for tilted approaches.** `find_objects.jaw_yaw` measures yaw from the base x axis, but `jaw_from_yaw` (`kinematics.py`) uses base x only for a straight-down approach and `cross(up, approach)` otherwise. Grasps off to the side or beyond ~27 cm turn the jaws by the target's angle from the y axis (~39° at (0.2, 0.25)). Fix in `planning.approach_for` / `skills._approach_for`, or make `jaw_from_yaw` always reference base x.
- **Wrist calibration never reaches the workspace.** `calibrate_camera.py` writes `data/camera_wrist.json`; the workspace reads `workspace/robot_config/camera_wrist.json`. Add a symlink like the one for `sit_pose.json`, or read one path.
- **Stale frames.** `workspace/tools/util/arm.py` `Arm.frame` accepts frames up to 2 s old (possibly from before the last move); `capture.freeze` records the pose after the frames. Require a frame newer than the last command, read q before and after capturing.
- **Silent failures.** `robot_config/control.py` (`Robot.step`/`call`) returns `{"error": ...}` instead of raising; `tools/home.py` swallows a failed lift.
- **`observe()` mixes moments**: `latest.json`, `state.json` and the fixed-name JPEGs are rewritten separately at 2 Hz. Publish one consistent snapshot (state inside `latest.json`, images under per-sequence names).
- Top camera ignores image size (`capture.TopCamera` never checks the calibration's `image_size`); `place_at` always releases 22 mm above the surface whatever `grasp_height` `pick_at` used.

Daemon / sharing the arm:
- **IK runs on the asyncio loop** (`robot.py` solve/plan) and stalls servo reading and the UI stream while it solves; `Kinematics` shares pinocchio data between calls, so it is not safe to call concurrently. Run IK with `asyncio.to_thread` and per-call `createData()`.
- **Tunnels bypass the access key**: requests from 127.0.0.1 need no key, and a reverse proxy or tunnel (cloudflared, ngrok, the Vite dev server) delivers remote requests from 127.0.0.1. Treat requests carrying `X-Forwarded-For` / `CF-Connecting-IP` as remote before exposing the daemon that way. Tailscale is fine as is.
- The access key travels over plain HTTP; fine on a team network, not on an open one.
- MCP shim and voice agent send single commands only; they get refused while someone else holds control instead of queueing.

Structure:
- Pick/place/approach logic exists twice (`skills.py` and `workspace/tools/`), as do the camera models (`vision.CameraModel` vs the workspace's homography `TopCamera`), FK (three times), `READY_DEG`, jaw masks and colour naming. Decide which path stays.

Simulation (goal: train policies):
- `simworld.py` is fine for integration tests and agent runs, not for training: real-time only (asyncio sleeps), not deterministic, no reset/step API, physics only runs inside camera rendering, blocks cannot rotate, arm and jaws pass through objects, the gripper ignores orientation (so it cannot catch the yaw bug above), flat rendering. Plan: MuJoCo from the URDF behind a gym-style `reset()/step(action)`, plus a `MujocoLink` with `SimLink`'s interface so daemon, UI and agent run on it unchanged.

Next features:
- Feedback loop: continuous visual servoing on the wrist camera during the approach (frames timestamped against joint state) instead of stop-look-move; a local executor that runs a whole plan without LLM round trips and retries known failures itself.
- Finding the requested object: colour-numbered scan first, an open-vocabulary detector (YOLO-World / OWLv2) only when the request is ambiguous.
- Goals: chess (board pose calibrated once, squares as fixed coordinates, occupancy check with the top camera, moves from an engine) and a mid-air handover between two arms (shared frame from AprilTags, one daemon per arm plus a coordinator).
