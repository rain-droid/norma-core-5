# elrobot-agent

Agent control for the ElRobot arm (seven joints and a parallel gripper, Feetech ST3215 servos) on top of the
NormaCore Station. An AI agent is started in `workspace/` and operates the arm with the Python tools there:
it sees the table through a camera above the mat and one on the wrist, plans in Python it can read and change,
and sends the robot one thing only, joint commands.

![Top camera view with the base frame, reach limits and mat tags](docs/images/top_view_frames.jpg)

## What it does

- Finds, tracks, picks, places and stacks objects on the table; a pick can be checked by the wrist camera
  before and after (`pick_verified`). Run on the real arm.
- Recovers by itself: aims again when an object is moved, tries again when a grasp comes up empty, and sends a
  move that was stopped as blocked once more, slower.
- Chess against a person, drawing with a pen, and grasp planning among neighbours: in the simulator so far.
- A web UI, an MCP server and a voice agent on the same daemon.
- A depth camera across the table measures heights and keeps what stands on the mat as obstacles.
- A second arm (an SO-101) shares the table: both arms live in one collision-checked model, shown together in
  the web UI's 3D view, and the ElRobot can hand a cube to it in the air. See `docs/SECOND_ARM.md`.

`docs/FEATURES.md` says how far each part is proven.

![Both arms and the scanned things in the web UI's 3D view](docs/images/two_arm/webui_two_arms.jpg)

## Run it

```bash
pixi run sim                    # simulated arm, no hardware
cd workspace && claude          # or any coding agent; it reads AGENTS.md
```

For the real arm: build the Station (see `CLAUDE.md`), then `scripts/cameras_ready.sh` and `pixi run up`.
With the second arm: `scripts/second_arm_up.sh` in another terminal.

```bash
uv run --with pytest pytest tests -q                       # simulator tests, about 5 minutes
(cd workspace && uv run --with pytest pytest -q tests)     # workspace code against a faked daemon, under a second
```

## Where to read

| Question | File |
|---|---|
| What can it do, and how far is each part proven? | `docs/FEATURES.md` |
| Why is it built this way? | `docs/DECISIONS.md` |
| Tools, joint commands, files, frames, units | `docs/INTERFACE.md` |
| The daemon's HTTP and WebSocket API | `docs/API.md` |
| The second arm, the depth camera, the shared space | `docs/SECOND_ARM.md` |
| Pictures and measurements from the real table | `docs/RESULTS.md` |
| How an agent operates the arm | `workspace/AGENTS.md`, `workspace/skills/` |
| Current state and next steps | `docs/HANDOFF.md`, `docs/PLAN.md` |

## Layout

`workspace/` is the agent's working directory (tools, skills, robot model, live perception, call history).
`src/elrobot/` is the robot side: the daemon that owns the arm, the servo link, kinematics and the simulator.
`station/` is the NormaCore Station config, `scripts/` bring-up and calibration helpers, `web/` the UI.
