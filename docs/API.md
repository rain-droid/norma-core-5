# Robot daemon API (v1)

This document gives the HTTP and WebSocket contract of the daemon. For the workspace tools that an agent calls,
the file formats, and the frames and units, refer to `docs/INTERFACE.md`. For what works and how far it is
proven, refer to `docs/FEATURES.md`.

One long-running process, the **daemon**, owns the robot. Everything else is a client:

```
NormaCore Station (Rust, :8888 TCP)  <->  daemon (Python, :8765 HTTP + WS)
                                             ^            ^            ^
                                   MCP stdio shim     web UI      voice agent
                                   (Claude Code)    (browser)   (Claude Agent SDK)
```

- Base URL: `http://127.0.0.1:8765`
- The daemon answers `403` to a request that is not a `GET` and has an `Origin` header from a page of another machine.
  The reason: a web page that is open in a browser must not be able to move the arm. Clients that are not browsers send no `Origin`.
- All bodies are JSON unless stated. All lengths are in **metres**. All angles are in **radians**.
- Robot base frame: the origin is on the table below the base, with **x = right, y = forward, z = up**
  (seen from behind the robot). This is the URDF frame, unchanged.
- The index of a joint is its servo id `1..8`. `1..7` are the arm. `8` is the gripper.

## WebSocket `/ws`

The server pushes JSON text messages. The client never needs to send anything.

### `hello` (once, on connect)

```json
{
  "type": "hello",
  "version": 1,
  "mode": "sim",
  "urdf": "/assets/elrobot/elrobot_follower.urdf",
  "joints": [{"id": 1, "name": "rev_motor_01", "lower": -1.5509, "upper": 1.5509}],
  "tools": [{"name": "pick", "description": "...", "input_schema": {"type": "object", "properties": {}}}],
  "gestures": ["dap", "wave", "nod"],
  "cameras": ["wrist"],
  "table_z": 0.0,
  "events": []
}
```

`events` is the backlog (the most recent 300 `event` messages, oldest first). With the backlog, a page reload
restores the timeline.

### `state` (about 30 Hz)

```json
{
  "type": "state",
  "t": 1795260000.123,
  "mode": "sim",
  "connected": true,
  "calibrated": true,
  "torque": true,
  "busy": false,
  "pace": 1.0,
  "activity": "idle",
  "source": null,
  "joints": [
    {"id": 1, "name": "rev_motor_01", "q": 0.0, "q_goal": 0.0, "lower": -1.5509, "upper": 1.5509,
     "tick": 2048, "current": 12, "load": 0, "temp": 31, "volt": 12.1, "torque": true, "error": 0}
  ],
  "gripper": {"open": 0.8, "width_mm": 41.0, "holding": false},
  "ee": {"pos": [0.0, 0.25, 0.12], "quat": [0, 0, 0, 1], "approach": [0, 0, -1]},
  "cameras": [{"name": "wrist", "width": 640, "height": 480, "fps": 14.8, "age_ms": 40}]
}
```

- `mode`: `"sim"` (no hardware, simulated servos) or `"real"`.
- `connected`: the link to Station (or the sim) is active and the state is fresh.
- `busy` / `activity` / `source`: whether a motion or skill runs, a short label for people
  (`"idle"`, `"moving"`, `"grasping"`, `"dap"`), and who started it
  (`"claude-code"`, `"voice"`, `"ui"`, or `null`).
- `joints[].q` is the measured angle. `q_goal` is the commanded angle. `current` is in raw servo
  units (roughly 6.5 mA each). `temp` is in °C. `error` is the servo status byte (0 = ok).
- `gripper.open` is 0 (closed) to 1 (fully open). `holding` is true after a verified grasp.
- `ee` is the tool centre point (between the fingertips) in the base frame. `quat` is
  `[x, y, z, w]`. `approach` is the unit vector along which the gripper points.

### `event`

Each event has `type: "event"`, an integer `id` that increases monotonically, a unix time `t`,
and a `kind`. The kinds are:

| kind | fields | meaning |
|---|---|---|
| `tool_start` | `call_id`, `tool`, `args`, `source` | a tool call started |
| `tool_end` | `call_id`, `tool`, `ok`, `summary`, `error`, `duration_ms`, `source` | the tool call ended; `summary` is one line for people |
| `plan` | `call_id`, `path`, `target` | the planned TCP polyline `[[x,y,z],...]` and `target: {"pos": [...], "approach": [...]}`; an empty `path` clears it |
| `markers` | `markers` | **replaces** the set of world markers: `[{"id", "label", "pos": [x,y,z], "color", "radius"}]` |
| `detections` | `camera`, `width`, `height`, `items` | 2D detections on a camera frame: `[{"label", "score", "box": [x1,y1,x2,y2], "point": [u,v], "polygon": [[u,v],...]}]`; `polygon` is optional; an empty `items` clears them |
| `log` | `level` (`info`/`warn`/`error`), `text` | a daemon log line that is worth showing |
| `chat` | `role` (`user`/`assistant`), `text`, `turn_id`, `final`, `via` (`voice`/`text`) | the conversation transcript. Assistant text can arrive as several events with the same `turn_id`. Each event contains the **full text so far**. The last event has `final: true` |
| `speech` | `turn_id`, `seq`, `text`, `url`, `duration_ms` | a synthesised sentence is ready. The browser plays these sentences in `seq` order |
| `steps` | `title`, `steps`, `current`, `failed`, `source` | the plan of the agent for its task, for the Steps panel of the UI: **replaces** the plan before. `steps` is a list of short sentences. `current` is the number of completed steps. Thus `current` is also the index of the step in work, and it is equal to the length when all steps are complete. `failed` marks the current step as failed. An empty `steps` clears the plan |
| `voice_state` | `state` | `idle`, `listening`, `transcribing`, `thinking`, `speaking`, `acting` |

## HTTP

### State and assets

- `GET /api/state` → the latest `state` message.
- `GET /api/meta` → the `hello` message without `events`.
- `GET /assets/elrobot/elrobot_follower.urdf` and `GET /assets/elrobot/assets/*.stl` → the robot
  model. The mesh paths in the URDF are relative (`assets/<name>.stl`, millimetres, scaled
  by 0.001 in the URDF).

### Camera

- `GET /api/camera/{name}/frame.jpg` → the latest JPEG (`404` if there is no frame yet).
- `GET /api/camera/{name}/stream.mjpg` → `multipart/x-mixed-replace` MJPEG stream.
- `GET /api/camera/{name}/vision` → what the vision sees in the newest frame, for the camera panel of the web UI
  (`src/elrobot/vision_live.py`): `{camera, width, height, seq, ms, layers}`. `layers.objects` are segmented
  outlines (`label` a colour word, `polygon`, `box`, `point`). `layers.tags` are AprilTags. For `top`, when its
  calibration matches the frame size, there is also `layers.frame` (base axes and reach limits). In the same
  condition, there is also `layers.arm` (the arm as the kinematics place it). The daemon computes the result on request and caches it for each frame.
  The result is for display only. `404` if there is no frame yet.

### Tools

- `POST /api/tools/{tool}` with body `{"args": {...}, "source": "ui"}`
  → `{"ok": true, "summary": "...", "data": {...}, "images": [{"mime": "image/jpeg", "b64": "..."}]}`
  or `{"ok": false, "error": "...", "summary": "..."}`. The request blocks until the tool finishes.
  Tool names and JSON schemas come from `hello.tools`. If a second motion tool is called while one
  runs, the second call returns `ok: false` with `error: "busy"`.
- `POST /api/stop` → stops motion immediately and holds the position. This is always allowed. `{"ok": true}`.
- `POST /api/pace` with `{"scale": 0.25, "hold_s": 0.5}` → motions run at that fraction (0.05 to 1) of
  their planned speed for the next `hold_s` seconds (at most 5). Then they return to full speed
  automatically. A motion that is in progress changes speed smoothly along its path. To keep the arm slow,
  continue to post. `state.pace` is the fraction that is in effect. The tracker of the workspace posts this while
  something moves on the table. `{"ok": true, "pace": 0.25}`.
- `GET /api/second_arm/assets/{path}` → a file of the model of the second arm (URDF, meshes). This daemon
  fetches the file from the daemon of the second arm. The state message and `GET /api/state` contain
  `second_arm` when `second_arm_url` is set:
  `{"robot", "urdf", "T", "connected", "torque", "activity", "joints", "gripper", "ee"}`. `T` is the base of the
  second arm in the base frame of this arm (4x4, null until `tools/calibrate_second_arm.py` has run). The whole
  value is null while the daemon of the second arm does not answer. This daemon never commands the second arm.
- `POST /api/guide` with `{"seconds": 30}` → a person leads the arm by hand. When a joint is pushed more than
  `guide_deadband` off its resting error, the command of that joint follows the push. It stays where it is left.
  The request blocks until the arm is moved and then left alone for `guide_idle_s`, until `seconds` (1 to 120)
  pass, or until `/api/stop`. `{"ok": true, "path": [[7 joint angles], ...], "moved": true, "ended": "left alone" | "time" | "stopped", "seconds": 6.4}`.
  The path starts where the arm stood. The joint limits and the table hold throughout (`"allow_low": false` also
  keeps the fingertips above the usual floor clearance). `busy` while another command runs.
- `POST /api/torque` with `{"enable": false}` → releases the torque (the arm goes limp) or enables it again.

The UI calls these tools directly (stable names): `home`, `rest`, `stop`, `gesture` (`{"name": "wave"}`),
`gripper` (`{"open": 0.0..1.0}`), `move_joints` (`{"joints": {"1": 0.2}, "relative": true}`).

### Conversation and voice

- `POST /api/chat` with `{"text": "dap me up"}` → `{"turn_id": "..."}`. The request returns immediately. The
  reply arrives as a stream of `chat` / `speech` / `tool_*` events.
- `POST /api/chat/interrupt` → stops the current turn of the agent and stops the robot. `{"ok": true}`.
- `POST /api/voice/utterance` (multipart form, field `audio`, any container that ffmpeg reads,
  for example `audio/webm;codecs=opus`) → `{"text": "dap me up", "turn_id": "..."}`. The daemon transcribes
  the audio. Then it handles the text exactly as it handles `/api/chat`, with `via: "voice"`. Audio that is
  empty or contains only noise returns `{"text": "", "turn_id": null}` and starts no turn.
- `GET /api/voice/audio/{id}.wav` → the audio for a `speech` event.

### Agent workspace (`workspace/`): joint commands only

The tools of the workspace run in the shell of the agent itself. This is all that they can ask of the arm:

- `POST /api/joints` with `{"path": [[q1..q7], ...], "gripper": 0..1, "speed": 0.8, "check": true,
  "allow_low": false, "label": "...", "source": "agent", "call_id": "..."}`. `path` is one or more
  waypoints of seven joint angles in radians. The arm travels them as one move. The daemon applies `gripper`
  after the arm arrives, and 0 closes the jaws until they stall. You can leave out either of them. The request
  blocks until it is done.
  It returns `{"ok": true, "joints": [7 measured], "gripper": {...}, "tcp": {...}}` or
  `{"ok": false, "error": "...", "kind": "invalid" | "busy" | "robot" | "blocked" | "stopped"}`.
  `blocked` is a joint that trails its setpoint. It is the one refusal that the workspace answers: the workspace
  sends the move one more time, slower.
  The daemon refuses a command outside the calibrated joint limits. It refuses a command outside the clearance
  envelope unless `check` is false (`allow_low` lets the fingertips reach the table surface). It refuses a
  command while any servo reports a fault. The command shares the one-motion-at-a-time lock with `/api/tools`.
  A command without `call_id` shows on the timeline as its own `joint_command` entry.
- `POST /api/event` with `{"kind": "tool_start" | "tool_end" | "markers" | "detections" | "log" | "steps", ...}`
  puts an event on the stream. Thus tool calls made in the workspace show in the web UI. This is for display only.
- `GET /api/meta` also contains `clearance` (`floor`, `body`, `min_y`, `limit_margin`). This is the envelope
  that the daemon holds joint commands to. It is for whoever plans the commands.
- While the daemon runs, it rewrites `workspace/perception/` at 2 Hz and after every joint command.
  With `--sim`, it also writes the calibration of the simulated top camera to
  `workspace/robot_config/camera_top.sim.json`.
