# Robot daemon API (v1)

One long-running process, the **daemon**, owns the robot. Everything else is a client:

```
NormaCore Station (Rust, :8888 TCP)  <->  daemon (Python, :8765 HTTP + WS)
                                             ^            ^            ^
                                   MCP stdio shim     web UI      voice agent
                                   (Claude Code)    (browser)   (Claude Agent SDK)
```

- Base URL: `http://127.0.0.1:8765`
- All bodies are JSON unless stated. All lengths are **metres**, all angles **radians**.
- Robot base frame: origin on the table under the base, **x = right, y = forward, z = up**
  (seen from behind the robot). This is the URDF frame, unchanged.
- Joints are indexed by servo id `1..8`. `1..7` are the arm, `8` is the gripper.

## WebSocket `/ws`

Server pushes JSON text messages. The client never needs to send anything.

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

`events` is the backlog (most recent ~200 `event` messages, oldest first) so a page reload
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
- `connected`: the link to Station (or the sim) is up and state is fresh.
- `busy` / `activity` / `source`: whether a motion or skill is running, a short human label
  (`"idle"`, `"moving"`, `"grasping"`, `"dap"`), and who started it
  (`"claude-code"`, `"voice"`, `"ui"`, or `null`).
- `joints[].q` is the measured angle, `q_goal` the commanded one. `current` is raw servo
  units (roughly 6.5 mA each), `temp` in °C, `error` is the servo status byte (0 = ok).
- `gripper.open` is 0 (closed) to 1 (fully open). `holding` is true after a verified grasp.
- `ee` is the tool centre point (between the fingertips) in the base frame. `quat` is
  `[x, y, z, w]`. `approach` is the unit vector the gripper points along.

### `event`

Every event has `type: "event"`, a monotonically increasing integer `id`, a unix time `t`,
and a `kind`. Kinds:

| kind | fields | meaning |
|---|---|---|
| `tool_start` | `call_id`, `tool`, `args`, `source` | a tool call began |
| `tool_end` | `call_id`, `tool`, `ok`, `summary`, `error`, `duration_ms`, `source` | it finished; `summary` is one human line |
| `plan` | `call_id`, `path`, `target` | planned TCP polyline `[[x,y,z],...]` and `target: {"pos": [...], "approach": [...]}`; an empty `path` clears it |
| `markers` | `markers` | **replaces** the set of world markers: `[{"id", "label", "pos": [x,y,z], "color", "radius"}]` |
| `detections` | `camera`, `width`, `height`, `items` | 2D detections on a camera frame: `[{"label", "score", "box": [x1,y1,x2,y2], "point": [u,v], "polygon": [[u,v],...]}]`; `polygon` optional; empty `items` clears |
| `log` | `level` (`info`/`warn`/`error`), `text` | daemon log line worth showing |
| `chat` | `role` (`user`/`assistant`), `text`, `turn_id`, `final`, `via` (`voice`/`text`) | conversation transcript. Assistant text may arrive as several events with the same `turn_id`; each carries the **full text so far**, and the last has `final: true` |
| `speech` | `turn_id`, `seq`, `text`, `url`, `duration_ms` | a synthesised sentence is ready; the browser plays these in `seq` order |
| `voice_state` | `state` | `idle`, `listening`, `transcribing`, `thinking`, `speaking`, `acting` |

## HTTP

### State and assets

- `GET /api/state` → the latest `state` message.
- `GET /api/meta` → the `hello` message without `events`.
- `GET /assets/elrobot/elrobot_follower.urdf` and `GET /assets/elrobot/assets/*.stl` → robot
  model. Mesh paths inside the URDF are relative (`assets/<name>.stl`, millimetres, scaled
  by 0.001 in the URDF).

### Camera

- `GET /api/camera/{name}/frame.jpg` → latest JPEG (`404` if no frame yet).
- `GET /api/camera/{name}/stream.mjpg` → `multipart/x-mixed-replace` MJPEG stream.

### Tools

- `POST /api/tools/{tool}` with body `{"args": {...}, "source": "ui"}`
  → `{"ok": true, "summary": "...", "data": {...}, "images": [{"mime": "image/jpeg", "b64": "..."}]}`
  or `{"ok": false, "error": "...", "summary": "..."}`. Blocks until the tool finishes.
  Tool names and JSON schemas come from `hello.tools`. A second motion tool called while one
  is running returns `ok: false` with `error: "busy"`.
- `POST /api/stop` → immediately halts motion and holds position. Always allowed. `{"ok": true}`.
- `POST /api/torque` with `{"enable": false}` → release (arm goes limp) or re-enable torque.

Tools the UI calls directly (stable names): `home`, `rest`, `stop`, `gesture` (`{"name": "wave"}`),
`gripper` (`{"open": 0.0..1.0}`), `move_joints` (`{"joints": {"1": 0.2}, "relative": true}`).

### Conversation and voice

- `POST /api/chat` with `{"text": "dap me up"}` → `{"turn_id": "..."}`. Returns at once; the reply
  streams as `chat` / `speech` / `tool_*` events.
- `POST /api/chat/interrupt` → stops the agent's current turn and the robot. `{"ok": true}`.
- `POST /api/voice/utterance` (multipart form, field `audio`, any container ffmpeg reads,
  e.g. `audio/webm;codecs=opus`) → `{"text": "dap me up", "turn_id": "..."}`. The daemon transcribes,
  then treats the text exactly like `/api/chat` with `via: "voice"`. Empty or noise-only audio
  returns `{"text": "", "turn_id": null}` and starts no turn.
- `GET /api/voice/audio/{id}.wav` → audio for a `speech` event.
