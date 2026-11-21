# Checkpointed cleanup session

One persistent process owns Station and exposes twelve operations through HTTP/CLI and MCP: `status`, `observe`, `act`, `assess`, `set_torque`, `stop`, `prepare_alignment`, `probe_alignment`, `align_target`, `select_target`, `locate_target`, `park`. It reuses the calibrated pgripper model and executor; it does not provide automatic object detection or whole-table grounding.

## Start

From the repository root:

```sh
.venv/bin/so101-cleanup serve --session-sim --port 8772
```

For this physical setup, the local `logs/cleanup-session.json` contains exact camera identities, bus serial, measured table heights, a pickup hover and taught cover pose. Inspect these before reusing after camera/base/cover movement. A hardware config must explicitly set `sim_only=false`.

```sh
.venv/bin/so101-cleanup serve --session logs/cleanup-session.json --enable-motion --port 8772 --out logs/cleanup-session
.venv/bin/so101-cleanup call status --url http://127.0.0.1:8772
.venv/bin/so101-cleanup call observe --url http://127.0.0.1:8772
```

Omitting `--enable-motion` makes the hardware connection read-only. Startup never enables torque. Session `speed_rad_s` defaults to 0.15 and is capped at 0.45; explicit hold applies matching arm servo RAM speed/acceleration. The current local config uses 0.45 at the operator’s request, while gripper speed stays separate. The service takes a per-Station/bus file lease; do not run older ad hoc motor scripts alongside it. The lease coordinates cooperating services, not external controllers.

For a session MCP client, set `SO101_CLEANUP_MODE=session` and `SO101_CLEANUP_URL=http://127.0.0.1:8772`, then launch `.venv/bin/so101-cleanup-mcp`. The older calibrated-table interface remains the default without the mode variable. A read-only protocol check is available:

```sh
SO101_CLEANUP_URL=http://127.0.0.1:8772 .venv/bin/python scripts/session_mcp_smoke.py
```

## Operate

Every mutation except emergency stop uses a unique `action_id` and fresh `observation_id`. Resubmit exactly the same arguments and ID after a transport timeout: it returns the stored result or in-progress status, without replaying motion. IDs belong to one process lifetime; observations from a previous process cannot authorize motion after restart. Do not substitute a new ID just to overcome a timeout.

The [target tools](TARGETS.md) remember a selected paper/wrapper patch and locate it in a supplied observation. They do not advance the task, capture another frame or write motors. Results are image positions tied to the source observation. A current wrist target can supply `target_id` to `probe_alignment` instead of a manually repeated bbox.

1. `observe`, then `set_torque(mode="hold")` explicitly if needed. Hold first targets measured joint positions.
2. `act(action="begin", params={"target_id":"paper-1"})` selects a new task identity.
3. `open`, `approach` to the saved hover, and `nudge` using `params={"delta_mm":[x,y,z]}`. Units are millimetres in the robot-base frame. Nudges are limited to 10 mm each and 120 mm cumulative per target. Each motion requires `scene_clear=true` based on the viewed corridor.
4. `grasp`, then `lift`. Gripper contact is only a candidate. The stage becomes `awaiting_lift` and **carry is unavailable**.
5. Inspect both returned views. `assess(outcome="held", source_clear=true, evidence="...")` permits carry. `empty` permits recovery; `uncertain` leaves carry blocked. Opening after a miss preserves the two-attempt budget.
6. `carry` uses the configured clearance and drop pose, then `release`. `retreat` clears the view. `assess(outcome="delivered", source_clear=true, on_cover=true, evidence="...")` counts delivery exactly once.

Use the post-action observation ID for the next step, or call `observe` again. Observations expire after 30 seconds and are invalidated by another operation, stop, or appreciable measured pose change. Both frames must be newly captured and within 250 ms of each other; timestamps remain separate. If post-action capture fails, `completed_without_observation` preserves the motion outcome and requires a new observation before further action.

`status` reports active action, stage, attempt and permitted actions. `stop` remains callable during motion and latches cancellation while attempting to hold. It does not make the arm limp. `set_torque(mode="limp", arm_supported=true)` is a separate deliberate operation and ends the session's task. Restart never resumes a task automatically.

## Evidence and limits

Every session creates its own directory with timestamped JSONL request/result/observation events and full JPEGs. MCP returns both images; CLI omits base64 data and prints their absolute paths. `scripts/session_mcp_smoke.py` exercises real stdio initialization, tool listing, HTTP forwarding and image packaging.

Synthetic tests cover state progression, stale assessments, duplicate requests, concurrent writers, stop, retry budget, camera errors and restart. Mock camera views are synthetic illustrations, not realistic wrist geometry or evidence for grasp physics. Codex still supplies visual assessments. Geometry checks cover joint limits, table/centerline clearance and bounded interpolation, not full environment collision checking. Moving the cover invalidates the taught destination operationally; automatic destination tracking is a later feature.

## Explicit held-object recovery

Only after a real operator instructs continuation, `act(action="resume_holding", params={"operator_confirmed":true,"evidence":"..."}, scene_clear=true)` may rearm a stopped held-object session. After process restart also supply `target_id`. It checks three fresh stable telemetry samples and geometry, sends no motor command, and enters awaiting_lift for a new held assessment from returned images. A newer stop wins. One recovery per session; adoption after restart conservatively sets the grasp attempt count to two. Never use this as automatic fault recovery.

A dedicated Astra operator owns live commands while the parent agent builds/tests. Do not run source-reloading servers or restart the hardware service during an operator action. The [local alignment routine](VISUAL_ALIGNMENT.md) adds neutral preparation, small measured probes and bounded image corrections. It validates hover alignment, not grasp geometry.

Arm command cadence defaults to 20 Hz; `command_hz: 10` halves arm goal updates while retaining the same nominal speed, following checks and 500 ms telemetry freshness cutoff. The live config selects 10 Hz as an experiment after transient stale telemetry during motion; serial saturation has not been established as the cause. Gripper cadence remains unchanged.

The standard [test lifecycle](REST.md) starts at folded rest and ends with `park`. Preparation returns to rest before moving to ready hover; configured-rest sessions require preparation before probing. Folding/unfolding uses longer execution segments while validating the entire path at fine intervals.
