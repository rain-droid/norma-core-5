# Codex cleanup loop

This demo uses Codex's own image understanding through MCP or shell tools. It does not call a second model API or require an API key. The physical robot remains disconnected.

## Start

```sh
.venv/bin/python -m station_robotics.app --port 8785 --station-port 8890
.venv/bin/python -m station_robotics.cleanup_cli prepare --robot so101
# The other profile uses the same tools:
# .venv/bin/python -m station_robotics.cleanup_cli prepare --robot elrobot
```

Port 8785 is the demo/tool server. Port 8890 is the **simulator's Station-compatible queue endpoint**. It implements setup/ping, latest/follow motor and camera reads, and ST3215 goal-position RAM writes using the vendored protobuf messages. It is not a full Station service. Cleanup motor commands use the upstream Station Python client over TCP to this endpoint. Nothing forwards to the team's physical Station on port 8888.

## Image-first task

User instruction: **“Pick up trash and put it in the bin.”**

1. Run `python -m station_robotics.cleanup_cli observe` and inspect the returned `image_path` with the image tool. The frame has no oracle object labels.
2. Identify all objects from their appearance. Only clearly **crumpled paper** and **empty snack wrapper** are eligible trash. A **computer mouse**, **pen**, and unknown/uncertain objects stay. Identify the open bin separately. Do not derive labels from colors, body names, object order or a previous scene.
3. Write a JSON array of observations. Each needs `label`, `decision` (`trash`, `keep`, `unknown`, `bin`), `confidence`, `bbox: [x,y,width,height]` in image pixels, and estimated `height_mm`. At least 0.9 confidence is required for trash and the bin. A bounding-box center is a first grasp hypothesis, not exact object geometry. Optional `jaw_yaw_deg` describes the jaw closing axis in the calibrated robot base frame; omit if uncertain.
4. Submit with `classify --observation-id ID --file observations.json`. The UI displays the decisions. Overlapping boxes, stale observations, protected labels and uncertain trash are rejected.
5. Run `execute`, then `status`. At `verify_grasp`, run `verify_grasp` and inspect both returned image paths. Only if the intended trash is visibly held above the table, run `assess_grasp --observation-id ID --held`. Otherwise omit `--held` to reject the grasp. If blocked, inspect the cause and reacquire; do not repeat blindly. Repeat this check for each object.
6. At `verify`, run `verify` and inspect the final image. Use `assess --observation-id ID --trash-in-bin --protected-untouched` **only after actually seeing both conditions**. Otherwise submit `assess` without the unverified flags.

Use `.venv/bin/python -m station_robotics.cleanup_cli` as the prefix for every command. `stop` holds the measured simulated pose and invalidates the plan. The UI's **Shuffle scene** resets the simulation and moves props; it always requires a new observation and selection.

## MCP

Run `.venv/bin/python -m station_robotics.cleanup_mcp` as a stdio MCP server. It exposes `cleanup_observe`, `cleanup_classify`, `cleanup_execute`, `cleanup_status`, `cleanup_stop`, `cleanup_verify_grasp`, `cleanup_assess_grasp`, `cleanup_verify` and `cleanup_assess`, returning native images to Codex. This repo also provides the CLI, so the active chat can use the tools immediately without restarting MCP configuration.

Example server configuration (merge with your existing servers; do not replace ElRobot's server):

```json
{
  "mcpServers": {
    "station-cleanup": {
      "command": "/absolute/path/to/repo/.venv/bin/python",
      "args": ["-m", "station_robotics.cleanup_mcp"]
    }
  }
}
```

## Scope

Objects are rigid approximations: no paper/foil deformation. Gripper gains, torque limiting and friction are simulation parameters. No object is teleported, attached, welded or made kinematic after initialization. The independent physics evaluator can measure trash-in-bin and protected-object displacement, but that information is excluded from agent observations and motion decisions.

The same executor and Station wire format serve both robot profiles. Real hardware additionally needs measured per-joint tick bindings, camera/workspace registration and its existing motion safeguards. **The synthetic servo zero offsets in this demo must never be used as hardware calibration.** No physical-hardware launch switch is exposed.
