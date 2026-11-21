# SO-101 tabletop cleanup

Codex identifies crumpled paper or empty snack wrappers in camera images. The harness grounds a grasp pixel to a measured table position, plans with the SO-101 model, executes bounded Station commands, and returns an after-image for Codex to verify. Targets may be anywhere inside the calibrated reachable workspace; there are no fixed pickup slots.

## Run offline

```bash
uv run --frozen python scripts/cleanup_demo.py
uv run --frozen so101-cleanup serve --sim
```

The demo exercises two synthetic objects and writes images/events under logs/cleanup-demo/. Fixture labels, pixels and simulator truth stand in for Codex vision. It tests orchestration/kinematics, not recognition or grasp physics.

With the service running, use another terminal:

```bash
uv run --frozen so101-cleanup call status
uv run --frozen so101-cleanup call get_observation
```

Open the returned absolute image_path with Codex's image-viewing tool. Follow [CODEX.md](CODEX.md) for the agent loop and JSON examples.

| Tool | Effect |
| --- | --- |
| status | Read robot state, tool XYZ, mode and verified count |
| get_observation | Fresh JPEG, timestamp, state and observation id |
| plan_pick | Pixel grounding and full checked pickup/drop path; no motion |
| execute_plan | Execute one fresh, single-use plan; return after-image |
| verify | Record visual evidence; count only source clear AND item in tray |
| stop | Interrupt every remaining stage and hold position; retain torque |

The same tools are exposed through stdio MCP:

```bash
uv run --frozen so101-cleanup-mcp
```

The MCP shim connects to http://127.0.0.1:8771 and returns native images. Override the URL with SO101_CLEANUP_URL. CLI tools work directly in this Codex workspace without registering MCP or obtaining another model API key.

## Hardware setup when connected

1. Run so101-station discover/status to check the exact bus, calibration and physical joint directions.
2. Mount a fixed overhead/external camera. The existing scripts/probe.py lists Station camera queues/identities. Keep resolution, crop, lens and camera position unchanged.
3. Copy examples/so101-cleanup.workspace.example.json. Measure at least six image-pixel/table-coordinate pairs spread across the area, in the robot URDF base_link frame (metres). Supply actual image dimensions, a convex workspace polygon within the measurements, a drop point, and table/tray heights. The empty template deliberately cannot execute.
4. Run so101-cleanup check-workspace YOUR_CONFIG.json. Validate separate held-out physical points: a small fitting residual alone is not proof of accuracy.
5. Preposition the supported arm in a clear downward-facing pose over the workspace near travel height. The planner refuses invalid/folded starting poses.
6. Start with motion disabled and inspect observations/plans:

```bash
uv run so101-cleanup serve --workspace YOUR_CONFIG.json \
  --station STATION_HOST:8888 --bus-serial SO101_BUS --camera-queue CAMERA_QUEUE
```

Add --enable-motion for the bench run with a clear work area and an operator at the physical stop/power control. Disable other teleop/policy writers. The service owns one bus, takes a local per-host/bus lock, and binds only to loopback.

Arm velocity is capped at 0.15 rad/s; segments are capped at 0.27 rad; a routine times out after 180 seconds. Stale state, wrong calibration, unreachable targets, changed images, lost tracking or stop prevent subsequent stages. Software hold can fail if the Station link is lost. No EEPROM offsets, PIDs or ElRobot poses are applied.

## Physical limits

A fixed camera and measured planar map are required. Select the object's table-contact footprint; the map does not infer depth, object height or a six-dimensional grasp. Grasp height is configured. Flat wrappers without a pinchable edge may not be graspable. The five-axis model uses downward approach with free jaw yaw and rejects unreachable combinations.

Clearance uses the table and conservative arm centerlines, not full meshes or scene obstacle reconstruction. Codex must inspect the visible corridor for people/obstacles; bench validation remains necessary. Gripper contact is an inference, not proof of a grasp. Only explicit after-image verification counts a cleanup.

Hardware trials now include a wrist round trip, one visually verified paper pickup using taught poses and Codex-selected corrections, and folded parking followed by torque-off. The generic mapped cleanup service has not been validated end-to-end on hardware, and the wrapper remains unpicked. See the [dynamic loop plan](workstreams/WS-2026-11-21-dynamic-loop.md) for the proposed next iterations.

[Architecture](ARCHITECTURE.md) · [Contracts](DATA_MODEL.md) · [Tests](EVALS.md) · [Handoff](HANDOFF.md) · [Calibration](../SO101.md)

New live session tools and operating commands: [SESSION.md](SESSION.md). Current physical state and exclusive robot ownership: [HANDOFF.md](HANDOFF.md). Final validation: 59 tests + 15 subtests, real MCP image smoke check, and one verified paper delivery with explicit fault recovery.
