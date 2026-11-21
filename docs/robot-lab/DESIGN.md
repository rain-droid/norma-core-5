# Station Robot Lab

Visual thesis: a calm charcoal control surface around a large, warm MuJoCo camera render, with mint overlays explaining observed error and the next correction.

Content plan: primary physics view; wrist observation and target reference; run/pause/step controls; compact error history and event explanation. Profile selection switches the robot, not the algorithm.

Interaction thesis: motion comes from MuJoCo physics and camera frames; the target/error overlay follows observations; a step button pauses at each observation so the correction can be inspected.

## Architecture and scope

All new control and UI code lives in `station_robotics`. Robot-specific geometry, limits, joint count, gripper and camera placement belong to profiles. SO-101 and ElRobot share the controller, physics backend and HTTP interface. Existing hardware packages remain available as model/transport adapters; this app never instantiates a Station connection.

First acceptance is visual XY alignment at clearance height, including injected coarse-position error and command deadband. Simulated objects are rigid proxies. Paper/wrapper deformation, physical grasp reliability, camera calibration and actuator parameters are not claimed to match hardware. Simulated alignment success must not be reported as a real pickup.

No robot service may be started while the teammate owns the hardware. A simulation launch has no hardware switch or Station credentials.

## Mixed-object cleanup extension

The main physics view remains the workspace. A task selector switches between alignment experiments and cleanup. Cleanup shows image-grounded decisions in a plain list: pick trash, preserve belongings, destination. Camera overlays and the event sequence make selection, motion, and verification visible. Objects are actual rigid MuJoCo bodies, including paper, packaging, a mouse, a pen and an open bin.

Codex is the semantic vision agent through MCP/CLI image tools; no external model API is introduced. Object identities in the physics evaluator are not returned to the agent. Selected pixels and observations are required; the controller never chooses trash from simulator body names. The shared executor sends generated ST3215 goal messages through the upstream Station Python client to a localhost simulator endpoint implementing the required Station queue-protocol subset. Hardware remains disconnected; physical Station integration needs real per-robot tick bindings and hardware acceptance.
