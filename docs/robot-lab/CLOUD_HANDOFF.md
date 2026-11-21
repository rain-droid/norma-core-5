# Cloud continuation — 21 November 2026

## Objective

Finish a Codex-controlled MuJoCo demo: from fresh camera images, identify crumpled paper and an empty snack wrapper, place only those in a bin, leave a computer mouse and pen untouched, and return to folded rest. Keep the shared policy and executor working with both SO-101 and ElRobot profiles. Show decisions and execution in the existing web lab.

This work is simulation only. The teammate owns the physical arm. Do not start a hardware motion service, connect to the real Station, or reuse synthetic simulator servo offsets as hardware calibration.

## Transfer status

- Repository: `https://github.com/rain-droid/norma-core-5.git`.
- Dedicated branch: `codex/so101-station`; checkpoint parent: `ba71868c0ec11e5483fa680bc9ef51cc19990f45`. Use this branch rather than `main`.
- This checkpoint includes `src/station_robotics`, the preceding `src/so101_station` harness, `assets/so101_pgripper`, tests, documentation, scripts and the simulation dependency lockfile.
- The account exposes a runnable default Codex Cloud environment, with no repository listed. No cloud task or remote deployment has been started.
- Clone the pushed branch with robot assets, upstream licenses, vendored Station messages, profiles, scripts, tests, `pyproject.toml` and `uv.lock` intact. `.env`, `.venv`, local recordings and runtime logs are excluded. Regenerate test artifacts in the cloud. Do not change the teammate's separate checkout.
- This chat cannot move itself using the available handoff tool; that tool also does not support cloud handoff. A separate cloud task needs the code and this handoff.

## Runtime

Python 3.12, MuJoCo 4.0.1 (pinned in the `simulation` extra), upstream Station Python client/protobufs vendored in the repo. Codex itself interprets the images; there is no second model API or API-key requirement for this loop.

Linux CPU rendering should use MuJoCo's OSMesa backend; a GPU is optional. The installed MuJoCo package includes that backend, but this checkout has only been tested on macOS so far. Validate Linux rendering before claiming cloud readiness.

Suggested Debian/Ubuntu setup, after the correct branch is present:

```sh
# For an environment without the repository already checked out:
git clone --branch codex/so101-station --single-branch https://github.com/rain-droid/norma-core-5.git
cd norma-core-5

# Install these system packages as the environment administrator:
apt-get update
apt-get install -y libosmesa6 libgl1 libglib2.0-0

uv sync --locked --extra simulation --python 3.12
uv pip install pytest

# Configure these in the cloud environment, or export in each execution shell.
export MUJOCO_GL=osmesa
export PYOPENGL_PLATFORM=osmesa

.venv/bin/python -m pytest tests/test_station_cleanup.py tests/test_station_robotics.py -q
.venv/bin/python scripts/robot_lab_cleanup_smoke.py
.venv/bin/python -m station_robotics.app --port 8785 --station-port 8890
```

The app and simulator Station endpoint bind to loopback. Use the cloud environment's authenticated preview/tunnel for the UI if available; do not expose the motion endpoint publicly. A rendered image/clip is sufficient if no preview is available. Current Codex Cloud documentation lists computer use as unsupported; use available browser testing tools if provided, or report visual UI validation as pending. See [Cloud environments](https://learn.chatgpt.com/docs/environments/cloud-environments).

## Current evidence

The complete offline suite passes: **246 tests and 15 subtests** (`python -m pytest tests -q`). This includes the 15 targeted simulator/Station tests. Simulator UI JavaScript also passes Node's syntax check. The actual upstream Station client drove both simulator profiles over local TCP in `scripts/robot_lab_cleanup_smoke.py`:

| Profile | Acknowledged goal packets | Paper and wrapper in bin | Both grasps held for 20 simulated seconds | Largest protected-object displacement |
| --- | ---: | --- | --- | ---: |
| SO-101 | 405 | yes | yes | 0.00098 mm |
| ElRobot | 405 | yes | yes | 0.00098 mm |

These are deterministic regression trials with explicit image-annotation fixtures in `examples/robot-lab-cleanup-observations.json`. They establish contact/transport behavior for these scenes, not general-purpose visual recognition. The independent evaluator uses body positions only for test scoring, never for the production agent's observations or plans.

An earlier interactive Codex run correctly identified the objects but a paper grasp slipped during a long pause. The agent rejected that grasp from images. Subsequent contact tuning passed the sustained-hold trials above; a fresh interactive image-driven run is still required.

## Architecture and limits

- `cleanup_cli.py` and `cleanup_mcp.py`: image observation, classification, execution and verification tools. Read `CODEX_CLEANUP.md` for exact usage.
- `cleanup_policy.py`: accepts only confident paper/wrapper trash labels; mouse, pen and uncertain objects stay. This validates supplied labels; it cannot guarantee the vision model never mislabels an object.
- `cleanup_world.py`: shared bounded task sequence, RGB target refinement, inverse kinematics, fresh grasp/final image verification and folded rest.
- `station_protocol.py`: actual upstream Station client and protobufs talking to a simulator-only subset of Station's queue protocol on port 8890. This is not the full Station service and has not been accepted against physical hardware.
- `profiles.py`: robot-specific geometry, joint names/limits and folded rest. Common controller handles both joint counts.
- `cleanup_scene.py` / `model.py`: MuJoCo rigid props and contact grasps. Paper/foil are rigid approximations. No weld, attachment or teleport is used during actions. Elliptic friction, impedance ratio 10 and three no-slip iterations stabilize holds; these are simulation parameters, not measured hardware parameters.
- A failed or uncertain grasp blocks for fresh observation. There is no general autonomous recovery planner or full swept-path collision planner yet.

## Next work, in order

1. Validate headless Linux rendering and rerun the two-profile Station acceptance script. Preserve before/after and held-object images.
2. Restart the lab on current code. The old local process was started before the latest contact and Station-camera changes and is not evidence for them.
3. Exercise the real CLI flow from `CODEX_CLEANUP.md`: freshly observe, visually identify all objects, submit annotations, execute, inspect wrist/overhead images at each `verify_grasp`, approve only visible holds, inspect the final scene, and assess the result. Do not substitute the regression fixtures or hidden physics truth for Codex's visual judgment.
4. Test the stdio MCP server end to end, including native images. Recent app changes route tool images through the upstream Station camera queue; that app path still needs interactive validation.
5. Check the updated cleanup UI (task selector, decisions, acknowledged packets, stage display, pause/reset) in a browser. Previous browser validation covered alignment, not the new cleanup UI.
6. Add meaningful regressions for stale grasp evidence and failed final assessment. Try shuffled layouts with fresh observations. Expand recovery only after a baseline visual loop works.
7. Record actual successes, failures and remaining limitations in `VALIDATION.md`; provide a short demo capture. Keep physical integration as a separately supervised task.

## Suggested cloud task prompt

Read `docs/robot-lab/CLOUD_HANDOFF.md` and `docs/robot-lab/CODEX_CLEANUP.md`. Continue the simulation-only trash-cleanup demo on the transferred branch. First validate headless rendering and existing tests, then complete fresh Codex image-driven cleanup, MCP verification and visual UI validation for both robot profiles. Use the actual Station client path. Never use test oracle labels or coordinates as perception. Do not contact physical hardware. Save reproducible evidence and clearly report what remains unverified.
