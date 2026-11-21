# Station Robot Lab — MuJoCo

An offline visual-alignment demo using MuJoCo 4.0.1 physics and rendering. Open **http://127.0.0.1:8785**. The browser displays JPEG frames produced by MuJoCo; it contains no Three.js scene or custom physics engine.

```sh
uv pip install --python .venv/bin/python -e '.[simulation]'
.venv/bin/python -m station_robotics.app --robot so101 --port 8785
# Or start with the team's other robot:
.venv/bin/python -m station_robotics.app --robot elrobot --port 8785
```

Run from this repository checkout. macOS must allow the process graphics access for offscreen OpenGL rendering. An `invalid CoreGraphics connection` error indicates the process lacks that access. The native MuJoCo viewer is not needed.

## What to demonstrate

1. Choose **SO-101** or **ElRobot**. Both use the same visual controller and interface.
2. Press **Run alignment**, or **Step** to pause after each observation/decision. Pause lets the current motor command finish and holds its target; it does not freeze physics.
3. Watch the wrist-image box, reference cross and error arrow. The chart records actual camera observations.
4. Compare **Single move only** against **Camera feedback** with the same injected calibration error. Changing settings resets the run.
5. Use **Move target**, or click near the object in the overhead view, then repeat. The first test region is local, within 45 mm of the default hover.
6. Export the run JSON for the observed pixels, commanded offsets, joint residuals, response matrix and events.

The experiment is hover alignment. It does not close the gripper, perform a simulated pickup, or claim cleanup success.

## Algorithm

Overhead RGB color detection gives a rough table point using the simulated camera calibration. A configurable XY bias represents a coarse localization/calibration error. The arm moves to hover near that point. Two sequential 4 mm commands measure the wrist-image response. Corrections accumulate from one fixed commanded joint origin. The controller waits for low motion, observes a new RGB frame, and computes a bounded pixel-error correction. Two fresh frames within 4 pixels establish simulated alignment. Joint equality is not the success signal.

The RGB detector finds the orange rigid trash proxy by color. It never receives MuJoCo object IDs or object coordinates. The reference cross uses the simulated tool and camera calibration at an assumed object-plane height; it is not a calibrated physical grasp reference. The response matrix is learned from rendered observations, not copied from the physics engine Jacobian.

## Robot compatibility

| Layer | Shared | Profile-specific |
| --- | --- | --- |
| Visual policy | Response estimation, bounded corrections, convergence | None |
| MuJoCo runtime | Physics stepping, RGB rendering, perception, events | URDF and model parameters |
| Interface | Same HTTP actions, state, camera frames and UI | Robot label and capabilities |
| Geometry | Common description-to-MJCF converter | Joint count/names/limits, gripper, tool transform, FK/IK adapter |

SO-101 has five arm joints plus its parallel-gripper motor. ElRobot has seven arm joints plus its parallel-gripper motor. No shared controller loop assumes either joint count. Original CAD/inertias/mimic relationships are imported; FK agreement is regression-tested against each existing model.

The existing real Station controllers remain separate adapters in `so101_station` and `elrobot`. They have **not** been replaced or connected by this demo. Hardware transfer of this new visual policy still needs each controller's command/observation integration and live acceptance; passing simulation is not a claim of complete hardware compatibility. Station unifies transport, not geometry, calibration or camera registration.

## Physics assumptions

- MuJoCo integrates position-actuated joints, gravity, inertia and floor/object contact. Robot joint positions are initialized only at reset; normal movement changes actuator targets and steps physics.
- Position gains, friction and the optional command deadband are simulation assumptions, not identified actuator parameters. The injected deadband holds an existing servo target until a command exceeds the selected tick difference; it is not a complete backlash model.
- Robot CAD meshes use convex collision hulls against the table/object. Robot self-collision is disabled because the source CAD includes overlapping assemblies; this demo must not be used to certify collision-free hardware trajectories.
- The orange trash proxy is rigid. A flat underside approximates its table contact; visual ellipsoids do not represent deformable paper or packaging.
- Baseline cameras are synthetic tool-relative/world cameras. The optional recorded bench estimate matches saved image geometry approximately; neither setup establishes physical camera calibration.
- ElRobot's upstream zero gear inertia is replaced with a positive `1e-8` numerical floor. Source licenses and the SO-101 download hashes are retained with the assets.

## Validation and endpoints

```sh
.venv/bin/python -m pytest -q tests/test_station_robotics.py
.venv/bin/python scripts/robot_lab_smoke.py
```

The smoke script renders six image-feedback trials (three target placements per robot) and two single-move comparisons; results and images are in `logs/robot-lab/acceptance/`. Initial acceptance passed all six with final errors 1.66–3.94 px; both single-move comparisons retained errors above 4 px. This is a small deterministic simulation check, not a reliability or sim-to-real benchmark.

Endpoints: `GET /api/state`, `/api/capabilities`, `/api/frame/{overview|overhead|wrist}`; `POST /api/action` with `command: run|step|pause|reset|relocate`; `POST /api/profile/{so101|elrobot}`; `POST /api/settings` with controller mode and disturbances. They are localhost simulation operations only. Unknown hardware profiles/actions are rejected.

No physical Station socket, serial device, motor service, real camera or hardware mode is used. Cleanup uses a separate localhost Station-compatible simulation socket. The teammate retains ownership of the physical robot.

## Match the saved bench views

Choose **SO-101 → Recorded bench estimate → Compare saved view**. Select the overhead or wrist tab, then drag the comparison slider to reveal the recorded photograph versus MuJoCo. The comparison is static and renders the saved joint pose using a separate physics-data object; it does not move the active simulated arm. **Back to live physics** returns to the running experiment with the same estimated camera rig.

The setup is stored in `src/station_robotics/scenes/recorded_bench.json`, separate from the visual policy. It uses the saved **853×480** image size, an oblique overhead camera estimated from six manually identified CAD joint centers, and the wrist camera's nominal CAD optical frame. The overhead fit has an approximately **5.4 px training residual**, with **no held-out validation**. Centered principal point, square pixels and a weak FOV prior are assumptions. The wrist FOV is estimated. Lens distortion is unknown and not fitted. These are simulation settings, **not** camera/hand-eye calibration and never accepted as hardware calibration.

The table color, transparent-cover rim and wrapper footprint approximate the recording. The orange target remains the rigid color-detection proxy. Background clutter, lighting, flexible objects, lens distortion and exact camera placement are not reproduced. Compare views before drawing conclusions about perception performance.

Saved reference images stay in the local ignored `logs/current-preflight/` folder. A fresh checkout needs those recordings for photo comparison; they are not embedded in the package. The scene's robot identity prevents accidentally assigning this SO-101 recording to ElRobot. ElRobot uses the same scene/controller mechanism but needs its own camera evidence.

```sh
.venv/bin/python scripts/fit_robot_lab_camera.py
.venv/bin/python scripts/robot_lab_camera_smoke.py
```

The first script reproduces the approximate camera fit from the stored manual landmarks. The second renders the recorded SO-101 setup and baseline ElRobot setup with nominal cameras and ±5° wrist camera tilt paired with ±10% vertical FOV changes. The policy learns a new image response for each run. This tests static camera variation, not a camera bumped midway through motion.

A fixed angle difference is a **sim-to-real geometry mismatch**; a bumped camera or moving mount causes **drift**. Learning the image response locally helps with the first, but cannot establish a correct grasp reference, recover an unseen target, or correct an obsolete camera mapping after a mount moves. Real transfer still needs a fresh coarse table mapping and checked grasp reference; invalidate/relearn the response after camera, hover-height or orientation changes. OpenCV describes [pose fitting from 3D–2D correspondences](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html); MuJoCo supports [camera geometry and intrinsics](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera).

Additional offline endpoints: `POST /api/scene/{baseline|recorded_bench}`, `GET /api/recording/{overhead|wrist}`, and `GET /api/match/{overhead|wrist}`. Recording routes serve only the allowlisted image files in the selected setup.


## Mixed-object cleanup through Station

The **Trash cleanup** task adds rigid paper and packaging, a protected mouse and pen, and an open bin. Codex inspects an unlabelled camera frame, submits the trash/protected/bin decisions, and runs the sequence. Fresh grasp images require Codex verification before carrying; a final image requires verification before success is recorded. See [CODEX_CLEANUP.md](CODEX_CLEANUP.md) for the native-image MCP server and immediately usable CLI.

Cleanup motion uses the vendored upstream Station client and generated ST3215 messages over TCP to a **local simulator endpoint** on port 8890. This is a deliberately limited emulation of Station's queue API, not a claim that the full Station service runs inside MuJoCo. The real Station on port 8888 is not contacted. The hardware bridge still needs each real arm's calibrated tick binding, camera registration and existing safety checks before live acceptance. The simulation's per-joint zero offsets are never hardware calibration.

Physical grasps use contact/friction with simplified flat finger pads instead of whole-jaw convex CAD hulls. Gripper torque, damping and contact parameters are simulation assumptions. Paper and packaging remain rigid. No weld, object attachment or object teleport is used to fake a grasp.
