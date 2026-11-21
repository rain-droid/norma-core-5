# MuJoCo validation — 21 November 2026

Seven model/controller/API regressions passed. Six rendered feedback trials and two rendered single-move comparisons passed on the two profiles. The UI was opened in Brave and both profiles were exercised.

| Robot | Trial | Final image error | Corrections |
| --- | --- | ---: | ---: |
| so101 | 0 | 2.61 px | 5 |
| so101 | 1 | 1.73 px | 5 |
| so101 | 2 | 1.66 px | 5 |
| so101 | single_move | 55.97 px | 0 |
| elrobot | 0 | 3.94 px | 4 |
| elrobot | 1 | 2.74 px | 4 |
| elrobot | 2 | 3.04 px | 4 |
| elrobot | single_move | 42.70 px | 0 |

Physics acceptance checks both model FK agreement and stable holding/object contact. Pause/resume preserves commanded goals and excludes teaching pauses from the next motion-observation deadline. Simulation-only acceptance does not establish physical robot calibration, object recognition, grasping or live Station compatibility.


## Recorded-camera update

Nine unit/contract checks pass, including recorded-scene identity checks, projection round trip at 853×480, missing recording behavior and bounded frame routes. The stored bench camera fit has six manually annotated landmarks and no held-out validation; its ~5.4 px residual is descriptive, not a hardware calibration claim.

Six additional rendered camera-variation trials passed. SO-101 used the saved-bench estimate; ElRobot used its own synthetic baseline. For each robot, the three wrist configurations were nominal, +5° tilt/+10% FOV, and −5° tilt/−10% FOV. Each run learned a fresh response. Final errors: SO-101 **2.30 / 2.17 / 2.53 px**; ElRobot **3.95 / 3.25 / 2.83 px**. Evidence: `logs/robot-lab/camera-match/results.json` plus per-run JSON and wrist images. These checks cover static camera geometry changes, not mid-run camera movement, real object detection or successful grasps.

## Mixed-object cleanup and Station transport

`tests/test_station_cleanup.py` and `tests/test_station_robotics.py`: **15 passed**. `scripts/robot_lab_cleanup_smoke.py` passed for both profiles using the upstream Station client over local TCP, with 405 acknowledged goal packets per profile. Both paper and wrapper reached the bin; both grasps survived a 20-second simulated inspection pause. Maximum mouse/pen displacement was approximately 0.00098 mm, below the 2 mm test threshold. Both arms returned to folded rest and awaited final visual assessment.

These trials use explicit image annotations as regression fixtures and independent physics scoring. They do not establish general visual recognition or physical-hardware compatibility. An earlier interactive grasp slipped and was correctly rejected from camera images; the fresh image-driven CLI/MCP demo still needs to be repeated after the contact changes. Artifacts are under `logs/robot-lab/cleanup/acceptance/`. Cloud continuation details and remaining checks are in `CLOUD_HANDOFF.md`.

Before the branch checkpoint, the full offline suite passed: **246 tests and 15 subtests** in 71 seconds (`python -m pytest tests -q`). The simulator UI script passed `node --check`, and the staged diff passed `git diff --cached --check`. No physical robot was contacted by these checks.
