# Two arms on one table

The ElRobot shares its table with a second arm: an SO-101 with the same parallel gripper and wrist camera
mount, standing 29 cm to its right. This page says how to start it, what the tools do, and how far each part
is proven. Reasons are in `docs/DECISIONS.md`; the history is in `docs/CHANGELOG.md`.

## Start

Both daemons belong in terminals of your own. Started from an agent session they are ended after two hours.

```bash
scripts/cameras_ready.sh && pixi run up     # Station, the ElRobot's daemon, web UI on :8765
scripts/second_arm_up.sh                    # the SO-101's daemon on :8770 (needs the Station running)
```

`scripts/second_arm_up.sh` checks out the `autopilot` branch of this repository beside it
(`../norma-core-5-so101`) and runs that branch's daemon with its SO-101 profile. Nothing of that branch is
merged here. `config.yaml` names the ElRobot's servo bus (`bus_serial`), since the Station now has two.

Once per machine, for the depth camera: `scripts/orbbec_setup.sh` (builds Orbbec's SDK into `bin/orbbec/`
and lists the two root-only settings it needs).

## Calibrate, in this order

| What | Tool | Needs | Writes |
|---|---|---|---|
| Depth camera in the ElRobot's frame | `calibrate_depth` | two mat tags in the `across` view | `robot_config/camera_depth.json` |
| Where the SO-101 stands | `calibrate_second_arm` | a person hands the SO-101 the pink block | `robot_config/second_arm.json` |
| The hand-over, shown by hand (optional) | `record_poses` | both arms limp, a person moves them | `state/two_arm/hand_over.json` |

Run them again when a camera, the mat or an arm has been moved.

## Tools

| Tool | What it does |
|---|---|
| `depth_scan` | heights of everything on the mat, and at given spots |
| `scan_obstacles` | keeps the height map as obstacles for both arms, for two minutes |
| `arms_clearance` | the smallest gap between the two arms right now |
| `second_arm` | the SO-101's `home`, `rest`, `move_to`, `move_joints`, `gripper`, `grasp`; positions in the ElRobot's frame |
| `hand_over` | the ElRobot passes what it holds to the SO-101 in the air |
| `teach_hand_over` | one hand-over adjusted by pushing the ElRobot |
| `record_poses`, `replay_poses` | poses shown with both arms limp, kept and played back |
| `second_arm_stack` | the SO-101 picks a thing off the mat and sets it on another |

## One space for both arms

`workspace/tools/util/shared_space.py` holds both arms in the ElRobot's base frame, each as six capsules (base,
upper arm, forearm, wrist, gripper, wrist camera), and the things on the table as the depth camera's height
map. Before a move is sent, its joint path is walked against the other arm where it stands and against the
map:

- a path that brings any part within 2 cm of the other arm is refused, with the parts named;
- a path that dips a part into something on the table is refused, with the thing's height and place;
- around where a move starts and ends the things on the table do not count, and at a hand-over the two hands
  may meet.

This covers every joint command of the ElRobot and the SO-101's `home`, `move_to` and `move_joints`. When
something stands on the way, `pick_at`, `place_at` and the SO-101's reach go up, across above the tallest
thing on the line, and down. The web UI's 3D view shows both arms, the scanned things as columns and the path
about to be taken.

What it does not know: what a gripper holds, an arm while it is moving (one arm moves at a time), the SO-101
daemon's own `pick`, `place` and gestures, and anything the depth camera does not see. It checks paths; apart
from the lift-over rule it does not search for another one.

## How far it is proven (real arms, 2026-11-22)

| Part | Result |
|---|---|
| Depth camera | tags fitted to 0.6 mm; a 30 mm cube read 29 mm; positions within about 1 cm of the top camera's |
| SO-101 calibration | six places fitted to 4 to 8 mm, in the air with level jaws; four more left out as over 15 mm off |
| Collision check, arm against arm | a move of the ElRobot at the SO-101's gripper was refused; a full pick and hand-over cycle went through |
| Obstacles and lift-over | a carry through a 47 mm block was refused, then went over it; the SO-101 went over the ring |
| Hand-over of a 3 cm cube, unattended | 0 of 3 before the SO-101's height was corrected; then 2 of 2 computed, 1 of 2 from one adjusted pose, 4 of 6 from poses shown by hand |
| Hand-over of the pink block | not possible: the ElRobot's wedge fingers cover it |
| Stacking by the SO-101 | not achieved: it picked the cube 2 times in 4 and set it beside the base both times |

## Known limits

- **The SO-101 is 2.5 to 4.5 cm off at the table.** Its place was fitted in the air; pointing down at the
  mat its joints put it somewhere else. `second_arm_stack` corrects with the top camera, which sees only a
  sliver of the gripper under its own camera and wanders by a centimetre. A fit at table level (set a cube down
  at five spots and measure, as was done for the ElRobot) or a tag on its camera mount would fix this. Until
  then the collision check is equally off for the SO-101 near the table.
- **The hand-over works about two times in three.** The jaws have 11 mm to spare each side of a 3 cm cube;
  the calibration (4 to 8 mm), where the ElRobot settles (2 mm, once 17) and how the cube sits in its fingers
  (about 10 mm) add up to that. `hand_over` retries by offering the cube 1 cm further, up to 3 cm.
- **The SO-101 stops 8 to 18 mm short when it rises** and is corrected from its own joint readings.
- **An obstacle map is two minutes old at most**, by design: the arms move the things it shows.
