# The body

`model.urdf` and `assets/` are the robot model (meshes in millimetres, scaled in the URDF).
To see the current posture, run `pixi run python tools/read_proprioception.py` and open the
`body_image` it returns: the model at the measured angles from behind, from the right, from above.

## Frames

Base frame: origin on the table under the base, **x right, y forward, z up**, seen from behind
the robot. The tool point (TCP) is between the finger pads, 12 mm back from the fingertips.
`pitch` is how far below horizontal the gripper points (90 = straight down). On the table the
arm reaches a half-disc of about 0.38 m; straight-down poses exist within about 0.27 m.

## Joints: the one hard contract

The robot takes joint commands and nothing else. A command is one or more waypoints of seven
absolute angles in radians (servos 1..7), and/or a jaw opening 0..1. `tools/joint_command.py`
sends exactly that; `control.py` is the same thing from Python; `tools/util/arm.py` is the
few lines that carry it, and says what the robot side does with it. Every other tool is a way
of working out which joint angles to send.

| Servo | Meaning | URDF limits (deg) | +10 deg from the ready pose moves the tool point |
|---|---|---|---|
| 1 | base rotation, about vertical | -89 .. 89 | 27 mm to the left (-x) |
| 2 | shoulder | -92 .. 92 | 28 mm down, tilts the gripper down |
| 3 | lower elbow | -101 .. 101 | 32 mm down, 11 mm back |
| 4 | upper elbow | -100 .. 100 | 32 mm down, 20 mm back |
| 5 | forearm roll | -150 .. 186 | 25 mm to the right (+x), swings the gripper sideways |
| 6 | wrist bend | -79 .. 101 | 23 mm back, 11 mm down |
| 7 | wrist roll | -183 .. 157 | nothing: turns the jaws about the pointing direction |
| 8 | gripper | 0 closed .. 1 open | jaws open 53 mm |

Ready pose (`tools/home.py`), degrees: `[0, -21, -29, 36, 0, 73.5, 0]`. The limits that apply
on this arm are the calibrated ones in `perception/proprioception/state.json`
(`lower_rad`/`upper_rad`), never wider than the table above. Joints 2, 3, 4 and 6 all pitch the
arm in the same vertical plane, which is why one tool position has many joint solutions.

## Cameras

`top` is fixed across the table from the robot, about 39 cm above the mat, looking back at
the arm, so the robot's right is on the left of its image. It is the camera to measure with.
`camera_top.json` holds its calibration, written by `tools/calibrate_top.py` from the four
AprilTags at the mat's corners: the mapping from a pixel to the mat surface, where the camera
stands (which extends that to points at a known height), and `base_in_mat`, where the robot's
base sits on the mat. That last one was tuned by eye; if picks land consistently off in one
direction, it is the number to correct. In simulation the file is `camera_top.sim.json`,
written exact by the simulator.

`wrist` rides on the gripper and looks along it; the black shapes at the bottom of its image
are the jaws. Its model (lens and mounting) is `camera_wrist.json` if someone has calibrated
it, otherwise a rough default that reads about 3 cm off on the real arm. Use it to look
closely and to check a grasp, not to measure.

## The servos

ST3215 bus servos behind the NormaCore Station. They are position-controlled with a soft gain,
so a loaded joint sags a few degrees; the daemon trims that out at the end of every move.
`load` in state.json is in 0.1 % of full output. Single `temp_c` readings glitch (the gripper
servo especially): trust a value only if it stays; it should stay under about 55.

Overload protection: a servo whose output stays above its threshold for 2 s drops to 20 % torque
and reports status 32 until its torque is switched off and on, and the joint sags meanwhile. The
threshold is stored in each servo. It is 60 % on the arm joints (`../scripts/servo_overload.py`
shows and sets it); the Station's calibration puts it back to 25 %, which the shoulder exceeds
on most lifts. Holding the arm stretched out keeps the shoulder near 35-45 %: go `home` or
`rest` between tasks. The robot side refuses joint commands while any servo reports a fault.

Moves follow a trapezoidal speed profile. The servos trail a moving setpoint, more the faster it
moves, and a joint more than 12.6 degrees behind for half a second counts as blocked. Unfolding
from the resting pose is the heaviest move: at speed 0.5 the upper elbow trails by about 9.5
degrees, which is why `home` defaults to that speed. Faster has been refused as blocked.

## Kinematics

`tools/util/planning.py` turns tool poses into joint angles: `solve` for one pose, `line` for
a straight path, `approach_for` for a grasp. The solver underneath is pink (Pinocchio), in the
arm's kinematics library at `../../src/elrobot/kinematics.py`; the robot side uses the same
library for its clearance check, so change your copy of the planning, not that file.
