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

A command is seven absolute angles in radians, servos 1..7, plus a jaw opening 0..1.
`tools/joint_command.py` sends exactly that; `control.py` is the same thing from Python.

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

`wrist` rides on the gripper and looks along it; the black shapes at the bottom of its image
are the jaws. Its model (intrinsics and mounting) is `../data/camera_wrist.json` if that file
exists, otherwise a rough default: results then carry `camera_calibrated: false` and table
positions can be off by a centimetre or more. `pick_at` absorbs that by looking again from close
up. `top` is fixed above the table for overview; it has no model, so its pixels cannot be turned
into positions.

## The servos

ST3215 bus servos behind the NormaCore Station. They are position-controlled with a soft gain,
so a loaded joint sags a few degrees; the daemon trims that out at the end of every move.
`load` in state.json is in 0.1 % of full output. Single `temp_c` readings glitch (the gripper
servo especially): trust a value only if it stays; it should stay under about 55.

Overload protection: a servo whose output stays above its threshold for 2 s drops to 20 % torque
and reports status 32 until its torque is switched off and on, and the joint sags meanwhile. The
threshold is stored in each servo. It is 50 % on the arm joints (`../scripts/servo_overload.py`
shows and sets it); the Station's calibration puts it back to 25 %, which the shoulder exceeds
on most lifts. Holding the arm stretched out keeps the shoulder near 35-45 %: go `home` or
`rest` between tasks. A tool refuses to start while any servo reports a fault.

Moves follow a trapezoidal speed profile. The servos trail a moving setpoint, more the faster it
moves, and a joint more than 12.6 degrees behind for half a second counts as blocked. Unfolding
from the resting pose is the heaviest move: at speed 0.5 the upper elbow trails by about 9.5
degrees, which is why `home` defaults to that speed. Faster has been refused as blocked.

Inverse kinematics is pink (Pinocchio): `../src/elrobot/kinematics.py`, `ik` for a pose and
`ik_line` for straight lines.
