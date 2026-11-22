---
name: recover
description: What to do when a command is refused, the arm is blocked, a servo faults, or a grasp fails.
---

Read the error first: each one is written to be acted on. Then, by what it says:

- **busy**: another motion is running. Wait for it, or `stop` it.
- **stopped**: someone pressed stop, or a tracker set to stop (`on_intrusion: "stop"`; by
  default it only slows the arm) detected unexplained motion.
  `perception/exteroception/intrusion.jpg` circles the last alarm's trigger;
  `intrusion.json` beside it records the report at that moment. These are historical:
  inspect the current scene with `track_objects` and `capture_camera` before deciding.
  `pick_object` and `place_object` wait out a tracker stop automatically, but quiet is not
  proof that people have left: stationary and occluded hands can be missed. Keep people
  clear of the arm. The other tools return this error rather than resuming.
- **blocked: joint N is X deg behind**: the arm met something, or the move was too heavy or
  too fast. It is holding where it stopped. Look with the top camera and
  `read_proprioception`. If something is in the way, go back the way you came, slowly. If
  nothing is, it was load: repeat slower (`home` at 0.5 or less when getting up from rest) or
  from a less stretched pose. `Arm.joints` has already sent the move a second time, slower,
  from where the arm stopped (a result that needed it carries `retried_blocked`), so a
  "blocked" error that reaches you has failed twice: look before trying a third time. How
  much slower is `blocked_retry_speed` in `robot_config/motion.json`, between 0 and 1: 0.5 is
  half speed, 0 switches the second try off.
- **contact: joint N (load ..., lag ...)**: collaboration mode is on and something touched the
  arm during a move, or the arm touched something. It has already stopped, backed off a little
  along the path it came by, and is holding. This move was not retried and must not be repeated
  unchanged. Look with the top camera: a hand or an object in the way means wait, or ask, or
  go round. If nothing is there and it happens again at the same place, it is the arm's own
  load (a heavy object, a stretched pose): lift from closer in, or tell the person the
  thresholds need raising (`collab_load_rise`, `collab_lag_rise` in the daemon's `config.yaml`).
- **servo fault on joint(s) ... (status 32 = overload)**: that joint has dropped to a fifth of
  its torque and may be sagging. No command will be accepted and none should be tried, least
  of all with `check: false`. A person has to hold the arm while torque is switched off and on.
  Say so plainly and wait.
- **fingertips would be N mm into the table**, **the arm body would hit the table**,
  **... would hit the ...**: the pose is outside the envelope. Choose another: higher, `yaw_deg` 0,
  a different `pitch_deg`.
- **can't reach**, **can't move in a straight line: stuck N%**: the edge of reach. Straight-
  down poses only exist within about 27 cm of the base; lean the gripper (a lower `pitch_deg`),
  work lower, or work closer in.
- **closed on nothing**: the object is not where you sent the gripper. Look again; never retry
  the same numbers.
- **slipped out while lifting**: look where it went. Pick again with the jaws across flat
  sides (`yaw_deg` from `find_objects`) and a lower `grasp_height`.
- **the robot daemon is not reachable**, **not connected**: nothing here can fix that. Say so.

When you are unsure what state the arm is in: `stop`, then `read_proprioception` and a top
capture, then `home` (it lifts the gripper clear before travelling). If the gripper holds
something you did not mean to pick up, `place_at` a clear spot.

Done when the arm is at `home` or `rest`, nothing is held by accident, and, if a person is
needed, you have told them what and why.
