---
name: recover
description: What to do when a command is refused, the arm is blocked, a servo faults, or a grasp fails.
---

Read the error first: each one is written to be acted on. Then, by what it says:

- **fingers touched or dug into the mat**: `stop` immediately. Read measured state and fresh
  top/across views, then retreat above the table if clear. Once home has finished, `depth_scan`
  at the target and visible nearby mat; inspect the height map, and refresh `scan_obstacles`.
  Missing depth is unknown. The 2026-11-22 correction adds 15 mm to table approach endpoints
  without raising the hover, and all outgoing joint paths enforce 25 mm modelled fingertip
  clearance. Do not reduce it, subtract it from target heights, or bypass it with raw commands.
  Depth verifies surfaces/object heights, not the robot's fingertip offset: investigate that
  calibration separately if contact disagrees with the model. Never repeat the same low path.

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
  sides (`yaw_deg` from `find_objects`) after checking its height with `depth_scan`. Keep the
  added descent margin; do not automatically lower the grasp to compensate.
- **the robot daemon is not reachable**, **not connected**: nothing here can fix that. Say so.

When you are unsure what state the arm is in: `stop`, then `read_proprioception` and a top
capture, then `home` (it lifts the gripper clear before travelling). If the gripper holds
something you did not mean to pick up, `place_at` a clear spot.

Done when the arm is at `home` or `rest`, nothing is held by accident, and, if a person is
needed, you have told them what and why.
