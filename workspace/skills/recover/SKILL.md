---
name: recover
description: What to do when a command is refused, the arm is blocked, a servo faults, or a grasp fails.
---

Read the error first: each one is written to be acted on. Then, by what it says:

- **busy**: another motion is running. Wait for it, or `stop` it.
- **stopped**: someone pressed stop. Do not carry on where you were. Look (`capture_camera`),
  find out why, and start again from what you see.
- **blocked: joint N is X deg behind**: the arm met something, or the move was too heavy or
  too fast. It is holding where it stopped. Look with the top camera and
  `read_proprioception`. If something is in the way, go back the way you came, slowly. If
  nothing is, it was load: repeat slower (`home` at 0.5 or less when getting up from rest) or
  from a less stretched pose.
- **servo fault on joint(s) ... (status 32 = overload)**: that joint has dropped to a fifth of
  its torque and may be sagging. No command will be accepted and none should be tried, least
  of all with `check: false`. A person has to hold the arm while torque is switched off and on.
  Say so plainly and wait.
- **fingertips would be N mm into the table**, **the arm body would hit the table**,
  **... would hit the ...**: the pose is outside the envelope. Choose another: higher, `yaw` 0,
  a different `pitch`.
- **can't reach**, **can't move in a straight line: stuck N%**: the edge of reach. Straight-
  down poses only exist within about 27 cm of the base; lean the gripper (a lower `pitch`),
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
