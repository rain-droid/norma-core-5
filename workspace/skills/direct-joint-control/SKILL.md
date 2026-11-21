---
name: direct-joint-control
description: Move the arm with joint commands of your own when no tool does what the task needs, such as gestures, pushes and unusual poses.
---

1. `read_proprioception` and open `body_image`. Know the pose before you change it.
2. Open the picture of each joint you mean to move, `robot_config/visuals/jointN.jpg`: the
   joint at -90, 0 and +90 degrees, what it carries in orange, its axis in cyan. The table in
   `robot_config/README.md` says how far 10 degrees of each joint moves the tool point.
3. Start from the measured angles in `perception/proprioception/state.json`. Change only the
   joints you mean to, and stay inside `lower_rad` .. `upper_rad` with a little to spare.
4. Send it with `joint_command`, or `Robot.step` in a script of your own
   (`robot_config/control.py`). Make the first moves small (0.3 rad or less) and slow
   (`speed` 0.4). A list of waypoints is travelled as one smooth move.
5. Leave `check` on. When the robot side refuses a pose it names what would be hit: choose
   another pose instead of switching the check off. `allow_low` is for a deliberate reach down
   to the table surface, nothing else.
6. Read each result. `worst_joint_error_deg` above a degree or so means the arm is pressing on
   something or straining. Look with a camera before the next command.
7. For a goal in space without a tool for it, plan in Python: `from util.planning import
   Planner`, then `solve` for a pose or `line` for a straight path, and send what it returns.
8. If it is worth doing twice, make it a tool: copy `tools/open_gripper.py`, write the
   docstring first, plan everything before the first command so a refusal leaves the arm where
   it was.

Done when a camera has shown the motion did what was asked. Then `home`.
