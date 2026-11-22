# ElRobot workspace

You are in the working directory of a robot arm: a NormaCore ElRobot, seven joints and a
parallel gripper, on a table, with a camera above the table (`top`) and one on its wrist.
Everything you need to operate it is in this directory, as files.

**The robot understands one thing: joint commands.** Seven joint angles in radians and a jaw
opening from 0 to 1. Nothing else crosses to the robot: no poses, no grasps, no named skills.
Inverse kinematics, grasping, finding objects, all of it is Python in `tools/` that runs in
your shell and ends in a joint command. You can read every line, run it, change it, or go
around it and send joint angles yourself.

| Path | What it is |
|---|---|
| `robot_config/` | The body: URDF and meshes, a picture of what each joint does (`visuals/`), frames, camera calibration, and `control.py`, the raw joint interface. Read `robot_config/README.md` first. |
| `tools/` | One Python file per tool: its whole algorithm and its own command. `tools/util/` is the shared code: `arm.py` is the contract with the robot, `planning.py` and `kinematics.py` the IK. |
| `perception/` | The live senses, rewritten twice a second and after every command: `latest.json`, `exteroception/{top,wrist}/rgb.jpg`, `proprioception/state.json`. While the tracker runs, also `exteroception/tracks.json`: every object on the table by name, ten times a second. |
| `skills/` | Task procedures in Markdown, written from experience with this arm. Take up the one that fits before starting (`activate_skill`). |
| `state/world.json` | The world model: every object by key and name, where it was last seen, its shape, what it stands on, what was done with it. |
| `tool_call_history/` | One folder per call: `input.json`, `result.json`, `joint_commands.jsonl` (every command sent), images. |
| `state/` | What a task remembers between calls: the chess game and board (`chess.json`), the drawing, sheet and pen (`drawing.json`). Delete a file to forget it. |

`perception/proprioception/state.json` says whether this is the real arm or a simulation
(`"mode": "real"` or `"sim"`). Work the same way in both.

Nothing here reaches outside this directory except the joint commands: it has its own
interpreter and libraries (`pyproject.toml`), its own copy of the robot model and its own
kinematics. The robot is a separate process that it talks to over local HTTP.

## Running tools

```bash
pixi run python tools/find_objects.py '{}'
pixi run python tools/pick_object.py '{"object": "red-1"}'
pixi run python tools/move_tcp.py --schema        # signature and full description
```

Arguments are one JSON object. The result is JSON on stdout; exit status 1 means it has an
`error`. Images come back as paths: open them with your image reader, a path alone shows you
nothing. If the robot daemon is not running the error says how to start it.

| Purpose | Tools | Moves the arm |
|---|---|---|
| See | `capture_camera`, `find_objects`, `read_proprioception`, `look_at_table` | only `look_at_table` |
| Follow objects live | `track_objects` | no, but its motion heuristic slows the arm down |
| Pixel to position | `pixel_to_table` | no |
| Heights, from the depth camera | `depth_scan`, `calibrate_depth` | no |
| Things on the table as obstacles | `scan_obstacles` | no |
| The second arm (SO-101) | `second_arm`, `hand_over`, `arms_clearance`, `calibrate_second_arm` | the second arm; `hand_over` and the calibration move this one too |
| Poses shown with both arms limp | `record_poses`, `replay_poses`, `teach_hand_over` | `record_poses`: the person moves them; the others yes |
| The contract itself | `joint_command`, or `robot_config/control.py` from Python | yes |
| Move the tool point | `move_tcp`, `adjust_tcp` | yes |
| Gripper | `open_gripper`, `close_gripper` | jaws only |
| Whole manipulations, by tracked name | `pick_object`, `place_object` | yes |
| Whole manipulations, by position | `pick_at`, `place_at` | yes |
| Objects by name: what is on the table, and pick and place by name | `describe_objects`, `name_objects`, `world_model`, `grasp_object`, `put_object` | only `grasp_object`, `put_object` |
| A pick the wrist camera checks before and after | `pick_verified` | yes |
| Choosing a grasp among neighbours | `plan_grasp` | only with `pick: true` |
| Chess on a physical board | `chess_setup`, `chess_see`, `chess_move`, `chess_record` | `chess_move`; `chess_setup` and `chess_see` turn the arm aside |
| Drawing with a pen | `draw_plan`, `draw_pen`, `draw_run` | `draw_pen`, `draw_run` |
| Pushing a flat T onto a target pose (Push-T) | `push_t` | yes; `max_pushes: 0` only looks |
| Poses and safety | `home`, `rest`, `stop` | `stop` always works |
| Shown by hand, then repeated | `hand_guide`, `replay_demo` | `hand_guide`: the person moves it; `replay_demo` yes |
| Skills | `activate_skill`, `deactivate_skill`, `create_skill` | no |
| Telling the people watching what comes next | `show_steps` | no |

A session usually goes: `activate_skill` for the task, `home` (wakes the arm and gets it out
of the camera's way), `track_objects` and open its image, `pick_object` and `place_object` by
name, look again to confirm, and `rest` when you are done or waiting. Do not leave the arm
stretched out: holding that is hard on the shoulder servo.

`pick_object` and `place_object` read the tracker while they move: they aim again when the
object has been moved, try again when a grasp comes up empty, and check with the camera where
a placed object ended up. While something else moves on the table the tracker slows the arm to
a quarter of its speed, and lets it return to full speed once the table is quiet. Slow is not
stopped, and quiet does not mean nobody is there: the arm runs at full speed with a stationary
hand in view. Keep people clear of the arm.
`pick_at` and `place_at` go
to the numbers you give them and check nothing; use them for what the tracker cannot see.

## Objects have names

`find_objects` and the tracker know colours, not things: a rubber ring and a cube are both `red`.
(`find_objects` outlines things with SAM 3 when its service runs, `uv run tools/util/sam3_server.py`, and
then takes a `prompt` such as "blue cube"; the tracker stays with colours.)
So before a task that names things, look and name them: `describe_objects` returns a picture
sheet with one cut-out per object; open it and write down what each is with `name_objects`
("blue cube", "rubber ring"). The names live in the world model (`state/world.json`) and stay
with an object when it is moved. After that, work by name: `grasp_object` "the ring",
`put_object` on "the pink box", `world_model` to read what is where and what was done with it.
`grasp_object` looks again before it reaches, grips a ring by its rim, and has the wrist camera
check the grasp. When the person talks about a thing, use their word for it as its name.

## Heights come from the depth camera

Across the table from the arm stands a depth camera. Its colour picture is camera `across`
(`capture_camera`): the scene from the front, good for seeing what is on top of what. Its depth
sensor is read by `tools/depth_scan.py`: every thing standing on the mat with the height of its
top, to about 3 mm, and a picture of the table from above coloured by height. Use it when height
is the question: how tall a stack is, whether one thing lies on another, whether a spot is free,
whether something was really put down or lifted (scan before and after). Pass the x, y that
`find_objects` gave for an object in `at` to get that object's height. For aiming a grasp keep
using the top camera's positions. The arm hides what is under and behind it: send it home before
a scan. If the camera's stand or the mat was moved, run `tools/calibrate_depth.py` once.

## There is a second arm

To this arm's right stands an SO-101 with an orange parallel gripper, on its own daemon
(`scripts/second_arm_up.sh`). `tools/second_arm.py` runs its tools: status, home, rest, move_to,
gripper, grasp, move_relative. It has five joints and reaches about 30 cm; with `frame="base"` you
positions are in this arm's frame. Both arms live in one model (`tools/util/shared_space.py`): a
move of either that would bring any part within 2 cm of the other arm is refused, naming the
parts; `arms_clearance` reads the gap. Then move the other arm away first (home, rest) or choose
another place. Only one arm moves at a time.

`scan_obstacles` adds what stands on the table to that model for two minutes: a move that would
dip a part of an arm into something is refused, naming the thing's height and place, and
`pick_at` and `place_at` carry over the tallest thing on their way instead of through it. Scan
before carrying something across the table, and again after things have been moved. The model
does not know what a gripper holds, and the second arm's own pick, place and gestures are not
checked. The web UI's 3D view shows both arms, the things scanned and the path about to be taken.

`tools/hand_over.py` passes what this arm holds to the second arm in the air. Pick the thing by
its top only (`pick_at` with `grasp_height` 0.03 for the 3 cm cube) so its lower part hangs free,
then call `hand_over(colour="blue")`. This arm holds it out where both reach, the depth camera
measures where it really is, the second arm comes up from below, closes, and this arm lets go and
goes home. Check `second_arm_holding` and `second_arm_has_it` in the result; on an error this arm
still holds the thing and you can call it again. Afterwards the second arm holds the thing: lower
it and open (`move_relative`, `gripper`) or carry it somewhere with `move_to`. If either arm or
the depth camera was moved, run `calibrate_second_arm` (a person hands the second arm the pink
block first).

## Shown by hand

"Let me show you", "put it here" with a hand on the arm, "do what I just did": use
`tools/hand_guide.py`. The person leads the gripper; the arm follows the push and stays where
it is left. Say when to start and that letting go for three seconds ends it. With `save_as`
the stretch is kept as a demonstration and `then_gripper` closes or opens the jaws at its end,
so a pick and place is taught in three or four calls and `tools/replay_demo.py` repeats it
without help. A demonstration repeats joint angles, not the intention: before replaying, check
that the objects are where they were, and afterwards check with a camera that it did what was
meant. When a taught motion is worth keeping, write a skill around it (`create_skill`).

## Leave a skill behind

When you finish a task that no skill in `skills/` covered, write one with `create_skill` before
you report: the steps that worked, what to check, what went wrong and what fixed it. It appends
the tool calls you actually made. The next agent, or you in the next session, starts from it
instead of from nothing. If you followed a skill and found a better way, replace it.

## The tools are a ladder, not a fence

`pick_object` is `pick_at` with the tracker's eyes; `pick_at` is `planning.py` choosing joint
angles and `arm.py` sending them, the same command
`joint_command` sends. When a tool does not do what the task needs: read it, call a lower
one, edit it (the next call runs your version), or write your own. A new `tools/NAME.py`
defining `def NAME(ctx, ...)` is a new tool; copy a short one such as `tools/open_gripper.py`.
For a one-off, plain Python is enough:

```python
from robot_config.control import Robot      # pixi run python my_script.py, from this directory
robot = Robot()
q = [j["rad"] for j in robot.observe()["joints"]]
q[0] += 0.2                                  # turn the base 11 degrees
robot.step(q)                                # one joint command
```

A crash in tool code comes back as `error_kind: tool_bug` with the traceback. The robot side
keeps its own guards whatever the code here does: joint limits, the table, self-collision,
blocked joints and servo faults. It refuses with a reason; it does not need you to be careful
on its behalf, but it only knows about the arm and the table, not about objects or people.

## Rules that keep you right

- Only the newest image shows the present. After any motion, by you or by a person at the
  table, look again before choosing a point. Never reuse a pixel or a position from before.
- Measure with the `top` camera. It is calibrated against the mat; it hangs 79 cm above
  the mat, with the robot at the right edge of its image reaching towards the left: the robot's
  forward is image-left and its right is image-up. The `wrist` camera is for looking closely and
  for checking a grasp; positions from it are about 1 cm off within 25 cm of the base and 2 cm
  or more beyond 30 cm on the real arm.
- A pixel belongs to its capture. Give `pixel_to_table` the `image_id` it came from; pixel
  coordinates are those of the unmodified image, and `rgb_grid.jpg` has gridlines to read from.
- Results are measurements, not verdicts. No error does not mean it worked: check the fields
  (`holding`, `width_mm`, `position_error_mm`) and then check with a camera.
- Units: metres, radians, gripper 0 (closed) to 1 (open). Base frame: origin on the table under
  the base, x to the robot's right, y forward, z up.
- `check: false` and `allow_low: true` remove or loosen the robot side's clearance guard. Use
  them deliberately, with small steps, after `read_proprioception`.
- If something looks wrong or a person reaches in, `tools/stop.py` first, think second.
  The tracker slows the arm on unexplained motion (or stops it, with `track_objects`
  `on_intrusion: "stop"`), but is not a person detector or safety
  interlock. Stationary hands, occluded areas, table-coloured objects and regions outside
  the top camera's search area can be missed. Its last alarm is recorded in
  `perception/exteroception/intrusion.jpg` and `intrusion.json`; those are historical,
  not live views. Never set `on_intrusion: "ignore"` automatically to work around repeated alarms.
- For a task of more than one step, say what you are going to do with `show_steps` before you
  start, and call it again as each step is finished or the plan changes. People follow the arm
  on a screen; it is how they know what comes next.
- When you learn something about this arm or this table that the next session should know,
  write it into the relevant skill or tool docstring.
