# ElRobot workspace

You are in the working directory of a real robot arm: a NormaCore ElRobot, seven joints and a
parallel gripper, on a table, with a camera on its wrist and one above the table. Everything
you need to operate it is in this directory, as files. There is no tool server.

| Path | What it is |
|---|---|
| `robot_config/` | The body: URDF, meshes, joint table, frames, and `control.py`, the raw joint interface. Read `robot_config/README.md` first. |
| `tools/` | One Python file per tool. The file is the whole algorithm and its own command. |
| `perception/` | The live senses, rewritten twice a second: `latest.json`, `exteroception/{wrist,top}/rgb.jpg`, `proprioception/state.json`. |
| `skills/` | Task procedures in Markdown. Read the one that fits before starting. |
| `tool_call_history/` | One folder per call: `input.json`, `result.json`, `joint_commands.jsonl`, images. |

## Running tools

```bash
pixi run python tools/look_at_table.py '{}'
pixi run python tools/pick_at.py '{"x": 0.07, "y": 0.20}'
pixi run python tools/move_tcp.py --schema        # signature and full description
```

Arguments are one JSON object. The result is JSON on stdout; exit status 1 means it has an
`error`. Images come back as paths: open them with your image reader, a path alone shows you
nothing. If the daemon is not running the error says how to start it.

| Purpose | Tools | Moves the arm |
|---|---|---|
| See | `capture_camera`, `look_at_table`, `find_objects`, `read_proprioception` | only `look_at_table` |
| Pixel to table position | `pixel_to_table` | no |
| One absolute joint command | `joint_command`, or `robot_config/control.py` from Python | yes |
| Move the tool point | `move_tcp`, `adjust_tcp` | yes |
| Gripper | `open_gripper`, `close_gripper` | yes |
| Whole manipulations | `pick_at`, `place_at` | yes |
| Poses and safety | `home`, `rest`, `stop` | `stop` always works |

The tools are a ladder, not a fence. `pick_at` is built from `move_tcp` and `close_gripper`,
which end in the same joint command you can send yourself. When a tool does not do what the
task needs: read it, call a lower one, or edit it. Edits take effect on the next call. A new
`tools/NAME.py` defining `async def NAME(ctx, ...)` is a new tool; copy a short one such as
`tools/home.py`. `ctx.robot` is `../src/elrobot/robot.py`, also readable. A crash in a tool
file comes back as `error_kind: tool_bug` with the traceback.

## Rules that keep you right

- Only the newest image shows the present. After any motion, by you or by a person at the
  table, look again before choosing a point. Never reuse a pixel or a position from before.
- A pixel belongs to its capture. Give `pixel_to_table` the `image_id` it came from; pixel
  coordinates are those of the unmodified image, and `rgb_grid.jpg` has gridlines to read from.
- Results are measurements, not verdicts. No error does not mean it worked: check the fields
  (`holding`, `position_error_mm`, `worst_joint_error_deg`) and then check with a camera.
- Units: metres, radians, gripper 0 (closed) to 1 (open). Base frame: origin on the table under
  the base, x to the robot's right, y forward, z up.
- `joint_command` with `check: false` and `allow_low: true` remove the table and self-collision
  guards. Use them deliberately, with small steps, after `read_proprioception`.
- If something looks wrong or a person reaches in, `tools/stop.py` first, think second.
- When you learn something about this arm or this table that the next session should know,
  write it into the relevant skill or tool docstring.
