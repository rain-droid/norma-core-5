# Changelog

What changed, when, and who made the change. Newest first. The reasons are in `docs/DECISIONS.md`; what works
today is in `docs/FEATURES.md`. Add an entry with each push.

## 2026-11-22

### `inspect_object`: the arm looks from the side where the top camera cannot tell
Modified by: Arian Malekzadeh

- New tool `workspace/tools/inspect_object.py`. Given a tracked name or a spot, the arm brings the wrist camera
  about 20 cm from it, looking down at a slant, from up to three sides. Each view is split by colour; colours
  one above the other in every view are a pile, colours next to each other are two objects touching.
- The verdict goes to the tracker through `.runtime/tracker_notes.json`: a pile becomes `on_source: "seen"`;
  side by side, the object without an outline of its own gets `beside` and is no longer `on`; a pile that is
  not there is dissolved. This also works for an object the tracker had simply lost.
- Only colours that every view shows at the spot count, so an object standing behind it does not.
- Test in `tests/test_workspace.py`, simulator only. On the real arm the viewpoints, the real camera's mounting
  and the colour reading are all untried.

### Tracker concludes piles it did not see being built
Modified by: Arian Malekzadeh

- Two objects on each other are one outline from above. Until now the tracker only knew a pile when the arm
  had built it. Now a track that goes missing without turning up elsewhere, while another outline has changed
  since (colour, area, height, or it stands in the missing one's place), is reported as lying on that one:
  `on`, with `on_source: "inferred"` and `on_evidence`. A pile the arm built has `on_source: "placed"`.
- The lower object keeps its name (it used to be renamed to the pile's colour after a second), and the upper
  one is no longer forgotten as `missing`. Taken off and put elsewhere, it is found again and `on` is cleared.
- `docs/OVERLAPPING_OBJECTS.md`: the plan for this and for the second half, a close look with the wrist camera
  when the tracker is unsure (not built yet).
- Test in `tests/test_workspace.py`, simulator only. Not run on the real arm.

### Web UI: camera switcher and live vision layers
Modified by: Atharva Bhorpe

- The camera panel has a tab for every camera the daemon reports (`top`, `wrist`, `across`), with a dot on the
  ones delivering frames. It opens on `top`.
- Five layers can be switched on over the picture: **Objects** (live segmentation outlines), **Tags**
  (AprilTags), **Frame** (the robot's axes and reach, top camera), **Arm** (the arm as the kinematics place it,
  top camera), **Tools** (what the tools and the tracker last reported).
- New daemon endpoint `GET /api/camera/{name}/vision` (`src/elrobot/vision_live.py`), computed only while a
  browser asks; test in `tests/test_vision_live.py`.
- `?camera=top&layers=objects,frame,arm` in the address opens the panel in that state.
- Fixed: the enlarged camera view collapsed to a thin bar.
- `config.yaml`: the two KD cameras' ids after the replug (wrist `5:17`, across `5:15`).

### Tracker slows the arm instead of stopping it
Modified by: Atharva Bhorpe

- While something else moves on the table, the tracker now slows the arm to a quarter of its speed and lets it
  return to full speed when the table is quiet. `track_objects` takes `on_intrusion` (`"slow"` by default,
  `"stop"`, `"ignore"`) in place of `stop_arm_on_intrusion`.
- New daemon endpoint `POST /api/pace` and `state.pace`; `Robot.set_pace` scales a motion under way.
- `pick_object`, `place_object`, `find_objects`, the pick-and-place and recover skills and `AGENTS.md` follow.

### Blocked moves retry once, slower
Modified by: Arian Malekzadeh

- A move the robot reports as blocked is tried once more at a lower speed; how much lower is in
  `workspace/robot_config/motion.json`. Tests in `workspace/tests/`.

### Docs and plan
Modified by: Atharva Bhorpe

- `docs/FEATURES.md`, `docs/DECISIONS.md`, `docs/INTERFACE.md`, `docs/PLAN.md`, this changelog; `docs/HANDOFF.md`
  brought up to date; `CLAUDE.md` points to all of them.

### `pick_verified`: the wrist camera checks a pick before and after
Modified by: Atharva Bhorpe

- From the hover point it measures how far the object is from the aim and lines up on it; after lifting it checks
  the object is between the fingers. Run on the real arm with the blue cube.

### UGREEN webcam as the top camera; 1280x720 streams; calibration
Modified by: Atharva Bhorpe

- The UGREEN above the mat is `top`; the KD camera across the table is `across`.
- `base_in_mat` refitted (it was 2.2 cm and 4.8 degrees off); top camera recalibrated from the mat tags.
- All cameras stream 1280x720 so frames match the wrist calibration; `scripts/cameras_ready.sh` prepares them.

### Wrist-camera calibration
Modified by: Arian Malekzadeh (merged from `main`)

- Lens and hand-eye calibration of the wrist camera, `data/camera_wrist.json`, `scripts/calibrate_camera.py`.

### Chess, drawing and a grasp planner
Modified by: Atharva Bhorpe

- `chess_setup`, `chess_see`, `chess_move`, `chess_record`; `draw_plan`, `draw_pen`, `draw_run`; `plan_grasp`;
  skills `play-chess` and `draw`. Simulator only.
