---
name: pick-and-place
description: Pick up an object from the table and put it down somewhere else, or stack it, with a camera check at each step.
---

1. `home`, so the arm is awake and out of the top camera's view of the table. Then
   `find_objects`. Open `annotated_image` and decide which outline is the object you want from
   what you see, not from the colour word alone. Check that its cross sits in the middle of
   where the object stands on the table.
2. If the object has no outline, shares one with another object, or its cross is off:
   `capture_camera`, read a pixel off `grid_image`, and use `pixel_to_table` with that
   `image_id`. Either click where the object meets the table (height 0; that is its near edge,
   so add half its depth away from the camera), or click the centre of its top face and give
   its height. Open `click_image` to confirm the crosshair is where you meant.
3. `pick_at` with the object's `x`, `y` and `yaw_deg`. `width_mm` over 50 will not fit the
   jaws: try the other side (`yaw_deg` plus 90), or push the object instead. Set `grasp_height`
   to about half the object's height (`height_mm`, which tends to read up to a centimetre high).
4. If it refuses or fails, read the reason:
   - "closed on nothing": the object is not at that x, y. Do not retry the same numbers; go
     back to step 1.
   - "fingertips would be N mm into the table": far objects need leaning jaws, and leaning,
     turned jaws dip. Raise `grasp_height` to 0.020 or more, or use `yaw` 0.
   - "can't reach": it is too far (about 38 cm from the base) or too close.
5. After a pick, confirm: `holding` true and a `width_mm` that matches the object. Then look:
   in a new `find_objects` the object should be gone from where it stood, and
   `capture_camera` with `camera: "wrist"` shows it between the jaws.
6. `place_at`. For stacking, `z` is the height of the object underneath (its `height_mm`, or
   its width if it is a cube), and x, y are that object's position from a look taken after the
   pick (the pick may have disturbed it).
7. `home`, then `find_objects` again, and confirm where the object came to rest. Expect a
   centimetre or two of error on the real arm. Report what you see, not what you commanded.
8. `rest` when the task is finished.

Stop when the object is visibly where it was asked to be, or say plainly that it is not.
