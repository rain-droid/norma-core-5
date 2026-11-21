---
name: pick-and-place
description: Pick up an object from the table and put it down somewhere else, or stack it, with visual checks at each step.
---

1. `look_at_table`, then `find_objects`. Open `annotated_image` and decide which outline is
   the object you want from what you see, not from the colour word alone. The view covers
   roughly y 0.18 to 0.38 m; for anything else open `perception/exteroception/top/rgb.jpg` and
   move the look pose with `x`.
2. If the object has no outline, or two objects share one, read its pixel off the image and use
   `pixel_to_table` with that `image_id`. Open `click_image` to confirm the crosshair is on it.
   Aim at where the object meets the table, or pass its height.
3. `pick_at` with the object's `x`, `y` and its `yaw_deg` if it has one. Narrow side over
   50 mm will not fit the jaws: try another `yaw`, or push it instead.
4. On "closed on nothing", do not retry the same numbers. Go back to step 1.
5. After a pick, confirm: `holding` true and a `width_mm` that matches the object. Then
   `capture_camera` and see the object between the jaws.
6. `place_at`. For stacking, `z` is the height of the object underneath, and x, y are that
   object's position from a look taken after the pick (the pick may have disturbed it).
7. `look_at_table` again and confirm where the object came to rest. Report what you see, not
   what you commanded.

Stop when the object is visibly where it was asked to be, or say plainly that it is not.
