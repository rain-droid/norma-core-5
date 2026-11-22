---
name: draw
description: Draw on a sheet of paper with a pen held in the gripper: copy a drawing shown to the camera or given as a picture, or draw something composed as strokes.
---

1. What to draw. To copy a drawing lying on the table: `home`, `capture_camera`, read the four
   corners of that drawing's sheet off `grid_image` and give them to `draw_plan` as
   `image_id` and `quad_px` (top-left, top-right, bottom-right, bottom-left as the drawing
   reads). From a file: `image_path`. From imagination, or to write words: compose `strokes`
   yourself, a list of polylines of [x, y] points with y upward. Open `preview_image`: it is
   exactly what the pen will do. Thick lines and filled areas come out as single centre lines;
   if the preview is a tangle, the picture has too much shading: raise `detail` or simplify.
2. Where to draw. The blank sheet lies flat within reach, its far edge no more than about
   30 cm from the base, taped down if it slides. Get its four corners in the base frame from a
   capture with `pixel_to_table` (height 0), in the order bottom-left, bottom-right, top-right,
   top-left of the drawing as it should read to whoever will look at it.
   After home has finished, use `depth_scan` at the sheet area to check the supporting surface
   and raised obstacles, with top-camera x/y. Null/grey is unknown, and roughly 3 mm depth
   accuracy cannot establish pen contact or paper thickness; retain the taught pen-tip method.
   All joint paths retain the configured extra mat clearance; never reduce it to reach paper.
3. The pen. `draw_pen` with those `corners`: the arm holds its open jaws over the middle of
   the sheet. Ask the person to stand the pen between the jaws, parallel to the fingers, cap
   off, tip resting on the paper, and to hold it there. Then `draw_pen` with `grip: true`.
   The tip is now known because it was resting on the paper when the jaws closed; nothing is
   measured and the pen is never pushed down to find the paper. `holding` must be true.
4. `draw_run` with `dry_run: true`. It plans every stroke; if one is out of reach it says
   which, and the fix is a larger `margin` or a sheet closer to the base.
5. `draw_run`. Then `capture_camera` and compare the paper with the preview.

Adjusting, from what the paper shows:

- Faint or broken lines: the tip is not pressing. `draw_run` again with `press` 0.002, then
  0.003 (it redraws over the same lines), or `first_stroke` to redo from one stroke on.
- The pen digs in, the paper drags, or the arm stops with a blocked joint: lower `press`
  (0, then -0.001) and slow down (`speed` 0.2).
- Lines near the far edge are lighter than near ones: the stretched arm sags less or more
  than at the middle where the pen was gripped. Use a smaller sheet area (`margin`) or move
  the sheet closer.
- The drawing is shifted or turned on the sheet: the corners were off or in the wrong order.
- The pen has slipped in the jaws (lines stop touching altogether): do step 3 again.

A felt-tip pen or marker forgives a millimetre of height error; a ballpoint does not.

Leave the skill with the pen handed back (`draw_pen` with `release: true`, the person holding
it) and the arm at `rest`.
