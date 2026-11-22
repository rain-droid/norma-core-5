---
name: pick-and-place
description: Pick up an object from the table and put it down somewhere else, or stack it, following it with the top camera so the task survives the object being moved.
---

1. `home`, so the arm is awake and out of the top camera's view of the table. Then
   `track_objects`. Open `image` and decide which name is the object you want from what you
   see, not from the colour word alone. Check that its cross sits in the middle of where the
   object stands.
2. `pick_object` with that name. It aims at where the object is at that moment (once it lies still), aims again if it moves, and tries again if the jaws come up empty.
   While something else moves on the table (a hand, most likely) the tracker slows the arm to
   a quarter of its speed, and lets it speed up again when the table is quiet. It keeps
   moving, and a stationary or occluded hand is not noticed at all. Keep people clear while
   it runs. Read `tried`: it says what happened on the way.
3. `place_object` with `on` (the name of the object to stack on) or `x`, `y`. It checks with
   the camera afterwards and, if the object is not where it should be, picks it up and places
   it once more. `checked: true` means the camera agreed.
4. Look yourself before you report: `track_objects` again and open its image. For a stack the
   top object has `on` set to the one below. Expect a centimetre or two of error on the real
   arm. Report what you see, not what you commanded.
   A stack the arm did not build (a person put one object on another) shows up the same way
   with `on_source: "inferred"`: the tracker concluded it from one object vanishing while the
   other changed its look. Before picking from or stacking on such a pile, look at it
   (`look_at_table` over it, or the top image) and check that it is a pile and not two objects
   side by side; if `z_mm` is 0 the lower one's height is unknown, so give `pick_at` the height
   yourself.
5. `rest` when the task is finished.

When the tracker does not have the object (no outline, or it shares one with another object;
the table's own colour and black things on the dark mat are not seen):

6. `capture_camera`, read a pixel off `grid_image`, and use `pixel_to_table` with that
   `image_id`. Either click where the object meets the table (height 0; that is its near edge,
   so add half its depth away from the camera), or click the centre of its top face and give
   its height. Open `click_image` to confirm the crosshair is where you meant.
7. `pick_verified` with that `x`, `y` and the object's height: it is `pick_at` with two looks
   through the wrist camera. From the hover point it measures how far the object is from where
   the jaws aim and lines up on it (`alignment_mm`, `corrections_mm`); after lifting it checks
   the object is between the fingers (`seen_in_jaws`). Open `before_image` and `after_image`.
   On this arm the top camera's position for an object is 1 to 2 cm off along the reach, and
   this is what takes that out. Plain `pick_at` remains for what the wrist camera cannot see
   (black or mat-coloured things). Then `place_at`. These check nothing, so look after each:
   `holding` true with a `width_mm` that matches the object, then `find_objects`. For
   stacking with `place_at`, `z` is the height of the object underneath.

When objects stand close together, or the object is long or lies at an angle:

8. `find_objects` (or `track_objects`), then `plan_grasp` with the object's id. It compares
   jaw directions, grip points and heights against the neighbours and the arm's reach, and
   says why it rejects the ones it does. Open `grasp_image`. `pick: true` performs the best
   one, opening the jaws only as far as it needs. If nothing survives, move a neighbour first.

A ring wider than the jaws (the brown rubber one is about 9 cm across, wall about 23 mm) is
picked by its rim, not by name: `pick_object` aims at the tracked centre, which is the hole
(and the tracker's cross sits on the ring's edge, not its middle). Read the ring's centre and
one rim point to the robot's left or right of it off `capture_camera`, `pixel_to_table` the rim
point, and `pick_at` there with `yaw_deg: 0`, `grasp_height: 0.02`: one finger goes down the
hole. `place_at` then puts the rim point, not the centre, at x, y.

The black tape roll (about 8 cm across, band about 16 mm, 19 mm high) goes the same way: the
tracker calls it `blue-N` and sees only a sliver of it, and `grasp_object` on it failed with
"not in view of the top camera" after its own arm covered it. What worked: `capture_camera`,
`pixel_to_table` (height 0.019) on the band 3 cm to the robot's left of the centre, `pick_at`
there with `yaw_deg: 0`, `grasp_height: 0.02` (`width_mm` about 16), then `place_at`. It landed
about 3 cm from the commanded spot.

Measured on the real arm, 2026-11-22, at half speed (`speed_scale` 0.5 in robot_config/motion.json),
with `find_objects` on SAM 3 (`prompt: "blue cube"`) giving the position and `pick_at` / `place_at`
doing the work: three pick-and-place cycles of the 30 mm blue cube, every grasp clean
(`width_mm` 27). Landings against the commanded spot: 4 mm off at 22 cm from the base, 8 and
12 mm long (further from the base) at 34 to 35 cm, where the gripper leans 60 to 70 degrees to
release. Too few to correct from, but if a far placement has to be exact, aim a centimetre short.

A stack (a cube on a box) is one outline to `find_objects` unless the prompt names its parts:
`find_objects` with `prompt: "blue cube, pink box"` (SAM 3 running) gives each its own entry,
and the upper one has `on` (the id of what it stands on), `z_mm` (how high its underside is) and
its own `height_mm`. Pick it with `pick_at` and `grasp_height` = (z_mm + height_mm / 2) / 1000.
Heights are read off the side face the camera sees and are good to about 5 mm; they tend to
come out a few millimetres low (the 30 mm cube reads 20 to 29). `z_mm` null means the lower
object is too hidden to measure: move the upper one off by a height you judge yourself, or look
from the wrist. Without the names, `pick_at` at the default height closed on the lower object
(`width_mm` 36, not the cube's 27) and carried the pair: always check `width_mm` against the
object you meant. Rings and rolls of tape get no `height_mm` (nothing of their side can be told
from the hole).

What the errors mean:

- "gave up after 25 s: the work area is not quiet (...)": only under a tracker set to stop
  (`track_objects` with `on_intrusion: "stop"`). Something keeps moving on the table: a hand,
  or an object the tracker measures unsteadily. Open the current image and the last alarm's
  `perception/exteroception/intrusion.jpg` and `intrusion.json`. The same applies when the
  arm stays slow for no visible reason. Do not set `on_intrusion: "ignore"` automatically:
  only after an operator has inspected the scene and agreed to supervise without it, and set
  it back afterwards.
- "... is out of sight (hidden)": it is under the arm even at home, or under something else.
  Objects right in front of the base are hidden by the arm in the home pose.
- "... is out of sight (missing)": it was taken away, or pushed out of the searched area.
- "could not pick ... in 4 attempts": read what each attempt says. Jaws closing on nothing
  several times means the measured position is off: use step 6.
- "can't reach": it is too far (about 38 cm from the base) or too close.
- "the height of ... could not be measured": give `place_object` a `z`, the height in metres
  of what you are stacking on (its width, if it is a cube).
- "... is not on ...": the stack failed twice. Look, and decide whether the base is too small
  or the release too high (`z` lower).

Stop when the object is visibly where it was asked to be, or say plainly that it is not.
