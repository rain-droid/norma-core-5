# Overlapping objects: knowing what is in a pile

Plan, written 2026-11-22. Two tasks: (1) the tracker concludes from what it has watched that one object
has been put on another, (2) when it is not sure, the arm looks with the wrist camera. Tick the steps off
here as they land.

## The problem

`find_objects` and the tracker (`workspace/tools/util/tracker.py`) have no object model. Whatever differs
from the table's colour is an object, and one connected outline is one object. Seen from the top camera, two
objects lying on each other are one outline:

- only the upper one is seen, or the outline reads as a mixture of both colours;
- the lower object's track is renamed after a second to whatever colour the pile now shows;
- the upper object's track finds its old place empty, becomes `missing`, and is forgotten after two minutes.

So the tracker ends up with one object where there are two, under the wrong name.

One case already works: when **the arm** puts an object down on another. The tracker sees the jaws let go
above a track and sets `on` on the object that was held (`Watcher._gripper`). The gap is everything the arm
did not do itself: a person stacking, an object pushed or fallen onto another.

## What a single top view cannot tell, and what else can

| Evidence | Where from | Strength |
|---|---|---|
| An object was there, and is now gone without turning up elsewhere | the tracker's history | strong: objects do not vanish |
| At about the same time another outline changed: other colour, larger, taller | the tracker's history | strong together with the first |
| Mixed colour or unusual height in one frame, no history | `find_objects` | weak: a two-coloured object looks the same |
| Layers seen from the side | wrist camera, slanted view | strong, but costs a motion and only reaches about 30 cm |

Task 1 uses the first two. Task 2 adds the last one where the first two leave doubt.

## Task 1: conclude a pile from the history

Following the object in a hand all the way is not possible with this segmentation: hand and object are one
outline that lies across the edge of the search area and is not measured. What can be observed is the state
before and after. The rule is conservation:

1. **A is gone.** Track A was in view, its place is now in view and empty (`missing`), and no outline of
   its colour has turned up elsewhere (the tracker already re-finds moved objects by colour).
2. **B changed after A was last seen.** A track B that is in view and lying still now differs from how it
   looked when it last lay undisturbed: its colour no longer fits its name, its outline is clearly larger,
   or its measured height is clearly greater.
3. **Then A is on B**, marked as a conclusion and not an observation: `on: B`, `on_source: "inferred"`,
   `on_evidence: ["colour", "area", "height"]` (those that held). A pile the arm built itself has
   `on_source: "placed"`.

Details:

- [x] **Undisturbed look.** Each track keeps how it looked while lying still and unremarkable (`calm`: outline
      area, height). It is not updated while any track is missing and unexplained, so the comparison is
      always with the time before A went away. It belongs to the place where it was taken: area and height
      only count as evidence while B has not been moved, because the same object has another outline elsewhere.
- [x] **No renaming while a pile may be forming.** A track whose colour stops fitting is not renamed while a
      track went missing in the last `STACK_WINDOW` seconds: the changed colour is evidence, not a new name.
- [x] **The conclusion** as above, once B has lain still for a moment and nothing is moving on the table. If
      several tracks are missing, the one whose colour B now shows is taken, else the one lost last.
      At most one new rider per base and look.
- [x] **Taking it back.** A reappearing elsewhere already clears `on` (existing code: "fell off what it was put
      on"). In addition an inferred pile whose base looks undisturbed again (its evidence gone for a second)
      is dissolved, and A is `missing` again.
- [x] **Report.** `on_source` and `on_evidence` in `tracks.json` and `track_objects`; the picture labels an
      inferred pile `on blue-1?`. `pick_object` needs no change: it already takes the rider from the top of
      the pile, at `z_mm` (the base's height when known).
- [x] **Test** in the simulator (`tests/test_workspace.py`): a block is moved onto another without the arm;
      the tracker reports `on` with `on_source: "inferred"`, the base keeps its name; the block taken off and
      put elsewhere is found again and `on` is cleared; a block that is simply removed stays `missing`.
- [x] Docstrings (`tracker.py`, `track_objects.py`), `skills/pick-and-place`, `docs/FEATURES.md`,
      `docs/CHANGELOG.md`.

Built on 2026-11-22 (`Watcher._piles`), tested in the simulator only. One addition to the rule: when the pile
reads as the upper object, the tracker first sees A "move" onto B's place and B go missing. That is turned
round: B gets its place back and A is on it, with `on_evidence: ["place"]`.

What it will not do:

- An object laid **against** another (touching, side by side) is also one outline that grew and changed
  colour, and is reported as `on`. That is what `inferred` says, and what task 2 settles.
- Two objects of the same colour: only area and height are left as evidence. Weaker, still usable.
- A pile that was already there when the tracker started has no history. Only task 2 (or the weak
  single-frame hints) can find it.
- An object taken off the table for good while its neighbour is nudged at the same moment can be taken for a
  pile. Again `inferred`.

## Task 2: when unsure, look with the wrist camera

A new tool `inspect_object(name)`:

1. Reads the track; goes `home` if the arm hides it.
2. Moves the wrist camera to a slanted view of the object, about 45 degrees below horizontal, from the
   robot's side, 12 to 15 cm away (IK through `ctx.plan.solve`, as `look_at_table` does; a few distances and
   pitches are tried until one is reachable).
3. Captures the wrist image and returns it for the agent to open, with a simple reading: the outline nearest
   to where the object should appear is cut into bands from bottom to top, and bands of clearly different
   colour are counted as `layers`, with a colour word each.
4. Writes the result back: a confirmed pile becomes `on_source: "seen"`, a refuted one is dissolved. The
   tracker is another process, so this goes through a small file it reads (`.runtime/tracker_notes.json`).

When to call it (a line in `workspace/AGENTS.md` and the pick-and-place skill): before picking from or
stacking on anything with `on_source: "inferred"`, and for an outline whose colour is a mixture or whose
height does not fit what it is said to be.

Limits:

- Beyond about 30 cm from the base only slanted views from the robot's side are reachable.
- Positions through the wrist camera are 1 to 2 cm off. Enough to count layers, not to measure them.
- Same-coloured layers do not separate into bands. The image is still returned, and the agent judges it.
- Black objects and ones the colour of the mat are not seen by the segmentation.

Built on 2026-11-22, simulator only. What came out differently from the plan above:

- The camera, not the tool point, is placed on the line of sight (the camera sits beside the tool axis), about
  20 cm from the spot at 35 degrees, from the robot's side and from 40 degrees left and right of it, further
  round if those are out of reach. Between views the arm goes by way of the ready pose.
- Not bands from bottom to top, but the outline split by colour and the colours' positions compared: above
  each other is a pile, next to each other is two objects. A pile is `certain` with two agreeing views.
- An object standing behind the spot gets into one view's outline: only colours seen in every view count.
- "Side by side" is kept in the tracker as `beside`, so the pair is not concluded to be a pile again.

Steps:

- [x] `tools/inspect_object.py`: the viewing pose and the capture.
- [x] The layer count in the wrist image.
- [x] `tracker_notes.json`, read by the tracker: confirm or dissolve a pile.
- [x] Test in the simulator.
- [ ] Test on the real arm with two cubes of different colours.
- [x] `workspace/AGENTS.md`, skill, `docs/FEATURES.md`, `docs/CHANGELOG.md`.

## Order

Task 1 first: it needs no motion, runs in the simulator's tests, and gives task 2 its trigger. Before either
is trusted on the real arm, `track_objects` has to be re-run there (open since the camera change, see
`docs/PLAN.md`). Both belong to the "world model above the tracker" item of that plan.
