# Overlapping objects: knowing what is in a pile

This document is a plan, written 2026-11-22. It has two tasks. (1) From what it has watched, the tracker
concludes that one object has been put on another. (2) When the tracker is not sure, the arm looks with the
wrist camera. Tick off the steps here when they are done.

## The problem

`find_objects` and the tracker (`workspace/tools/util/tracker.py`) have no object model. Anything that differs
from the table's colour is an object, and one connected outline is one object. From the top camera, two
objects that lie on each other are one outline:

- Only the upper object is seen, or the outline reads as a mixture of both colours.
- After a second, the tracker renames the lower object's track to the colour that the pile now shows.
- The upper object's track finds its old place empty and becomes `missing`. The tracker forgets it after two
  minutes.

As a result, the tracker has one object where there are two, and that object has the wrong name.

One case already works: when **the arm** puts an object down on another. The tracker sees that the jaws
release above a track. It then sets `on` on the object that was held (`Watcher._gripper`). The missing part is
everything that the arm did not do itself: a person who stacks, or an object that was pushed or fell onto
another.

## What a single top view cannot tell, and what else can

| Evidence | Where from | Strength |
|---|---|---|
| An object was there. It is now gone, and it has not appeared elsewhere | the tracker's history | strong: objects do not vanish |
| At about the same time another outline changed: other colour, larger, taller | the tracker's history | strong together with the first |
| Mixed colour or unusual height in one frame, no history | `find_objects` | weak: a two-coloured object looks the same |
| Layers seen from the side | wrist camera, slanted view | strong, but it costs a motion and reaches only about 30 cm |

Task 1 uses the first two. Task 2 adds the last one where the first two leave doubt.

## Task 1: conclude a pile from the history

With this segmentation, it is not possible to follow the object in a hand all the way. The hand and the object
are one outline. That outline lies across the edge of the search area and is not measured. It is possible to
observe the state before and after. The rule is conservation:

1. **A is gone.** Track A was in view. Its place is now in view and empty (`missing`). No outline of its
   colour has appeared elsewhere (the tracker already finds moved objects again by colour).
2. **B changed after A was last seen.** A track B is in view and lies still. It now differs from how it
   looked when it last lay undisturbed. Its colour no longer fits its name, its outline is clearly larger,
   or its measured height is clearly greater.
3. **Then A is on B.** The tracker marks this as a conclusion and not an observation: `on: B`,
   `on_source: "inferred"`, `on_evidence: ["colour", "area", "height"]` (those that applied). A pile that
   the arm built itself has `on_source: "placed"`.

Details:

- [x] **Undisturbed look.** Each track keeps how it looked while it lay still and unremarkable (`calm`: outline
      area, height). The tracker does not update this look while any track is missing and unexplained. Thus
      the comparison is always with the time before A went away. The look belongs to the place where it was
      taken. Area and height count as evidence only while B has not been moved, because the same object has
      another outline elsewhere.
- [x] **No renaming while a pile may be forming.** If a track went missing in the last `STACK_WINDOW` seconds,
      the tracker does not rename a track whose colour stops fitting. The changed colour is evidence, not a
      new name.
- [x] **The conclusion** is as above. The tracker makes it when B has lain still for a moment and nothing
      moves on the table. If several tracks are missing, the tracker takes the track whose colour B now
      shows. Otherwise, the tracker takes the track that was lost last. There is at most one new rider per
      base and look.
- [x] **Taking it back.** When A appears again elsewhere, that already clears `on` (existing code: "fell off
      what it was put on"). In addition, the tracker dissolves an inferred pile whose base looks undisturbed
      again (its evidence gone for a second). A is then `missing` again.
- [x] **Report.** `on_source` and `on_evidence` are in `tracks.json` and `track_objects`. The picture labels an
      inferred pile `on blue-1?`. `pick_object` needs no change. It already takes the rider from the top of
      the pile, at `z_mm` (the base's height when known).
- [x] **Test** in the simulator (`tests/test_workspace.py`). The test moves a block onto another without the
      arm. The tracker reports `on` with `on_source: "inferred"`, and the base keeps its name. When the block
      is taken off and put elsewhere, it is found again and `on` is cleared. A block that is simply removed
      stays `missing`.
- [x] Docstrings (`tracker.py`, `track_objects.py`), `skills/pick-and-place`, `docs/FEATURES.md`,
      `docs/CHANGELOG.md`.

This was built on 2026-11-22 (`Watcher._piles`), and tested in the simulator only. There is one addition to
the rule. When the pile reads as the upper object, the tracker first sees A "move" onto B's place and B go
missing. The tracker reverses that: B gets its place back and A is on it, with `on_evidence: ["place"]`.

What it does not do:

- An object laid **against** another (touching, side by side) is also one outline that grew and changed
  colour. The tracker reports it as `on`. That is what `inferred` says, and what task 2 settles.
- Two objects of the same colour: only area and height stay as evidence. This is weaker, but still usable.
- A pile that was already there when the tracker started has no history. Only task 2 (or the weak
  single-frame hints) can find it.
- If an object is taken off the table permanently and its neighbour is nudged at the same moment, the tracker
  can read this as a pile. Again `inferred`.

## Task 2: when unsure, look with the wrist camera

A new tool `inspect_object(name)`:

1. It reads the track. It goes `home` if the arm hides it.
2. It moves the wrist camera to a slanted view of the object. The view is about 45 degrees below horizontal,
   from the robot's side, 12 to 15 cm away. The IK goes through `ctx.plan.solve`, as `look_at_table` does.
   The tool tries a few distances and pitches until one is reachable.
3. It captures the wrist image and returns it for the agent to open, with a simple reading. The tool cuts the
   outline nearest to the expected position of the object into bands from bottom to top. It counts bands of
   clearly different colour as `layers`, with a colour word each.
4. It writes the result back. A confirmed pile becomes `on_source: "seen"`. A refuted pile is dissolved. The
   tracker is another process, so the result goes through a small file that the tracker reads
   (`.runtime/tracker_notes.json`).

When to call it (a line in `workspace/AGENTS.md` and the pick-and-place skill): Call it before you pick from
or stack on anything with `on_source: "inferred"`. Also call it for an outline whose colour is a mixture, or
whose height does not fit what it is said to be.

Limits:

- Beyond about 30 cm from the base, only slanted views from the robot's side are reachable.
- Positions through the wrist camera are 1 to 2 cm off. That is enough to count layers, but not to measure
  them.
- Same-coloured layers do not separate into bands. The tool still returns the image, and the agent judges it.
- The segmentation does not see black objects and objects the colour of the mat.

Steps:

- [ ] `tools/inspect_object.py`: the viewing pose and the capture.
- [ ] The layer count in the wrist image.
- [ ] `tracker_notes.json`, read by the tracker: confirm or dissolve a pile.
- [ ] Test in the simulator, then on the real arm with two cubes of different colours.
- [ ] `workspace/AGENTS.md`, skill, `docs/FEATURES.md`, `docs/CHANGELOG.md`.

## Order

Task 1 comes first. It needs no motion, it runs in the simulator's tests, and it gives task 2 its trigger.
Before you trust either task on the real arm, re-run `track_objects` there. This is open since the camera
change (refer to `docs/PLAN.md`). Both tasks belong to the "world model above the tracker" item of that plan.
