---
name: push-t
description: Push a flat T-shaped block across the table with the closed fingertips until it lies on a target position and heading (Push-T).
---

Proven in the simulation only (2026-11-22: two starts, 3 to 8 cm and 25 to 60 degrees off,
ended within 3 mm and 5 degrees in 8 to 11 pushes). Not yet tried on the real arm: when you
do, write what happened here.

1. Clear the table between the T and its target, and around the T: `push_t` knows about
   nothing but the T. `home`, wait for motion to finish, then `depth_scan` at fresh top-camera
   T/target coordinates to measure thickness and check the surface. Inspect the height map;
   missing depth is unknown. `scan_obstacles` installs the separate collision map.
2. Look first, moving nothing: `push_t` with the target and `max_pushes: 0`. Open `image`: the
   cyan outline must sit on the T and point the way it points (the short line from its centre
   goes towards the bar). If it sits on something else or `seen_overlap` is below about 0.7,
   the shape is wrong: measure the T and give `shape_mm` [bar width, bar depth, stem width,
   stem length] and `height_mm`.
3. `push_t` with `x`, `y`, `yaw_deg`. It looks, pushes at most 12 mm, and looks again until the
   T is within tolerance. Read `pushes`: the error after each should shrink overall; single
   pushes that make it worse are normal.
4. Open `image` and check the cyan outline on the magenta one before you report. `overlap`
   above about 0.75 is a T that visibly lies on its target.

What to expect on the real arm, and has not been measured: since mat contact on 2026-11-22,
approaches add a further 15 mm clearance on top of the 10 mm model correction and every joint
path enforces a 25 mm modelled fingertip floor. The old 12 mm minimum T thickness is no longer
adequate evidence of contact: measure its thickness with depth and check that the fingers can
reach its side while retaining the margin. Do not lower the floor to reach a thin T.
Positions are 1 to 2 cm off beyond 30 cm from the base, so keep
T and target within 30 cm and ask for `tolerance_mm` 10, `tolerance_deg` 10; the T needs a
colour the top camera tells from the mat (not black, not the mat's own).

Errors: "no T in view" (covered by the arm or a hand, off the searched area x -0.24..0.24,
y 0.14..0.40, or the wrong `shape_mm`); "no useful push that the arm can make" (the T is beyond
reach or against the base: move it by hand, or pick a nearer target).
