# Reusable local probe and alignment

The session sequence is `prepare_alignment` → `probe_alignment` → `align_target` → `park`. [Folded rest](REST.md) is the standard start and finish. The Astra operator selects the target patch and inspects checkpoints; the local controller handles each probe and correction without a separate model/tool round trip. The routine starts from fresh observations and has no model API dependency.

This version aligns a tracked **image feature at hover height**. It does not discover table height, detect contact, determine a grasp pixel, or make arbitrary positions across the whole table reachable. The first physical acceptance exercise is perturb-and-return to the initial image location. That establishes a local feedback loop, not a grasp calibration.

## Start fresh

Start the checkpointed service using the real measured session config, as described in [SESSION.md](SESSION.md). Never run a second motion controller. With the operator's task authorized, use an empty/open gripper at the configured travel clearance and inspect a clear horizontal corridor. The current setup uses 70 mm modelled tip height and 0.45 rad/s nominal arm speed.

```sh
.venv/bin/python scripts/visual_alignment.py observe
```

Inspect both returned image paths and the route to the saved neutral ready hover. If the empty gripper needs returning to neutral:

```sh
.venv/bin/python scripts/visual_alignment.py prepare \
  --observation-id FRESH_INSPECTED_ID --empty-gripper-confirmed --scene-clear
```

This holds the measured open gripper, checks five stable telemetry samples, invalidates old probe maps and returns to configured folded rest if needed, verifies it, then moves to saved `pickup_hover`. Neutral is a coarse ready pose; it does not imply that the gripper is over the moved paper. After a fault or attempted pickup, add `--operator-confirmed` only when the user has actually directed a fresh start. A new stop still cancels preparation.

Inspect the returned fresh images and reacquire the paper. Select a distinctive textured patch **on the stationary target**, excluding the gripper and background. Bounding box units are wrist-image pixels `[x,y,width,height]`, at the image's actual resolution.

```sh
.venv/bin/python scripts/visual_alignment.py probe \
  --observation-id FRESH_INSPECTED_ID --bbox X Y WIDTH HEIGHT --scene-clear
```

The controller checks all probe destinations, then moves +5 mm X, returns, −5 mm X, returns, +5 mm Y, returns, −5 mm Y, returns. The command frame stays anchored to the initial robot pose. It tracks the original patch after each move, fits a 2×2 pixel-per-commanded-mm response, and validates on a separate `[3,3]` mm move before returning. Source timestamps and actual measured positions accompany each sample.

Before accepting a measurement, the controller requires a 150 ms quiet encoder window (at most two ticks of variation), then samples telemetry throughout camera capture. It checks the camera observation's own motor reading too. After a commanded move, each arm joint must be within 0.0047 rad of the target. Three consecutive fresh images must have a maximum pairwise displacement of 1.5 px at a consistent measured pose, with gaps no greater than 250 ms. Settling has a cumulative three-second budget; image collection has its own cumulative 1.2-second budget. Movement during capture discards the burst and returns to settling without resetting either budget; it never sends extra motor commands to force settling. Real telemetry older than 100 ms cannot establish a quiet sample, while the existing transport fault cutoff remains 500 ms.

Zero-offset returns restore the exact initial joint angles, including wrist orientation, instead of solving a new IK pose. Logged `visual_stable_sample` events and per-step `sampling` fields contain the observation's measured pose, frame/state timestamps, encoder variation and image drift. These bracketed samples improve timing evidence; they are not hardware-synchronized measurements. The latest separated-budget implementation is offline-tested; hardware outcomes are recorded in HANDOFF.md.

After each visual move, bounded encoder refinement now distinguishes the nominal endpoint from the servo command. It waits for a 150 ms window with at most one tick of motion and telemetry age at most 100 ms. A settled error may adjust a joint goal by at most two ticks per correction, with total bias limited to six ticks, measured deviation limited to eight ticks, five corrections and a four-second deadline. Every compensated path and measured waypoint retains the joint, table and horizontal-corridor checks. Stalls, changed gripper, stale state and stops abort the refinement.

Acceptance requires each joint within three ticks of the quantized nominal target (the original approximately 0.0047 rad joint bound), model endpoint error at most 0.5 mm and rotation-matrix difference at most 0.006. Small opposing joint errors can cancel at the tool; the first live experiment showed that chasing an extra encoder tick despite acceptable tool pose caused an unnecessary stall. The endpoint geometry and original joint checks remain active throughout the subsequent image burst. `sampling.endpoint` records measured/desired/command ticks, residuals, timing and correction count. This tests encoder/model repeatability; it does not prove physical tool accuracy or replace the five-pixel image-map acceptance checks. It is applied only to visual hover moves, not folding or carrying.

The returned `probe_id`, `reference_pixel`, response matrix, fit error, held-out error and return drift describe that local experiment. Poor texture, ambiguous tracking, weak response, excessive drift or a failed independent check reject the map. Lack of visible motion does **not** mean table contact and never authorizes pushing harder.

Patch tracking remains the default. `probe --tracker features` selects the optional fixed-reference multipoint tracker, with bidirectional optical flow, RANSAC and original-patch corroboration. See [TRACKING.md](TRACKING.md) for its tested limits. It has synthetic and offline validation, not demonstrated physical accuracy superiority.

Alternatively, [select and locate a reusable wrist target](TARGETS.md), then use `probe --target-id ID` with that exact observation. The probe records the target selection generation; later reselection, loss or expiry prevents using its map. The target's stored tracker is used. Pixel coordinates still do not define a grasp reference.

## Start with one axis

Use `probe --axis x` for the next acceptance test: +5 mm X, return, −5 mm X, return. `--axis y` is also available. Select a wrist target from the fresh ready-hover observation as usual. Run `park` after a normally completed test.

```sh
.venv/bin/python scripts/visual_alignment.py probe \
  --observation-id FRESH_INSPECTED_ID --target-id paper-wrist --axis x --scene-clear
```

This returns an `axis_test_id`, response vector, fit/return errors and `repeatable`. It never supplies a `probe_id` or caches a usable two-axis map. Weak or inconsistent measured responses report `repeatable=false` as diagnostics; no alignment or grasp is authorized. Actual sampling/motion faults still latch stop. The default `axis=xy` retains the full two-axis probe and independent diagonal validation.

Sampling failures now include `diagnostic.code`, `phase`, `metrics` and `timing` in action results and events. Codes distinguish joint target error, encoder movement, telemetry freshness/unavailability, image drift, frame gap/order, camera capture failure, lost tracking and image-processing timeout. Metrics retain per-joint errors, tick spans and frame drift. A rolling three-frame burst must stay quiet across **all** its motor samples; opposite excursions cannot cancel in the check.

## Align locally

After a small checked perturbation, take and inspect a new observation. Use `reference_pixel` as the goal for the initial return test:

```sh
.venv/bin/python scripts/visual_alignment.py align \
  --observation-id FRESH_INSPECTED_ID --probe-id PROBE_ID \
  --goal-pixel U V --scene-clear
```

The local loop uses the measured response to compute bounded XY corrections, reacquires frames after each move, and requires two fresh frames within 4 pixels. It does not descend, close the gripper or count a cleanup. A later grasp-alignment use requires a separately validated grasp reference; do not substitute image centre or visible tip midpoint by assumption.

Limits: 5 mm maximum correction, 25 mm cumulative correction path, eight moves, 10 mm neighbourhood around the probe origin (9.5 mm command margin), 60 pixel initial error, 45 second alignment timeout. Height, camera identity, resolution and orientation must remain compatible. Maps expire after 300 seconds, are cleared by stop, and cannot be loaded into a new process. Saved probe JSON is an evidence artifact, not an automatically trusted calibration cache. Rerun the reusable routine in a fresh setup.

`status.active_progress` shows the current phase/sample or pixel error. Existing stop, one-writer, stale telemetry, geometry and following-error checks remain active. If vision fails after movement, the controller latches stop and holds where possible; it does not blindly move back. It saves all intermediate full images and probe/step events in the session log.

The CLI records request/result JSON under `logs/visual-client`. It prints the request ID before sending: after a timeout, retry with the **same** `--request-id`, observation ID and arguments. Never obtain an unseen fresh observation merely to make an old decision pass the freshness check.

## Validation boundary

Synthetic image-motion tests validate the fitted direction, closed-loop convergence and failure gates. They include servo quantization through the mock arm model, but not real backlash, flexible trash or full camera geometry. Physical test outcomes are recorded in [HANDOFF.md](HANDOFF.md) and [EVALS.md](EVALS.md).
