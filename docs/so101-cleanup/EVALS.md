# Evaluation

## Bounded endpoint refinement — current build

Added hover-only goal compensation after a fresh, quiet encoder window, retaining nominal versus compensated targets, path checks, gripper invariants, stop, small bias limits and deadlines. The initial build required one tick, 0.5 mm model displacement and 0.006 rotation-matrix difference throughout image sampling. Acceptance still requires the unchanged image-map criteria.

Offline validation: 52 pure policy tests and nine integration tests passed. Integration covers steady bias through reversals, settling without unnecessary correction, bounded stalled-servo failure, stop, compensated-path rejection, measured corridor escape between waypoints, stale telemetry/gripper changes, post-correction drift and a synthetic image probe with biased encoders. The broader focused run passed 92 of 93 before correcting a test's write-count expectation: each trim uses two checked segments with two executor writes per segment. The final nine integration tests passed after that test correction and the final pre-write gripper check. Other visual, sampling and parking checks passed in the broader run. A prior image-drift test encountered the equally rejecting image-processing deadline on one run, then passed on rerun; no production limits were loosened.

First physical trial `d2306247774b4608b7dec746f280bd11` faulted on a persistent two-tick elbow residual before collecting the first probe image. Initial endpoint error was already 0.356 mm with zero orientation difference; after correction it was 0.377 mm and 0.00217 rotation-matrix difference. The experimental one-tick criterion caused unnecessary correction despite acceptable modeled tool pose. No map was produced.

Revised acceptance retains the original three-tick/0.0047 rad joint bound together with the 0.5 mm/0.006 pose bounds; both the rounded nominal target and the original fractional IK target must satisfy their joint limits. The exact physical endpoint is now an offline regression proving no unnecessary correction, while an additional fractional-target regression prevents convergence followed by immediate capture rejection. Endpoint diagnostics now propagate into session results. **107 focused tests passed** after the pose-criterion and diagnostic changes; **two targeted tests passed** after the final fractional-IK consistency fix (one new test, one repeated). Physical acceptance of the revised implementation remains pending; synthetic results do not establish improved real grasp accuracy.

Run `uv run --offline --frozen --with pytest python -m pytest -q` with cached dependencies (omit --offline initially). Run `uv run python scripts/cleanup_demo.py` for the two-object synthetic routine.

Tests cover imported calibration, protobuf registers/cameras, continuous pixel mapping, invalid maps, IK/reachability, joint continuity, trash-only decisions, stale/changed observations, exact-bus six-motor writes, latched stop, single-use plans, explicit verification, HTTP and MCP images.

2026-11-21: full offline suite passed, **43 tests and 15 subtests**. The two-object synthetic demo completed paper and wrapper pickup/drop and recorded two simulator-verified cleanups. Image/event evidence is under logs/cleanup-demo/. A moved-object regression checks the local grasp region as well as the whole image before motion.

A real localhost process smoke test also passed: CLI status, MCP initialization/listing of all six tools, native JPEG delivery, and MCP stop against the simulated service. The temporary service was shut down afterward. The unfilled hardware workspace template correctly fails validation.

Station camera stamps were checked against the pinned upstream [Unix-epoch implementation](https://github.com/norma-core/norma-core/blob/6b494f77fac366a878ae71222458b196f8400dee/software/station/shared/systime-rs/src/local.rs) and [USB frame pipeline](https://github.com/norma-core/norma-core/blob/6b494f77fac366a878ae71222458b196f8400dee/software/drivers/usbvideo/src/pipeline.rs). Cross-host clock synchronization is required.

Synthetic fixtures use the actual codecs and robot model. They do not validate visual-recognition accuracy, friction, paper compression, wrapper graspability, live calibration, torque settings or mesh collisions. Hardware must follow the README bench sequence.

2026-11-21, first hardware test: Station bus 5B91029197, motor 5, actual encoder 2647 → 2636 → 2647; one-degree wrist test completed and returned. Only wrist torque enabled; other motors remain off. Exact result is logs/so101-first-control/wrist-test-executed.json. This establishes one-joint Station control, not full-arm or cleanup validation. The new bench tests and existing backend tests passed together (7 tests), including preview without writes, refusing changed starting positions/other powered joints, and stopping on passive-joint movement.

Later on 2026-11-21: one real paper cleanup succeeded on the second grasp attempt using taught poses and Codex-guided visual corrections. Lift and placement were inspected; paper is visible on the plastic cover and its former location is clear. See logs/taught-pickup/result.json and second-after-drop.jpg. Folded parking and subsequent torque-off were also confirmed. The wrapper was not picked. These are single-run observations, not a statistical reliability result; earlier test counts predate subsequent live-script/speed/UI changes.

## Session-controller validation

Full suite before speed adjustment: 55 tests and 15 subtests passed. After matching controller/servo speed to the user request: 14 focused backend/session tests passed. Real MCP stdio discovery returned all six session tools and both live camera images. Physical second grasp/lift verified; carry was cancelled due to stale motor telemetry, with paper still held near the cover after telemetry recovered. Delivery unverified; do not count a successful cleanup.

After explicit operator instruction, held-object recovery and fresh assessment enabled remaining carry/release/retreat. Paper delivery was visually verified on the cover; one cleanup counted. This is one physical trial with two grasps and an operator-mediated transport-fault recovery, not an autonomous reliability result.

Final verification after explicit recovery support: **59 tests and 15 subtests passed**. Documentation links and git diff whitespace checks passed.

## Reusable visual probe and fresh start

Full suite: **98 tests and 15 subtests passed**. Includes local response fitting, independent validation, image convergence, stop/expired-map/lost-target gates, horizontal path bounds, preservation of the open gripper, neutral preparation and lower command cadence. Optional multipoint tracking has 15 synthetic tests; offline replay has 12 tests. Real MCP smoke against session 33e2235a26904e9da8b5fb1aa7173c0a exposes nine tools and two fresh live images.

Earlier approach trial a26ad9026f0a4c52ac0ceffc875cc86a stopped on stale motor timestamps before probing. After the user explicitly requested a neutral fresh start, the tested service restarted with 10 Hz arm writes as an experiment; speed and freshness bounds remained unchanged.

Physical trial 33e2235a26904e9da8b5fb1aa7173c0a: neutral preparation completed in 9.26 s; one +10 mm X nudge completed in 1.38 s. An initial probe was rejected before motion because the patch moved since selection; a freshly inspected selection then ran eight probe moves in 8.67 s. Tracking NCC ranged 0.930–0.995. The fit was rejected at 9.0 px maximum residual; no held-out diagonal, offset/return alignment or grasp followed. Stop latched and maps remained empty. Paper unchanged; open gripper and all six motor torques ON in final observation 1bfa43c942024395b7c280274093c43c.

Repeated commanded returns spanned 12.48 px X / 6.06 px Y and modelled positions spanned 1.20 mm X / 2.11 mm Y / 2.81 mm Z. An exploratory affine fit using measured XY still had 5.85 px max residual; switching to measured XY alone is not validated as a fix. Position was recorded before the associated image, so temporal alignment and settling need investigation. No stale-telemetry interruption occurred in this trial; this does not establish a transport fix. Synthetic alignment success is not evidence of a grasp calibration.

## Stable sample implementation

After that trial, added quiet motor windows, stricter final visual-move target checks, telemetry sampling during capture, the observation's own state in the quiet check, two stable fresh image samples, and exact initial joint restoration for zero-offset returns. `visual_stable_sample` records frame/state timing and variation; no extra motion commands are sent while waiting. The existing map-fit limits remain unchanged.

Offline validation: **31 focused tests passed** using `tests/test_cleanup_visual.py` and `tests/test_cleanup_session.py`. Seven new regression tests cover delayed settling, movement during capture, timeout/cancellation, target-image drift, exact anchor restoration, old telemetry and an anomalous observation-state sample. Independent read-only review verified cancellation and state/image checks. The runtime remains unchanged while Claude owns the arm. No claim of improved physical accuracy or successful object coordinates is made until a new hardware trial passes.

## Target memory and probe identity

Target tools remember an explicit image selection, report tracked/shifted/lost states, preserve camera/observation provenance, latch loss/expiry/view changes, and bind probes to a selection generation. Five integration tests plus the existing session/visual suites passed: **36 tests**. They verify no motor/camera calls from selection/location, stale/busy/duplicate gates, HTTP schema behavior, current-observation target resolution and map invalidation after reselection. MCP schema inspection exposes eleven tools without network access.

Offline recorded-image check used the selected paper ROI and all eight probe sample images from run 33e2235a26904e9da8b5fb1aa7173c0a. All eight matched, with minimum normalized correlation 0.930; each reported image shift, as expected from the moving wrist camera. Evidence: logs/target-memory-recorded-check.json. This is not a ground-truth identity/position benchmark or a new hardware run. The physical motion service remains off while Claude owns the arm.

Final full offline suite after target memory and stable sampling: **129 tests and 15 subtests passed**. Documentation links and whitespace checks passed. No physical service was started or robot command issued.

## Folded rest and transfer pacing

User requested folded rest as the start/end of every test and quicker motion. Added configured `poses.rest`, `park`, rest-first preparation, probe preparation gate, pending-return status and dense-validation/longer-execution transfer paths. The initial physical park in d0fd3b4b337543f99de5822fbbe5dab0 stopped after 0.896 s on the following-error gate; no folded arrival was claimed. The old check compared the next unsent waypoint to actual position, so a faster command step consumed the following-error allowance.

Pacing now checks tracking against the previous sent goal and waits, without further writes, before sending a next goal that would exceed the same 0.08 rad bound. Actual tracking error, stale state, stop or a one-second catch-up timeout still terminates motion. Tests cover delayed following, a stalled arm, stop during catch-up, dense path checks and rest lifecycle. **43 focused tests passed** (36 parking/session/visual and 7 legacy harness). User explicitly approved finishing the fold with this revised pacing; physical recovery outcome follows separately.

Approved physical recovery in session6c86146640904bf7b13fe7b886bbecf6 completed park in **8.85s**. Both final images (observation874ac1d7fcbe4cec9cda9fba5583d72e) show the arm folded, empty/open gripper, paper/cover untouched. at_rest=true, allsix torqueON, no telemetry issues. Service shut down and port8772 released. This validates one folded-rest recovery with adaptive pacing; no numerical speedup or new probe success is claimed.

## Live object memory and improved probe trial

Run5e1e6b093b884436ba6c73f9a84835e2 began at folded rest. Overhead paper and wrapper selections were located in fresh frames: centers[519.499,284.034] and[630.037,345.554], NCC0.9982/0.9930. One wrapper selection used an unsupported label and was corrected to snack_wrapper without motion. Rest-first preparation took7.75s; paper appeared in wrist view without coarse nudges.

A stale-observation probe request was rejected before motion; fresh inspection/location preceded the actual probe. It accepted +5mm X and zero-return samples, then stopped at−5mm X because no two stable image samples arrived within the3s deadline. Elapsed7.36s; no map or pickup. Accepted sample image drift was0.589px/0.744px, NCC0.9025/0.9746. Last failed-sample images shifted about4.6px while wrist-flex/roll readings changed2ticks; the later post-stop frame shifted only0.21px. These observations motivate sampling-timing work but do not validate a map.

After user confirmed path clear and explicitly approved recovery, park completed in8.71s. Final observation4861894d96d14976a7699ee2ab687ead shows folded empty arm and untouched objects. AlltorqueON. Service terminated and port8772 released.

## Separated sampling budgets and one-axis diagnostic

Implemented3s cumulative motor settling plus separate1.2s cumulative image budget, three-frame pairwise1.5px stability, <=250ms frame gap, retained joint/telemetry thresholds, structured failure codes/metrics/timing, and one-axis mode without a2D map.74 distinct focused tests passed across overlapping runs; independent review prompted regression fixes for cross-frame motor extrema, failure timing and synchronous processing deadlines.

Physical rune61fbc6a95b44be6917e28106194e32b: rest→hover7.397s; X diagnostic7.882s overall, allfour samples passed three-frame stability. Motor settling0.257/0.420/1.987/0.745s, image sampling0.218/0.408/0.329/0.208s; frame drift0.624/1.006/0.492/1.247px. No sampling/motion fault. However repeatable=false:10.4336px return/fit residual>5px. Gain6.5233px/mm. Artifactaxis-test-41d53568ed28460492579e0ff2e27dc4.json.

Measured joint offsets on zero returns were up to3ticks. FK offsets from initial pose were[0.047,0.683,-0.285]mm and[-0.709,-1.345,-0.616]mm. These support investigating endpoint repeatability/tolerance before full calibration; they are not independent physical position measurements.

Park completed8.307s, finalobs4c4ede0566ea41e3873757f848668076 confirmed folded and objects untouched. AlltorqueON. Service shut down and port8772 released. No2D map, alignment or pickup was claimed.
