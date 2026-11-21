# Next iterations

See the [dynamic loop plan](workstreams/WS-2026-11-21-dynamic-loop.md) for acceptance criteria. The first controller slice is implemented; later iterations remain proposed.

[Recent research](coordination/RESEARCH-2026-11-21.md) now informs the plan: separate execution from visual success, preserve uncertainty, validate local image corrections, resume from observed checkpoints and count aborted trials. Research itself did not move hardware; subsequent session-controller implementation and physical tests are recorded in SESSION.md and HANDOFF.md.

1. Implemented persistent CLI/MCP session controller, dual-camera results, action IDs, stage gates, explicit torque and stop. Physical paper delivery completed after one empty grasp and an operator-approved recovery from stale telemetry.
2. Add dynamic pickup within a validated region, keeping the drop fixed. Validate local image-to-motion corrections and evaluate five untaught placements.
3. Add bounded recovery for target motion, empty grasp, failed lift and uncertain placement. Transport/tracking faults remain stop conditions.
4. Validate coverage across the reachable table, dynamic destination grounding, multiple objects and a separate wrapper grasp strategy.
5. Measure latency, coordinate controller/servo speed profiles and add overlays/replay.

Immediate: Claude owns the arm; Codex operator stays idle and its motion service is off. Stable sampling, exact anchor returns and reusable target selection/location are implemented. A fresh wrist target can feed probing by ID; loss/reselection/expiry invalidates its map. After user returns ownership, load the new runtime and repeat the neutral/probe acceptance trial, then establish a verified grasp image reference. The preceding 9.0 px residual rejected the map; no successful physical map exists. Keep fit gates unchanged. Investigate the telemetry gap without loosening freshness checks.

Outstanding physical evidence: successful local probe/return validation, a validated grasp image reference, workspace-wide image grounding, robust grasp verification, general collision clearance and wrapper pickup. Last state: stopped, powered, open/empty at approximately 70.7 mm modelled tip height near the paper. Read live state before operating.

Latest requested workflow: every test starts with rest-first `prepare_alignment` and finishes with `park`. Folded pose is recorded in the session config and REST.md. Probe paused while completing the user-requested parking recovery; no fresh physical map exists.

Latest hardware result: both overhead objects located/tracked; improved probe still rejected−5mm sample on image stability timeout. Arm returned to folded rest after approved recovery; serviceOFF. Next: investigate settling/sample timing using saved diagnostics, retain existing fit/stability thresholds, then retry after explicit hardware direction.

Implemented next sampling revision: separate cumulative3s motor-settling and1.2s image budgets, three-frame burst, exact structured gate diagnostics, and diagnostic one-axis mode. Next physical acceptance: rest→hover→X-axis test→rest; full2D map and grasp reference remain pending.

Latest acceptance: separated sampling budgets worked onhardware; allfour X-axis samples passed three-frame stability withoutfault. Repeatability remainsfalse (10.43px return error). Joint returns differed by up to3ticks; investigate tighter endpoint control/consistent approach while retaining current map/image criteria. Arm folded, serviceOFF.
