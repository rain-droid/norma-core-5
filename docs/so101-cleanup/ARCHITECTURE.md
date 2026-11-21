# Implemented architecture

Codex vision → CLI/stdio MCP → loopback FastAPI → Harness → Executor → Station TCP/protobuf → six servos. A mock backend supplies synthetic camera images and motor state.

ArmModel reads the five joint origins/axes from the pinned SO-101 Xacro and uses the pgripper tool0 transform. SciPy IK constrains position/downward approach with free jaw yaw. TableMap fits measured planar correspondences and checks residual, coverage and workspace bounds. Planning runs in a worker thread to keep stop responsive.

One service owns motion. Stop stays latched across all stages. Execution checks current pose and a new image globally and near the target, then opens, raises, approaches, descends, grasps, lifts, carries, releases and retreats. A new post-motion frame is required. Completion remains unverified until Codex supplies visual evidence.

No ROS or separate model API is required. Collision checks are coarse centerline/table checks, not full environmental collision avoidance. A fixed camera is required.

## Actual live pilot

The successful physical paper pickup used separate taught_pickup, visual_nudge and finish_pickup scripts over the shared backend/model/executor. Codex inspected overhead/wrist images and chose corrections after an empty grasp, then verified lift and placement. The taught drop stayed fixed. This did not exercise the full generic TableMap/plan_pick service, and no complete camera-to-table map was validated. Proposed consolidation is described in [the dynamic loop plan](workstreams/WS-2026-11-21-dynamic-loop.md).

## Checkpointed live session

`cleanup/session.py` now supplies an alternative to the original monolithic mapped-table workflow. `serve --session` selects it; `SO101_CLEANUP_MODE=session` selects its MCP contracts. Both clients share one service and bus lease. Fresh dual-camera observations, idempotent bounded actions and explicit held/delivered assessments control task progression. Station backend camera discovery matches camera identities rather than a boot-specific queue prefix. This path completed the paper demo with two grasps and an explicitly approved recovery from one stale-telemetry interruption. See [SESSION.md](SESSION.md).

The root agent builds/tests; the low-reasoning gpt-6-astra operator owns all live commands. Runtime motion stays in the local executor. The operator is dispatched whole tasks via followup_task; this is not a continuously running autonomous visual servo. Local tracking/probing remains the next iteration.
