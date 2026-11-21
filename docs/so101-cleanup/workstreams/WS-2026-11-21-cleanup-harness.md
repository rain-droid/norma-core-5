# WS-2026-11-21-cleanup-harness

## Objective

Build a runnable Codex cleanup harness for arbitrary calibrated tabletop points, with Station transport, SO-101 IK, bounded execution, camera observations, and explicit visual verification.

## Status

complete for disconnected implementation scope; physical validation pending hardware

## Branch

- `codex/so101-station`

## Scope

- Offline mock backend and demo; real Station backend; CLI tools and MCP shim; table calibration and workspace bounds; one-item pickup/drop plans and stop/hold.

## Non-Goals

- Hardware is disconnected. No physical calibration, real motion, or success claims until bench validation. No new model API: Codex supplies visual recognition from tool images.

## Expected Files

- src/so101_station/cleanup/, scripts/cleanup_demo.py, tests/test_cleanup_*.py, docs/so101-cleanup/, pyproject.toml.

## Interfaces Affected

- Observation, proposed pickup, planned action, execution result and verification schemas; see coordination/INTERFACES.md.

## Acceptance Evidence

- IK/FK round trips, table projection bounds, guard failures, stop during multi-stage motion, protobuf camera/command checks, and end-to-end simulated pickup/drop with verification.
- Full regression suite: 43 tests and 15 subtests passed. Two-object synthetic demo: two verified cleanups using simulator truth. No physical action or visual-recognition accuracy claim.

## Handoff Log

| Date | Owner | Update |
| --- | --- | --- |
| 2026-11-21 | Codex | Workstream created. |
| 2026-11-21 | Codex | Implemented and verified offline; startup commands and Codex operating loop documented. Remaining bench/calibration work is listed in HANDOFF.md. |
