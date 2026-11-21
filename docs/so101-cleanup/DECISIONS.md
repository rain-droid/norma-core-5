# Decisions

- 2026-11-21: User requires arbitrary table positions; fixed pickup slots rejected. Use measured planar grounding and SO-101 IK.
- 2026-11-21: User says hardware is disconnected. Build/test offline; no live motion.
- 2026-11-21: Codex supplies recognition and final visual verification; no additional model API/key.
- 2026-11-21: One local service owns the bus for CLI/MCP. Stop cancels all remaining stages while retaining torque.
- 2026-11-21: First version uses a fixed camera. Moving wrist-camera grounding needs additional hand-eye/pose calibration.

- 2026-11-21 update: Hardware was connected and a paper pickup completed using taught poses plus Codex-selected visual corrections. The earlier disconnected-only scope is historical. No workspace-wide camera map was validated.
- 2026-11-21 update: User requested analysis and incremental planning after folding and torque-off. The dynamic-loop workstream is a proposal, not implemented behavior. Start with a persistent controller around proven actions; the objective does not require a printed mat or fixed pickup slots.

- 2026-11-21 implementation: added checkpointed session mode alongside the existing calibrated-table service; dual-camera observations, bounded actions, explicit torque and visual stage gates. User requested 3× motion speed; configured 0.45 rad/s and matching arm RAM speed, retaining separate gripper control.
- 2026-11-21 physical test: second grasp/lift verified; carry stopped on stale Station telemetry. Current state supersedes prior torque-off notes: paper held near cover, torque on, stop latched. No delivery success counted.

- Final update: user explicitly instructed finishing placement after the fault. Tested resume_holding performs no motion, checks fresh stable state and requires a new visual held assessment. Carry/release/retreat then succeeded; paper delivery counted once. Current state: open empty gripper above cover, torque on. User requested dedicated gpt-6-astra operator at low reasoning in this same chat; parent builds, operator exclusively owns live actions.
