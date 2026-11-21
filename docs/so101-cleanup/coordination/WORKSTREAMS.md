# Workstreams

| ID | Status | Owner | Branch | Scope |
| --- | --- | --- | --- | --- |
| WS-2026-11-21-cleanup-harness | complete for disconnected scope | Codex | codex/so101-station | Arbitrary-position cleanup, Station transport, geometry, CLI/MCP; 43 tests + 15 subtests passed |
| WS-2026-11-21-dynamic-loop | local probe ran; fit rejected, stop latched | Codex | codex/so101-station | Persistent stages, stable sampling, target memory, local image response/alignment, optional multipoint tracking and offline replay; 129 tests + 15 subtests |

Hardware owner: Claude, explicitly assigned by the user while Codex builds. `/root/robot_operator` (gpt-6-astra, low reasoning) is idle. No Codex live calls/restarts until the user returns ownership. Stable-sampling changes are offline-tested (31 focused tests) but not loaded into the running service.

Latest: user returned ownership from Claude; requested folded rest and quicker transfers. Folded parking completed and both cameras verified it after approved pacing recovery. Service is now OFF; torques remain ON holding rest. Operator/root idle. Future tests must use rest-first preparation and finish with park. Latest rest/pacing validation:43 focused tests.
