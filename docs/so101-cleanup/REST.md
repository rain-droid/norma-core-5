# Folded rest and test lifecycle

User preference: folded rest is the standard start and finish for every test. This is a powered parking pose; parking does not switch off torque.

The session config stores the setup-specific five arm angles in `poses.rest`. The current SO-101 pose comes from the first observation of recorded session `2f7a378764ef470db0fd46c60a1d7b59`, after the user's earlier folded parking and torque-off:

```json
[0.0030679615757712823, -1.7165245016440325, 1.5753982691585535, -0.23930100291016002, 0.9188544919434991]
```

Joint order is shoulder pan, shoulder lift, elbow flex, wrist flex, wrist roll; units are radians. This belongs to bus 5B91029197 and its pinned pgripper calibration, not every SO-101. No gripper endpoint is replayed: its measured open position is retained.

## Routine

1. Inspect fresh views and telemetry; establish one controller owner and a clear full folding/unfolding path.
2. `prepare_alignment` checks whether each arm joint is within 0.015 rad of rest. If needed, it moves to rest and verifies arrival, then moves to the saved ready hover. Select/locate the target from the new images.
3. Probe and, if validated, perform the bounded local alignment test. Configured-rest sessions reject a probe if preparation has not run.
4. Finish every successful test with `park`, inspect the returned view and `at_rest`, then close the service when handing control away.

```sh
.venv/bin/python scripts/visual_alignment.py park \
  --observation-id FRESH_INSPECTED_ID --empty-gripper-confirmed --scene-clear
```

`park` uses the same freshness, stable telemetry, open/empty gripper, full path and cancellation checks as preparation. It clears local maps and preparation state, leaves torque on, and reports `at_rest`. Status exposes `return_to_rest_pending` during a prepared test. The operator must call `park` at completion; individual probe/alignment calls do not immediately fold because a validated local map is needed for the next step.

A fault holds the arm and invalidates preparation. The routine never blindly parks through stale telemetry or uncertain physical state. An actual operator instruction, fresh observations and `operator_confirmed=true` are required to park after a fault or attempted pickup. A newer stop always wins. Do not use empty-gripper confirmation for a held object.

## Faster transfers

Folding/unfolding paths are validated every 0.025 rad, then executed in segments no larger than 0.20 rad. This avoids stopping at every tiny validation point. Executor velocity, following checks, calibrated joint bounds and telemetry checks remain active. Current speed is 0.45 rad/s; runtime cadence is 10 Hz. Before each write, actual joints must remain within 0.08 rad of the previous command, and the next command must also stay within that bound. If only the next command is too far ahead, the controller waits in 20 ms intervals (maximum one second) for the arm to catch up without sending farther goals. A real tracking violation, stale telemetry or timeout still stops the move. Visual correction paths retain their fine segments.

Offline parking tests verify rest → hover → rest, mandatory preparation, explicit fault recovery, torque preservation and dense validation with bounded execution segments. Physical outcomes are recorded in HANDOFF.md and EVALS.md.
