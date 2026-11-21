"""Pure, bounded encoder-goal trim for the five arm joints.

Small endpoint errors can have several causes, including servo deadband or
load-dependent offset. This policy does not identify the cause. The caller owns
fresh telemetry, settling, calibrated joint/path limits, convergence checks,
timeouts and stall/oscillation detection. No gripper value is accepted or changed.
"""
from __future__ import annotations

from numbers import Integral

from .geometry import CleanupError


def _ticks(value, label):
    try:
        values = list(value)
    except TypeError as exc:
        raise CleanupError(f"{label} must contain exactly five integer arm ticks") from exc
    # Integral excludes NaN/infinity and fractional floats. Exclude bool too:
    # truth values must not silently become motor tick commands.
    if len(values) != 5 or any(isinstance(tick, bool) or not isinstance(tick, Integral) for tick in values):
        raise CleanupError(f"{label} must contain exactly five integer arm ticks")
    return [int(tick) for tick in values]


def next_trim(desired, measured, command, *, tolerance=1) -> list[int]:
    """Return one bounded correction to the previous five-joint command.

    Measurements more than eight ticks from the desired endpoint are rejected.
    By default, errors of one tick or less retain the current goal. Explicit
    ``tolerance=0`` also corrects residual one-tick errors by one tick. Larger
    errors adjust the goal toward the desired measurement by at most two ticks. Existing and new
    commands must remain within six ticks of the desired endpoint; exhausting
    that allowance raises ``CleanupError`` rather than silently clamping.

    Inputs must each contain exactly five integers; NumPy integer scalars are
    supported, but booleans, strings and floating-point values are rejected.
    Inputs are not mutated. A returned goal is not a motion authorization or a
    convergence result: the caller must validate and execute it separately.
    """
    if isinstance(tolerance, bool) or not isinstance(tolerance, Integral) or tolerance not in (0, 1):
        raise CleanupError("trim tolerance must be integer zero or one tick")
    desired_ticks = _ticks(desired, "desired")
    measured_ticks = _ticks(measured, "measured")
    command_ticks = _ticks(command, "command")
    errors = [goal - actual for goal, actual in zip(desired_ticks, measured_ticks)]
    for index, (goal, previous, error) in enumerate(zip(desired_ticks, command_ticks, errors), 1):
        if abs(error) > 8:
            raise CleanupError(f"arm joint {index}: measured deviation exceeds eight ticks")
        if abs(previous - goal) > 6:
            raise CleanupError(f"arm joint {index}: existing command bias exceeds six ticks")
    trimmed = [previous + (min(abs(error), 2) if error > tolerance else -min(abs(error), 2) if error < -tolerance else 0)
               for previous, error in zip(command_ticks, errors)]
    for index, (goal, proposed) in enumerate(zip(desired_ticks, trimmed), 1):
        if abs(proposed - goal) > 6:
            raise CleanupError(f"arm joint {index}: trim exhausted the six-tick command-bias allowance")
    return trimmed
