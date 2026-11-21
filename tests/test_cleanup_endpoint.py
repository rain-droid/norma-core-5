import numpy as np
import pytest

from so101_station.cleanup.endpoint import next_trim
from so101_station.cleanup.geometry import CleanupError


DESIRED = [2000, 2100, 2200, 2300, 2400]


def test_mixed_error_signs_step_by_two_and_preserve_one_tick_deadband():
    measured = [1997, 2103, 2199, 2301, 2400]
    command = DESIRED.copy()
    assert next_trim(DESIRED, measured, command) == [2002, 2098, 2200, 2300, 2400]
    assert command == DESIRED
    assert measured == [1997, 2103, 2199, 2301, 2400]


@pytest.mark.parametrize("offset", [-3, 3])
def test_constant_three_tick_offset_converges_within_one_tick(offset):
    command = DESIRED.copy()
    for _ in range(4):
        measured = [value + offset for value in command]
        if all(abs(goal - actual) <= 1 for goal, actual in zip(DESIRED, measured)):
            break
        command = next_trim(DESIRED, measured, command)
    else:
        pytest.fail("constant bias did not converge")
    assert max(abs(goal - actual) for goal, actual in zip(DESIRED, measured)) == 1
    assert max(abs(goal - value) for goal, value in zip(DESIRED, command)) == 2
    assert next_trim(DESIRED, measured, command) == command


def test_alternating_small_bias_stays_bounded_but_does_not_claim_convergence():
    command = DESIRED.copy()
    seen = []
    for iteration in range(40):
        # A changing two-tick reading can sustain an oscillation. The caller's
        # time/stall limits must end it; this stateless policy must remain bounded.
        offset = -2 if iteration % 2 == 0 else 2
        measured = [value + offset for value in command]
        proposed = next_trim(DESIRED, measured, command)
        assert max(abs(new - old) for new, old in zip(proposed, command)) <= 2
        assert max(abs(new - goal) for new, goal in zip(proposed, DESIRED)) <= 6
        seen.append(proposed)
        command = proposed
    assert len({tuple(values) for values in seen}) == 2


def test_stalled_encoder_exhausts_allowance_instead_of_unbounded_commands():
    measured = [value - 3 for value in DESIRED]
    command = DESIRED.copy()
    for expected_bias in [2, 4, 6]:
        command = next_trim(DESIRED, measured, command)
        assert command == [value + expected_bias for value in DESIRED]
    with pytest.raises(CleanupError, match="exhausted"):
        next_trim(DESIRED, measured, command)
    assert command == [value + 6 for value in DESIRED]


@pytest.mark.parametrize("sign", [-1, 1])
def test_eight_tick_measurement_boundary_is_allowed_but_nine_rejected(sign):
    measured = DESIRED.copy()
    measured[2] += sign * 8
    expected = DESIRED.copy()
    expected[2] -= sign * 2
    assert next_trim(DESIRED, measured, DESIRED) == expected
    measured[2] += sign
    with pytest.raises(CleanupError, match="measured deviation"):
        next_trim(DESIRED, measured, DESIRED)


@pytest.mark.parametrize("sign", [-1, 1])
def test_existing_and_proposed_command_bias_are_both_checked(sign):
    command = DESIRED.copy()
    command[4] += sign * 7
    with pytest.raises(CleanupError, match="existing command bias"):
        next_trim(DESIRED, DESIRED, command)
    command[4] = DESIRED[4] + sign * 5
    measured = DESIRED.copy()
    measured[4] -= sign * 3
    with pytest.raises(CleanupError, match="exhausted"):
        next_trim(DESIRED, measured, command)
    assert command[4] == DESIRED[4] + sign * 5  # No partial update on rejection.


def test_at_bias_limit_deadband_is_preserved_and_reverse_correction_is_allowed():
    command = [value + 6 for value in DESIRED]
    assert next_trim(DESIRED, [value - 1 for value in DESIRED], command) == command
    assert next_trim(DESIRED, [value + 2 for value in DESIRED], command) == [value + 4 for value in DESIRED]


@pytest.mark.parametrize("position", [0, 1, 2])
@pytest.mark.parametrize("invalid", [None, [1, 2, 3, 4], [1, 2, 3, 4, 5, 6],
                                    [1, 2, 3, 4, True], [1, 2, 3, 4, 5.],
                                    [1, 2, 3, 4, .5], [1, 2, 3, 4, float("nan")],
                                    [1, 2, 3, 4, float("inf")], [1, 2, 3, 4, "5"],
                                    [[1], [2], [3], [4], [5]]])
def test_invalid_tick_inputs_rejected(position, invalid):
    args = [DESIRED, DESIRED, DESIRED]
    args[position] = invalid
    with pytest.raises(CleanupError, match="exactly five integer"):
        next_trim(*args)


def test_numpy_integer_ticks_return_plain_integer_list_without_mutating_inputs():
    desired = np.array(DESIRED, dtype=np.int64)
    measured = desired - 2
    command = desired.copy()
    result = next_trim(desired, measured, command)
    assert result == [value + 2 for value in DESIRED]
    assert all(type(value) is int for value in result)
    assert np.array_equal(command, desired)
    assert np.array_equal(measured, desired - 2)


def test_zero_tolerance_corrects_residual_one_tick_errors_by_one_tick():
    measured = [1999, 2101, 2200, 2297, 2403]
    assert next_trim(DESIRED, measured, DESIRED) == [2000, 2100, 2200, 2302, 2398]
    assert next_trim(DESIRED, measured, DESIRED, tolerance=0) == [2001, 2099, 2200, 2302, 2398]


def test_stricter_final_step_can_finish_constant_three_tick_offset():
    measured = [value - 3 for value in DESIRED]
    command = next_trim(DESIRED, measured, DESIRED)
    measured = [value - 3 for value in command]
    assert next_trim(DESIRED, measured, command) == command
    corrected = next_trim(DESIRED, measured, command, tolerance=0)
    assert [value - 3 for value in corrected] == DESIRED


def test_zero_tolerance_does_not_relax_bias_or_measured_deviation_caps():
    with pytest.raises(CleanupError, match="exhausted"):
        next_trim(DESIRED, [value - 1 for value in DESIRED], [value + 6 for value in DESIRED], tolerance=0)
    with pytest.raises(CleanupError, match="measured deviation"):
        next_trim(DESIRED, [value - 9 for value in DESIRED], DESIRED, tolerance=0)


@pytest.mark.parametrize("tolerance", [-1, 2, .5, 1., True, None, "0", float("nan")])
def test_invalid_tolerance_rejected(tolerance):
    with pytest.raises(CleanupError, match="tolerance"):
        next_trim(DESIRED, DESIRED, DESIRED, tolerance=tolerance)
