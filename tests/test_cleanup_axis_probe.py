import json

import numpy as np
import pytest

from so101_station.cleanup.axis_probe import summarize_axis_probe
from so101_station.cleanup.geometry import CleanupError


def samples(axis="x", response=(2., -3.), commands=(5., 0., -5., 0.)):
    command = np.asarray(commands)
    offsets = np.zeros((len(command), 2))
    offsets[:, 0 if axis == "x" else 1] = command
    reference = np.array([130., 210.])
    pixels = reference + command[:, None] * np.asarray(response)
    return offsets, pixels, reference


@pytest.mark.parametrize("axis,response", [("x", (2., -3.)), ("y", (-4., 1.5))])
def test_fit_preserves_axis_sign_scale_and_diagnostic_scope(axis, response):
    summary = summarize_axis_probe(axis, *samples(axis, response))
    assert np.allclose(summary["response_px_per_commanded_mm"], response)
    assert summary["response_gain_px_per_mm"] == pytest.approx(np.linalg.norm(response))
    assert summary["max_fit_error_px"] < 1e-10
    assert summary["max_return_error_px"] == 0
    assert summary["repeatable"] is True and summary["diagnostic_only"] is True
    assert summary["validated_2d_map"] is False and summary["grasp_calibrated"] is False
    assert "probe_id" not in summary
    json.dumps(summary, allow_nan=False)


def test_reverse_order_and_additional_complete_excursions_are_supported():
    data = samples(commands=(-5, 0, 5, 0, -5, 0, 5, 0))
    summary = summarize_axis_probe("x", *data)
    assert summary["sample_count"] == 8 and summary["return_sample_count"] == 4
    assert np.allclose(summary["response_px_per_commanded_mm"], [2, -3])
    assert summary["repeatable"] is True


def test_nonlinear_response_is_reported_not_discarded():
    offsets, pixels, reference = samples(response=(2, 0))
    pixels[2] = reference + [-40, 0]
    summary = summarize_axis_probe("x", offsets, pixels, reference)
    assert summary["repeatable"] is False
    assert summary["response_px_per_commanded_mm"] == [5., 0.]
    assert summary["max_fit_error_px"] == 15
    assert summary["max_return_error_px"] == 0
    assert summary["diagnostic_reasons"] == ["fit_error_exceeds_5_px"]


def test_return_drift_is_reported_with_original_reference():
    offsets, pixels, reference = samples(response=(2, 0))
    pixels[[1, 3], 1] += 6
    summary = summarize_axis_probe("x", offsets, pixels, reference)
    assert summary["repeatable"] is False
    assert summary["max_return_error_px"] == 6
    assert summary["max_fit_error_px"] == 6
    assert summary["response_px_per_commanded_mm"] == [2., 0.]
    assert "return_error_exceeds_5_px" in summary["diagnostic_reasons"]


@pytest.mark.parametrize("gain,repeatable", [(0., False), (.49, False), (.5, True), (40., True), (40.1, False)])
def test_gain_limits_are_diagnostics_with_inclusive_boundaries(gain, repeatable):
    summary = summarize_axis_probe("x", *samples(response=(gain, 0)))
    assert summary["repeatable"] is repeatable
    assert summary["response_gain_px_per_mm"] == pytest.approx(gain)


def test_five_pixel_fit_and_return_error_are_accepted():
    offsets, pixels, reference = samples(response=(2, 0))
    pixels[[1, 3], 1] += 5
    summary = summarize_axis_probe("x", offsets, pixels, reference)
    assert summary["repeatable"] is True
    assert summary["max_fit_error_px"] == summary["max_return_error_px"] == 5


@pytest.mark.parametrize("commands,reason", [
    ((5, 0, 5, 0), "both"),
    ((0, 0, 0, 0), "both"),
    ((5, 0, -5), "at least four"),
    ((5, 0, -5, 0, 5), "finish at zero"),
    ((5, -5, 0, 0), "immediately followed"),
    ((1, 0, -1, 0), "exactly"),
])
def test_malformed_excitation_or_missing_returns_rejected(commands, reason):
    with pytest.raises(CleanupError, match=reason):
        summarize_axis_probe("x", *samples(commands=commands))


def test_other_axis_excitation_rejected():
    offsets, pixels, reference = samples()
    offsets[0, 1] = .01
    with pytest.raises(CleanupError, match="other axis"):
        summarize_axis_probe("x", offsets, pixels, reference)


@pytest.mark.parametrize("field", [0, 1, 2])
@pytest.mark.parametrize("value", [float("nan"), float("inf")])
def test_nonfinite_samples_rejected(field, value):
    data = list(samples())
    data[field].flat[0] = value
    with pytest.raises(CleanupError, match="finite"):
        summarize_axis_probe("x", *data)


def test_invalid_axis_and_shapes_rejected_without_mutating_samples():
    offsets, pixels, reference = samples()
    original = [a.copy() for a in (offsets, pixels, reference)]
    for axis in ["z", "", None, []]:
        with pytest.raises(CleanupError, match="axis"):
            summarize_axis_probe(axis, offsets, pixels, reference)
    for data in [(offsets[:, 0], pixels, reference), (offsets, pixels[:-1], reference),
                 (offsets, pixels, reference.reshape(1, 2))]:
        with pytest.raises(CleanupError):
            summarize_axis_probe("x", *data)
    summarize_axis_probe("x", offsets, pixels, reference)
    assert all(np.array_equal(a, b) for a, b in zip((offsets, pixels, reference), original))
