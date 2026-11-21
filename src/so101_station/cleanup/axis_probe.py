"""Offline diagnostic summary of a single commanded XY probe axis.

This fits a two-pixel response to one scalar command. It cannot identify or
validate a two-axis image map, a grasp point, or physical positioning accuracy.
"""
from __future__ import annotations

import numpy as np

from .geometry import CleanupError


def _finite_array(value, label):
    try:
        if np.iscomplexobj(value):
            raise ValueError("complex samples are not supported")
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise CleanupError(f"invalid {label}: expected finite numeric samples") from exc
    if not np.isfinite(array).all():
        raise CleanupError(f"invalid {label}: expected finite numeric samples")
    return array


def summarize_axis_probe(axis, offsets, pixels, reference):
    """Summarize measured +5/return/-5/return samples without issuing motion.

    ``offsets`` and ``pixels`` are matching N×2 arrays. Commands must excite only
    ``axis`` (``'x'`` or ``'y'``), contain both +5 mm and -5 mm, and immediately
    return to zero after every excursion. Either sign can be first; additional
    complete excursions and stationary return samples are allowed.

    Malformed command/sample data raises ``CleanupError``. Valid measured data
    with weak gain, nonlinearity or return drift is returned with
    ``repeatable=False``; it remains useful diagnostic evidence. Correlation or
    motor/camera freshness must be checked by the caller before supplying data.
    """
    if not isinstance(axis, str) or axis not in {"x", "y"}:
        raise CleanupError("probe axis must be x or y")
    commands = _finite_array(offsets, "offsets")
    observations = _finite_array(pixels, "pixels")
    origin = _finite_array(reference, "reference")
    if commands.ndim != 2 or commands.shape[1:] != (2,) or len(commands) < 4:
        raise CleanupError("offsets must contain at least four [x,y] command samples")
    if observations.shape != commands.shape or origin.shape != (2,):
        raise CleanupError("pixels must match the N×2 offsets and reference must contain two pixels")
    index = 0 if axis == "x" else 1
    command = commands[:, index]
    if np.any(commands[:, 1 - index] != 0):
        raise CleanupError("one-axis probe must not excite the other axis")
    if not np.isin(command, [-5., 0., 5.]).all():
        raise CleanupError("one-axis probe commands must be exactly -5, 0 or +5 mm")
    if not (np.any(command == 5) and np.any(command == -5)):
        raise CleanupError("one-axis probe requires both +5 mm and -5 mm excitation")
    returns = command == 0
    if int(returns.sum()) < 2 or command[-1] != 0:
        raise CleanupError("one-axis probe requires at least two zero returns and must finish at zero")
    if any(command[i + 1] != 0 for i in range(len(command) - 1) if command[i] != 0):
        raise CleanupError("each one-axis excursion must be immediately followed by a zero return")

    # Through-reference least squares: pixel_delta = scalar_command * response.
    # Zero rows contribute to the diagnostic residual, not to the fitted slope.
    with np.errstate(over="ignore", invalid="ignore"):
        displacement = observations - origin
        weights = command / np.dot(command, command)
        response = np.sum(displacement * weights[:, None], axis=0)
        residual = displacement - command[:, None] * response
        fit_errors = np.hypot(residual[:, 0], residual[:, 1])
        return_errors = np.hypot(displacement[returns, 0], displacement[returns, 1])
        gain = float(np.hypot(*response))
    if not (np.isfinite(response).all() and np.isfinite(fit_errors).all()
            and np.isfinite(return_errors).all() and np.isfinite(gain)):
        raise CleanupError("probe sample arithmetic exceeds the finite numeric range")
    fit_error, return_error = float(fit_errors.max()), float(return_errors.max())
    reasons = []
    if gain < .5:
        reasons.append("response_gain_below_0.5_px_per_mm")
    if gain > 40:
        reasons.append("response_gain_above_40_px_per_mm")
    if fit_error > 5:
        reasons.append("fit_error_exceeds_5_px")
    if return_error > 5:
        reasons.append("return_error_exceeds_5_px")
    return {"axis": axis, "sample_count": len(commands), "return_sample_count": int(returns.sum()),
            "response_px_per_commanded_mm": response.tolist(),
            "max_fit_error_px": fit_error, "max_return_error_px": return_error,
            "response_gain_px_per_mm": gain, "repeatable": not reasons,
            "diagnostic_reasons": reasons, "diagnostic_only": True,
            "validated_2d_map": False, "grasp_calibrated": False}
