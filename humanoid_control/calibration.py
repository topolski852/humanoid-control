"""
Per-joint position_offset calibration — the AS5600 is single-turn absolute but the
multi-turn zero is lost on power-down, so ``position_offset`` must be recomputed every
power-up before any motion.

Math vendored from humanoid-studio ``backend/humanoid/range_cal.py`` (the control-truth
reference). Procedure: with ``position_offset = 0`` written to the ESC, capture the RAW
position at each mechanical hardstop (lower → ``min_rad``, upper → ``max_rad``), then:

    offset = lower_pos - min_rad          # so the lower hardstop reads exactly min_rad

A backward-counting encoder (upper_pos < lower_pos) would need a gear-ratio sign flip; we
only write ``position_offset`` here (gear sign is set at commissioning), so we FLAG a flip /
range error rather than silently changing gear.
"""
from __future__ import annotations

# ~20° range tolerance, matching studio's Motor Cal page.
_MAX_RANGE_ERROR_RAD = 0.35


def compute_offset(
    lower_pos: float,
    upper_pos: float,
    min_rad: float,
    max_rad: float,
) -> dict:
    """Compute position_offset from two raw hardstop captures.

    Returns {position_offset, measured_range_rad, expected_range_rad, range_error_rad,
    range_ok, flipped}. Caller should refuse to apply when ``flipped`` is True (indicates
    captures swapped or a wrong gear sign — not a plain offset calibration).
    """
    if max_rad <= min_rad:
        raise ValueError(f"min_rad ({min_rad}) must be < max_rad ({max_rad})")

    flipped = upper_pos < lower_pos          # backward-counting encoder
    offset = lower_pos - min_rad             # lower hardstop → min_rad
    measured_range = abs(upper_pos - lower_pos)
    expected_range = max_rad - min_rad
    range_error = abs(measured_range - expected_range)
    return {
        "position_offset": offset,
        "measured_range_rad": measured_range,
        "expected_range_rad": expected_range,
        "range_error_rad": range_error,
        "range_ok": range_error <= _MAX_RANGE_ERROR_RAD,
        "flipped": flipped,
    }
