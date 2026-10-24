"""Closed-zero interpretation for geometry/admission only, never MIT feedback."""
import math


def closed_zero(angle, tolerance):
    if not math.isfinite(angle) or not math.isfinite(tolerance) or not 0 <= tolerance <= math.radians(5) + 1e-12:
        raise ValueError('INVALID_GRIPPER_MEASUREMENT_TOLERANCE')
    # Match float32 SDK boundary roundoff in the C++ owner.
    return 0. if abs(angle) <= tolerance + 1e-9 else angle


def gripper_start_error(measured, commanded, tolerance):
    return abs(closed_zero(measured, tolerance) - closed_zero(commanded, tolerance))
