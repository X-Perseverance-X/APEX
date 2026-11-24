"""Pure tracking math shared by SIL nodes and unit tests."""


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def turn_command(error, distance_m, base_gain=0.65, distance_boost=0.9,
                 near_m=1.0, far_m=3.0, maximum=0.85):
    """Match detection_hud.py's gain-scheduled normalized turn command."""
    error = clamp(float(error), -1.0, 1.0)
    if distance_m is None or distance_m <= 0.0:
        distance_factor = 0.0
    else:
        distance_factor = clamp((distance_m - near_m) / (far_m - near_m), 0.0, 1.0)
    value = error * base_gain * (1.0 + distance_factor * distance_boost)
    return clamp(value, -maximum, maximum)


def forward_command(distance_m, error, target_m=1.2, gain=0.40, maximum=0.70):
    """Approach only; reduce forward motion until the target is aligned."""
    if distance_m is None or distance_m <= 0.0:
        return 0.0
    raw = clamp((distance_m - target_m) * gain, 0.0, maximum)
    return raw * clamp(1.0 - abs(error), 0.0, 1.0)
