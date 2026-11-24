"""Firmware-derived tripod gait and URDF joint-space inverse kinematics."""

import math


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def approach(current, target, maximum_delta):
    if current < target:
        return min(current + maximum_delta, target)
    if current > target:
        return max(current - maximum_delta, target)
    return current


def solve_leg_ik(mount_yaw, radial_m, body_dx, body_dy, z_m,
                 coxa_m=0.06966, femur_m=0.10324, tibia_m=0.16841):
    """Return URDF (coxa, femur, tibia) radians for one foot target.

    The nominal foot lies ``radial_m`` along the mechanical mount yaw.  Gait
    displacement is expressed in the canonical body frame (+X forward,+Y left).
    The result matches the URDF's -Y femur/tibia axes and knee-down branch.
    """
    cos_yaw = math.cos(mount_yaw)
    sin_yaw = math.sin(mount_yaw)
    body_x = radial_m * cos_yaw + body_dx
    body_y = radial_m * sin_yaw + body_dy
    local_x = cos_yaw * body_x + sin_yaw * body_y
    local_y = -sin_yaw * body_x + cos_yaw * body_y

    coxa = math.atan2(local_y, local_x)
    planar = max(0.005, math.hypot(local_x, local_y) - coxa_m)
    cosine_tibia = clamp(
        (planar * planar + z_m * z_m - femur_m * femur_m - tibia_m * tibia_m)
        / (2.0 * femur_m * tibia_m),
        -1.0,
        1.0,
    )
    tibia = -math.acos(cosine_tibia)
    femur = math.atan2(z_m, planar) - math.atan2(
        tibia_m * math.sin(tibia), femur_m + tibia_m * math.cos(tibia)
    )
    # Firmware ultimately constrains every electrical servo to 5..175 deg.
    # In the URDF's zero-centered geometric convention that is +/-85 deg for
    # coxa/femur and -175..-5 deg for the knee-down tibia branch.
    return (
        clamp(coxa, math.radians(-85.0), math.radians(85.0)),
        clamp(femur, math.radians(-85.0), math.radians(85.0)),
        clamp(tibia, math.radians(-175.0), math.radians(-5.0)),
    )


def tripod_targets(phase, forward, lateral, turn_right, mount_yaws,
                   phase_offsets, radial_m=0.150, stance_z_m=-0.100,
                   step_length_m=0.100, step_lift_m=0.050):
    """Reproduce v55's two-tripod swing/stance curve in URDF joint space."""
    targets = []
    motion_scale = clamp(math.sqrt(forward * forward + lateral * lateral + turn_right * turn_right), 0.0, 1.0)
    for index, mount_yaw in enumerate(mount_yaws):
        leg_phase = (phase + phase_offsets[index]) % 1.0
        side_sign = 1.0 if index < 3 else -1.0
        leg_forward = forward * step_length_m - side_sign * turn_right * step_length_m
        leg_lateral = lateral * step_length_m
        if leg_phase < 0.5:
            t = leg_phase * 2.0
            curve = -math.cos(t * math.pi)
            z_m = stance_z_m + step_lift_m * math.sin(t * math.pi) * motion_scale
            swing = True
        else:
            t = (leg_phase - 0.5) * 2.0
            curve = 1.0 - 2.0 * t
            z_m = stance_z_m
            swing = False
        body_dx = 0.5 * leg_forward * curve
        body_dy = 0.5 * leg_lateral * curve
        targets.append((
            solve_leg_ik(mount_yaw, radial_m, body_dx, body_dy, z_m),
            swing,
        ))
    return targets
