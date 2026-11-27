"""Reachable, separated footholds for a simulated stair tread."""

from __future__ import annotations

import numpy as np

MIN_PAD_GAP_M = 0.075


def select_foothold(env, leg, nominal, occupied, root, rotation):
    """Search near the nominal landing without clipping a joint to reach it."""
    course = env.course
    direction = np.sign(env.hip[leg, 1])
    choices = (
        np.array(
            [
                (dx, direction * dy, 0.0)
                for dx in (0.0, 0.025, 0.050, 0.075, -0.025)
                for dy in (0.0, 0.024, 0.046, -0.018)
            ]
        )
        + nominal
    )
    pad = env.config["foot_radius_m"] + 0.007
    level = course.height(*nominal[:2])
    offsets = ((pad, 0), (-pad, 0), (0, pad), (0, -pad))
    safe_tread = np.array(
        [
            all(course.height(x + dx, y + dy) == level for dx, dy in offsets)
            for x, y in choices[:, :2]
        ]
    )

    body = (choices - root) @ rotation - env.hip[leg]
    c, s = np.cos(env.mount[leg]), np.sin(env.mount[leg])
    points = np.column_stack(
        (body[:, 0] * c + body[:, 1] * s, -body[:, 0] * s + body[:, 1] * c, body[:, 2])
    )
    angles = env._solve_ik(points, leg=leg)
    coxa, shoulder, knee = angles.T
    a, b, d = env.lengths
    radius = a + b * np.cos(shoulder) + d * np.cos(shoulder + knee)
    reached = np.column_stack(
        (
            radius * np.cos(coxa),
            radius * np.sin(coxa),
            b * np.sin(shoulder) + d * np.sin(shoulder + knee),
        )
    )
    reachable = np.linalg.norm(points - reached, axis=1) <= 0.010
    limits = slice(3 * leg, 3 * leg + 3)
    reserve = np.minimum(angles - env.joint_low[limits], env.joint_high[limits] - angles).min(
        axis=1
    )
    if len(occupied):
        gaps = np.linalg.norm(choices[:, None, :2] - occupied[None, :, :2], axis=2).min(axis=1)
    else:
        gaps = np.full(len(choices), 0.15)
    # Keep the smallest gap first, then favor progress and modest outward spread.
    score = (
        np.minimum(gaps, 0.145)
        + 0.42 * (choices[:, 0] - nominal[0])
        + 0.14 * direction * (choices[:, 1] - nominal[1])
        + 0.018 * np.minimum(reserve, 0.35)
    )
    valid = safe_tread & reachable & (gaps >= MIN_PAD_GAP_M)
    if not valid.any():
        return None
    return choices[int(np.argmax(np.where(valid, score, -np.inf)))].copy()
