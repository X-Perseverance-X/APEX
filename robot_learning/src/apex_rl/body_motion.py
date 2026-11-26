"""Terrain-aware wave gait with planted feet and an explicit body target."""

from __future__ import annotations

import math

import numpy as np

WAVE = (0, 3, 1, 4, 2, 5)
DOWN_WAVE = (2, 5, 1, 4, 0, 3)
DT = 0.02


def ease(x):
    u = float(np.clip(x, 0, 1))
    return u**3 * (10 - 15 * u + 6 * u * u)


def support_target(points, target, *, inset=0.018):
    """Project an XY target into the inset convex hull of the supporting feet."""
    points = sorted(set(map(tuple, points)))
    if len(points) < 3:
        return None

    def half(ordered):
        chain = []
        for p in ordered:
            while len(chain) >= 2:
                a, b = chain[-2:]
                if (b[0] - a[0]) * (p[1] - b[1]) > (b[1] - a[1]) * (p[0] - b[0]):
                    break
                chain.pop()
            chain.append(p)
        return chain[:-1]

    hull = np.array(half(points) + half(reversed(points)))
    if len(hull) < 3:
        return None
    edges = np.roll(hull, -1, axis=0) - hull
    normals = np.column_stack((-edges[:, 1], edges[:, 0]))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    centre = hull.mean(axis=0)
    limit = min(inset, float(np.min(np.sum((centre - hull) * normals, axis=1))) * 0.5)
    result = np.asarray(target, dtype=float).copy()
    for _ in range(4):
        for corner, normal in zip(hull, normals, strict=True):
            result += normal * max(0.0, limit - float((result - corner) @ normal))
    return result


class BodyMotion:
    """Plan one swing leg at a time and keep the other foothold targets fixed."""

    def __init__(self, env):
        self.env = env
        self.terrain = env.course
        self.radius = env.config["foot_radius_m"]
        neutral = env._neutral_foot
        c, s = np.cos(env.mount), np.sin(env.mount)
        self.neutral_xy = env.hip[:, :2] + np.column_stack(
            (neutral[:, 0] * c - neutral[:, 1] * s, neutral[:, 0] * s + neutral[:, 1] * c)
        )
        self.foot_geom_ids = [
            env.model.geom(f"{label}_foot_geom").id
            for label in ("rf", "rm", "rr", "lf", "lm", "lr")
        ]
        self.support_geom_ids = (env.model.geom("floor").id, *env._terrain_ids)
        self.base_id = env.model.body("base").id

    def reset(self):
        data = self.env.data
        self.body = data.qpos[:3].copy()
        self.path_x = float(self.body[0])
        self.velocity = np.zeros(3)
        self.pitch = 0.0
        self.anchors = data.xpos[self.env._foot_ids].copy()
        self.leg = None
        self.next_leg = 0
        self.elapsed = 0.0
        self.settle = 0.20
        self.swing_height = 0.04
        self.rear_edge_swing = False
        self.support_count = 0

    def contacts(self):
        contact = self.env.data.contact
        pairs = contact.geom[np.abs(contact.frame.reshape(-1, 3, 3)[:, 0, 2]) > 0.72]
        if self.env.course.link_contacts:
            pairs = pairs[np.isin(pairs, self.support_geom_ids).any(axis=1)]
        return np.isin(self.foot_geom_ids, pairs)

    def rear_riser_contact(self):
        if not self.env.hybrid or not self.env._terrain_ids:
            return False
        contact = self.env.data.contact
        side_pairs = contact.geom[np.abs(contact.frame.reshape(-1, 3, 3)[:, 0, 2]) < 0.55]
        if not len(side_pairs):
            return False
        rear = np.isin(side_pairs, (self.foot_geom_ids[2], self.foot_geom_ids[5]))
        step = np.isin(side_pairs, self.env._terrain_ids)
        return bool(np.any(np.any(rear, axis=1) & np.any(step, axis=1)))

    def _pick_landing(self, foot, support, pace):
        course = self.terrain
        actual = self.env.data.qpos[:3]
        x = max(
            foot[0] + (0.028 if self.env.hybrid else 0.035),
            actual[0]
            + self.neutral_xy[self.leg, 0]
            + pace * (1.86 if self.env.short_climb else 2.2),
        )
        y = self.neutral_xy[self.leg, 1] + 0.25 * actual[1]
        # Do not place a spherical foot with its centre on a riser edge.
        rgb_rear = self.env.edge_control and course is not self.env.course and self.leg in (2, 5)
        edge_margin = self.radius + (0.016 if rgb_rear else 0.008)
        for i in range(course.risers):
            edge = course.edge_x + i * course.tread_m
            if abs(x - edge) < edge_margin:
                x = edge + edge_margin
        max_x = min(actual[0], float(support[0]) + 0.027) + self.neutral_xy[self.leg, 0] + 0.185
        x = min(x, max_x)
        return np.array([x, y, course.height(x, y) + self.radius - 0.0015])

    def targets(self, residual):
        env = self.env
        data = env.data
        self.elapsed += DT
        contacts = self.contacts()
        feet = data.xpos[env._foot_ids].copy()
        steep_descent = env.hybrid and self.terrain.descending and self.terrain.rise_m >= 0.08
        pace = (0.032 + 0.010 * residual[0]) if steep_descent else (0.037 + 0.012 * residual[0])
        if env.short_climb:
            pace = 0.033 + 0.010 * residual[0]
        order = DOWN_WAVE if steep_descent else WAVE
        duration = 0.80
        if (
            self.leg is not None
            and self.elapsed >= duration
            and (contacts[self.leg] or self.elapsed >= duration + 0.22)
        ):
            self.anchors[self.leg] = feet[self.leg] if contacts[self.leg] else self.landing
            self.leg = None
            self.next_leg += 1
            self.elapsed = 0.0
        if self.leg is None and self.elapsed >= self.settle:
            candidate = order[self.next_leg % len(order)]
            stance = contacts.copy()
            stance[candidate] = False
            if stance.sum() >= 3:
                self.leg = candidate
                self.lift_off = feet[candidate].copy()
                self.landing = self._pick_landing(
                    self.lift_off, feet[stance, :2].mean(axis=0), pace
                )
                rise = max(0.0, self.landing[2] - self.lift_off[2])
                self.swing_height = float(
                    np.clip(0.040 + 0.25 * rise + 0.028 * residual[1], 0.018, 0.080)
                )
                edge_pad = self.radius + 0.012
                self.rear_edge_swing = (
                    env.edge_control
                    and candidate in (2, 5)
                    and any(
                        self.lift_off[0] - edge_pad <= edge <= self.landing[0] + edge_pad
                        for edge in (
                            self.terrain.edge_x + i * self.terrain.tread_m
                            for i in range(self.terrain.risers)
                        )
                    )
                )
                self.elapsed = 0.0
                self.settle = 0.07
        stance = contacts.copy()
        if self.leg is not None:
            stance[self.leg] = False
        self.support_count = int(stance.sum())
        root = data.qpos[:3]
        com_offset = data.subtree_com[self.base_id, :2] - root[:2]
        xy = self.body[:2].copy()
        self.path_x = min(self.path_x + DT * pace, root[0] + (0.018 if env.short_climb else 0.022))
        front = self.terrain.height(root[0] + 0.17, root[1])
        centre = self.terrain.height(root[0] - 0.03, root[1])
        if self.support_count >= 3:
            support = feet[stance]
            target = [
                self.path_x + 0.014 * residual[2],
                0.30 * float(support[:, 1].mean()) + 0.026 * residual[4],
            ]
            bounded = support_target(support[:, :2], np.array(target) + com_offset)
            if bounded is not None:
                xy = bounded - com_offset
            levels = support[:, 2] - self.radius
            support_height = 0.55 * float(levels.mean()) + 0.45 * float(levels.max())
        else:
            support_height = float(self.body[2] - env.stand_height)
        rise_ahead = max(0.0, front - centre)
        z = env.stand_height + support_height + min(0.025, rise_ahead * 0.52) + 0.028 * residual[3]
        if env.short_climb:
            z -= 0.008
        if env.hybrid and self.support_count >= 3 and front > centre:
            # Raise the leading low-mounted hardware before it reaches the riser.
            ahead = self.terrain.height(root[0] + 0.24, root[1])
            z = max(z, ahead + 0.076)
        target = np.array([*xy, z])
        rate = np.clip(5 * (target - self.body), [-0.065, -0.065, -0.05], [0.08, 0.065, 0.05])
        self.velocity += np.clip(rate - self.velocity, -0.50 * DT, 0.50 * DT)
        self.body += self.velocity * DT
        pitch = float(np.clip((front - centre) * 0.7 + 0.16 * residual[5], -0.14, 0.18))
        self.pitch += float(np.clip(pitch - self.pitch, -0.34 * DT, 0.34 * DT))
        footholds = self.anchors.copy()
        if self.leg is not None:
            u = min(1.0, self.elapsed / duration)
            lift_end, travel_start, travel_span, lower_start, lower_span = (
                (0.33, 0.32, 0.35, 0.73, 0.27)
                if self.rear_edge_swing
                else (0.30, 0.22, 0.51, 0.70, 0.30)
            )
            t = ease((u - travel_start) / travel_span)
            foot = self.lift_off + t * (self.landing - self.lift_off)
            crest = max(self.landing[2], self.lift_off[2]) + self.swing_height
            if self.rear_edge_swing:
                crest += 0.012
            if u < lift_end:
                foot[2] = self.lift_off[2] + ease(u / lift_end) * (crest - self.lift_off[2])
            else:
                foot[2] = crest + ease((u - lower_start) / lower_span) * (self.landing[2] - crest)
            if self.terrain is not env.course and not contacts[self.leg]:
                # Probe gently below an uncertain RGB landing height until
                # the sole touches or the existing swing timeout expires.
                foot[2] -= 0.06 * max(0.0, self.elapsed - duration)
            footholds[self.leg] = foot
        c, s = math.cos(self.pitch), math.sin(self.pitch)
        rotation = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
        body_frame = (footholds - self.body) @ rotation - env.hip
        mc, ms = np.cos(env.mount), np.sin(env.mount)
        return np.column_stack(
            (
                body_frame[:, 0] * mc + body_frame[:, 1] * ms,
                -body_frame[:, 0] * ms + body_frame[:, 1] * mc,
                body_frame[:, 2],
            )
        )
