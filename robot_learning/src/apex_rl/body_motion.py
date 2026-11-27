"""Terrain-aware wave gait with planted feet and an explicit body target."""

from __future__ import annotations

import math

import numpy as np

from .ascent_pose import attitude_matrix

WAVE = (0, 3, 1, 4, 2, 5)
DOWN_WAVE = (2, 5, 1, 4, 0, 3)
DT = 0.02
UPPER_SUPPORT = (0, 1, 3, 4)
REAR_SUPPORT = (2, 5)
MIDDLE_SUPPORT = (1, 4)


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
        self.roll = 0.0
        self.yaw = 0.0
        self.anchors = data.xpos[self.env._foot_ids].copy()
        self.leg = None
        self.next_leg = 0
        self.elapsed = 0.0
        self.settle = 0.20
        self.swing_height = 0.04
        self.rear_edge_swing = False
        self.support_count = 0
        self.rear_transfer = False
        self.fore_probe = False
        self.transfer_peak_m = 0.0
        self.peak_upper_contacts = 0
        self.ledge_index = 0
        self.swing_edge = None
        self.retreat_swing = False

    @property
    def ledge_x(self):
        return self.terrain.step_edges[self.ledge_index]

    @property
    def ledge_top(self):
        return (self.ledge_index + 1) * self.terrain.rise_m

    @property
    def high_home_step(self):
        return self.terrain.home_stairs is not None and self.terrain.rise_m >= 0.15

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

    def upper_contacts(self, feet, contacts):
        return (
            contacts
            & (feet[:, 0] >= self.ledge_x + 0.017)
            & (feet[:, 2] >= self.ledge_top + self.radius - 0.012)
        )

    def high_edge_swing(self, u, crest, observed):
        """Withdraw the sole from the riser before lifting it over the lip."""
        edge = self.swing_edge
        start, end = self.lift_off, self.landing
        retreat = start.copy()
        retreat[0] = min(start[0], edge - 0.09)
        retreat[1] += math.copysign(0.025, start[1])
        retreat[2] += 0.031
        if edge - self.radius - 0.022 < observed[0] < edge:
            u = min(u, 0.28)
        if u < 0.28:
            foot = start + ease((u - 0.055) / 0.225) * (retreat - start)
            foot[2] = start[2] + ease(u / 0.18) * (retreat[2] - start[2])
            return foot
        if u < 0.52 or (observed[2] < end[2] + 0.011 and observed[0] < edge):
            foot = retreat.copy()
            foot[2] += ease((u - 0.28) / 0.24) * (crest - retreat[2])
            return foot
        t = ease((u - 0.52) / 0.28)
        foot = retreat + t * (end - retreat)
        lower = ease((u - 0.80) / 0.20) if observed[0] >= edge + 0.012 else 0.0
        foot[2] = crest + lower * (end[2] - crest)
        return foot

    def _pick_landing(self, foot, support, pace, middle_probe=False, offset=None):
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
        for edge in course.step_edges:
            if abs(x - edge) < edge_margin:
                x = edge + edge_margin
        max_x = min(actual[0], float(support[0]) + 0.027) + self.neutral_xy[self.leg, 0] + 0.185
        if self.fore_probe and self.leg in (0, 3):
            x = max(x, self.ledge_x + 0.052)
            if self.high_home_step:
                y += math.copysign(0.045, y)
        if middle_probe and self.leg in MIDDLE_SUPPORT:
            if foot[0] < self.ledge_x - 0.132:
                # Reposition an extended trailing leg on its current tread
                # before asking it to fold vertically over the next riser.
                x = min(foot[0] + 0.16, self.ledge_x - 0.065)
            else:
                x = max(x, self.ledge_x + 0.044)
                y += math.copysign(0.044, y)
        if self.rear_transfer and self.leg in REAR_SUPPORT:
            if self.high_home_step and max_x < self.ledge_x + 0.047:
                x = min(foot[0] + 0.15, self.ledge_x - 0.059)
            else:
                x = max(x, self.ledge_x + 0.065)
                y += math.copysign(0.034, y)
        if offset is not None:
            x += 0.027 * offset[0]
            y += 0.035 * offset[1]
            for edge in course.step_edges:
                if abs(x - edge) < edge_margin:
                    x = edge + math.copysign(edge_margin, x - edge)
        x = min(x, max_x)
        if self.env.rubble:
            y = self.neutral_xy[self.leg, 1] + course.route_y(actual[0] + 0.09)
            # Search near the nominal foothold for enough flat space for the
            # entire pad and a swing path around blocks taller than the gait.
            offsets = (
                (0, 0),
                (0, -0.035),
                (0, 0.035),
                (-0.035, 0),
                (0, -0.065),
                (0, 0.065),
                (-0.035, -0.05),
                (-0.035, 0.05),
                (-0.060, 0),
                (0.035, 0),
                (-0.060, -0.065),
                (-0.060, 0.065),
            )
            for dx, dy in offsets:
                px, py = min(x + dx, max_x), y + dy
                h = course.height(px, py)
                pad = self.radius + 0.004
                if h > 0.06 or any(
                    course.height(px + sx * pad, py + sy * pad) != h
                    for sx, sy in ((1, 0), (-1, 0), (0, 1), (0, -1))
                ):
                    continue
                if all(
                    course.height(foot[0] + t * (px - foot[0]), foot[1] + t * (py - foot[1]))
                    <= 0.06
                    for t in (0.15, 0.35, 0.55, 0.75, 0.90)
                ):
                    return np.array([px, py, h + self.radius - 0.0015])
            return foot.copy()
        return np.array([x, y, course.height(x, y) + self.radius - 0.0015])

    def targets(self, residual):
        env = self.env
        data = env.data
        self.elapsed += DT
        contacts = self.contacts()
        feet = data.xpos[env._foot_ids].copy()
        ledge = env.course.ledge_transfer
        upper = self.upper_contacts(feet, contacts) if ledge else None
        home = env.course.home_stairs is not None
        brace = env.brace
        brace_xy = None if brace is None else brace.xy
        if (
            home
            and ledge
            and self.leg is None
            and upper.all()
            and self.ledge_index + 1 < self.terrain.risers
        ):
            self.ledge_index += 1
            upper = self.upper_contacts(feet, contacts)
        # Anchor state is updated on touchdown; brief unloading of a planted
        # forefoot must not restart the front transfer under the body.
        planted = (
            self.upper_contacts(self.anchors, np.ones(6, dtype=bool))
            if ledge and self.high_home_step
            else upper
        )
        middle_probe = (
            ledge
            and self.high_home_step
            and planted[0]
            and planted[3]
            and not all(planted[i] for i in MIDDLE_SUPPORT)
        )
        low_probe = env.course.fore_probe and self.terrain.rise_m <= 0.12
        if ledge:
            self.peak_upper_contacts = max(self.peak_upper_contacts, int(upper.sum()))
            if self.leg is None:
                lead = 0.315 if low_probe or self.high_home_step else 0.280
                self.fore_probe = (
                    env.course.fore_probe
                    and not (planted[0] and planted[3])
                    and self.ledge_x - lead <= data.qpos[0] < self.ledge_x
                )
                self.rear_transfer = (
                    all(planted[i] for i in UPPER_SUPPORT)
                    and not all(planted[i] for i in REAR_SUPPORT)
                    and data.qpos[0] >= self.ledge_x + (0.035 if self.high_home_step else 0.16)
                )
        steep_descent = env.hybrid and self.terrain.descending and self.terrain.rise_m >= 0.08
        pace = (0.032 + 0.010 * residual[0]) if steep_descent else (0.037 + 0.012 * residual[0])
        if env.short_climb:
            pace = 0.033 + 0.010 * residual[0]
        order = DOWN_WAVE if steep_descent else WAVE
        duration = 1.38 if self.retreat_swing else (1.02 if self.rear_transfer else 0.80)
        if (
            self.leg is not None
            and self.elapsed >= duration
            and (contacts[self.leg] or self.elapsed >= duration + 0.22)
        ):
            self.anchors[self.leg] = feet[self.leg] if contacts[self.leg] or ledge else self.landing
            self.leg = None
            self.next_leg += 1
            self.elapsed = 0.0
        if self.leg is None and self.elapsed >= self.settle:
            candidate = order[self.next_leg % len(order)]
            if self.fore_probe:
                candidate = 3 if planted[0] else 0
            if self.rear_transfer:
                # Alternate trailing feet that are still on the lower tread.
                rears = (
                    REAR_SUPPORT[::-1]
                    if self.high_home_step and self.next_leg % 2
                    else REAR_SUPPORT
                )
                candidate = next((i for i in rears if not planted[i]), candidate)
            if middle_probe:
                candidates = (
                    MIDDLE_SUPPORT if data.qpos[0] >= self.ledge_x - 0.119 else REAR_SUPPORT
                )
                candidate = next((i for i in candidates if not planted[i]), candidate)
                if candidates == REAR_SUPPORT:
                    candidate = candidates[self.next_leg % len(candidates)]
            if brace is not None:
                # Let the policy choose the side within the current pair.
                other = (candidate + 3) % 6
                if self.fore_probe or middle_probe or self.rear_transfer:
                    other_allowed = not planted[other] and not planted[candidate]
                else:
                    other_allowed = True
                if other_allowed:
                    prefer_other = residual[10] * (1 if candidate < 3 else -1) > 0.22
                    if prefer_other:
                        candidate = other
            stance = contacts.copy()
            stance[candidate] = False
            minimum_support = 4 if middle_probe and brace_xy is None else 3
            if stance.sum() >= minimum_support:
                self.leg = candidate
                self.lift_off = feet[candidate].copy()
                self.landing = self._pick_landing(
                    self.lift_off,
                    feet[stance, :2].mean(axis=0),
                    pace,
                    middle_probe,
                    residual[8:10] if brace is not None else None,
                )
                self.swing_edge = (
                    next(
                        (
                            edge
                            for edge in self.terrain.step_edges
                            if self.lift_off[0] < edge <= self.landing[0]
                        ),
                        None,
                    )
                    if home
                    else None
                )
                self.retreat_swing = (
                    self.swing_edge is not None
                    and self.high_home_step
                    and self.landing[2] >= self.lift_off[2] + 0.065
                    and not self.terrain.descending
                )
                duration = 1.38 if self.retreat_swing else (1.02 if self.rear_transfer else 0.80)
                rise = max(0.0, self.landing[2] - self.lift_off[2])
                lift = 0.034 + 0.14 * rise if env.course.steep_approach else 0.040 + 0.25 * rise
                self.swing_height = float(np.clip(lift + 0.028 * residual[1], 0.018, 0.080))
                edge_pad = self.radius + 0.012
                self.rear_edge_swing = (
                    env.edge_control
                    and candidate in (2, 5)
                    and any(
                        self.lift_off[0] - edge_pad <= edge <= self.landing[0] + edge_pad
                        for edge in self.terrain.step_edges
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
        if self.fore_probe:
            setback = 0.270 if self.high_home_step else (0.260 if low_probe else 0.238)
            self.path_x = min(self.path_x, self.ledge_x - setback)
        if self.rear_transfer:
            self.path_x = min(self.path_x, self.ledge_x + 0.27)
        if middle_probe:
            # Keep the centre behind the lip while two forefeet carry the
            # high side and the middle foothold is still being moved.
            setback = 0.056 if not any(planted[i] for i in MIDDLE_SUPPORT) else 0.027
            slack = 0.055 if brace_xy is not None else 0.0
            self.path_x = min(self.path_x, self.ledge_x - setback + slack)
        tall_ascent = env.contact_ascent and self.terrain.rise_m >= 0.07
        steep = env.course.steep_approach
        preview = 0.29 if steep else (0.24 if tall_ascent else 0.17)
        front = self.terrain.height(root[0] + preview, root[1])
        centre = self.terrain.height(root[0] - 0.03, root[1])
        if self.support_count >= 3:
            support = feet[stance]
            support_xy = support[:, :2]
            shift = 0.0
            if brace is not None:
                if self.leg is not None:
                    shift = -math.copysign(0.016, self.neutral_xy[self.leg, 1])
                if brace_xy is not None:
                    support_xy = np.vstack((support_xy, brace_xy))
            target = [
                self.path_x + 0.014 * residual[2],
                (
                    self.terrain.route_y(root[0] + 0.07)
                    if env.rubble
                    else 0.30 * float(support[:, 1].mean())
                )
                + 0.026 * residual[4]
                + shift,
            ]
            bounded = support_target(support_xy, np.array(target) + com_offset)
            if bounded is not None:
                xy = bounded - com_offset
            if self.rear_transfer:
                shifted = support_target(support_xy, xy + com_offset + np.array([0.026, 0.0]))
                if shifted is not None:
                    transfer = float(np.clip(shifted[0] - xy[0] - com_offset[0], 0.0, 0.026))
                    self.transfer_peak_m = max(self.transfer_peak_m, transfer)
                    xy = shifted - com_offset
            levels = support[:, 2] - self.radius
            support_height = (
                0.45 * float(levels.mean()) + 0.55 * float(levels.max())
                if tall_ascent
                else 0.55 * float(levels.mean()) + 0.45 * float(levels.max())
            )
        else:
            support_height = float(self.body[2] - env.stand_height)
        rise_ahead = max(0.0, front - centre)
        z = env.stand_height + support_height + min(0.025, rise_ahead * 0.52) + 0.028 * residual[3]
        if env.short_climb:
            z -= 0.008
        if env.hybrid and self.support_count >= 3 and front > centre:
            # Raise the leading low-mounted hardware before it reaches the riser.
            ahead = self.terrain.height(root[0] + 0.24, root[1])
            z = max(z, ahead + (0.094 if tall_ascent else 0.076))
        if self.fore_probe:
            probe_raise = 0.073 if self.high_home_step else (0.051 if low_probe else 0.059)
            z = env.stand_height + probe_raise + self.ledge_index * self.terrain.rise_m
        if middle_probe:
            nose_clearance = 0.041 * ease((root[0] - self.ledge_x + 0.25) / 0.11)
            z = self.ledge_top + 0.052 + nose_clearance + 0.009 * residual[3]
        if self.rear_transfer:
            z = max(z, self.ledge_top + env.stand_height - 0.009)
        if brace is not None and (middle_probe or self.rear_transfer):
            # Permit a controlled lowering of the nose onto the top tread.
            # A negative policy output keeps the body in a higher stance.
            z -= 0.045 * max(0.0, residual[11])
        target = np.array([*xy, z])
        rate = np.clip(5 * (target - self.body), [-0.065, -0.065, -0.05], [0.08, 0.065, 0.05])
        self.velocity += np.clip(rate - self.velocity, -0.50 * DT, 0.50 * DT)
        self.body += self.velocity * DT
        pitch_gain = 2.0 if steep else (1.3 if tall_ascent else 0.7)
        pitch_limit = 0.26 if steep else 0.18
        pitch = float(
            np.clip((front - centre) * pitch_gain + 0.16 * residual[5], -0.14, pitch_limit)
        )
        if self.fore_probe:
            pitch = 0.50 if self.high_home_step else (0.36 if low_probe else 0.28)
        if middle_probe:
            pitch = 0.26
        if self.rear_transfer:
            pitch = max(pitch, 0.14) if self.high_home_step else min(pitch, -0.055)
        if brace is not None:
            pitch = np.clip(pitch + 0.12 * residual[5], -0.18, 0.53)
        self.pitch += float(np.clip(pitch - self.pitch, -0.34 * DT, 0.34 * DT))
        if env.attitude_control:
            # Estimate left/right support slope from measured feet, then let
            # PPO adjust the three attitude axes without moving planted feet.
            roll_ref = 0.0
            if self.support_count >= 3:
                left = support[support[:, 1] > root[1]]
                right = support[support[:, 1] < root[1]]
                if len(left) and len(right):
                    dz = float(left[:, 2].mean() - right[:, 2].mean())
                    dy = float(left[:, 1].mean() - right[:, 1].mean())
                    roll_ref = 0.45 * math.atan2(dz, max(0.15, dy))
            roll_target = np.clip(roll_ref + 0.30 * residual[6], -0.15, 0.15)
            yaw_target = np.clip(0.45 * residual[7], -0.16, 0.16)
            if self.fore_probe and self.high_home_step and self.leg in (0, 3):
                roll_target = np.clip(
                    roll_target + (0.075 if self.leg == 3 else -0.075), -0.20, 0.20
                )
            if middle_probe and self.leg in MIDDLE_SUPPORT:
                offset = 0.046 if self.leg == 4 else -0.046
                roll_target = np.clip(roll_target + offset, -0.20, 0.20)
            if (
                env.course.diagonal_transfer
                and self.leg in (0, 3)
                and self.landing[2] > self.lift_off[2] + 0.04
            ):
                # Lift the swinging shoulder and advance that side so the
                # opposite side can carry the load through the support hull.
                side = 1 if self.leg == 0 else -1
                roll_target = np.clip(roll_target - side * 0.075, -0.20, 0.20)
                yaw_target = np.clip(yaw_target + side * 0.090, -0.20, 0.20)
            self.roll += float(np.clip(roll_target - self.roll, -0.30 * DT, 0.30 * DT))
            self.yaw += float(np.clip(yaw_target - self.yaw, -0.27 * DT, 0.27 * DT))
        footholds = self.anchors.copy()
        if self.leg is not None:
            u = min(1.0, self.elapsed / duration)
            lift_end, travel_start, travel_span, lower_start, lower_span = (
                (0.33, 0.32, 0.35, 0.73, 0.27)
                if self.rear_edge_swing
                else (0.30, 0.22, 0.51, 0.70, 0.30)
            )
            t = ease((u - travel_start) / travel_span)
            if self.swing_edge is not None:
                observed = feet[self.leg]
                min_z = max(self.landing[2], self.lift_off[2]) + 0.012
                if observed[0] < self.swing_edge and observed[2] < min_z:
                    edge_fraction = (self.swing_edge - self.radius - 0.006 - self.lift_off[0]) / (
                        self.landing[0] - self.lift_off[0]
                    )
                    t = min(t, max(0.0, edge_fraction))
                if observed[0] < self.swing_edge + 0.012:
                    # Keep lifting rather than lowering a foot still behind
                    # the edge. The swing timeout remains bounded.
                    u = min(u, lower_start)
            foot = self.lift_off + t * (self.landing - self.lift_off)
            crest = max(self.landing[2], self.lift_off[2]) + self.swing_height
            if env.rubble:
                peak = max(
                    self.terrain.height(
                        self.lift_off[0] + t * (self.landing[0] - self.lift_off[0]),
                        self.lift_off[1] + t * (self.landing[1] - self.lift_off[1]),
                    )
                    for t in (0.25, 0.50, 0.75)
                )
                crest = max(crest, peak + self.radius + 0.027)
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
            if self.retreat_swing:
                if self.fore_probe or middle_probe:
                    crest = min(crest, self.landing[2] + 0.036)
                foot = self.high_edge_swing(
                    min(1.0, self.elapsed / duration), crest, feet[self.leg]
                )
            footholds[self.leg] = foot
        if env.attitude_control:
            rotation = attitude_matrix(self.roll, self.pitch, self.yaw)
        else:
            c, s = math.cos(self.pitch), math.sin(self.pitch)
            rotation = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
        body_frame = (footholds - self.body) @ rotation - env.hip
        if steep and self.leg is not None:
            # Keep the swing foot out of the minimum-reach circle implied
            # by the knee stop, preserving its vertical clearance target.
            leg = self.leg
            coxa, femur, tibia = env.lengths
            fold_limit = env.joint_low[3 * leg + 2]
            inner = (
                math.sqrt(femur**2 + tibia**2 + 2 * femur * tibia * math.cos(fold_limit)) + 0.002
            )
            x, y, z = body_frame[leg]
            radius = math.hypot(x, y)
            if abs(z) < inner and radius > 1e-5:
                required = coxa + math.sqrt(inner**2 - z**2)
                body_frame[leg, :2] *= max(1.0, required / radius)
        mc, ms = np.cos(env.mount), np.sin(env.mount)
        return np.column_stack(
            (
                body_frame[:, 0] * mc + body_frame[:, 1] * ms,
                -body_frame[:, 0] * ms + body_frame[:, 1] * mc,
                body_frame[:, 2],
            )
        )
