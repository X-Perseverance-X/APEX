"""Staged single-step ascent: plan each foot move against reach, support and torque.

A separate experiment next to the ``hybrid-brace-transfer-up-*`` controller.
The courses ``hybrid-staged-up-13`` … ``hybrid-staged-up-20`` use the same
stair geometry, mass, servo torque and speed limits, knee stop, link contact
model and evaluation limits as the brace-transfer courses. Only the stepping
logic near the riser differs:

1. The tripod approach and the existing wave walk bring the robot to about
   36 cm before the riser (unchanged code path).
2. From there every foot move is planned before it starts: the next leg in
   phase order (front → middle → rear), a landing whose whole pad fits on the
   tread with 7.5 cm to its neighbours, and a body pose ``(x, y, z, pitch)``
   for which the remaining stance legs reach their soles with joint margin,
   the centre of mass lies inside their support polygon, and the static joint
   torques stay below 82 % of this episode's servo limits. The swing path is
   checked over its whole length for reach, knee margin, sole/riser clearance,
   femur/tibia clearance to the step and neighbouring soles.
3. The body moves to that pose with every sole planted. Before lift-off the
   measured centre of mass is checked against the measured contacts. If the
   preferred step is not feasible, a trailing foot is moved first, another leg
   is tried, or the body pose is chosen again.
4. Once six soles are on the tread, control returns to the existing wave gait.

PPO adds small corrections to the planned body pose and landing point.
"""

from __future__ import annotations

import itertools
import math

import numpy as np

from .ascent_planner import (
    COM_INSET,
    NEIGHBOUR_GAP,
    LegGeometry,
    check_stance,
    check_swing,
    hull_margin,
    rotation,
    swing_path,
)
from .body_motion import BodyMotion, ease
from .locomotion import COURSES, LocomotionEnv

TAKEOVER_M = 0.36
TRIPOD_HANDOFF_M = 0.50
MAX_LAG_M = 0.15
POSES_PER_LANDING = 120
LEG_BUDGET = 420
FRONT, MIDDLE, REAR = (0, 3), (1, 4), (2, 5)
LEG_NAMES = ("RF", "RM", "RR", "LF", "LM", "LR")
STAGED = {}
for _height in range(13, 21):
    _name = f"hybrid-staged-up-{_height:02}"
    COURSES[_name] = COURSES[f"hybrid-brace-transfer-up-{_height:02}"]
    STAGED[_name] = _height / 100

CHASSIS_BOTTOM = np.array(
    [
        [x, y, z]
        for x, z in ((0.155, -0.050), (0.115, -0.050), (-0.070, -0.025), (-0.140, -0.025))
        for y in (0.045, -0.045)
    ]
    + [[x, y, 0.0] for x in (0.15, -0.15) for y in (0.055, -0.055)]
)


class StagedAscent(BodyMotion):
    """Wave walk to the riser, then one planned and checked foot move at a time."""

    def __init__(self, env):
        super().__init__(env)
        self.geom = LegGeometry.from_env(env)

    # ------------------------------------------------------------------ state
    def reset(self):
        super().reset()
        self.mode = "legacy"
        self.state = "plan"
        self.plan = None
        self.timer = 0.0
        self.blocked = {}
        self.plans = 0
        self.failed_plans = 0
        self.log = []
        self.pose_pitch = 0.0
        self.stage_body = None
        self.last_shift = None

    @property
    def edge(self):
        return self.terrain.step_edges[0]

    @property
    def top(self):
        return self.terrain.rise_m

    def _upper(self, points):
        return (points[:, 0] >= self.edge + 0.017) & (
            points[:, 2] >= self.top + self.radius - 0.012
        )

    def staged_phase(self):
        if self.mode == "legacy":
            return "approach"
        if self.mode == "top":
            return "top"
        up = self._upper(self.anchors)
        if not (up[0] and up[3]):
            return "front"
        if not (up[1] and up[4]):
            return "middle"
        if not (up[2] and up[5]):
            return "rear"
        return "top"

    # ------------------------------------------------------------- main hook
    def targets(self, residual):
        env = self.env
        if self.mode == "legacy":
            root = env.data.qpos[:3]
            if (
                self.leg is None
                and env.gait == "wave"
                and root[0] >= self.edge - TAKEOVER_M
                and self.contacts().sum() >= 5
            ):
                self._take_over()
            else:
                return super().targets(residual)
        if self.mode == "top":
            return self._top_targets(residual)
        return self._staged_targets(residual)

    def _take_over(self):
        data = self.env.data
        self.mode = "staged"
        self.state = "plan"
        self.anchors = data.xpos[self.env._foot_ids].copy()
        self.body = data.qpos[:3].copy()
        self.velocity[:] = 0.0
        self.leg = None
        self.elapsed = 0.0

    def _hand_back(self):
        """All six soles are on the tread: walk on with the flat-ground tripod."""
        from .gait_lab import PhaseTripod

        env = self.env
        data = env.data
        self.mode = "top"
        feet = data.xpos[env._foot_ids].copy()
        self.anchors = feet
        self.leg = None
        self.elapsed = 0.0
        rot = data.xmat[self.base_id].reshape(3, 3)
        body_frame = (feet - data.qpos[:3]) @ rot
        self.top_gait = PhaseTripod(env)
        self.top_gait.start_from(body_frame[:, :2])

    def _top_targets(self, residual):
        env = self.env
        data = env.data
        self.body = data.qpos[:3].copy()
        self.pitch += float(np.clip(-self.pitch, -0.25 * 0.02, 0.25 * 0.02))
        self.support_count = int(self.contacts().sum())
        speed = 0.10 + 0.02 * float(residual[0])
        # Steer back to the lane centre and the course heading.
        heading = env._heading()
        command = (
            speed,
            float(np.clip(-0.8 * data.qpos[1], -0.03, 0.03)),
            float(np.clip(-1.5 * heading, -0.2, 0.2)),
        )
        return self.top_gait.feet(command)

    # -------------------------------------------------------------- planning
    def _com_body(self):
        data = self.env.data
        rot = data.xmat[self.base_id].reshape(3, 3)
        return rot.T @ (data.subtree_com[self.base_id] - data.qpos[:3])

    def _weight(self):
        model = self.env.model
        return float(model.body_mass.sum() * abs(model.opt.gravity[2]))

    def _joint_margin_now(self):
        """Joint margin each planted leg needs: 4 degrees, or 1 degree less than it has now."""
        from .ascent_planner import JOINT_MARGIN, KNEE_PLAN_MIN

        env = self.env
        q = env.data.qpos[env._leg_positions].reshape(6, 3)
        low = env.joint_low.reshape(6, 3).copy()
        low[:, 2] = np.maximum(low[:, 2], KNEE_PLAN_MIN)
        now = np.minimum(q - low, env.joint_high.reshape(6, 3) - q).min(axis=1)
        return np.minimum(JOINT_MARGIN, np.maximum(0.0, now - math.radians(1.0)))

    def _torque_limit(self):
        env = self.env
        return env.model.actuator_forcerange[env._legs, 1].copy()

    def _layout(self, phase, feet):
        """Target sole positions and body pose for the current stage of the climb.

        Returns (targets (6, 3), pose (x, z, pitch), movers) where ``movers``
        are the legs this stage lifts onto the tread; the other targets are
        preparatory moves on the sole's current level.
        """
        e, h, r = self.edge, self.top, self.radius
        stand = self.env.stand_height
        up = self._upper(feet)
        low, high = r - 0.0015, h + r - 0.0015
        side = np.sign(self.neutral_xy[:, 1])
        targets = feet.copy()
        front_lip = e - r - 0.030
        if phase == "front":
            # One layout for both front lifts, so the second needs no new gather.
            x = e - 0.27
            pose = (x, stand + 0.55 * h + 0.005, math.radians(15 if h < 0.17 else 18))
            for leg in FRONT:
                targets[leg] = (e + 0.075, side[leg] * 0.205, high) if not up[leg] else feet[leg]
            for leg in MIDDLE:
                targets[leg] = (min(x + 0.02, front_lip), side[leg] * 0.228, low)
            for leg in REAR:
                targets[leg] = (x - 0.20, side[leg] * 0.200, low)
            return targets, pose, FRONT
        if phase == "middle":
            ready = all(feet[leg, 0] >= e + 0.18 for leg in FRONT)
            x = e - (0.02 if ready else 0.12)
            pose = (x, h + 0.07, math.radians(5 if ready else 10))
            for leg in FRONT:
                targets[leg] = (e + 0.25, side[leg] * 0.205, high)
            for leg in MIDDLE:
                targets[leg] = (
                    (e + 0.09, side[leg] * 0.245, high)
                    if ready
                    else (min(x + 0.02, front_lip), side[leg] * 0.228, low)
                )
            for leg in REAR:
                targets[leg] = (x - 0.22, side[leg] * 0.200, low)
            return targets, pose, MIDDLE if ready else ()
        if phase == "rear":
            ready = all(feet[leg, 0] >= e + 0.29 for leg in FRONT) and all(
                feet[leg, 0] >= e + 0.12 for leg in MIDDLE
            )
            x = e + (0.16 if ready else 0.08)
            pose = (x, h + (0.10 if ready else 0.09), math.radians(10))
            for leg in FRONT:
                targets[leg] = (e + 0.33, side[leg] * 0.205, high)
            for leg in MIDDLE:
                targets[leg] = (e + 0.15, side[leg] * 0.245, high)
            for leg in REAR:
                targets[leg] = (
                    (e + 0.065, side[leg] * 0.205, high)
                    if ready
                    else (min(x - 0.20, front_lip), side[leg] * 0.200, low)
                )
            return targets, pose, REAR if ready else ()
        return targets, (float(self.body[0]), stand, 0.0), ()

    def _separate(self, targets, feet):
        """Keep a rear target 8 cm behind its middle while that middle is still below."""
        up = self._upper(feet)
        for middle, rear in ((1, 2), (4, 5)):
            if not up[middle] and not up[rear] and targets[rear, 2] < self.top:
                targets[rear, 0] = min(targets[rear, 0], feet[middle, 0] - 0.08)
        return targets

    def _candidates(self, phase, feet, *, allow_lift=True):
        """(leg, kind, target) in preference order: preparatory moves, then lifts.

        With ``allow_lift=False`` and a preparatory move still far from its
        target, only preparatory moves are offered: a lift that leaves a sole
        far behind makes the following steps infeasible.
        """
        targets, _, movers = self._layout(phase, feet)
        targets = self._separate(targets, feet)
        up = self._upper(feet)
        error = np.linalg.norm(targets[:, :2] - feet[:, :2], axis=1)
        prep = [
            leg
            for leg in range(6)
            if leg not in movers and error[leg] > 0.018 and (targets[leg, 2] > self.top) == up[leg]
        ]
        prep.sort(key=lambda leg: -error[leg])
        lifts = [leg for leg in movers if not up[leg]]
        big = [leg for leg in prep if error[leg] > 0.04]
        if big and not allow_lift:
            lifts = []
        order = [(leg, "advance") for leg in big] + [(leg, "up") for leg in lifts]
        order += [(leg, "advance") for leg in prep if leg not in big]
        return [
            (leg, kind, targets[leg]) for leg, kind in order if self.blocked.get((leg, kind), 0) < 3
        ]

    def _landings(self, leg, kind, feet, target):
        side = math.copysign(1.0, self.neutral_xy[leg, 1])
        r = self.radius
        if kind == "up":
            for dx, dy in ((0, 0), (0.02, 0), (0, 0.025), (0.02, 0.025), (-0.015, 0), (0, -0.02)):
                yield np.array([target[0] + dx, target[1] + side * dy, target[2]])
            return
        start = feet[leg]
        step = target[:2] - start[:2]
        length = float(np.linalg.norm(step))
        longest = 0.14 if leg in REAR else 0.11
        if length > longest:
            step *= longest / length
        limit = self.edge - r - 0.028
        upper = start[2] > self.top
        for scale, dy in ((1.0, 0.0), (0.7, 0.0), (1.0, -0.02), (0.5, 0.0)):
            point = np.array(
                [start[0] + scale * step[0], start[1] + scale * step[1] + side * dy, target[2]]
            )
            if not upper:
                point[0] = min(point[0], limit)
            if np.linalg.norm(point[:2] - start[:2]) > 0.012:
                yield point

    def _poses(self, phase, leg, kind, landing, feet):
        """Body pose candidates, closest first, on the way to the stage pose."""
        _, (x0, z0, p0), _ = self._layout(phase, feet)
        cx, cz, cp = float(self.body[0]), float(self.body[2]), self.pitch
        stand = self.env.stand_height
        nx = cx + float(np.clip(x0 - cx, -0.06, 0.06))
        nz = cz + float(np.clip(z0 - cz, -0.04, 0.04))
        npitch = cp + float(np.clip(p0 - cp, -math.radians(6), math.radians(6)))
        side = math.copysign(1.0, self.neutral_xy[leg, 1]) if leg is not None else 0.0
        dxs = (0.0, -0.03, 0.03, -0.06)
        dzs = (0.0, 0.015, -0.015, 0.03, -0.03)
        dps = tuple(math.radians(v) for v in (0, 5, -5, 10))
        dys = (0.0, -0.015 * side, -0.03 * side) if side else (0.0,)
        cands = []
        for dx, dz, dp, dy in itertools.product(dxs, dzs, dps, dys):
            cost = abs(dx) / 0.03 + abs(dz) / 0.015 + abs(dp) / math.radians(5) + abs(dy) / 0.015
            cands.append((cost, nx + dx, dy, max(stand - 0.015, nz + dz), npitch + dp))
        cands.sort(key=lambda c: c[0])
        if kind == "advance":
            cands.insert(0, (0.0, cx, float(self.body[1]), cz, cp))
        return cands

    def _max_lag(self, body, feet, skip=None):
        """How far the most trailing lower-level sole is behind its neutral spot."""
        lag = body[0] + self.neutral_xy[:, 0] - feet[:, 0]
        lag[self._upper(feet)] = -np.inf
        if skip is not None:
            lag[skip] = -np.inf
        return float(lag.max())

    def _chassis_clear(self, body, pitch):
        world = CHASSIS_BOTTOM @ rotation(pitch).T + body
        for edge, top in ((self.edge, self.top),):
            over = world[:, 0] >= edge - 0.004
            if np.any(over & (world[:, 2] < top + 0.010)):
                return False
        return True

    def _plan(self, residual):
        feet = self.anchors.copy()
        phase = self.staged_phase()
        if phase == "top":
            self._hand_back()
            return None
        geom = self.geom
        torque = self._torque_limit()
        com_body = self._com_body()
        weight = self._weight()
        steps = [(self.edge, self.top)]
        margin = self._joint_margin_now()
        budget = 1400
        checks = 0
        reasons = self.last_reasons = {}

        def note(leg, kind, why):
            key = f"{LEG_NAMES[leg]} {kind}: {why}"
            reasons[key] = reasons.get(key, 0) + 1

        first = self._candidates(phase, feet, allow_lift=False)
        full = self._candidates(phase, feet)
        candidates = first + [
            item for item in full if not any(item[0] == f[0] and item[1] == f[1] for f in first)
        ]
        for leg, kind, target in candidates:
            leg_checks = 0
            stance = np.ones(6, dtype=bool)
            stance[leg] = False
            upper_others = self._upper(feet) & stance
            for landing in self._landings(leg, kind, feet, target):
                landing = landing.copy()
                landing[:2] += (0.020 * residual[8], 0.020 * residual[9] * np.sign(landing[1]))
                on_tread = kind == "up" or self._upper(feet)[leg]
                if on_tread and landing[0] < self.edge + self.radius + 0.007:
                    note(leg, kind, "pad off tread")
                    continue
                others = (
                    feet[upper_others]
                    if landing[2] > self.top
                    else feet[stance & ~self._upper(feet)]
                )
                if (
                    len(others)
                    and np.linalg.norm(others[:, :2] - landing[:2], axis=1).min() < NEIGHBOUR_GAP
                ):
                    note(leg, kind, "neighbour sole")
                    continue
                neighbours = feet[
                    [n for n in (leg - 1, leg + 1) if 0 <= n < 6 and n // 3 == leg // 3]
                ]
                for _, x, y, z, pitch in self._poses(phase, leg, kind, landing, feet)[
                    :POSES_PER_LANDING
                ]:
                    checks += 1
                    leg_checks += 1
                    if checks > budget or leg_checks > LEG_BUDGET:
                        break
                    body = np.array([x, y, z])
                    if not self._chassis_clear(body, pitch):
                        note(leg, kind, "chassis over tread")
                        continue
                    if self._max_lag(body, feet, skip=leg) > MAX_LAG_M:
                        note(leg, kind, "leaves a sole behind")
                        continue
                    pre = check_stance(
                        geom,
                        torque,
                        feet,
                        np.ones(6, dtype=bool),
                        body,
                        pitch,
                        com_body,
                        weight,
                        steps,
                        required_margin=margin,
                    )
                    if not pre.feasible:
                        note(leg, kind, "six-foot " + pre.reason)
                        continue
                    hold = check_stance(
                        geom,
                        torque,
                        feet,
                        stance,
                        body,
                        pitch,
                        com_body,
                        weight,
                        steps,
                        required_margin=margin,
                    )
                    if not hold.feasible:
                        note(leg, kind, "five-foot " + hold.reason)
                        continue
                    crossing = feet[leg, 0] < self.edge <= landing[0]
                    lift = 0.030 + 0.015 * residual[1]
                    for retreat, bulge, clearance in itertools.product(
                        (0.030, 0.060), (0.0, 0.03, 0.06), (0.026, 0.040)
                    ):
                        if not crossing and (retreat != 0.030 or bulge or clearance != 0.026):
                            continue
                        crest = (
                            self.top + self.radius + clearance
                            if crossing
                            else max(landing[2], feet[leg, 2]) + lift
                        )
                        path, waypoints = swing_path(
                            feet[leg],
                            landing,
                            crest,
                            self.edge if crossing else None,
                            self.radius,
                            retreat=retreat,
                            bulge=bulge,
                        )
                        swing = check_swing(geom, leg, path, body, pitch, neighbours, steps)
                        if not swing.feasible:
                            note(leg, kind, swing.reason)
                        if swing.feasible:
                            self.plans += 1
                            # A level step may start while the body is still
                            # moving if the stance also holds at the present pose
                            # and the whole swing is reachable from there.
                            concurrent = False
                            if kind == "advance" and not crossing:
                                here = np.array(self.body)
                                now = check_stance(
                                    geom,
                                    torque,
                                    feet,
                                    stance,
                                    here,
                                    self.pitch,
                                    com_body,
                                    weight,
                                    steps,
                                    required_margin=margin,
                                )
                                concurrent = (
                                    now.feasible
                                    and check_swing(
                                        geom, leg, path, here, self.pitch, neighbours, steps
                                    ).feasible
                                )
                            return {
                                "phase": phase,
                                "leg": leg,
                                "kind": kind,
                                "landing": landing,
                                "waypoints": waypoints,
                                "body": body,
                                "pitch": pitch,
                                "torque_ratio": hold.torque_ratio,
                                "worst_joint": hold.worst_joint,
                                "com_margin": hold.com_margin,
                                "checks": checks,
                                "concurrent": concurrent,
                            }
                if checks > budget or leg_checks > LEG_BUDGET:
                    break
            if checks > budget:
                break
        shift = self._shift_plan(phase, feet, geom, torque, com_body, weight, steps, margin)
        if shift is not None:
            self.plans += 1
            return shift
        self.failed_plans += 1
        self.log.append(
            {"failed": phase, "reasons": sorted(reasons.items(), key=lambda kv: -kv[1])[:6]}
        )
        return None

    def _shift_plan(self, phase, feet, geom, torque, com_body, weight, steps, margin):
        """Move only the body towards the stage pose, if every check allows it."""
        _, (x0, z0, p0), _ = self._layout(phase, feet)
        cx, cz, cp = float(self.body[0]), float(self.body[2]), self.pitch
        options = []
        if not (abs(x0 - cx) < 0.01 and abs(z0 - cz) < 0.01 and abs(p0 - cp) < math.radians(2)):
            for frac in (1.0, 0.6, 0.3):
                options.append(
                    (
                        cx + frac * float(np.clip(x0 - cx, -0.06, 0.06)),
                        cz + frac * float(np.clip(z0 - cz, -0.04, 0.04)),
                        cp + frac * float(np.clip(p0 - cp, -math.radians(6), math.radians(6))),
                    )
                )
        # Relief poses: back towards the trailing soles and a little lower,
        # which unloads stretched rear legs and lets them be lifted again.
        for dx, dz in ((-0.04, -0.02), (-0.06, 0.0), (-0.03, -0.035)):
            options.append((cx + dx, max(self.env.stand_height, cz + dz), cp))
        for x, z, pitch in options:
            body = np.array([x, 0.0, z])
            if not self._chassis_clear(body, pitch) or self._max_lag(body, feet) > MAX_LAG_M:
                continue
            check = check_stance(
                geom,
                torque,
                feet,
                np.ones(6, dtype=bool),
                body,
                pitch,
                com_body,
                weight,
                steps,
                required_margin=margin,
            )
            if check.feasible:
                # Do not carry the body so far that the most trailing foot can
                # no longer be lifted from the new pose.
                lag = body[0] + self.neutral_xy[:, 0] - feet[:, 0]
                lag[self._upper(feet)] = -np.inf
                worst = int(np.argmax(lag))
                if lag[worst] > 0.03:
                    stance = np.ones(6, dtype=bool)
                    stance[worst] = False
                    if not check_stance(
                        geom,
                        torque,
                        feet,
                        stance,
                        body,
                        pitch,
                        com_body,
                        weight,
                        steps,
                        required_margin=margin,
                    ).feasible:
                        continue
                if self.last_shift is not None and np.allclose(
                    self.last_shift, (x, z, pitch), atol=1e-3
                ):
                    continue
                self.last_shift = (x, z, pitch)
                return {
                    "phase": phase,
                    "leg": None,
                    "kind": "shift",
                    "landing": None,
                    "waypoints": None,
                    "body": body,
                    "pitch": pitch,
                    "torque_ratio": check.torque_ratio,
                    "worst_joint": check.worst_joint,
                    "com_margin": check.com_margin,
                    "checks": 0,
                    "concurrent": False,
                }
        return None

    # ------------------------------------------------------------- execution
    def _measured_ready(self, plan):
        """Measured support polygon and centre of mass before lifting ``plan['leg']``."""
        env = self.env
        data = env.data
        contacts = self.contacts()
        stance = contacts.copy()
        stance[plan["leg"]] = False
        if stance.sum() < 3:
            return False, "fewer than three soles"
        feet = data.xpos[env._foot_ids]
        com = data.subtree_com[self.base_id, :2]
        margin = hull_margin(feet[stance, :2], com)
        if margin < 0.6 * COM_INSET:
            return False, "centre of mass"
        if stance.sum() == 3:
            r = data.xmat[self.base_id].reshape(3, 3)
            pitch = math.atan2(r[2, 0], math.hypot(r[2, 1], r[2, 2]))
            check = check_stance(
                self.geom,
                self._torque_limit(),
                feet,
                stance,
                data.qpos[:3].copy(),
                pitch,
                self._com_body(),
                self._weight(),
                [(self.edge, self.top)],
                torque_use=0.95,
            )
            if not check.feasible:
                return False, check.reason
        return True, "ok"

    def _staged_targets(self, residual):
        env = self.env
        data = env.data
        self.elapsed += 0.02
        self.timer += 0.02
        contacts = self.contacts()
        feet = data.xpos[env._foot_ids].copy()
        upper = self.upper_contacts(feet, contacts)
        self.peak_upper_contacts = max(self.peak_upper_contacts, int(upper.sum()))
        if self.state == "plan":
            self.plan = self._plan(residual)
            if self.mode == "top":
                return super().targets(residual)
            if self.plan is None:
                # Nothing feasible from here: settle into the stand pose over
                # the current feet and try again shortly.
                support = self.anchors[contacts]
                if len(support) >= 3:
                    self.body[:2] += np.clip(
                        support[:, :2].mean(axis=0) - self.body[:2], -0.002, 0.002
                    )
                self.state, self.timer = "wait", 0.0
                self.blocked.clear()
            else:
                self.state, self.timer = "shift", 0.0
                self.stage_body = self.plan["body"].copy()
                self.pose_pitch = self.plan["pitch"]
                entry = {
                    k: self.plan[k] for k in ("phase", "leg", "kind", "torque_ratio", "worst_joint")
                }
                entry.update(
                    t=round(self.env.steps * 0.02, 2),
                    body=np.round(self.plan["body"], 3).tolist(),
                    pitch_deg=round(math.degrees(self.plan["pitch"]), 1),
                )
                if self.plan["leg"] is not None:
                    entry.update(
                        landing=np.round(self.plan["landing"], 3).tolist(),
                        lift_off=np.round(self.anchors[self.plan["leg"]], 3).tolist(),
                    )
                self.log.append(entry)
        if self.state == "wait" and self.timer >= 0.4:
            self.state = "plan"
        target = self.body.copy() if self.stage_body is None else self.stage_body.copy()
        target += (0.012 * residual[2], 0.012 * residual[4], 0.012 * residual[3])
        pitch_target = self.pose_pitch + 0.05 * residual[5]
        if self.state == "shift":
            error = np.abs(target - data.qpos[:3])
            r = data.xmat[self.base_id].reshape(3, 3)
            pitch_now = math.atan2(r[2, 0], math.hypot(r[2, 1], r[2, 2]))
            # A level step tolerates more residual body motion than a lift
            # onto the tread; both still pass the measured support check.
            tol = 0.012 if self.plan["kind"] == "up" else 0.022
            settled = (
                error[0] < tol
                and error[1] < tol
                and error[2] < tol
                and abs(pitch_now - pitch_target) < math.radians(3.0 if tol < 0.02 else 5.0)
                and np.abs(self.velocity).max() < (0.01 if tol < 0.02 else 0.03)
            )
            settled = settled or self.plan.get("concurrent", False)
            if (settled or self.timer > 2.6) and self.plan["leg"] is None:
                self.state, self.timer = "settle", 0.0
            elif settled or self.timer > 2.6:
                ready, reason = self._measured_ready(self.plan)
                if ready:
                    self.state, self.timer = "swing", 0.0
                    self.leg = self.plan["leg"]
                    self.lift_off = self.anchors[self.leg].copy()
                    self.landing = self.plan["landing"]
                    self.elapsed = 0.0
                elif self.timer > 3.4:
                    key = (self.plan["leg"], self.plan["kind"])
                    self.blocked[key] = self.blocked.get(key, 0) + 1
                    self.log.append({"blocked": key, "reason": reason})
                    self.state, self.timer = "plan", 0.0
        if self.state == "swing":
            done = self._swing_update(contacts, feet)
            if done:
                self.log.append(
                    {
                        "t": round(self.env.steps * 0.02, 2),
                        "landed": LEG_NAMES[self.leg],
                        "at": np.round(feet[self.leg], 3).tolist(),
                        "contact": bool(contacts[self.leg]),
                        "body_z": round(float(data.qpos[2]), 3),
                    }
                )
                self.anchors[self.leg] = feet[self.leg]
                self.leg = None
                self.state, self.timer = "settle", 0.0
                self.blocked.clear()
        if self.state == "settle" and self.timer >= 0.06:
            self.state = "plan"
        # Body: rate- and acceleration-limited motion to the pose target.
        rate = np.clip(6 * (target - self.body), [-0.06, -0.04, -0.05], [0.08, 0.04, 0.05])
        self.velocity += np.clip(rate - self.velocity, -0.8 * 0.02, 0.8 * 0.02)
        self.body += self.velocity * 0.02
        self.pitch += float(np.clip(pitch_target - self.pitch, -0.25 * 0.02, 0.25 * 0.02))
        self.roll += float(np.clip(-self.roll, -0.3 * 0.02, 0.3 * 0.02))
        self.yaw += float(np.clip(-self.yaw, -0.27 * 0.02, 0.27 * 0.02))
        stance = contacts.copy()
        if self.leg is not None:
            stance[self.leg] = False
        self.support_count = int(stance.sum())
        footholds = self.anchors.copy()
        if self.leg is not None:
            footholds[self.leg] = self.swing_point
        return self._to_leg_frames(footholds)

    def _swing_update(self, contacts, feet):
        """Follow the checked waypoints, gated on the measured sole position.

        The sole only starts across the lip once it is measured near the crest
        height, and the crossing pauses while the measured sole is still below
        the lip, so a lagging servo does not drag the sole up the riser face.
        """
        plan = self.plan
        waypoints = plan["waypoints"]
        lengths = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
        crossing = plan["landing"][2] > self.lift_off[2] + 0.05
        duration = (1.25 if crossing else 0.34) + 0.55 * min(1.0, float(lengths.sum()) / 0.30)
        if self.elapsed <= 0.02:
            self.segment, self.progress = 0, 0.0
        shares = (0.30, 0.42, 0.28)
        observed = feet[self.leg]
        self.progress += 0.02 / (shares[self.segment] * duration)
        if self.segment == 0 and self.progress >= 1.0:
            if (
                not crossing
                or observed[2] >= waypoints[1][2] - 0.012
                or self.elapsed > 2.5 * duration
            ):
                self.segment, self.progress = 1, 0.0
            else:
                self.progress = 1.0
        elif self.segment == 1:
            lip = self.edge - self.radius - 0.004 < observed[0] < self.edge + self.radius + 0.004
            if crossing and lip and observed[2] < self.top + self.radius + 0.006:
                self.progress -= 0.02 / (
                    shares[1] * duration
                )  # hold until the sole is above the lip
            if self.progress >= 1.0:
                self.segment, self.progress = 2, 0.0
        self.progress = min(self.progress, 1.0)
        t = ease(self.progress)
        point = waypoints[self.segment] + t * (
            waypoints[self.segment + 1] - waypoints[self.segment]
        )
        if self.segment == 1 and crossing:
            point[2] += 0.006  # a little extra lift while crossing
        done_moving = self.segment == 2 and self.progress >= 1.0
        if done_moving and not contacts[self.leg]:
            # Probe gently below the planned height until the sole touches.
            self.probe = getattr(self, "probe", 0.0) + 0.02
            point = waypoints[-1].copy()
            point[2] -= min(0.012, 0.03 * self.probe)
        else:
            self.probe = 0.0
        self.swing_point = point
        near = np.linalg.norm(observed[:2] - plan["landing"][:2]) < 0.022
        lowering = self.segment == 2 and self.progress >= (1.0 if crossing else 0.7)
        touched = contacts[self.leg] and (done_moving or lowering) and near
        return touched or self.elapsed > 2.8 * duration + 0.8

    def _to_leg_frames(self, footholds):
        env = self.env
        from .ascent_pose import attitude_matrix

        rot = attitude_matrix(self.roll, self.pitch, self.yaw)
        body_frame = (footholds - self.body) @ rot - env.hip
        mc, ms = np.cos(env.mount), np.sin(env.mount)
        return np.column_stack(
            (
                body_frame[:, 0] * mc + body_frame[:, 1] * ms,
                -body_frame[:, 0] * ms + body_frame[:, 1] * mc,
                body_frame[:, 2],
            )
        )


# Residual dimensions the staged controller uses: body forward, height,
# sideways and pitch, and the landing point. The others are held at zero.
STAGED_ACTIONS = (2, 3, 4, 5, 8, 9)
_SKIP = {"env", "terrain", "geom", "top_gait", "log", "last_reasons"}
_ENV_FIELDS = (
    "steps",
    "phase",
    "supply_scale",
    "pack_voltage_proxy_v",
    "command",
    "gait",
    "switch_step",
    "approach_speed",
    "approach_x",
    "adjustment",
    "previous_action",
    "last_contacts",
    "last_feet",
    "upper_rewarded",
    "upper_reward_index",
    "min_body_z",
    "max_body_z",
    "max_lateral_com",
    "min_knee_position",
    "attitude_peak",
    "_max_target_rate",
    "touchdowns",
    "slip_m",
    "work_j",
    "max_clearance_m",
)


def capture_state(env):
    """Snapshot of a running staged episode that ``restore_state`` can resume."""
    import copy

    import mujoco

    model, data = env.model, env.data
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    physics = np.zeros(mujoco.mj_stateSize(model, spec))
    mujoco.mj_getState(model, data, physics, spec)
    return {
        "course": env.course_name,
        "physics": physics,
        "model": {
            name: getattr(model, name).copy()
            for name in (
                "actuator_forcerange",
                "actuator_gainprm",
                "actuator_biasprm",
                "body_mass",
                "body_inertia",
                "geom_friction",
            )
        },
        "env": {name: copy.deepcopy(getattr(env, name)) for name in _ENV_FIELDS},
        "controller": {
            k: copy.deepcopy(v) for k, v in env.body_motion.__dict__.items() if k not in _SKIP
        },
        "brace": copy.deepcopy(env.brace.__dict__) if env.brace is not None else None,
        "phase_name": env.body_motion.staged_phase(),
    }


def restore_state(env, snapshot):
    import copy

    import mujoco

    model, data = env.model, env.data
    if snapshot["course"] != env.course_name:
        raise ValueError("snapshot belongs to another course")
    for name, value in snapshot["model"].items():
        getattr(model, name)[:] = value
    mujoco.mj_setConst(model, data)
    mujoco.mj_setState(model, data, snapshot["physics"], mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(model, data)
    for name, value in snapshot["env"].items():
        setattr(env, name, copy.deepcopy(value))
    env.body_motion.__dict__.update(copy.deepcopy(snapshot["controller"]))
    env.body_motion.log = []
    if snapshot["brace"] is not None:
        env.brace.__dict__.update(copy.deepcopy(snapshot["brace"]))
    if env.link_metrics is not None:
        env.link_metrics.reset()


class StagedEnv(LocomotionEnv):
    """``LocomotionEnv`` with the staged controller; same observations and actions.

    ``start_states`` (a list of ``capture_state`` snapshots from successful
    climbs) lets training begin a ``start_fraction`` of its episodes from a
    recorded intermediate pose, so the rear transfer is practised often.
    Evaluation environments never receive start states.
    """

    fingerprint_sources = ("staged_ascent.py", "ascent_planner.py", "gait_lab.py")

    def __init__(self, course, *, start_states=None, start_fraction=0.0, **kwargs):
        if course not in STAGED:
            raise ValueError(f"{course} is not a staged ascent course")
        super().__init__(course, **kwargs)
        self.body_motion = StagedAscent(self)
        self.body_motion.reset()
        self.start_states = [s for s in (start_states or ()) if s["course"] == course]
        self.start_fraction = start_fraction
        self.started_from = None
        self.action_mask = np.zeros(self.action_space.shape, dtype=np.float32)
        self.action_mask[list(STAGED_ACTIONS)] = 1.0

    def _approach_complete(self, x: float) -> bool:
        # The fast tripod walk continues to 0.50 m before the riser (0.72 m on
        # the brace-transfer courses); the planted wave gait takes over there.
        return self.course.edge_x - x <= TRIPOD_HANDOFF_M

    def reset(self, *, seed=None, options=None):
        obs, info = super().reset(seed=seed, options=options)
        self.started_from = None
        if self.start_states and self.np_random.random() < self.start_fraction:
            snapshot = self.start_states[int(self.np_random.integers(len(self.start_states)))]
            restore_state(self, snapshot)
            self.started_from = snapshot["phase_name"]
            obs = self._observation()
        info["started_from"] = self.started_from
        return obs, info

    def step(self, action):
        action = np.asarray(action, dtype=np.float32) * self.action_mask
        obs, reward, done, truncated, info = super().step(action)
        controller = self.body_motion
        info["staged_phase"] = controller.staged_phase()
        info["staged_plans"] = controller.plans
        info["staged_failed_plans"] = controller.failed_plans
        return obs, reward, done, truncated, info


def record_start_states(course, seeds, *, phases=("front", "middle", "rear"), every_s=2.0):
    """Intermediate states from successful deterministic climbs, for training resets."""
    states = []
    for seed in seeds:
        env = StagedEnv(course)
        try:
            env.reset(seed=seed)
            kept = []
            while True:
                controller = env.body_motion
                phase = controller.staged_phase()
                if (
                    controller.mode == "staged"
                    and phase in phases
                    and controller.leg is None
                    and controller.state == "plan"
                    and (not kept or env.steps * 0.02 - kept[-1][0] >= every_s)
                ):
                    kept.append((env.steps * 0.02, capture_state(env)))
                _, _, done, truncated, info = env.step(np.zeros(env.action_space.shape))
                if done or truncated:
                    break
            clean = (
                info["success"]
                and not info["fallen"]
                and not info.get("body_impact", False)
                and info["riser_contact_substeps"] * env.model.opt.timestep <= 0.25
            )
            if clean:
                states.extend(snapshot for _, snapshot in kept)
        finally:
            env.close()
    return states


def make_env(course, **kwargs):
    return StagedEnv(course, **kwargs) if course in STAGED else LocomotionEnv(course, **kwargs)
