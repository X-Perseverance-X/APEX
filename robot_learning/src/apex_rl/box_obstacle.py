"""Climb onto a box, walk on it, and step down the far side. Simulation only.

The box is 16 cm high, 60 cm wide and 60 cm long (``hybrid-box-16``). The climb is the
staged ascent of ``staged_ascent.py`` unchanged. On top, the flat tripod walks
until the front soles are near the far edge and halts. The descent is planned
one sole at a time with the same static checks as the climb (reach and joint
margins, centre of mass inside the support polygon, joint torque, femur/tibia
clearance to the box, the whole swing path), evaluated in a mirrored frame:
stepping forward down an edge is, geometrically, stepping backward up it.
After the last rear sole is down the flat tripod walks to the goal.

Nothing here edits the sources that recorded policies are fingerprinted against.
"""

from __future__ import annotations

import dataclasses
import itertools
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

from .ascent_planner import NEIGHBOUR_GAP, check_stance, check_swing, hull_margin, swing_path
from .body_motion import ease
from .body_visual import fmt
from .locomotion import COURSES, Course
from .staged_ascent import (
    CHASSIS_BOTTOM,
    FRONT,
    LEG_NAMES,
    MIDDLE,
    REAR,
    StagedAscent,
    StagedEnv,
    rotation,
)

MIRROR = [2, 1, 0, 5, 4, 3]  # front <-> rear when x is mirrored
SIDE_Y = np.array([0.205, 0.245, 0.205, 0.205, 0.245, 0.205])
TOP_MARGIN = 0.045  # sole centre at least this far back from the far edge while on top
FLOOR_MARGIN = 0.060  # and this far beyond it once down
STOP_FRONT_M = 0.10  # tripod on top halts with the front soles this far from the edge


@dataclass(frozen=True)
class BoxCourse(Course):
    length_m: float = 0.60
    width_m: float = 0.60

    @property
    def far_edge(self) -> float:
        return self.edge_x + self.length_m

    @property
    def step_edges(self) -> tuple[float, ...]:
        return (self.edge_x,)

    @property
    def last_edge(self) -> float:
        # Success needs every sole beyond the far edge, back on the floor.
        return self.far_edge

    @property
    def goal(self) -> float:
        return self.far_edge + 0.45

    @property
    def terrain_names(self) -> tuple[str, ...]:
        return ("course_step_1",)

    def height(self, x: float, y: float = 0.0) -> float:
        inside = self.edge_x <= x <= self.far_edge and abs(y) <= self.width_m / 2
        return self.rise_m if inside else 0.0

    def scene(self, config: dict) -> str:
        root = ET.fromstring(Course.scene(dataclasses.replace(self, risers=0), config))
        ET.SubElement(
            root.find("worldbody"),
            "geom",
            name="course_step_1",
            type="box",
            pos=fmt([self.edge_x + self.length_m / 2, 0.0, self.rise_m / 2]),
            size=fmt([self.length_m / 2, self.width_m / 2, self.rise_m / 2]),
            friction="1 0.005 0.0001",
            rgba="0.40 0.22 0.13 1",
        )
        return ET.tostring(root, encoding="unicode")


_BASE = COURSES["hybrid-brace-transfer-up-16"]
COURSES["hybrid-box-16"] = BoxCourse(
    **{f.name: getattr(_BASE, f.name) for f in dataclasses.fields(Course)}
    | {"rise_m": 0.16, "risers": 1, "home_stairs": None}
)
BOX_COURSES = ("hybrid-box-16",)
BOX_TIME_LIMIT_S = 150.0  # climb, walk on top and step down take about two minutes


class BoxTraverse(StagedAscent):
    """Staged climb, a halting tripod walk on top, then a planned descent."""

    def reset(self):
        super().reset()
        self.stuck = 0
        self.shifts = 0
        self.descended = False
        self.halting = False
        self.halted_s = 0.0
        self.descent_log = []

    # The controller plans with the box it is told about. ``geometry_error`` on the
    # environment (near edge, far edge, height; metres) models a measured box that
    # differs from the real one; the physics always uses the real box.
    @property
    def far(self):
        return self.terrain.far_edge + self.env.geometry_error[1]

    @property
    def edge(self):
        # During the descent the near edge is behind the robot; no checks use it.
        if self.mode == "descend":
            return 1e3
        return self.terrain.step_edges[0] + self.env.geometry_error[0]

    @property
    def top(self):
        return self.terrain.rise_m + self.env.geometry_error[2]

    def staged_phase(self):
        if self.mode == "descend":
            down = self._down(self.anchors)
            if not (down[0] and down[3]):
                return "down-front"
            if not (down[1] and down[4]):
                return "down-middle"
            return "down-rear"
        if self.mode == "top" and self.descended:
            return "floor"
        return super().staged_phase()

    def _down(self, points):
        return (points[:, 0] > self.far + self.radius) & (points[:, 2] < self.top / 2)

    # --------------------------------------------------------------- top walk
    def targets(self, residual):
        if self.mode == "descend":
            return self._staged_targets(residual)
        return super().targets(residual)

    def _top_targets(self, residual):
        if self.descended:
            return super()._top_targets(residual)
        env = self.env
        feet = env.data.xpos[env._foot_ids]
        if feet[[0, 3], 0].max() >= self.far - STOP_FRONT_M:
            self.halting = True
        if not self.halting:
            return super()._top_targets(residual)
        self.body = env.data.qpos[:3].copy()
        self.support_count = int(self.contacts().sum())
        out = self.top_gait.feet((0.0, 0.0, 0.0))
        if self.top_gait.halted:
            self.halted_s += 0.02
            # A halted tripod can leave one sole just short of loading; after a
            # second, five loaded soles are enough: every descent move re-checks the
            # measured support polygon before it lifts anything.
            if self.contacts().sum() == 6 or (self.halted_s > 1.0 and self.contacts().sum() >= 5):
                self._start_descent()
        return out

    def _start_descent(self):
        data = self.env.data
        self.mode = "descend"
        self.state = "plan"
        self.anchors = data.xpos[self.env._foot_ids].copy()
        self.body = data.qpos[:3].copy()
        r = data.xmat[self.base_id].reshape(3, 3)
        self.pitch = math.atan2(r[2, 0], math.hypot(r[2, 1], r[2, 2]))
        self.pose_pitch = self.pitch
        self.stage_body = None
        self.velocity[:] = 0.0
        self.leg = None
        self.elapsed = 0.0
        self.blocked.clear()
        self.last_shift = None

    # -------------------------------------------------------- mirrored checks
    def _mirrored(self, feet, body, pitch):
        mf = feet[MIRROR].copy()
        mf[:, 0] *= -1
        mb = np.array(body, dtype=float).copy()
        mb[0] *= -1
        return mf, mb, -pitch

    def _stance_ok(self, feet, stance, body, pitch, margin):
        mf, mb, mp = self._mirrored(feet, body, pitch)
        com = self._com_body() * np.array([-1.0, 1.0, 1.0])
        torque = self._torque_limit().reshape(6, 3)[MIRROR].reshape(-1)
        return check_stance(
            self.geom, torque, mf, stance[MIRROR], mb, mp, com, self._weight(),
            [(-self.far, self.top)], required_margin=margin[MIRROR],
        )  # fmt: skip

    def _swing_ok(self, leg, path, body, pitch, neighbours):
        mpath = path.copy()
        mpath[:, 0] *= -1
        mnb = neighbours.copy()
        mnb[:, 0] *= -1
        _, mb, mp = self._mirrored(np.zeros((6, 3)), body, pitch)
        return check_swing(self.geom, MIRROR[leg], mpath, mb, mp, mnb, [(-self.far, self.top)])

    def _chassis_ok(self, body, pitch):
        world = CHASSIS_BOTTOM @ rotation(pitch).T + body
        over = (world[:, 0] >= self.terrain.edge_x - 0.004) & (world[:, 0] <= self.far + 0.004)
        floor = world[:, 2] < 0.010
        return not np.any(over & (world[:, 2] < self.top + 0.010)) and not np.any(floor)

    def _down_path(self, start, landing, clearance, retreat, bulge):
        """Rise over the lip, travel out beyond the face, lower onto the floor."""
        ms, ml = start.copy(), landing.copy()
        ms[0] *= -1
        ml[0] *= -1
        crest = self.top + self.radius + clearance
        path, points = swing_path(
            ml, ms, crest, -self.far, self.radius, retreat=retreat, bulge=bulge
        )
        path, points = path[::-1].copy(), points[::-1].copy()
        path[:, 0] *= -1
        points[:, 0] *= -1
        return path, points

    # ---------------------------------------------------------------- planning
    def _layout(self, phase, feet):
        if self.mode != "descend":
            return super()._layout(phase, feet)
        return self._descent_layout(feet)

    def _descent_layout(self, feet):
        """Sole targets, body pose and moving legs for the present descent stage.

        These are the climb's stage layouts for a 16 cm step, mirrored and in reverse
        order: the descent's front pair steps down where the climb's rear pair stepped
        up, then the middle pair, then the rear pair. Returns (targets, pose, movers).
        """
        e, h, r = self.far, self.top, self.radius
        stand = self.env.stand_height
        down = self._down(feet)
        high, low = h + r - 0.0015, r - 0.0015
        side = np.sign(self.neutral_xy[:, 1])
        t = feet.copy()

        def put(legs, x, y, z):
            for leg in legs:
                t[leg] = (x, side[leg] * y, z)

        if not (down[0] and down[3]):
            ready = all(feet[i, 0] <= e - 0.29 for i in REAR) and all(
                e - 0.20 <= feet[i, 0] for i in MIDDLE
            )
            put(REAR, e - 0.33, 0.205, high)
            put(MIDDLE, e - 0.15, 0.245, high)
            for i in FRONT:
                t[i] = (
                    (e + 0.048, side[i] * 0.205, low)
                    if (ready or down[i])
                    else (e - 0.065, side[i] * 0.205, high)
                )
            return t, (e - 0.16, h + 0.10, math.radians(-10)), FRONT if ready else ()
        if not (down[1] and down[4]):
            ready = all(feet[i, 0] >= e + 0.18 for i in FRONT)
            x = e + (0.02 if ready else 0.12)
            put(FRONT, e + 0.24, 0.205, low)
            put(REAR, e - 0.25, 0.205, high)
            for i in MIDDLE:
                t[i] = (
                    (e + 0.048, side[i] * 0.228, low)
                    if (ready or down[i])
                    else (e - 0.09, side[i] * 0.245, high)
                )
            return t, (x, h + 0.07, math.radians(-5 if ready else -10)), MIDDLE if ready else ()
        # The rear soles first walk up to the lip on top (where the climb put its
        # front soles), then step down.
        ready = all(down[i] or feet[i, 0] >= e - 0.13 for i in REAR)
        put(FRONT, e + 0.47, 0.200, low)
        put(MIDDLE, e + 0.25, 0.228, low)
        for i in REAR:
            t[i] = (
                (e + 0.08, side[i] * 0.205, low)
                if (ready or down[i])
                else (e - 0.075, side[i] * 0.205, high)
            )
        x = e + (0.27 if ready else 0.12)
        return t, (x, stand + 0.55 * h + 0.005, math.radians(-15)), REAR if ready else ()

    def _max_lag(self, body, feet, skip=None):
        # After repeated failed climb plans, accept a trailing sole 4 cm further back.
        relief = 0.04 if self.mode == "staged" and self.stuck >= 5 else 0.0
        return super()._max_lag(body, feet, skip) - relief

    def _climb_plan(self, residual):
        before = self.failed_plans
        plan = super()._plan(residual)
        self.stuck = self.stuck + 1 if self.failed_plans > before else 0
        kind = None if plan is None else plan["kind"]
        self.shifts = self.shifts + 1 if kind == "shift" else 0
        if (
            (plan is None or self.shifts >= 4)
            and self.mode == "staged"
            and self.staged_phase() == "rear"
        ):
            # The body is swinging between two shift poses without a sole moving:
            # lift a rear sole first and tilt the body while it hangs.
            feet = self.anchors.copy()
            targets, _, movers = self._layout("rear", feet)
            for leg in movers:
                if self._upper(feet)[leg]:
                    continue
                neighbours = feet[
                    [n for n in (leg - 1, leg + 1) if 0 <= n < 6 and n // 3 == leg // 3]
                ]
                for landing in self._landings(leg, "up", feet, targets[leg]):
                    tilt = self._lift_then_tilt(
                        "rear", leg, landing, feet, neighbours, lambda *a: None, residual
                    )
                    if tilt is not None:
                        self.shifts = 0
                        self.plans += 1
                        return tilt
        return plan

    def _plan(self, residual):
        if self.mode != "descend":
            return self._climb_plan(residual)
        feet = self.anchors.copy()
        down = self._down(feet)
        if down.all():
            self.descended = True
            self._hand_back()
            return None
        margin = self._joint_margin_now()
        targets, pose, movers = self._descent_layout(feet)
        nominal = (pose[0] + 0.012 * float(residual[2]), pose[1], pose[2])
        error = np.linalg.norm(targets[:, :2] - feet[:, :2], axis=1)
        prep = [
            i for i in range(6)
            if i not in movers and error[i] > 0.018 and (targets[i, 2] > self.top) == (not down[i])
        ]  # fmt: skip
        prep.sort(key=lambda i: -error[i])
        lifts = [i for i in movers if not down[i]]
        big = [i for i in prep if error[i] > 0.04]
        order = [(i, "advance") for i in big] + [(i, "down") for i in lifts]
        order += [(i, "advance") for i in prep if i not in big]
        reasons = self.last_reasons = {}
        for leg, kind in order:
            if self.blocked.get((leg, kind), 0) >= 3:
                continue
            landing = targets[leg].copy()
            if kind == "advance":
                step = landing[:2] - feet[leg, :2]
                length = float(np.linalg.norm(step))
                longest = 0.14 if leg in REAR else 0.11
                if length > longest:
                    landing[:2] = feet[leg, :2] + step * longest / length
            landing[0] += 0.02 * float(residual[8])
            sgn = math.copysign(1.0, self.neutral_xy[leg, 1])
            variants = (
                ((0.0, 0.0), (0.03, 0.0), (0.0, 0.02), (0.03, 0.02), (0.06, 0.0))
                if kind == "down"
                else ((0.0, 0.0),)
            )
            for dx, dy in variants:
                option = landing + (dx, sgn * dy, 0.0)
                plan = self._descent_move(leg, kind, option, feet, margin, nominal, reasons)
                if plan is not None:
                    self.plans += 1
                    return plan
        # Nothing can move: carry the body part of the way towards the stage pose.
        nx, nz, npitch = nominal
        cx, cz, cp = float(self.body[0]), float(self.body[2]), float(self.pitch)
        for frac in (1.0, 0.6, 0.3):
            body = np.array(
                [
                    cx + frac * float(np.clip(nx - cx, -0.04, 0.04)),
                    0.0,
                    cz + frac * float(np.clip(nz - cz, -0.03, 0.03)),
                ]
            )
            p = cp + frac * float(np.clip(npitch - cp, -math.radians(5), math.radians(5)))
            key = (round(body[0], 3), round(body[2], 3), round(p, 3))
            if np.allclose(body, (cx, 0.0, cz), atol=0.004) and abs(p - cp) < 0.01:
                continue
            if key == self.last_shift:
                continue
            if (
                self._chassis_ok(body, p)
                and self._stance_ok(feet, np.ones(6, bool), body, p, margin).feasible
            ):
                self.last_shift = key
                self.plans += 1
                return {
                    "phase": self.staged_phase(), "leg": None, "kind": "shift",
                    "landing": None, "waypoints": None, "body": body, "pitch": p,
                    "torque_ratio": 0.0, "worst_joint": None, "com_margin": 0.0,
                    "checks": 0, "concurrent": False,
                }  # fmt: skip
        self.failed_plans += 1
        self.descent_log.append(
            {
                "t": round(self.env.steps * 0.02, 2),
                "failed": sorted(reasons.items(), key=lambda kv: -kv[1])[:5],
            }
        )
        return None

    def _pose_grid(self, nominal):
        """Body poses between the present pose and the stage pose, closest first."""
        nx, nz, npitch = nominal
        cx, cz, cp = float(self.body[0]), float(self.body[2]), float(self.pitch)
        xs = {round(cx + f * float(np.clip(nx - cx, -0.06, 0.06)), 4) for f in (0.0, 0.5, 1.0)}
        out = []
        for x, dz, dp in itertools.product(
            xs,
            (0.0, -0.02, 0.02, -0.04, 0.04, 0.06, -0.06, 0.08),
            np.radians((0, -5, 5, -10, 10, 15)),
        ):
            z = float(np.clip(nz + dz, 0.10, self.top + 0.17))
            p = float(np.clip(npitch + dp, math.radians(-30), math.radians(10)))
            cost = abs(x - cx) / 0.06 + abs(dz) / 0.02 + abs(dp) / math.radians(5)
            cost += abs(z - cz) / 0.08 + abs(p - cp) / math.radians(15)
            out.append((cost, x, z, p))
        out.sort(key=lambda c: c[0])
        return [(x, z, p) for _, x, z, p in out]

    def _descent_move(self, leg, kind, landing, feet, margin, nominal, reasons):
        def note(why):
            key = f"{LEG_NAMES[leg]} {kind}: {why}"
            reasons[key] = reasons.get(key, 0) + 1

        stance = np.ones(6, dtype=bool)
        stance[leg] = False
        same = [i for i in range(6) if i != leg and abs(feet[i, 2] - landing[2]) < 0.05]
        if same and np.linalg.norm(feet[same, :2] - landing[:2], axis=1).min() < NEIGHBOUR_GAP:
            note("neighbour sole")
            return None
        neighbours = feet[[n for n in (leg - 1, leg + 1) if 0 <= n < 6 and n // 3 == leg // 3]]
        sgn = math.copysign(1.0, self.neutral_xy[leg, 1])
        holding = []  # poses where both the six- and the five-foot stance hold
        for (x, z, p), dy in itertools.product(self._pose_grid(nominal), (0.0, 0.02, 0.04)):
            body = np.array([x, -sgn * dy, z])
            if not self._chassis_ok(body, p):
                note("chassis")
                continue
            pre = self._stance_ok(feet, np.ones(6, bool), body, p, margin)
            if not pre.feasible:
                note("six-foot " + pre.reason)
                continue
            hold = self._stance_ok(feet, stance, body, p, margin)
            if not hold.feasible:
                note("five-foot " + hold.reason)
                continue
            holding.append((body, p, hold))
            if len(holding) >= 80:
                break
        options = (
            list(itertools.product((0.03, 0.06), (0.0, 0.03), (0.03, 0.045)))
            if kind == "down"
            else [(0.03, 0.0, 0.0)]
        )
        paths = []
        for retreat, bulge, clearance in options:
            if kind == "down":
                paths.append(self._down_path(feet[leg], landing, clearance, retreat, bulge))
            else:
                crest = max(landing[2], feet[leg, 2]) + 0.03
                paths.append(swing_path(feet[leg], landing, crest, None, self.radius))
        # One pose for the whole swing.
        for body, p, hold in holding:
            for path, waypoints in paths:
                swing = self._swing_ok(leg, path, body, p, neighbours)
                if swing.feasible:
                    return self._move(leg, kind, landing, waypoints, body, p, hold)
                note(swing.reason)
        if kind != "down":
            return None
        # Lift and cross the lip from one pose, lower the sole from a second one:
        # the body tilts nose-down while the sole hangs beyond the edge.
        for path, waypoints in paths[:4]:
            for split in (15, 14, 16, 12, 9):
                first = [
                    h
                    for h in holding
                    if self._swing_ok(leg, path[: split + 1], h[0], h[1], neighbours).feasible
                ]
                if not first:
                    continue
                rest = [
                    h
                    for h in holding
                    if self._swing_ok(leg, path[split:], h[0], h[1], neighbours).feasible
                ]
                for a in first[:15]:
                    for b in rest[:15]:
                        mid, mp = 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])
                        if (
                            self._chassis_ok(mid, mp)
                            and self._stance_ok(feet, stance, mid, mp, margin).feasible
                        ):
                            plan = self._move(leg, kind, landing, waypoints, a[0], a[1], b[2])
                            plan.update(post_body=b[0], post_pitch=b[1], split_index=split)
                            return plan
                note("no two-pose swing")
        return None

    def _move(self, leg, kind, landing, waypoints, body, pitch, hold):
        self.descent_log.append(
            {"t": round(self.env.steps * 0.02, 2), "leg": LEG_NAMES[leg], "kind": kind,
             "landing": np.round(landing, 3).tolist(), "body": np.round(body, 3).tolist(),
             "pitch_deg": round(math.degrees(pitch), 1)}
        )  # fmt: skip
        return {
            "phase": self.staged_phase(), "leg": leg, "kind": kind, "landing": landing,
            "waypoints": waypoints, "body": body, "pitch": pitch,
            "torque_ratio": hold.torque_ratio, "worst_joint": hold.worst_joint,
            "com_margin": hold.com_margin, "checks": 0, "concurrent": False,
        }  # fmt: skip

    def _swing_update(self, contacts, feet):
        """Descent two-pose swings tilt the body exactly where the check split the path."""
        plan = self.plan
        if self.mode != "descend" or plan.get("post_body") is None:
            return super()._swing_update(contacts, feet)
        if self.elapsed <= 0.02:
            self.segment, self.progress = 0, 0.0
        if not self.post_settled:
            index = plan["split_index"]
            seg, frac = index // 6, (index % 6) / 6
            reached = self.segment > seg or (self.segment == seg and ease(self.progress) >= frac)
            if reached:
                data = self.env.data
                if not self.post_started:
                    self.post_started = True
                    self.stage_body = np.array(plan["post_body"], dtype=float)
                    self.pose_pitch = float(plan["post_pitch"])
                r = data.xmat[self.base_id].reshape(3, 3)
                pitch_now = math.atan2(r[2, 0], math.hypot(r[2, 1], r[2, 2]))
                near = (
                    np.abs(self.stage_body - data.qpos[:3]).max() < 0.012
                    and abs(pitch_now - self.pose_pitch) < math.radians(3.0)
                    and np.abs(self.velocity).max() < 0.02
                )
                self.post_wait += 0.02
                if near or self.post_wait > 4.0:
                    self.post_settled = True
                else:
                    self.elapsed -= 0.02  # the swing waits; do not run into its time-out
                    return False
        view = {k: v for k, v in plan.items() if k != "post_body"}
        self.plan = view
        try:
            return super()._swing_update(contacts, feet)
        finally:
            self.plan = plan

    def _measured_ready(self, plan):
        if self.mode != "descend":
            return super()._measured_ready(plan)
        env = self.env
        contacts = self.contacts()
        stance = contacts.copy()
        stance[plan["leg"]] = False
        if stance.sum() < 3:
            return False, "fewer than three soles"
        feet = env.data.xpos[env._foot_ids]
        com = env.data.subtree_com[self.base_id, :2]
        if hull_margin(feet[stance, :2], com) < 0.6 * 0.020:
            return False, "centre of mass"
        return True, "ok"


class BoxEnv(StagedEnv):
    """Box course with the staged climb, halting top walk and planned descent."""

    fingerprint_sources = (*StagedEnv.fingerprint_sources, "box_obstacle.py")

    def __init__(self, course, *, geometry_error=(0.0, 0.0, 0.0), stress=False, **kwargs):
        self.geometry_error = tuple(float(v) for v in geometry_error)
        self.stress = bool(stress)
        if course not in BOX_COURSES:
            raise ValueError(f"{course} is not a box course")
        # StagedEnv only accepts staged ascent courses; set up the same way here.
        from .locomotion import LocomotionEnv

        LocomotionEnv.__init__(self, course, **kwargs)
        if kwargs.get("episode_steps") is None:
            self.episode_steps = round(BOX_TIME_LIMIT_S / 0.02)
        self.body_motion = BoxTraverse(self)
        self.body_motion.reset()
        self.start_states = []
        self.start_fraction = 0.0
        self.started_from = None
        from .staged_ascent import STAGED_ACTIONS

        self.action_mask = np.zeros(self.action_space.shape, dtype=np.float32)
        self.action_mask[list(STAGED_ACTIONS)] = 1.0

    def _randomize_dynamics(self):
        super()._randomize_dynamics()
        if getattr(self, "stress", False) and self.domain_randomization:
            # Wider than the pilot randomisation: floor friction 0.6-1.2 of nominal
            # and body mass +-15 %. Applied with the other dynamics, before the
            # episode's initial state is set.
            import mujoco

            model, rng = self.model, self.np_random
            model.geom_friction[0, 0] = self._nominal_floor_friction[0] * rng.uniform(0.6, 1.2)
            base = model.body("base").id
            scale = rng.uniform(0.85, 1.15)
            model.body_mass[base] = self._nominal_body_mass[base] * scale
            model.body_inertia[base] = self._nominal_body_inertia[base] * scale
            mujoco.mj_setConst(model, self.data)

    def reset(self, *, seed=None, options=None):
        obs, info = super().reset(seed=seed, options=options)
        if self.stress:
            # Every leg servo up to 15 % weaker (after the supply scaling of reset).
            self.model.actuator_forcerange[self._legs] *= self.np_random.uniform(
                0.85, 1.0, size=(18, 1)
            )
        return obs, info
