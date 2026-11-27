"""Static checks for one single-leg step on a stair: reach, support, torque, clearance.

All functions take world-frame positions and a candidate body pose
``(x, y, z, pitch)`` with zero roll and yaw. They use the contract's link
lengths, mount angles, joint command limits and the episode's actual servo
torque limits (``model.actuator_forcerange``), so a derated battery in the
simulation is reflected in the check. Nothing here changes the robot model.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

import numpy as np

KNEE_PLAN_MIN = math.radians(-146.0)  # 3 degrees inside the -149 degree command stop
JOINT_MARGIN = math.radians(4.0)
# Static estimate vs. MuJoCo: 1.2-1.45 x the measured servo load in planted
# climbing poses (it ignores link weight carried below each joint). 1.15 of the
# estimate is about 0.80-0.95 of the episode's servo limit.
TORQUE_USE = 1.15
COM_INSET = 0.020
TIBIA_CLEARANCE = 0.030  # tibia CAD: 2.4 cm half thickness in the leg plane + 6 mm
FEMUR_CLEARANCE = 0.034
SOLE_RISER_GAP = 0.012
NEIGHBOUR_GAP = 0.075


def rotation(pitch: float) -> np.ndarray:
    """Body-to-world rotation with positive pitch lifting the nose."""
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])


@dataclass
class LegGeometry:
    hip: np.ndarray  # (6, 3) body frame
    mount: np.ndarray  # (6,)
    lengths: tuple[float, float, float]
    low: np.ndarray  # (18,)
    high: np.ndarray  # (18,)
    radius: float

    @classmethod
    def from_env(cls, env):
        return cls(
            env.hip.copy(),
            env.mount.copy(),
            tuple(env.lengths),
            env.joint_low.copy(),
            env.joint_high.copy(),
            env.config["foot_radius_m"],
        )

    def to_leg(self, leg, body_points):
        c, s = math.cos(self.mount[leg]), math.sin(self.mount[leg])
        p = body_points - self.hip[leg]
        return np.stack(
            (p[..., 0] * c + p[..., 1] * s, -p[..., 0] * s + p[..., 1] * c, p[..., 2]), -1
        )

    def from_leg(self, leg, leg_points):
        c, s = math.cos(self.mount[leg]), math.sin(self.mount[leg])
        p = leg_points
        return (
            np.stack((p[..., 0] * c - p[..., 1] * s, p[..., 0] * s + p[..., 1] * c, p[..., 2]), -1)
            + self.hip[leg]
        )

    def ik(self, leg, leg_points):
        """Unclipped joint angles, a reachability flag and the smallest joint margin."""
        length, a, b = self.lengths
        x, y, z = leg_points[..., 0], leg_points[..., 1], leg_points[..., 2]
        radius = np.hypot(x, y) - length
        cos_knee = (radius**2 + z**2 - a * a - b * b) / (2 * a * b)
        reach = np.abs(cos_knee) <= 0.995
        knee = -np.arccos(np.clip(cos_knee, -0.995, 0.995))
        shoulder = np.arctan2(z, radius) - np.arctan2(b * np.sin(knee), a + b * np.cos(knee))
        coxa = np.arctan2(y, x)
        q = np.stack((coxa, shoulder, knee), -1)
        lim = slice(3 * leg, 3 * leg + 3)
        low = self.low[lim].copy()
        low[2] = max(low[2], KNEE_PLAN_MIN)
        margin = np.minimum(q - low, self.high[lim] - q).min(axis=-1)
        return q, reach & (margin >= JOINT_MARGIN), margin

    def jacobian(self, q):
        length, a, b = self.lengths
        c, s, k = q[..., 0], q[..., 1], q[..., 2]
        r = length + a * np.cos(s) + b * np.cos(s + k)
        dr_s = -a * np.sin(s) - b * np.sin(s + k)
        dr_k = -b * np.sin(s + k)
        zero = np.zeros_like(c)
        return np.stack(
            (
                np.stack((-r * np.sin(c), np.cos(c) * dr_s, np.cos(c) * dr_k), -1),
                np.stack((r * np.cos(c), np.sin(c) * dr_s, np.sin(c) * dr_k), -1),
                np.stack((zero, a * np.cos(s) + b * np.cos(s + k), b * np.cos(s + k)), -1),
            ),
            -2,
        )


def world_to_leg(geom, leg, points, body, rot):
    return geom.to_leg(leg, (points - body) @ rot)


def leg_to_world(geom, leg, leg_points, body, rot):
    return geom.from_leg(leg, leg_points) @ rot.T + body


def vertical_loads(points_xy, com_xy, weight, compliance=None):
    """Non-negative vertical sole forces that hold the weight at ``com_xy``.

    With ``compliance`` (vertical sole compliance of each position-servoed
    leg) this is the minimum complementary-energy split of linear springs,
    i.e. the share a statically indeterminate stance actually takes; without
    it, the minimum-norm split.
    """
    active = list(range(len(points_xy)))
    c_all = np.ones(len(points_xy)) if compliance is None else np.asarray(compliance)
    for _ in range(len(points_xy)):
        if len(active) < 3:
            return None
        p = points_xy[active] - com_xy
        a = np.vstack((np.ones(len(active)), p[:, 0], p[:, 1]))
        w = 1.0 / np.maximum(c_all[active], 1e-9)
        try:
            f = w * (a.T @ np.linalg.solve((a * w) @ a.T, np.array([weight, 0.0, 0.0])))
        except np.linalg.LinAlgError:
            return None
        if f.min() >= -1e-9:
            out = np.zeros(len(points_xy))
            out[active] = f
            return out
        active.pop(int(np.argmin(f)))
    return None


def hull_margin(points_xy, target):
    """Signed distance of ``target`` inside the convex hull (negative outside)."""
    pts = sorted(set(map(tuple, np.round(points_xy, 6))))
    if len(pts) < 3:
        return -1.0

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

    hull = np.array(half(pts) + half(list(reversed(pts))))
    if len(hull) < 3:
        return -1.0
    edges = np.roll(hull, -1, axis=0) - hull
    normals = np.column_stack((-edges[:, 1], edges[:, 0]))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    return float(np.min(np.sum((np.asarray(target) - hull) * normals, axis=1)))


@dataclass
class StepCheck:
    feasible: bool
    reason: str
    torque_ratio: float = math.inf
    com_margin: float = -1.0
    reach_margin: float = -1.0
    worst_joint: str = ""
    loads: np.ndarray | None = None


JOINT_NAMES = ("coxa", "femur", "tibia")
LEGS = ("RF", "RM", "RR", "LF", "LM", "LR")


def _batch_leg_frames(geom, legs, points_world, body, rot):
    c, s = np.cos(geom.mount[legs]), np.sin(geom.mount[legs])
    p = (points_world - body) @ rot - geom.hip[legs]
    return np.stack((p[:, 0] * c + p[:, 1] * s, -p[:, 0] * s + p[:, 1] * c, p[:, 2]), -1), c, s


def _batch_world(geom, legs, leg_points, c, s, body, rot):
    """leg_points (n, k, 3) for legs (n,) to world."""
    x, y, z = leg_points[..., 0], leg_points[..., 1], leg_points[..., 2]
    cc, ss = c[:, None], s[:, None]
    bp = np.stack((x * cc - y * ss, x * ss + y * cc, z), -1) + geom.hip[legs][:, None, :]
    return bp @ rot.T + body


def _step_distance(points, steps):
    """Distance in the sagittal plane from points (..., 3) to each step solid."""
    worst = np.full(points.shape[:-1], np.inf)
    for edge, top in steps:
        x, z = points[..., 0], points[..., 2]
        d = np.hypot(np.maximum(edge - x, 0.0), np.maximum(z - top, 0.0))
        inside = (x >= edge) & (z <= top)
        d = np.where(inside, -np.minimum(x - edge, top - z), d)
        worst = np.minimum(worst, d)
    return worst


def stance_links(geom, legs, q, c, s, body, rot):
    """Femur, knee and sole positions in world for joint angles q (n, 3)."""
    length, a, b = geom.lengths
    co, sh, kn = q[:, 0], q[:, 1], q[:, 2]
    femur = np.stack((length * np.cos(co), length * np.sin(co), np.zeros_like(co)), -1)
    rk = length + a * np.cos(sh)
    knee = np.stack((rk * np.cos(co), rk * np.sin(co), a * np.sin(sh)), -1)
    rf = rk + b * np.cos(sh + kn)
    foot = np.stack((rf * np.cos(co), rf * np.sin(co), a * np.sin(sh) + b * np.sin(sh + kn)), -1)
    pts = _batch_world(geom, legs, np.stack((femur, knee, foot), 1), c, s, body, rot)
    return pts[:, 0], pts[:, 1], pts[:, 2]


def links_clearance(geom, legs, q, c, s, body, rot, steps):
    """Per-leg femur/tibia clearance to the step solids minus the link half thickness."""
    femur, knee, foot = stance_links(geom, legs, q, c, s, body, rot)
    direction = foot - knee
    norm = np.maximum(np.linalg.norm(direction, axis=1, keepdims=True), 1e-6)
    tibia_end = knee + direction * np.maximum(0.0, 1.0 - 0.04 / norm)
    t = np.linspace(0.0, 1.0, 9)[None, :, None]
    femur_pts = femur[:, None] + t * (knee - femur)[:, None]
    tibia_pts = knee[:, None] + t * (tibia_end - knee)[:, None]
    return np.minimum(
        _step_distance(femur_pts, steps).min(axis=1) - FEMUR_CLEARANCE,
        _step_distance(tibia_pts, steps).min(axis=1) - TIBIA_CLEARANCE,
    )


def check_stance(
    geom,
    torque_limit,
    feet,
    stance,
    body,
    pitch,
    com_body,
    weight,
    steps=(),
    *,
    torque_use=TORQUE_USE,
    com_inset=COM_INSET,
    required_margin=None,
):
    """Reach, centre of mass, joint torque and link/step clearance for the stance legs.

    ``required_margin`` (6,) lowers the joint margin for legs that already
    stand closer to a limit, so a pose is judged against where the robot is.
    """
    rot = rotation(pitch)
    legs = np.flatnonzero(stance)
    p, c, s = _batch_leg_frames(geom, legs, feet[legs], body, rot)
    length, a, b = geom.lengths
    radius = np.hypot(p[:, 0], p[:, 1]) - length
    cos_knee = (radius**2 + p[:, 2] ** 2 - a * a - b * b) / (2 * a * b)
    knee = -np.arccos(np.clip(cos_knee, -0.995, 0.995))
    shoulder = np.arctan2(p[:, 2], radius) - np.arctan2(b * np.sin(knee), a + b * np.cos(knee))
    q = np.stack((np.arctan2(p[:, 1], p[:, 0]), shoulder, knee), -1)
    low = geom.low.reshape(6, 3)[legs].copy()
    low[:, 2] = np.maximum(low[:, 2], KNEE_PLAN_MIN)
    margin = np.minimum(q - low, geom.high.reshape(6, 3)[legs] - q).min(axis=1)
    need = JOINT_MARGIN if required_margin is None else np.asarray(required_margin)[legs]
    ok = (np.abs(cos_knee) <= 0.995) & (margin >= need)
    if not ok.all():
        bad = legs[int(np.argmin(np.where(ok, np.inf, margin)))]
        return StepCheck(False, f"{LEGS[bad]} reach", reach_margin=float(margin.min()))
    reach_margin = float(margin.min())
    com = body + rot @ com_body
    stance_xy = feet[legs, :2]
    com_margin = hull_margin(stance_xy, com[:2])
    if com_margin < com_inset:
        return StepCheck(False, "centre of mass", com_margin=com_margin, reach_margin=reach_margin)
    jac = geom.jacobian(q)
    # World vertical row of each leg Jacobian: z_world = rot[2] . body, body = mount(leg).
    up_body = rot[2]
    up_leg = np.stack(
        (
            up_body[0] * c + up_body[1] * s,
            -up_body[0] * s + up_body[1] * c,
            np.full_like(c, up_body[2]),
        ),
        -1,
    )
    vertical_row = np.einsum("ni,nij->nj", up_leg, jac)
    loads = vertical_loads(stance_xy, com[:2], weight, (vertical_row**2).sum(axis=1))
    if loads is None:
        return StepCheck(False, "load split", com_margin=com_margin, reach_margin=reach_margin)
    force_leg = loads[:, None] * up_leg  # world (0, 0, f) in each leg frame
    tau = np.einsum("nij,ni->nj", jac, force_leg)
    ratio = np.abs(tau) / torque_limit.reshape(6, 3)[legs]
    k = np.unravel_index(int(np.argmax(ratio)), ratio.shape)
    worst = float(ratio[k])
    worst_name = f"{LEGS[legs[k[0]]]}_{JOINT_NAMES[k[1]]}"
    if steps:
        clearance = links_clearance(geom, legs, q, c, s, body, rot, steps)
        if clearance.min() < 0:
            bad = legs[int(np.argmin(clearance))]
            return StepCheck(False, f"{LEGS[bad]} link at riser", worst, com_margin, reach_margin)
    if worst > torque_use:
        return StepCheck(
            False, f"torque {worst_name}", worst, com_margin, reach_margin, worst_name, loads
        )
    return StepCheck(True, "ok", worst, com_margin, reach_margin, worst_name, loads)


def swing_path(lift_off, landing, crest_z, edge=None, radius=0.018, *, retreat=0.030, bulge=0.0):
    """Lift, travel and lower waypoints.

    Over a riser the sole first rises while it backs away from the face and
    swings outwards by ``bulge`` (keeping the knee away from its fold stop),
    then crosses the lip above ``crest_z`` and lowers onto the tread.
    """
    start = np.asarray(lift_off, dtype=float)
    end = np.asarray(landing, dtype=float)
    side = math.copysign(1.0, start[1])
    up = start.copy()
    on_lip = edge is not None and abs(start[0] - edge) < radius + 0.02 and start[2] > crest_z - 0.06
    if edge is not None and start[0] < edge <= end[0] and not on_lip:
        up[0] = min(start[0], edge - radius - retreat)
    up[1] += side * bulge
    up[2] = crest_z
    over = end.copy()
    over[1] += side * bulge * 0.5
    over[2] = crest_z
    points = [start, up, over, end]
    path = []
    for p, q in itertools.pairwise(points):
        for t in np.linspace(0, 1, 6, endpoint=False):
            path.append(p + t * (q - p))
    path.append(end)
    return np.array(path), np.array(points)


def check_swing(geom, leg, path, body, pitch, others, steps, *, gap=0.05):
    """Reach and knee margin along the whole swing, sole/riser and neighbour clearance."""
    rot = rotation(pitch)
    p = world_to_leg(geom, leg, path, body, rot)
    q, ok, margin = geom.ik(leg, p)
    if not ok.all():
        return StepCheck(False, f"{LEGS[leg]} swing reach", reach_margin=float(margin.min()))
    start = path[0]
    for point in path[1:]:
        if np.allclose(point[:2], start[:2]) and point[2] >= start[2]:
            continue  # rising straight up from where the sole already is
        for edge, top in steps:
            near_lip = (
                edge - geom.radius - SOLE_RISER_GAP < point[0] < edge + geom.radius + SOLE_RISER_GAP
            )
            below_lip = geom.radius + 0.004 < point[2] < top + geom.radius + SOLE_RISER_GAP
            if near_lip and below_lip:
                return StepCheck(False, "sole at riser")
    if steps:
        legs = np.full(len(path) - 1, leg)
        c = np.cos(geom.mount[legs])
        s_ = np.sin(geom.mount[legs])
        clearance = links_clearance(geom, legs, q[:-1], c, s_, body, rot, steps)
        if clearance.min() < 0:
            return StepCheck(False, f"{LEGS[leg]} swing link at riser")
    if len(others):
        d = np.linalg.norm(path[:, None, :2] - others[None, :, :2], axis=2).min()
        if d < gap:
            return StepCheck(False, "neighbour")
    return StepCheck(True, "ok", reach_margin=float(margin.min()))
