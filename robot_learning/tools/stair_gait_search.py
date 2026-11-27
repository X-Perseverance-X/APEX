"""Simulation-only feasibility study. Search for a periodic, statically checked foot sequence on a uniform straight flight.

State: for each leg pair (front, middle, rear) a tread level and a position on the
tread (fraction of the usable band). Left and right legs of a pair move one after
the other. A transition lifts one pair up one tread (or slides it on its tread);
it is accepted when, for each of the two single-leg lifts, some body pose holds
the five standing soles (reach, joint margins, centre of mass, torque, link and
chassis clearance) and the whole swing path is reachable and clear of the riser.
A gait exists if a sequence returns to the same relative state one tread higher.

    python robot_learning/tools/stair_gait_search.py --rise 0.17 --tread 0.27 --output out.json

The output lists, for each pair, how many level-up moves were tried and accepted,
and the cycle as (level, position index) per pair; positions are 10, 30, 50, 70
and 90 % of the usable tread depth.
"""

import argparse
import collections
import itertools
import json
import time
from pathlib import Path

import numpy as np

from apex_rl.ascent_planner import NEIGHBOUR_GAP, check_stance, check_swing, rotation, swing_path
from apex_rl.staged_ascent import CHASSIS_BOTTOM, StagedEnv

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--rise", type=float, default=0.20)
parser.add_argument("--tread", type=float, default=0.27)
parser.add_argument(
    "--torque-scale", type=float, default=1.0, help="what-if only; the model limits stay"
)
parser.add_argument(
    "--tibia", type=float, help="what-if knee-to-sole length in metres (model: 0.16841)"
)
parser.add_argument("--rear-y", type=float, default=0.205, help="rear sole |y| in metres")
parser.add_argument(
    "--descent",
    action="store_true",
    help="walking down facing downhill: the same search for the front/rear mirrored robot",
)
parser.add_argument("--budget-s", type=float, default=2400)
parser.add_argument("--output", type=Path, required=True)
ARGS = parser.parse_args()
RISE, TREAD = ARGS.rise, ARGS.tread
NOSE = 0.02
env = StagedEnv("hybrid-staged-up-20")
env.reset(seed=1)
c = env.body_motion
SCALE = ARGS.torque_scale
geom, torque, com_body, weight = c.geom, c._torque_limit() * SCALE, c._com_body(), c._weight()
if ARGS.descent:
    # Walking down a flight facing downhill is, step for step, the time reverse of
    # walking up it backwards; mirroring x turns that into walking up forwards with
    # front and rear swapped. Reach, torque and swing checks are unchanged under
    # both (legs and joint limits are front/rear symmetric); the chassis outline,
    # the centre of mass and the per-leg torque limits are mirrored.
    CHASSIS_BOTTOM = CHASSIS_BOTTOM * np.array([-1.0, 1.0, 1.0])
    com_body = com_body * np.array([-1.0, 1.0, 1.0])
    torque = torque.reshape(6, 3)[[2, 1, 0, 5, 4, 3]].reshape(-1)
if ARGS.tibia:
    geom.lengths = (geom.lengths[0], geom.lengths[1], ARGS.tibia)
R = c.radius
NL = 9
EDGES = [i * TREAD for i in range(NL + 1)]  # level i spans EDGES[i]..EDGES[i+1]
STEPS = [(EDGES[i], RISE * i) for i in range(1, NL + 1)]
FRACS = (0.1, 0.3, 0.5, 0.7, 0.9)
Y = (0.205, 0.245, ARGS.rear_y)  # front, middle, rear |y|


def foot(level, frac, side, pair):
    lo, hi = EDGES[level] + R + 0.007, EDGES[level + 1] + NOSE - R - 0.015
    return np.array([lo + frac * (hi - lo), side * Y[pair], RISE * level + R - 0.0015])


def feet_of(state, moved=None):
    """state = ((level, fi) for front, middle, rear); moved = (pair, (level, fi)) on the right side only."""
    out = np.zeros((6, 3))
    for pair, (lv, fi) in enumerate(state):
        out[pair] = foot(lv, FRACS[fi], -1, pair)  # right legs 0..2
        out[3 + pair] = foot(lv, FRACS[fi], 1, pair)  # left legs 3..5
    if moved is not None:
        pair, (lv, fi) = moved
        out[pair] = foot(lv, FRACS[fi], -1, pair)
    return out


def chassis_clear(body, pitch):
    world = CHASSIS_BOTTOM @ rotation(pitch).T + body
    for e, t in STEPS:
        if np.any((world[:, 0] >= e - 0.004) & (world[:, 2] < t + 0.010)):
            return False
    return True


POSE_GRID = list(
    itertools.product(
        np.arange(-0.12, 0.121, 0.03), np.arange(0.06, 0.30, 0.03), np.radians(np.arange(0, 45, 5))
    )
)


def holds(feet, stance, swing=None):
    centre = feet[stance].mean(axis=0)
    lateral = (0.0,) if swing is None else (0.0, 0.03, 0.06)
    side = 0.0 if swing is None else (-1.0 if swing[0] < 3 else 1.0)
    for (bx, h, p), dy in itertools.product(POSE_GRID, lateral):
        body = np.array([centre[0] + bx, -side * dy, centre[2] + h])
        if not chassis_clear(body, p):
            continue
        if not check_stance(geom, torque, feet, stance, body, p, com_body, weight, STEPS).feasible:
            continue
        if swing is None:
            return (body, p)
        leg, landing = swing
        crossing = feet[leg, 2] < landing[2] - 0.05
        edge = EDGES[round((landing[2] - R + 0.0015) / RISE)] if crossing else None
        nb = feet[[n for n in (leg - 1, leg + 1) if 0 <= n < 6 and n // 3 == leg // 3]]
        for retreat, bulge, clr in itertools.product((0.03, 0.06), (0.0, 0.03), (0.026, 0.04)):
            if not crossing and (retreat != 0.03 or bulge or clr != 0.026):
                continue
            crest = landing[2] + clr if crossing else max(landing[2], feet[leg, 2]) + 0.03
            path, _ = swing_path(feet[leg], landing, crest, edge, R, retreat=retreat, bulge=bulge)
            if check_swing(geom, leg, path, body, p, nb, STEPS).feasible:
                return (body, p)
    return None


def holds_two_pose(feet, stance, leg, landing):
    """Lift at one pose, finish the swing from a second pose (both, and halfway, hold)."""
    centre = feet[stance].mean(axis=0)
    side = -1.0 if leg < 3 else 1.0
    poses = []
    for (bx, h, p), dy in itertools.product(POSE_GRID, (0.0, 0.03, 0.06)):
        body = np.array([centre[0] + bx, -side * dy, centre[2] + h])
        if (
            chassis_clear(body, p)
            and check_stance(geom, torque, feet, stance, body, p, com_body, weight, STEPS).feasible
        ):
            poses.append((body, p))
    if not poses:
        return None
    edge = EDGES[round((landing[2] - R + 0.0015) / RISE)]
    nb = feet[[n for n in (leg - 1, leg + 1) if 0 <= n < 6 and n // 3 == leg // 3]]
    for retreat, bulge, clr in itertools.product((0.03, 0.06), (0.0, 0.03, 0.06), (0.026, 0.04)):
        path, _ = swing_path(
            feet[leg], landing, landing[2] + clr, edge, R, retreat=retreat, bulge=bulge
        )
        for split in (3, 5, 7, 9):
            first = [
                q
                for q in poses
                if check_swing(geom, leg, path[: split + 1], q[0], q[1], nb, STEPS).feasible
            ]
            if not first:
                continue
            rest = [
                q
                for q in poses
                if check_swing(geom, leg, path[split:], q[0], q[1], nb, STEPS).feasible
            ]
            for a in first[:25]:
                for b in rest[:25]:
                    mid, mp = 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])
                    if (
                        chassis_clear(mid, mp)
                        and check_stance(
                            geom, torque, feet, stance, mid, mp, com_body, weight, STEPS
                        ).feasible
                    ):
                        return a, b
    return None


def neighbour_ok(feet, leg, landing):
    same = [i for i in range(6) if i != leg and abs(feet[i, 2] - landing[2]) < 0.05]
    return not same or np.linalg.norm(feet[same, :2] - landing[:2], axis=1).min() >= NEIGHBOUR_GAP


stand_cache, move_cache = {}, {}


def normal(state):
    base = min(lv for lv, _ in state)
    return tuple((lv - base, fi) for lv, fi in state), base


def stand_ok(state):
    key, _ = normal(state)
    if key not in stand_cache:
        st = tuple((lv + 2, fi) for lv, fi in key)
        stand_cache[key] = holds(feet_of(st), np.ones(6, bool)) is not None
    return stand_cache[key]


def move_ok(state, pair, target):
    key, base = normal(state)
    tkey = (target[0] - base, target[1])
    k = (key, pair, tkey)
    if k not in move_cache:
        st = tuple((lv + 2, fi) for lv, fi in key)
        tg = (tkey[0] + 2, tkey[1])
        feet = feet_of(st)
        ok = True
        # right leg first, then the left leg with the right one already moved
        for leg, before in ((pair, feet), (3 + pair, feet_of(st, moved=(pair, tg)))):
            side = -1 if leg < 3 else 1
            landing = foot(tg[0], FRACS[tg[1]], side, pair)
            stance = np.ones(6, bool)
            stance[leg] = False
            if not neighbour_ok(before, leg, landing):
                ok = False
                break
            if holds(before, stance, (leg, landing)) is None and (
                landing[2] <= before[leg, 2] + 0.05
                or holds_two_pose(before, stance, leg, landing) is None
            ):
                ok = False
                break
        move_cache[k] = ok
    return move_cache[k]


def successors(state):
    for pair in range(3):
        lv, fi = state[pair]
        for nlv in (lv, lv + 1):
            for nfi in range(len(FRACS)):
                if (nlv, nfi) == (lv, fi):
                    continue
                if nlv == lv and nfi < fi:
                    continue  # no backwards slides
                new = list(state)
                new[pair] = (nlv, nfi)
                new = tuple(new)
                # keep the pairs ordered front-to-back and within reach of each other
                xs = [EDGES[l] + FRACS[f] * TREAD for l, f in new]
                if not (xs[0] > xs[1] > xs[2]) or xs[0] - xs[2] > 0.75:
                    continue
                if stand_ok(new) and move_ok(state, pair, (nlv, nfi)):
                    yield new


t0 = time.time()
starts = [
    s
    for s in itertools.product(*[[(lv, fi) for lv in range(4) for fi in range(len(FRACS))]] * 3)
    if min(lv for lv, _ in s) == 0
    and EDGES[s[0][0]] + FRACS[s[0][1]] * TREAD
    > EDGES[s[1][0]] + FRACS[s[1][1]] * TREAD
    > EDGES[s[2][0]] + FRACS[s[2][1]] * TREAD
]
good = [s for s in starts if stand_ok(s)]
print(
    f"tibia {geom.lengths[2]:.3f} torque x{SCALE:g} rise {RISE} tread {TREAD}: {len(good)}/{len(starts)} six-foot states hold ({time.time() - t0:.0f}s)",
    flush=True,
)
# Breadth-first search for a cycle that climbs every pair by one tread.
found = None
for start in good:
    frontier = collections.deque([(start, (start,))])
    seen = {start}
    while frontier and found is None:
        state, path = frontier.popleft()
        for nxt in successors(state):
            if all(n[0] == s[0] + 1 and n[1] == s[1] for n, s in zip(nxt, start)):
                found = path + (nxt,)
                break
            if nxt not in seen and max(lv for lv, _ in nxt) - start[0][0] <= 2 and len(path) < 10:
                seen.add(nxt)
                frontier.append((nxt, path + (nxt,)))
    if found or time.time() - t0 > ARGS.budget_s:
        break
print(f"checked {len(move_cache)} transitions in {time.time() - t0:.0f}s")
print("PERIODIC GAIT:", json.dumps(found) if found else "none found")
ups = collections.Counter()
upsok = collections.Counter()
for (key, pair, tkey), ok in move_cache.items():
    if tkey[0] > key[pair][0]:
        ups[pair] += 1
        upsok[pair] += ok
print(
    "level-up moves tried/accepted per pair (front, middle, rear):",
    [(ups[p], upsok[p]) for p in range(3)],
)
report = {
    "direction": "down" if ARGS.descent else "up",
    "torque_scale": SCALE,
    "tibia_m": geom.lengths[2],
    "rear_y_m": ARGS.rear_y,
    "ups": [(ups[p], upsok[p]) for p in range(3)],
    "rise": RISE,
    "tread": TREAD,
    "standing_states": len(good),
    "transitions": len(move_cache),
    "accepted": sum(move_cache.values()),
    "cycle": found,
}
ARGS.output.write_text(json.dumps(report, indent=1), encoding="utf-8")
