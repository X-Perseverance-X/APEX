"""Phase-by-phase measurement of a single-step ascent.

The trace labels every 20 ms control step with the climb phase that the foot
positions show, and records the body height target next to the measured
height, every sole position and load, and every joint's torque relative to its
limit. Phases, in the order a successful climb passes through them:

``approach``      tripod walk towards the riser
``front``         until both front soles stand on the upper tread
``transfer``      fronts up, the body is carried forward and up, no middle swing
``middle``        a middle foot is swinging onto the upper tread
``rear``          fronts and middles up, rears still below
``top``           all six soles on the upper tread
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from .contract import LEG_ORDER

PHASES = ("approach", "front", "transfer", "middle", "rear", "top")
SATURATION = 0.98
JOINTS = ("coxa", "femur", "tibia")


def climb_phase(env, feet, contacts) -> str:
    controller = env.body_motion
    if env.gait != "wave":
        return "approach"
    upper = controller.upper_contacts(feet, contacts) | controller.upper_contacts(
        controller.anchors, np.ones(6, dtype=bool)
    )
    if not (upper[0] and upper[3]):
        return "front"
    if not (upper[1] and upper[4]):
        return "middle" if controller.leg in (1, 4) else "transfer"
    if not (upper[2] and upper[5]):
        return "rear"
    return "top"


class SoleLoads:
    """Physics-rate sole normal force and joint load, averaged per control step."""

    def __init__(self, env):
        self.env = env
        model = env.model
        self.feet = np.array(
            [model.geom(f"{leg.lower()}_foot_geom").id for leg in LEG_ORDER], dtype=int
        )
        self.weight = float(model.body_mass.sum() * abs(model.opt.gravity[2]))
        self.wrench = np.zeros(6)
        self.leg_of = np.full(model.ngeom, -1)
        from .leg_contacts import leg_geoms

        for i, leg in enumerate(LEG_ORDER):
            for name in leg_geoms(leg):
                self.leg_of[model.geom(name).id] = i
        self.terrain = np.zeros(model.ngeom, dtype=bool)
        self.terrain[env._terrain_ids] = True
        self.clear()

    def clear(self):
        self.n = 0
        self.riser = np.zeros(6)
        self.foot = np.zeros(6)
        self.joint = np.zeros(18)
        self.saturated = np.zeros(18)

    def observe(self, contacts):
        env = self.env
        model, data = env.model, env.data
        self.n += 1
        a, b = contacts.geom.T
        for leg, geom in enumerate(self.feet):
            for j in np.flatnonzero((a == geom) | (b == geom)):
                mujoco.mj_contactForce(model, data, int(j), self.wrench)
                self.foot[leg] += abs(float(self.wrench[0])) / self.weight
        normal_z = np.abs(contacts.frame.reshape(-1, 3, 3)[:, 0, 2]) if len(a) else np.zeros(0)
        side = (normal_z < 0.60) & (contacts.dist <= 0) if len(a) else np.zeros(0, dtype=bool)
        touching = np.zeros(6, dtype=bool)
        for j in np.flatnonzero(side):
            leg = (
                self.leg_of[a[j]]
                if self.terrain[b[j]]
                else (self.leg_of[b[j]] if self.terrain[a[j]] else -1)
            )
            if leg >= 0:
                touching[leg] = True
        self.riser += touching
        load = np.abs(data.actuator_force[env._legs]) / model.actuator_forcerange[env._legs, 1]
        self.joint += load
        self.saturated += load >= SATURATION

    def sample(self):
        n = max(1, self.n)
        out = self.foot / n, self.joint / n, self.saturated / n
        self.last_riser = self.riser.copy()
        self.clear()
        return out


def _pitch(env):
    r = env.data.xmat[env.body_motion.base_id].reshape(3, 3)
    return math.atan2(r[2, 0], math.hypot(r[2, 1], r[2, 2]))


def trace_episode(env, policy=None, seed=0, *, max_steps=None):
    """Run one evaluation episode from the course start and record every control step."""
    loads = SoleLoads(env)
    original = env._observe_link_contacts

    def observe(contacts):
        original(contacts)
        loads.observe(contacts)

    env._observe_link_contacts = observe
    try:
        obs, _ = env.reset(seed=seed)
        loads.weight = float(env.model.body_mass.sum() * abs(env.model.opt.gravity[2]))
        rows = []
        info = {}
        while True:
            action = (
                np.zeros(env.action_space.shape, dtype=np.float32)
                if policy is None
                else policy.predict(obs, deterministic=True)[0]
            )
            obs, _, done, truncated, info = env.step(action)
            feet = env.data.xpos[env._foot_ids].copy()
            contacts = env.body_motion.contacts()
            foot_load, joint_load, saturated = loads.sample()
            c = env.body_motion
            rows.append(
                {
                    "t": round(env.steps * 0.02, 2),
                    "phase": climb_phase(env, feet, contacts),
                    "leg": c.leg,
                    "x": float(env.data.qpos[0]),
                    "z": float(env.data.qpos[2]),
                    "target_x": float(c.body[0]),
                    "target_z": float(c.body[2]),
                    "pitch": _pitch(env),
                    "target_pitch": float(c.pitch),
                    "feet": np.round(feet, 4).tolist(),
                    "contacts": contacts.astype(int).tolist(),
                    "foot_load": np.round(foot_load, 4).tolist(),
                    "joint_load": np.round(joint_load, 4).tolist(),
                    "saturated": np.round(saturated, 3).tolist(),
                    "chassis_load": float(env.brace.support_load) if env.brace else 0.0,
                    "support_count": int(c.support_count),
                    "riser_substeps": loads.last_riser.astype(int).tolist(),
                }
            )
            if done or truncated or (max_steps and env.steps >= max_steps):
                break
        return rows, info
    finally:
        env._observe_link_contacts = original


def phase_summary(rows, stand_height):
    """Per-phase height error, sole loads and joint loads."""
    summary = {}
    for phase in PHASES:
        part = [r for r in rows if r["phase"] == phase]
        if not part:
            continue
        z = np.array([r["z"] for r in part])
        target = np.array([r["target_z"] for r in part])
        foot = np.array([r["foot_load"] for r in part])
        joint = np.array([r["joint_load"] for r in part])
        sat = np.array([r["saturated"] for r in part])
        loaded = foot > 0.05
        # Joint saturation while that joint's sole carries load.
        loaded_sat = {}
        for leg, name in enumerate(LEG_ORDER):
            mask = loaded[:, leg]
            for k, joint_name in enumerate(JOINTS):
                if mask.any():
                    loaded_sat[f"{name}_{joint_name}"] = round(
                        float(sat[mask, 3 * leg + k].mean()), 3
                    )
        deficit = target - z
        summary[phase] = {
            "duration_s": round(0.02 * len(part), 2),
            "mean_target_z_m": round(float(target.mean()), 4),
            "mean_z_m": round(float(z.mean()), 4),
            "mean_height_deficit_m": round(float(deficit.mean()), 4),
            "max_height_deficit_m": round(float(deficit.max()), 4),
            "mean_target_rise_above_stand_m": round(float(target.mean() - stand_height), 4),
            "mean_foot_load_weight": dict(
                zip(LEG_ORDER, np.round(foot.mean(axis=0), 3).tolist(), strict=True)
            ),
            "mean_chassis_load_weight": round(float(np.mean([r["chassis_load"] for r in part])), 3),
            "mean_joint_load": {
                f"{name}_{j}": round(float(joint[:, 3 * i + k].mean()), 3)
                for i, name in enumerate(LEG_ORDER)
                for k, j in enumerate(JOINTS)
            },
            "saturated_while_loaded": loaded_sat,
            "x_start_m": round(part[0]["x"], 3),
            "x_end_m": round(part[-1]["x"], 3),
            "feet_start_m": dict(zip(LEG_ORDER, part[0]["feet"], strict=True)),
            "feet_end_m": dict(zip(LEG_ORDER, part[-1]["feet"], strict=True)),
            "riser_contact_s": round(
                0.002 * float(np.sum([r.get("riser_substeps", [0] * 6) for r in part])), 3
            ),
        }
    return summary


def deficit_attribution(rows, phase="front", threshold=0.02):
    """Which soles carry the load, and which joints are at their limit, while short of target."""
    part = [r for r in rows if r["phase"] == phase and r["target_z"] - r["z"] > threshold]
    if not part:
        return None
    foot = np.array([r["foot_load"] for r in part])
    sat = np.array([r["saturated"] for r in part])
    joint = np.array([r["joint_load"] for r in part])
    swing = [r["leg"] for r in part]
    return {
        "samples": len(part),
        "duration_s": round(0.02 * len(part), 2),
        "mean_deficit_m": round(float(np.mean([r["target_z"] - r["z"] for r in part])), 4),
        "mean_foot_load_weight": dict(
            zip(LEG_ORDER, np.round(foot.mean(axis=0), 3).tolist(), strict=True)
        ),
        "loaded_fraction": dict(
            zip(LEG_ORDER, np.round((foot > 0.05).mean(axis=0), 3).tolist(), strict=True)
        ),
        "saturated_fraction": {
            f"{name}_{j}": round(float(sat[:, 3 * i + k].mean()), 3)
            for i, name in enumerate(LEG_ORDER)
            for k, j in enumerate(JOINTS)
            if sat[:, 3 * i + k].mean() >= 0.05
        },
        "mean_joint_load": {
            f"{name}_{j}": round(float(joint[:, 3 * i + k].mean()), 3)
            for i, name in enumerate(LEG_ORDER)
            for k, j in enumerate(JOINTS)
            if joint[:, 3 * i + k].mean() >= 0.5
        },
        "swing_leg_share": {
            (LEG_ORDER[leg] if leg is not None else "none"): round(swing.count(leg) / len(swing), 3)
            for leg in sorted(set(swing), key=lambda v: -1 if v is None else v)
        },
        "mean_chassis_load_weight": round(float(np.mean([r["chassis_load"] for r in part])), 3),
    }
