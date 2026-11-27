"""Flat-ground omnidirectional gaits and the measurements used to compare them.

Two controllers share the tripod timing, inverse kinematics, joint limits,
target-rate limits and servo torque limits of ``LocomotionEnv``:

``reference``
    The existing flat tripod profile (period 1.24 s, duty 0.64) with its
    stride vector rotated to the commanded body velocity. A stop command
    removes the stride immediately, as the original reference does.
``phase``
    Feet carry their own body-frame offset. Stance feet move with the
    commanded body velocity, swing feet land half a stride ahead, and the
    command is acceleration limited. A stop lets each swing foot land at its
    neutral foothold and holds the gait clock once all six feet are down.

Nothing here talks to hardware. Measurements are simulation-only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np

from .body_motion import ease
from .env import _rotation_from_quaternion
from .locomotion import TRIPOD, LocomotionEnv

PERIOD = 1.24
DUTY = 0.64
LIFT = 0.045
SATURATION = 0.98
# A touchdown is hard above 0.20 m/s closing speed or 0.9 x robot weight
# within 40 ms on one sole (a planted tripod shares about 0.33 each).
HARD_SPEED = 0.20
HARD_LOAD = 0.90
MIDDLE_INSET_M = 0.020

COMMANDS = {
    "forward": (0.12, 0.0, 0.0),
    "backward": (-0.10, 0.0, 0.0),
    "left": (0.0, 0.08, 0.0),
    "right": (0.0, -0.08, 0.0),
    "turn": (0.0, 0.0, 0.35),
    "stop": (0.12, 0.0, 0.0),
}


def _neutral_xy(env):
    neutral = env._neutral_foot
    c, s = np.cos(env.mount), np.sin(env.mount)
    return env.hip[:, :2] + np.column_stack(
        (neutral[:, 0] * c - neutral[:, 1] * s, neutral[:, 0] * s + neutral[:, 1] * c)
    )


def _to_leg(env, body_xy):
    c, s = np.cos(env.mount), np.sin(env.mount)
    return np.column_stack(
        (body_xy[:, 0] * c + body_xy[:, 1] * s, -body_xy[:, 0] * s + body_xy[:, 1] * c)
    )


def foot_velocity(neutral_xy, command):
    """Velocity of each planted foot relative to a body moving with ``command``."""
    vx, vy, wz = command
    return np.column_stack((vx - wz * neutral_xy[:, 1], vy + wz * neutral_xy[:, 0]))


class ReferenceTripod:
    name = "reference"

    def __init__(self, env):
        self.env = env
        self.neutral_xy = _neutral_xy(env)
        self.command = np.zeros(3)
        self.steps = 0

    def reset(self):
        self.steps = 0

    def targets(self, command):
        env = self.env
        self.command = np.asarray(command, dtype=float)
        phase = (self.steps * 0.02 / PERIOD + TRIPOD) % 1.0
        swing = phase >= DUTY
        u = np.clip((phase - DUTY) / (1 - DUTY), 0, 1)
        ramp = min(1.0, self.steps * 0.02 / 0.75)
        stride = foot_velocity(self.neutral_xy, self.command) * PERIOD * DUTY
        travel = np.clip((u - 0.22) / 0.58, 0, 1)
        profile = np.where(swing, -0.5 + (3 * travel**2 - 2 * travel**3), 0.5 - phase / DUTY)
        offset = ramp * profile[:, None] * stride
        foot = env._neutral_foot.copy()
        foot[:, :2] += _to_leg(env, offset)
        foot[:, 2] += ramp * LIFT * np.sin(np.pi * u) ** 2
        self.steps += 1
        return env._solve_ik(foot)

    @property
    def standing(self):
        return False


class PhaseTripod:
    """Velocity-integrated tripod with an explicit walk-to-stand transition."""

    name = "phase"
    accel = np.array([0.20, 0.20, 0.70])  # m/s^2 and rad/s^2

    def __init__(self, env, *, middle_inset_m: float = MIDDLE_INSET_M):
        self.env = env
        self.neutral_xy = _neutral_xy(env)
        # A middle leg carries its side of the tripod alone. Pulling that
        # sole 2 cm towards the body shortens the femur's load lever.
        self.neutral = env._neutral_foot.copy()
        self.neutral[[1, 4], 0] -= middle_inset_m

    def reset(self):
        self.clock = 0.0
        self.command = np.zeros(3)
        self.offset = np.zeros((6, 2))
        self.lift_off = np.zeros((6, 2))
        self.was_swing = np.zeros(6, dtype=bool)
        self.halted = True
        self.stop_requested = False

    @property
    def standing(self):
        return self.halted

    def targets(self, command):
        return self.env._solve_ik(self.feet(command))

    def start_from(self, body_xy_feet):
        """Continue from planted soles at these body-frame XY positions."""
        self.reset()
        self.offset = np.asarray(body_xy_feet, dtype=float) - self.neutral_xy
        self.offset[[1, 4], 1] -= np.sign(self.neutral_xy[[1, 4], 1]) * (
            self.neutral[[1, 4], 0] - self.env._neutral_foot[[1, 4], 0]
        )

    def feet(self, command):
        """Leg-frame sole targets for one 20 ms control step."""
        env = self.env
        goal = np.asarray(command, dtype=float)
        dt = 0.02
        self.command += np.clip(goal - self.command, -self.accel * dt, self.accel * dt)
        moving = bool(np.any(np.abs(goal) > 1e-6))
        if moving:
            self.halted = False
        phase = (self.clock / PERIOD + TRIPOD) % 1.0
        swing = phase >= DUTY
        if not moving and not self.halted:
            # Finish the stride: stop the clock only at the moment when the
            # swing tripod has landed and every offset is back near neutral.
            settled = np.linalg.norm(self.offset, axis=1).max() < 0.004
            if not swing.any() and settled and np.abs(self.command).max() < 1e-3:
                self.halted = True
        if self.halted:
            self.offset *= 0.9
            foot = self.neutral.copy()
            foot[:, :2] += _to_leg(env, self.offset)
            return foot
        velocity = foot_velocity(self.neutral_xy, self.command)
        stride = velocity * PERIOD * DUTY
        u = np.clip((phase - DUTY) / (1 - DUTY), 0, 1)
        lifted = swing & ~self.was_swing
        self.lift_off[lifted] = self.offset[lifted]
        stance = ~swing
        self.offset[stance] -= velocity[stance] * dt
        travel = np.array([ease((v - 0.18) / 0.62) for v in u])
        landing = 0.5 * stride if moving else np.zeros_like(stride)
        self.offset[swing] = self.lift_off[swing] + travel[swing, None] * (
            landing[swing] - self.lift_off[swing]
        )
        self.was_swing = swing
        foot = self.neutral.copy()
        foot[:, :2] += _to_leg(env, self.offset)
        # Faster lift, slower final descent: the sole arrives with low speed.
        rise = np.array([ease(v / 0.38) for v in u])
        fall = np.array([ease((v - 0.55) / 0.45) for v in u])
        foot[:, 2] += LIFT * np.where(swing, rise * (1 - fall), 0.0)
        self.clock += dt
        return foot


CONTROLLERS = {"reference": ReferenceTripod, "phase": PhaseTripod}


@dataclass
class FlatMonitor:
    """Physics-rate sole, body and servo measurements."""

    env: LocomotionEnv

    def __post_init__(self):
        env = self.env
        self.foot_geoms = np.array(
            [env.model.geom(f"{p}_foot_geom").id for p in ("rf", "rm", "rr", "lf", "lm", "lr")]
        )
        self.floor = env.model.geom("floor").id
        self.weight = float(env.model.body_mass.sum() * abs(env.model.opt.gravity[2]))
        self.wrench = np.zeros(6)
        self.dt = env.model.opt.timestep
        self.substeps = 0
        self.saturated_any = 0
        self.saturated_joint = 0
        self.saturated_by_joint = np.zeros(18)
        self.sole_slide_m = 0.0
        self.loaded_slide_m = 0.0
        self.recontacts = 0
        self.lifted_for = np.zeros(6)
        self.down = np.zeros(6, dtype=bool)
        self.since_touch = np.full(6, 1e9)
        self.window_peak = np.zeros(6)
        self.touchdowns = []  # [speed m/s, peak load / weight]
        self.pending = {}
        self.prev_feet = env.data.xpos[env._foot_ids].copy()

    def foot_forces(self):
        """Normal force on each sole and the horizontal sliding speed at its contact."""
        model, data = self.env.model, self.env.data
        force = np.zeros(6)
        sliding = np.zeros(6)
        velocity = np.zeros(6)
        for j in range(data.ncon):
            c = data.contact[j]
            a, b = int(c.geom1), int(c.geom2)
            for leg, geom in enumerate(self.foot_geoms):
                if geom in (a, b):
                    mujoco.mj_contactForce(model, data, j, self.wrench)
                    force[leg] += abs(float(self.wrench[0]))
                    # Material-point velocity of the sole at the contact point:
                    # rolling moves the sphere centre but not this point.
                    mujoco.mj_objectVelocity(
                        model, data, mujoco.mjtObj.mjOBJ_GEOM, geom, velocity, 0
                    )
                    point = velocity[3:] + np.cross(velocity[:3], c.pos - data.geom_xpos[geom])
                    sliding[leg] = max(sliding[leg], float(np.hypot(point[0], point[1])))
        return force, sliding

    def observe(self, _contacts):
        env = self.env
        data = env.data
        self.substeps += 1
        load = np.abs(data.actuator_force[env._legs]) / env.model.actuator_forcerange[env._legs, 1]
        saturated = load >= SATURATION
        self.saturated_any += int(saturated.any())
        self.saturated_joint += int(saturated.sum())
        self.saturated_by_joint += saturated
        force, sliding = self.foot_forces()
        feet = data.xpos[env._foot_ids]
        speed = (feet[:, 2] - self.prev_feet[:, 2]) / self.dt
        self.prev_feet = feet.copy()
        down = force > 0.02 * self.weight
        self.sole_slide_m += float(sliding[down].sum()) * self.dt
        self.loaded_slide_m += float(sliding[force > 0.10 * self.weight].sum()) * self.dt
        touch = down & ~self.down
        # A sole that comes back within 0.10 s of leaving the ground bounced.
        self.recontacts += int(np.sum(touch & (self.lifted_for < 0.10)))
        self.lifted_for = np.where(down, 0.0, self.lifted_for + self.dt)
        for leg in np.flatnonzero(touch):
            self.touchdowns.append([float(-speed[leg]), 0.0])
            self.pending[int(leg)] = len(self.touchdowns) - 1
            self.since_touch[leg] = 0
        for leg, index in list(self.pending.items()):
            record = self.touchdowns[index]
            record[1] = max(record[1], float(force[leg]) / self.weight)
            if self.since_touch[leg] >= 0.04 / self.dt:
                del self.pending[leg]
        self.since_touch += 1
        self.down = down


def _attitude(q):
    r = _rotation_from_quaternion(q)
    roll = math.atan2(r[2, 1], r[2, 2])
    pitch = math.atan2(-r[2, 0], math.hypot(r[2, 1], r[2, 2]))
    yaw = math.atan2(r[1, 0], r[0, 0])
    return roll, pitch, yaw


def run_flat_trial(
    controller: str, mode: str, seed: int, *, seconds: float = 8.0, stop_at=5.0, options=None
):
    """Run one flat-ground command and return the measurements."""
    env = LocomotionEnv("flat", randomize=True)
    try:
        env.reset(seed=seed)
        gait = CONTROLLERS[controller](env, **(options or {}))
        gait.reset()
        monitor = FlatMonitor(env)
        command = np.array(COMMANDS[mode], dtype=float)
        steps = int(seconds / 0.02)
        log = []
        slip = 0.0
        slip_stop = 0.0
        prev_feet = env.data.xpos[env._foot_ids].copy()
        prev_down = monitor.down.copy()
        settle_time = None
        start_yaw = _attitude(env.data.qpos[3:7])[2]
        for k in range(steps):
            t = k * 0.02
            requested = command if mode != "stop" or t < stop_at else np.zeros(3)
            targets = gait.targets(requested)
            env._apply_joint_targets(targets, observe_contacts=monitor.observe)
            feet = env.data.xpos[env._foot_ids].copy()
            both = monitor.down & prev_down
            step_slip = float(np.linalg.norm(feet[both, :2] - prev_feet[both, :2], axis=1).sum())
            slip += step_slip
            if mode == "stop" and t >= stop_at:
                slip_stop += step_slip
            prev_feet, prev_down = feet, monitor.down.copy()
            q = env.data.qpos
            roll, pitch, yaw = _attitude(q[3:7])
            r = _rotation_from_quaternion(q[3:7])
            v = r.T @ env.data.qvel[:3]
            log.append((t, q[0], q[1], q[2], roll, pitch, yaw, v[0], v[1], env.data.qvel[5]))
            stopping = mode == "stop" and t >= stop_at and settle_time is None
            if stopping and np.linalg.norm(env.data.qvel[:2]) < 0.005 and monitor.down.all():
                settle_time = t - stop_at
            if q[2] < 0.05:
                break
        log = np.array(log)
        warm = log[:, 0] >= 1.5
        steady = warm & ((log[:, 0] < stop_at) if mode == "stop" else True)
        z = log[steady, 3]
        yaw_track = np.unwrap(log[:, 6]) - start_yaw
        duration = log[steady, 0][-1] - log[steady, 0][0]
        span = log[steady]
        distance = float(np.hypot(span[-1, 1] - span[0, 1], span[-1, 2] - span[0, 2]))
        records = np.array(monitor.touchdowns).reshape(-1, 2)
        speeds, peaks = records[:, 0], records[:, 1]
        groups = monitor.saturated_by_joint.reshape(6, 3) * monitor.dt
        result = {
            "controller": controller,
            "mode": mode,
            "seed": seed,
            "command": command.tolist(),
            "fallen": bool(log[-1, 3] < 0.05),
            "mean_vx_m_s": float(span[:, 7].mean()),
            "mean_vy_m_s": float(span[:, 8].mean()),
            "mean_wz_rad_s": float(span[:, 9].mean()),
            "yaw_change_deg": float(np.degrees(yaw_track[-1])),
            "distance_m": distance,
            "sole_slide_mm": 1000 * monitor.sole_slide_m,
            "sole_slide_mm_per_m": 1000 * monitor.sole_slide_m / max(distance, 0.05)
            if mode != "turn"
            else None,
            "loaded_sole_slide_mm": 1000 * monitor.loaded_slide_m,
            "foot_centre_travel_m": slip,
            "recontacts": monitor.recontacts,
            "roll_rms_deg": float(np.degrees(np.sqrt(np.mean(span[:, 4] ** 2)))),
            "pitch_rms_deg": float(np.degrees(np.sqrt(np.mean(span[:, 5] ** 2)))),
            "height_std_mm": float(1000 * z.std()),
            "lateral_sway_mm": float(
                1000
                * np.std(span[:, 2] - np.polyval(np.polyfit(span[:, 0], span[:, 2], 1), span[:, 0]))
            )
            if mode in ("forward", "backward", "stop")
            else None,
            "touchdowns": len(speeds),
            "touchdown_speed_p95_m_s": float(np.percentile(speeds, 95)) if len(speeds) else 0.0,
            "touchdown_peak_p95_weight": float(np.percentile(peaks, 95)) if len(peaks) else 0.0,
            "hard_touchdowns": int(np.sum((speeds > HARD_SPEED) | (peaks > HARD_LOAD))),
            "saturated_any_s": monitor.saturated_any * monitor.dt,
            "saturated_fraction": monitor.saturated_any / max(1, monitor.substeps),
            "saturated_joint_s": monitor.saturated_joint * monitor.dt,
            "saturated_s_by_joint": {
                f"{row}_{joint}": float(groups[legs, k].sum())
                for row, legs in (("front", [0, 3]), ("middle", [1, 4]), ("rear", [2, 5]))
                for k, joint in enumerate(("coxa", "femur", "tibia"))
            },
            "steady_s": float(duration),
        }
        if mode == "stop":
            result.update(
                stop_foot_centre_travel_m=slip_stop,
                settle_s=settle_time,
                standing=bool(gait.standing),
                stop_overshoot_m=float(log[-1, 1] - log[log[:, 0] <= stop_at][-1, 1]),
            )
        return result
    finally:
        env.close()
