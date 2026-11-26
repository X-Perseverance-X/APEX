"""Gymnasium task for offline stand and velocity-control experiments."""

from __future__ import annotations

import math
from typing import ClassVar

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

from .contract import LEG_ORDER, load_contract
from .model import build_xml, stand_joint_angles


def _rotation_from_quaternion(q: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(v) for v in q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class Meta1Env(gym.Env):
    """Proprioceptive 18-joint residual policy; never accesses Pi or ESP.

    Action: normalized 18-joint offsets around firmware stand pose.
    Observation: body velocity, angular velocity, gravity, 18 joint positions,
    18 velocities, 18 previous actions, 3 commanded velocities, 6 foot contacts.
    Dynamics, contact and servo parameters are provisional until identification.
    """

    metadata: ClassVar[dict] = {"render_modes": []}

    def __init__(
        self, task: str = "stand", episode_steps: int = 1000, domain_randomization: bool = False
    ):
        super().__init__()
        if task not in {"stand", "velocity"}:
            raise ValueError("task must be stand or velocity")
        self.task = task
        self.episode_steps = episode_steps
        self.domain_randomization = domain_randomization
        self.config = load_contract()
        self.model = mujoco.MjModel.from_xml_string(self._model_xml())
        self.data = mujoco.MjData(self.model)
        self.nominal = np.array(stand_joint_angles(self.config), dtype=np.float64)
        self._legs = slice(0, self.nominal.size)
        self._leg_positions = slice(7, 7 + self.nominal.size)
        self._leg_velocities = slice(6, 6 + self.nominal.size)
        self.joint_low = self.model.actuator_ctrlrange[self._legs, 0].copy()
        self.joint_high = self.model.actuator_ctrlrange[self._legs, 1].copy()
        rates = self.config["leg_target_rate_deg_s"]
        self._max_target_rate = np.deg2rad([rates[j] for j in ("coxa", "femur", "tibia")] * 6)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(18,), dtype=np.float32)
        bounds = np.array(
            [5.0] * 3
            + [20.0] * 3
            + [1.1] * 3
            + [4.0] * 18
            + [30.0] * 18
            + [1.0] * 18
            + [2.0] * 3
            + [1.0] * 6,
            dtype=np.float32,
        )
        self.observation_space = spaces.Box(-bounds, bounds, dtype=np.float32)
        self._base_observation_space = self.observation_space
        self.previous_action = np.zeros(18, dtype=np.float32)
        self.command = np.zeros(3, dtype=np.float32)
        self.steps = 0
        self.control_steps = 10  # 0.002 s physics; 50 Hz policy and command interface
        self.stand_height = float(self.model.key_qpos[0, 2])
        self._nominal_floor_friction = self.model.geom_friction[0].copy()
        self._nominal_body_mass = self.model.body_mass.copy()
        self._nominal_body_inertia = self.model.body_inertia.copy()
        self._nominal_gain = self.model.actuator_gainprm.copy()
        self._nominal_bias = self.model.actuator_biasprm.copy()
        self._foot_sensor_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, f"{leg.lower()}_contact")
            for leg in LEG_ORDER
        ]

    def _model_xml(self) -> str:
        return build_xml(self.config)

    def _randomize_dynamics(self) -> None:
        self.model.geom_friction[0] = self._nominal_floor_friction
        self.model.body_mass[:] = self._nominal_body_mass
        self.model.body_inertia[:] = self._nominal_body_inertia
        self.model.actuator_gainprm[:] = self._nominal_gain
        self.model.actuator_biasprm[:] = self._nominal_bias
        if not self.domain_randomization:
            return
        # Narrow pilot ranges; widen only after physical system identification.
        self.model.geom_friction[0, 0] *= float(self.np_random.uniform(0.85, 1.15))
        base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base")
        body_scale = float(self.np_random.uniform(0.95, 1.05))
        self.model.body_mass[base_id] *= body_scale
        self.model.body_inertia[base_id] *= body_scale
        gain_scale = self.np_random.uniform(0.9, 1.1, size=18)
        self.model.actuator_gainprm[self._legs, 0] *= gain_scale
        self.model.actuator_biasprm[self._legs, 1] *= gain_scale
        mujoco.mj_setConst(self.model, self.data)

    def _contacts(self) -> np.ndarray:
        return np.array(
            [
                float(self.data.sensordata[self.model.sensor_adr[sensor_id]] > 0.2)
                for sensor_id in self._foot_sensor_ids
            ],
            dtype=np.float32,
        )

    def _observation(self) -> np.ndarray:
        rotation = _rotation_from_quaternion(self.data.qpos[3:7])
        base_linear = rotation.T @ self.data.qvel[:3]
        base_angular = self.data.qvel[3:6]
        gravity = rotation.T @ np.array([0.0, 0.0, -1.0])
        obs = np.concatenate(
            (
                base_linear,
                base_angular,
                gravity,
                self.data.qpos[self._leg_positions] - self.nominal,
                self.data.qvel[self._leg_velocities],
                self.previous_action,
                self.command,
                self._contacts(),
            )
        ).astype(np.float32)
        if not np.isfinite(obs).all() or not self._base_observation_space.contains(obs):
            raise RuntimeError("non-finite or incorrect-size simulation observation")
        return obs

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        self._randomize_dynamics()
        # mj_setConst inside randomization rewrites reference state; restore
        # the named stand keyframe only after changing model constants.
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.previous_action.fill(0)
        self.steps = 0
        self.command.fill(0)
        if self.task == "velocity":
            command = None if options is None else options.get("command")
            if command is None:
                self.command[:] = [
                    self.np_random.uniform(-0.15, 0.25),
                    self.np_random.uniform(-0.10, 0.10),
                    self.np_random.uniform(-0.3, 0.3),
                ]
            else:
                arr = np.asarray(command, dtype=np.float32)
                if (
                    arr.shape != (3,)
                    or not np.isfinite(arr).all()
                    or np.any(np.abs(arr[:2]) > 0.4)
                    or abs(float(arr[2])) > 1.0
                ):
                    raise ValueError("velocity command must be finite and within 0.4 m/s, 1 rad/s")
                self.command[:] = arr
        mujoco.mj_forward(self.model, self.data)
        return self._observation(), {
            "task": self.task,
            "domain_randomization": self.domain_randomization,
        }

    def _apply_joint_targets(self, desired: np.ndarray) -> None:
        delta = self._max_target_rate * self.model.opt.timestep
        leg_controls = self.data.ctrl[self._legs]
        for _ in range(self.control_steps):
            leg_controls[:] = np.clip(desired, leg_controls - delta, leg_controls + delta)
            mujoco.mj_step(self.model, self.data)

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (18,) or not np.isfinite(action).all():
            raise ValueError("action must have 18 finite normalized joint offsets")
        action = np.clip(action, -1.0, 1.0)
        desired = np.clip(self.nominal + 0.25 * action, self.joint_low, self.joint_high)
        self._apply_joint_targets(desired)
        rotation = _rotation_from_quaternion(self.data.qpos[3:7])
        gravity = rotation.T @ np.array([0.0, 0.0, -1.0])
        height = float(self.data.qpos[2])
        upright = math.exp(-4.0 * float(gravity[:2] @ gravity[:2]))
        height_score = math.exp(-80.0 * (height - self.stand_height) ** 2)
        action_rate = float(np.mean((action - self.previous_action) ** 2))
        effort = float(np.mean(np.square(self.data.actuator_force[self._legs])))
        if self.task == "stand":
            reward = 1.5 * upright + height_score - 0.05 * action_rate - 0.002 * effort
        else:
            body_velocity = rotation.T @ self.data.qvel[:3]
            xy_error = body_velocity[:2] - self.command[:2]
            yaw_error = float(self.data.qvel[5] - self.command[2])
            tracking = math.exp(-float(xy_error @ xy_error) / 0.04 - yaw_error * yaw_error / 0.25)
            reward = 2.0 * tracking + 0.5 * upright + 0.2 * height_score
            reward -= 0.05 * action_rate + 0.002 * effort
        self.previous_action[:] = action
        self.steps += 1
        terminated = bool(
            height < 0.055 or gravity[2] > -0.65 or not np.isfinite(self.data.qpos).all()
        )
        truncated = self.steps >= self.episode_steps
        info = {
            "height_m": height,
            "upright": upright,
            "contacts": int(self._contacts().sum()),
            "forward_m_s": float((rotation.T @ self.data.qvel[:3])[0]),
            "command": self.command.copy(),
        }
        return self._observation(), float(reward), terminated, truncated, info

    def close(self):
        self.data = None
        self.model = None
