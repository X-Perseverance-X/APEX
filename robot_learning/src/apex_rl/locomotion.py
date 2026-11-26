"""Offline forward locomotion with a tripod reference and terrain observations."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np
from gymnasium import spaces

from .contract import LEG_ORDER
from .env import Meta1Env, _rotation_from_quaternion
from .model import build_xml


@dataclass(frozen=True)
class Course:
    rise_m: float
    risers: int
    edge_x: float = 0.62
    tread_m: float = 0.42
    descending: bool = False

    @property
    def start_height(self) -> float:
        return self.rise_m * self.risers if self.descending else 0.0

    def height(self, x: float, y: float = 0.0) -> float:
        if self.descending:
            if abs(y) > 0.60 or x < -1.25:
                return 0.0
            crossed = max(0, 1 + math.floor((x - self.edge_x) / self.tread_m))
            return max(0, self.risers - crossed) * self.rise_m
        if abs(y) > 0.60 or x < self.edge_x or x >= self.last_edge + 1.25 or not self.risers:
            return 0.0
        return (
            min(self.risers, 1 + math.floor((x - self.edge_x) / self.tread_m + 1e-9)) * self.rise_m
        )

    @property
    def last_edge(self) -> float:
        return self.edge_x + max(0, self.risers - 1) * self.tread_m

    @property
    def goal(self) -> float:
        return self.last_edge + 0.43 if self.risers else 1.20

    def scene(self, config: dict) -> str:
        root = ET.fromstring(build_xml(config))
        world = root.find("worldbody")
        base = world.find("body[@name='base']")
        if self.descending:
            key = root.find("keyframe/key[@name='stand']")
            for element, attribute in ((key, "qpos"), (base, "pos")):
                values = element.attrib[attribute].split()
                values[2] = str(float(values[2]) + self.start_height)
                element.set(attribute, " ".join(values))
        # Keep the existing box's mass/inertia. Use separate contact envelopes
        # for the CAD deck and visible low-mounted hardware near the risers.
        base.find("geom[@name='body_shell']").attrib.update(contype="0", conaffinity="0")
        for name, pos, half_size in (
            ("deck", "0 0 0.044", "0.150 0.055 0.044"),
            ("speaker", "0.135 0 -0.025", "0.020 0.045 0.025"),
            ("main_pack", "-0.105 0 -0.0125", "0.035 0.030 0.0125"),
            ("middle_pack", "-0.019 0 -0.013", "0.029 0.027 0.013"),
        ):
            ET.SubElement(
                base,
                "geom",
                name=f"course_chassis_{name}",
                type="box",
                pos=pos,
                size=half_size,
                mass="0",
                rgba="0 0 0 0",
            )
        for i in range(self.risers):
            if self.descending:
                start = -1.25 if i == 0 else self.edge_x + (i - 1) * self.tread_m
                depth = self.edge_x - start if i == 0 else self.tread_m
                height = (self.risers - i) * self.rise_m
            else:
                start = self.edge_x + i * self.tread_m
                depth = self.tread_m if i < self.risers - 1 else 1.25
                height = (i + 1) * self.rise_m
            ET.SubElement(
                world,
                "geom",
                name=f"course_step_{i + 1}",
                type="box",
                pos=f"{start + depth / 2} 0 {height / 2}",
                size=f"{depth / 2} 0.60 {height / 2}",
                friction="1 0.005 0.0001",
                rgba=("0.24 0.38 0.46 1" if i % 2 == 0 else "0.30 0.47 0.55 1"),
            )
        ET.SubElement(
            world,
            "geom",
            name="goal_marker",
            type="box",
            pos=f"{self.goal} 0 {self.height(self.goal) + 0.001}",
            size="0.009 0.59 0.001",
            rgba="0.22 0.86 0.45 1",
            contype="0",
            conaffinity="0",
        )
        return ET.tostring(root, encoding="unicode")


COURSES = {
    "flat": Course(0, 0),
    "step-02": Course(0.02, 1),
    "down-02": Course(0.02, 1, descending=True),
    "step-03": Course(0.03, 1),
    "down-03": Course(0.03, 1, descending=True),
    "step-04": Course(0.04, 1),
    "down-04": Course(0.04, 1, descending=True),
    "step-05": Course(0.05, 1),
    "down-05": Course(0.05, 1, descending=True),
    "stairs-05": Course(0.05, 3),
    "stairs-down-05": Course(0.05, 3, descending=True),
    "stairs-10": Course(0.10, 3),
    "stairs-down-10": Course(0.10, 3, descending=True),
    "stairs-15": Course(0.15, 3),
    "stairs-down-15": Course(0.15, 3, descending=True),
    "stairs-20": Course(0.20, 3),
    "stairs-down-20": Course(0.20, 3, descending=True),
}

TRIPOD = np.array([0, 1, 0, 1, 0, 1], dtype=float) * 0.5


class LocomotionEnv(Meta1Env):
    """PPO adjusts stride, clearance and forward bias on top of inverse kinematics."""

    def __init__(
        self,
        course: str = "flat",
        *,
        randomize: bool = True,
        episode_steps: int | None = None,
        record_pose: bool = False,
    ):
        self.course_name = course
        self.course = COURSES[course]
        self.adapt = self.course.descending or self.course.rise_m > 0.02
        self.climbing = self.adapt and not self.course.descending
        self.period = (
            2.65
            if self.climbing
            else (2.20 if self.adapt else (1.24 if course == "flat" else 1.80))
        )
        self.record_pose = record_pose
        self.episode = -1
        super().__init__(
            task="velocity",
            episode_steps=episode_steps or (1700 if self.adapt else 1100),
            domain_randomization=randomize,
        )
        self.stand_height -= self.course.start_height
        self.action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(85,), dtype=np.float32)
        self._base_observation_space = spaces.Box(-np.inf, np.inf, shape=(72,), dtype=np.float32)
        self.mount = np.deg2rad([self.config["mount_yaw_deg"][leg] for leg in LEG_ORDER])
        self.hip = np.array([self.config["hip_xyz_m"][leg] for leg in LEG_ORDER])
        self.lengths = [self.config["links_m"][joint] for joint in ("coxa", "femur", "tibia")]
        self._stand = self.nominal.reshape(6, 3)
        co, fe, ti = self._stand.T
        l, a, b = self.lengths
        radius = l + a * np.cos(fe) + b * np.cos(fe + ti)
        self._neutral_foot = np.column_stack(
            (radius * np.cos(co), radius * np.sin(co), a * np.sin(fe) + b * np.sin(fe + ti))
        )
        self._foot_ids = [self.model.body(f"{leg.lower()}_foot").id for leg in LEG_ORDER]
        self._chassis_geoms = [
            self.model.geom(f"course_chassis_{name}").id
            for name in ("deck", "speaker", "main_pack", "middle_pack")
        ]
        self._leg_force_range = self.model.actuator_forcerange[self._legs].copy()
        self._terrain_ids = [
            self.model.geom(f"course_step_{i + 1}").id for i in range(self.course.risers)
        ]

    def _model_xml(self) -> str:
        return self.course.scene(self.config)

    def _observation(self) -> np.ndarray:
        base = super()._observation()
        x, y = self.data.qpos[:2]
        heading = self._heading()
        heights = [
            self.course.height(x + d * math.cos(heading), y + d * math.sin(heading))
            - self.course.height(x, y)
            for d in (0.15, 0.25, 0.35, 0.45, 0.55, 0.70)
        ]
        phase = 2 * math.pi * self.phase
        extras = [
            math.sin(phase),
            math.cos(phase),
            *heights,
            self.pack_voltage_proxy_v / 8.0,
            y,
            math.sin(heading),
            math.cos(heading),
            self.course.goal - x,
        ]
        return np.concatenate((base, extras)).astype(np.float32)

    def _heading(self) -> float:
        w, x, y, z = self.data.qpos[3:7]
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def reset(self, *, seed=None, options=None):
        self.episode += 1
        self.phase = 0.0
        self.supply_scale = 1.0
        self.pack_voltage_proxy_v = 8.0
        self.touchdowns = 0
        self.hold = 0
        self.work_j = 0.0
        self.slip_m = 0.0
        self.max_clearance_m = 0.0
        self.adjustment = np.zeros(3)
        self.model.actuator_forcerange[self._legs] = self._leg_force_range
        super().reset(seed=seed, options={"command": [0.12, 0.0, 0.0]})
        if self.domain_randomization:
            speed = (0.075, 0.11) if self.adapt else (0.10, 0.15)
            self.command[0] = self.np_random.uniform(*speed)
            self.pack_voltage_proxy_v = float(self.np_random.uniform(6.8, 8.0))
            self.supply_scale = self.pack_voltage_proxy_v / 8.0
        elif self.adapt:
            self.command[0] = 0.09
        self.model.actuator_forcerange[self._legs] = self._leg_force_range * self.supply_scale
        self.model.geom_friction[self._terrain_ids, 0] = self.model.geom_friction[0, 0]
        self.last_contacts = self._contacts().astype(bool)
        self.last_feet = self.data.xpos[self._foot_ids].copy()
        return self._observation(), {"course": self.course_name}

    def _targets(self, residual: np.ndarray) -> np.ndarray:
        period = self.period
        duty = 0.68 if self.climbing else (0.72 if self.adapt else 0.64)
        phase = (self.phase + TRIPOD) % 1.0
        swing = phase >= duty
        u = np.clip((phase - duty) / (1 - duty), 0, 1)
        ramp = min(1.0, self.steps * 0.02 / 0.75)
        stride = (
            float(self.command[0])
            * (2.20 if self.climbing else period)
            * duty
            * (1.0 + 0.35 * residual[0])
        )
        delay, sweep = (0.28, 0.53) if self.climbing else (0.22, 0.58)
        travel = np.clip((u - delay) / sweep, 0, 1)
        dx = stride * np.where(swing, -0.5 + (3 * travel**2 - 2 * travel**3), 0.5 - phase / duty)
        dx = ramp * dx + 0.013 * residual[2]
        lift_scale = 0.30 if self.climbing else 0.65
        lift = np.clip(0.045 + self.course.rise_m * lift_scale + 0.035 * residual[1], 0.015, 0.13)
        foot = self._neutral_foot.copy()
        foot[:, 0] += dx * np.cos(self.mount)
        foot[:, 1] -= dx * np.sin(self.mount)
        x, y, body_z = self.data.qpos[:3]
        yaw = self._heading()
        c, s = math.cos(yaw), math.sin(yaw)
        foot_x = self.hip[:, 0] + foot[:, 0] * np.cos(self.mount) - foot[:, 1] * np.sin(self.mount)
        foot_y = self.hip[:, 1] + foot[:, 0] * np.sin(self.mount) + foot[:, 1] * np.cos(self.mount)
        world_x = x + foot_x * c - foot_y * s
        world_y = y + foot_x * s + foot_y * c
        terrain = np.array(
            [self.course.height(xx, yy) for xx, yy in zip(world_x, world_y, strict=True)]
        )
        for i in np.flatnonzero(swing):
            preview = max(0.0, stride * 0.5 - dx[i])
            if self.climbing:
                preview += self.config["foot_radius_m"] + 0.009
            landing = self.course.height(world_x[i] + preview * c, world_y[i] + preview * s)
            if self.adapt:
                initial = self.course.height(
                    world_x[i] - max(0.0, dx[i] + stride / 2) * c,
                    world_y[i] - max(0.0, dx[i] + stride / 2) * s,
                )
                transfer = travel[i] if self.course.descending else min(1.0, u[i] / 0.28)
                terrain[i] = max(terrain[i], initial + (landing - initial) * transfer)
            else:
                terrain[i] = max(terrain[i], landing * min(1.0, u[i] / 0.28))
        # Bounded vertical compensation helps the feet reach a tread as the body rises.
        height_adjustment = terrain - max(0.0, body_z - self.stand_height)
        foot[:, 2] += np.clip(height_adjustment, -0.12, 0.14)
        foot[:, 2] += ramp * lift * np.sin(np.pi * u) ** 2
        if self.adapt:
            # Express the planned foot motion relative to gravity when the pelvis pitches.
            relative = np.column_stack((foot_x, foot_y, self.hip[:, 2] + foot[:, 2]))
            world = relative @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
            body = world @ _rotation_from_quaternion(self.data.qpos[3:7]) - self.hip
            mc, ms = np.cos(self.mount), np.sin(self.mount)
            foot = np.column_stack(
                (body[:, 0] * mc + body[:, 1] * ms, -body[:, 0] * ms + body[:, 1] * mc, body[:, 2])
            )

        length, a, b = self.lengths
        radius = np.hypot(foot[:, 0], foot[:, 1]) - length
        z = foot[:, 2]
        cos_knee = np.clip((radius**2 + z**2 - a * a - b * b) / (2 * a * b), -0.999, 0.999)
        knee = -np.arccos(cos_knee)
        shoulder = np.arctan2(z, radius) - np.arctan2(b * np.sin(knee), a + b * cos_knee)
        coxa = np.arctan2(foot[:, 1], foot[:, 0])
        return np.clip(
            np.column_stack((coxa, shoulder, knee)).ravel(), self.joint_low, self.joint_high
        )

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (3,) or not np.isfinite(action).all():
            raise ValueError("locomotion action must be three finite residuals")
        action = np.clip(action, -1, 1)
        self.adjustment = 0.75 * self.adjustment + 0.25 * action
        x0 = float(self.data.qpos[0])
        targets = self._targets(self.adjustment)
        self._apply_joint_targets(targets)
        self.steps += 1
        self.phase = (self.phase + 0.02 / self.period) % 1.0
        self.previous_action[:] = (targets - self.nominal) / 0.25
        self.previous_action[:] = self.previous_action.clip(-1.0, 1.0)
        x, y, z = self.data.qpos[:3]
        upright = 1 - 2 * float(self.data.qpos[4:6] @ self.data.qpos[4:6])
        soles = self.data.xpos[self._foot_ids].copy()
        contact = self._contacts().astype(bool)
        touchdown = int(np.count_nonzero(contact & ~self.last_contacts))
        self.touchdowns += touchdown
        slip = float(
            np.linalg.norm(
                soles[contact & self.last_contacts, :2]
                - self.last_feet[contact & self.last_contacts, :2],
                axis=1,
            ).sum()
        )
        self.slip_m += slip
        self.last_contacts, self.last_feet = contact, soles
        clearance = [
            sz - self.config["foot_radius_m"] - self.course.height(sx, sy)
            for (sx, sy, sz), grounded in zip(soles, contact, strict=True)
            if not grounded
        ]
        if clearance:
            self.max_clearance_m = max(self.max_clearance_m, max(clearance))
        energy = float(np.abs(self.data.actuator_force[:18] * self.data.qvel[6:24]).sum()) * 0.02
        self.work_j += energy
        heading = self._heading()
        chassis_hit = bool(np.isin(self.data.contact.geom, self._chassis_geoms).any())
        fallen = bool(chassis_hit or upright < 0.68 or z < 0.05 + self.course.height(x, y))
        out = bool(abs(y) > 0.42 or x < -0.24)
        in_goal = (
            x >= self.course.goal
            and upright > 0.92
            and contact.sum() >= 3
            and (not self.course.risers or np.min(soles[:, 0]) > self.course.last_edge)
        )
        self.hold = self.hold + 1 if in_goal else 0
        success = self.hold >= 8 and not fallen and not out
        reward = (
            14 * float(x - x0)
            + 0.012 * upright
            - 0.015 * abs(y)
            - 0.007 * abs(heading)
            - 0.03 * slip
            - 0.0015 * energy
            - 0.003 * float(np.mean(self.adjustment**2))
            + 5.0 * success
            - 3.0 * (fallen or out)
        )
        info = {
            "success": bool(success),
            "fallen": fallen,
            "chassis_hit": chassis_hit,
            "out": out,
            "progress_m": float(x),
            "touchdowns": self.touchdowns,
            "feet_xyz_m": soles.tolist(),
            "mean_forward_m_s": float(x / (self.steps * 0.02)),
            "slip_m": self.slip_m,
            "mechanical_work_j": self.work_j,
            "max_clearance_m": self.max_clearance_m,
            "supply_scale": self.supply_scale,
            "pack_voltage_proxy_v": self.pack_voltage_proxy_v,
        }
        if self.record_pose and (
            self.steps % 5 == 0 or success or fallen or out or self.steps == self.episode_steps
        ):
            info["pose"] = self.data.qpos.round(5).tolist()
            info["episode_id"] = self.episode
            info["episode_step"] = self.steps
        return (
            self._observation(),
            reward,
            success or fallen or out,
            self.steps >= self.episode_steps,
            info,
        )
