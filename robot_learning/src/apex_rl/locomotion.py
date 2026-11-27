"""Offline forward locomotion with a tripod reference and terrain observations."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np
from gymnasium import spaces

from .ascent_pose import apply_knee_stops
from .body_motion import BodyMotion
from .contract import LEG_ORDER
from .env import Meta1Env, _rotation_from_quaternion
from .leg_contacts import LinkContactMetrics, add_leg_contacts
from .model import build_xml


@dataclass(frozen=True)
class Course:
    rise_m: float
    risers: int
    edge_x: float = 0.62
    tread_m: float = 0.42
    descending: bool = False
    link_contacts: bool = False
    attitude_control: bool = False
    steep_approach: bool = False
    diagonal_transfer: bool = False

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

    @property
    def terrain_names(self) -> tuple[str, ...]:
        return tuple(f"course_step_{i + 1}" for i in range(self.risers))

    @property
    def goal_y(self) -> float:
        return 0.0

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


@dataclass(frozen=True)
class RubbleCourse(Course):
    # Entries are centre X/Y, half lengths X/Y and height, in metres.
    blocks: tuple[tuple[float, float, float, float, float], ...] = ()
    waypoints: tuple[tuple[float, float], ...] = ()

    @property
    def goal(self) -> float:
        return self.waypoints[-1][0]

    @property
    def goal_y(self) -> float:
        return self.waypoints[-1][1]

    @property
    def terrain_names(self) -> tuple[str, ...]:
        return tuple(f"rock_{i + 1}" for i in range(len(self.blocks)))

    def height(self, x: float, y: float = 0.0) -> float:
        return max(
            (h for bx, by, dx, dy, h in self.blocks if abs(x - bx) <= dx and abs(y - by) <= dy),
            default=0.0,
        )

    def route_y(self, x: float) -> float:
        for (a, left), (b, right) in zip(self.waypoints, self.waypoints[1:]):
            if x <= b:
                return left + (right - left) * max(0.0, (x - a) / (b - a))
        return self.goal_y

    def scene(self, config: dict) -> str:
        root = ET.fromstring(super().scene(config))
        world = root.find("worldbody")
        for i, (name, (x, y, dx, dy, h)) in enumerate(
            zip(self.terrain_names, self.blocks, strict=True)
        ):
            ET.SubElement(
                world,
                "geom",
                name=name,
                type="box",
                pos=f"{x} {y} {h / 2}",
                size=f"{dx} {dy} {h / 2}",
                friction="1 0.005 0.0001",
                rgba="0.42 0.36 0.29 1" if i % 2 else "0.33 0.38 0.29 1",
            )
        marker = world.find("geom[@name='goal_marker']")
        marker.set(
            "pos", f"{self.goal} {self.goal_y} {self.height(self.goal, self.goal_y) + 0.001}"
        )
        marker.set("size", "0.025 0.10 0.001")
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
    "hybrid-step-05": Course(0.05, 1, edge_x=1.05),
    "hybrid-down-05": Course(0.05, 1, edge_x=1.05, descending=True),
    "hybrid-narrow-05": Course(0.05, 3, edge_x=1.05, tread_m=0.23),
    "hybrid-narrow-down-05": Course(0.05, 3, edge_x=1.05, tread_m=0.23, descending=True),
    "hybrid-narrow-08": Course(0.08, 3, edge_x=1.05, tread_m=0.23),
    "hybrid-narrow-down-08": Course(0.08, 3, edge_x=1.05, tread_m=0.23, descending=True),
    "hybrid-narrow-10": Course(0.10, 3, edge_x=1.05, tread_m=0.21),
    "hybrid-narrow-down-10": Course(0.10, 3, edge_x=1.05, tread_m=0.21, descending=True),
    "hybrid-lab-up-07": Course(0.07, 1, edge_x=1.05),
    "hybrid-lab-up-08": Course(0.08, 3, edge_x=1.05, tread_m=0.23),
    "hybrid-lab-up-07-slow": Course(0.07, 1, edge_x=1.05),
    "hybrid-lab-down-10": Course(0.10, 3, edge_x=1.05, tread_m=0.21, descending=True),
    "hybrid-lab-down-12": Course(0.12, 3, edge_x=1.05, tread_m=0.21, descending=True),
    "hybrid-lab-down-15": Course(0.15, 3, edge_x=1.05, tread_m=0.21, descending=True),
    "hybrid-lab-down-15-single": Course(0.15, 1, edge_x=1.05, descending=True),
    "hybrid-lab-down-15-wide": Course(0.15, 3, edge_x=1.05, tread_m=0.32, descending=True),
    "hybrid-contact-down-10": Course(
        0.10, 3, edge_x=1.05, tread_m=0.21, descending=True, link_contacts=True
    ),
    "hybrid-contact-down-15": Course(0.15, 1, edge_x=1.05, descending=True, link_contacts=True),
    "hybrid-contact-up-05": Course(0.05, 1, edge_x=1.05, link_contacts=True),
    "hybrid-contact-up-07": Course(0.07, 1, edge_x=1.05, link_contacts=True),
    "hybrid-contact-up-10": Course(0.10, 1, edge_x=1.05, link_contacts=True),
    "hybrid-contact-up-15": Course(0.15, 1, edge_x=1.05, link_contacts=True),
    "hybrid-contact-stairs-up-10": Course(0.10, 3, edge_x=1.05, tread_m=0.21, link_contacts=True),
    "hybrid-rubble-01": RubbleCourse(
        0,
        0,
        edge_x=0.78,
        link_contacts=True,
        blocks=(
            (0.85, 0.29, 0.12, 0.12, 0.020),
            (1.03, -0.30, 0.12, 0.13, 0.022),
            (1.20, 0.26, 0.12, 0.12, 0.026),
            (1.34, -0.35, 0.12, 0.13, 0.016),
            (1.52, 0.26, 0.12, 0.12, 0.022),
            (1.65, -0.22, 0.11, 0.11, 0.016),
        ),
        waypoints=((0, 0), (0.56, 0), (0.94, -0.045), (1.35, -0.045), (1.99, 0.065)),
    ),
    "hybrid-rubble-02": RubbleCourse(
        0,
        0,
        edge_x=0.78,
        link_contacts=True,
        blocks=(
            (0.82, 0.32, 0.10, 0.12, 0.041),
            (1.02, -0.16, 0.12, 0.094, 0.12),
            (1.26, 0.28, 0.12, 0.15, 0.036),
            (1.48, -0.29, 0.12, 0.14, 0.042),
            (1.65, 0.18, 0.12, 0.11, 0.023),
            (1.76, -0.20, 0.10, 0.09, 0.031),
        ),
        waypoints=((0, 0), (0.52, 0), (0.90, 0.11), (1.29, 0.12), (1.99, -0.04)),
    ),
    "hybrid-pose-up-05": Course(0.05, 1, edge_x=1.05, link_contacts=True, attitude_control=True),
    "hybrid-pose-up-07": Course(0.07, 1, edge_x=1.05, link_contacts=True, attitude_control=True),
    "hybrid-pose-up-10": Course(0.10, 1, edge_x=1.05, link_contacts=True, attitude_control=True),
    "hybrid-pose-up-15": Course(0.15, 1, edge_x=1.05, link_contacts=True, attitude_control=True),
    "hybrid-steep-up-08": Course(
        0.08, 1, edge_x=1.05, link_contacts=True, attitude_control=True, steep_approach=True
    ),
    "hybrid-steep-up-09": Course(
        0.09, 1, edge_x=1.05, link_contacts=True, attitude_control=True, steep_approach=True
    ),
    "hybrid-steep-up-10": Course(
        0.10, 1, edge_x=1.05, link_contacts=True, attitude_control=True, steep_approach=True
    ),
    "hybrid-steep-up-12": Course(
        0.12, 1, edge_x=1.05, link_contacts=True, attitude_control=True, steep_approach=True
    ),
    "hybrid-steep-up-15": Course(
        0.15, 1, edge_x=1.05, link_contacts=True, attitude_control=True, steep_approach=True
    ),
    "hybrid-diagonal-up-12": Course(
        0.12,
        1,
        edge_x=1.05,
        link_contacts=True,
        attitude_control=True,
        steep_approach=True,
        diagonal_transfer=True,
    ),
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
        body_control: bool = False,
    ):
        self.course_name = course
        self.course = COURSES[course]
        self.attitude_control = self.course.attitude_control
        self.hybrid = course.startswith("hybrid-")
        self.rubble = isinstance(self.course, RubbleCourse)
        self.contact_ascent = (
            self.course.link_contacts and self.course.risers > 0 and not self.course.descending
        )
        self.edge_control = course.startswith("hybrid-lab-") or (
            self.course.link_contacts and not self.course.descending
        )
        self.short_climb = course == "hybrid-lab-up-07-slow"
        body_control |= self.hybrid
        self.adapt = self.course.descending or self.course.rise_m > 0.02
        self.climbing = self.adapt and not self.course.descending
        self.period = (
            2.65
            if self.climbing
            else (2.20 if self.adapt else (1.24 if course == "flat" else 1.80))
        )
        self.record_pose = record_pose
        self.gait = "tripod" if self.hybrid else "reference"
        self.switch_step = None
        self.approach_speed = 0.0
        self.approach_x = 0.0
        self.episode = -1
        super().__init__(
            task="velocity",
            episode_steps=episode_steps
            or (3000 if body_control else (1700 if self.adapt else 1100)),
            domain_randomization=randomize,
        )
        self.stand_height -= self.course.start_height
        self.action_space = spaces.Box(
            -1.0,
            1.0,
            shape=(8 if self.attitude_control else 6 if body_control else 3,),
            dtype=np.float32,
        )
        self.observation_space = spaces.Box(
            -np.inf,
            np.inf,
            shape=(105 if self.attitude_control else 103 if body_control else 85,),
            dtype=np.float32,
        )
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
        self._terrain_ids = [self.model.geom(name).id for name in self.course.terrain_names]
        self.link_metrics = (
            LinkContactMetrics(self.model, self._terrain_ids) if self.course.link_contacts else None
        )
        self.body_motion = BodyMotion(self) if body_control else None
        if self.body_motion:
            self.body_motion.reset()

    def _model_xml(self) -> str:
        xml = self.course.scene(self.config)
        if self.attitude_control:
            xml = apply_knee_stops(xml)
        return (
            add_leg_contacts(xml, self.course.terrain_names) if self.course.link_contacts else xml
        )

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
            y - self.course.route_y(x) if self.rubble else y,
            math.sin(heading),
            math.cos(heading),
            self.course.goal - x,
        ]
        if self.body_motion:
            controller = self.body_motion
            relative_feet = [
                sz - self.config["foot_radius_m"] - self.course.height(sx, sy)
                for sx, sy, sz in self.data.xpos[self._foot_ids]
            ]
            leg = [float(controller.leg == i) for i in range(6)]
            extras.extend(
                [
                    *(controller.body - self.data.qpos[:3]),
                    controller.pitch,
                    controller.support_count / 6,
                    min(1.0, controller.elapsed),
                    *leg,
                    *relative_feet,
                ]
            )
        if self.attitude_control:
            extras.extend((self.body_motion.roll, self.body_motion.yaw))
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
        self.rear_riser_contact_steps = 0
        if self.link_metrics is not None:
            self.link_metrics.reset()
        self.max_clearance_m = 0.0
        self.adjustment = np.zeros(self.action_space.shape)
        self.gait = "tripod" if self.hybrid else "reference"
        self.switch_step = None
        self.approach_speed = 0.0
        self.approach_x = 0.0
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
        if self.body_motion:
            self.body_motion.reset()
        self.min_body_z = self.max_body_z = float(self.data.qpos[2])
        self.max_lateral_com = 0.0
        self.min_knee_position = float(self.data.qpos[self._leg_positions][2::3].min())
        self.attitude_peak = np.zeros(3)
        return self._observation(), {"course": self.course_name}

    def _targets(self, residual: np.ndarray) -> np.ndarray:
        adjusted = residual * 0.28 if self.contact_ascent or self.rubble else residual
        if self.hybrid and self.gait != "wave":
            x = float(self.data.qpos[0])
            # Hand off after touchdown, so the wave gait inherits a planted
            # footprint and never has to flatten a tripod mid-swing.
            if self._approach_complete(x) and self.body_motion.contacts().sum() >= 5:
                self.switch_step = self.steps
                self.approach_speed = x / max(0.02, self.steps * 0.02)
                self.approach_x = x
                self.body_motion.reset()
                self.gait = "wave"
            else:
                return self._tripod_targets(
                    np.zeros_like(residual)
                    if self.short_climb or self.contact_ascent or self.rubble
                    else residual,
                    flat=True,
                )
        if self.body_motion:
            return self._solve_ik(self.body_motion.targets(adjusted))
        return self._tripod_targets(residual)

    def _approach_complete(self, x: float) -> bool:
        return self.course.edge_x - x <= 0.58

    def _tripod_targets(self, residual: np.ndarray, *, flat: bool = False) -> np.ndarray:
        period = 1.24 if flat else self.period
        duty = 0.64 if flat else (0.68 if self.climbing else (0.72 if self.adapt else 0.64))
        base_phase = self.steps * 0.02 / period if flat else self.phase
        phase = (base_phase + TRIPOD) % 1.0
        swing = phase >= duty
        u = np.clip((phase - duty) / (1 - duty), 0, 1)
        ramp = min(1.0, self.steps * 0.02 / 0.75)
        stride = (
            (0.13 if flat else float(self.command[0]))
            * (2.20 if self.climbing and not flat else period)
            * duty
            * (1.0 + 0.35 * residual[0])
        )
        delay, sweep = (0.28, 0.53) if self.climbing and not flat else (0.22, 0.58)
        travel = np.clip((u - delay) / sweep, 0, 1)
        dx = stride * np.where(swing, -0.5 + (3 * travel**2 - 2 * travel**3), 0.5 - phase / duty)
        dx = ramp * dx + 0.013 * residual[2]
        lift_scale = 0.30 if self.climbing else 0.65
        rise = 0.0 if flat else self.course.rise_m
        lift = np.clip(0.045 + rise * lift_scale + 0.035 * residual[1], 0.015, 0.13)
        foot = self._neutral_foot.copy()
        foot[:, 0] += dx * np.cos(self.mount)
        foot[:, 1] -= dx * np.sin(self.mount)
        if flat:
            foot[:, 2] += ramp * lift * np.sin(np.pi * u) ** 2
            return self._solve_ik(foot)
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

        return self._solve_ik(foot)

    def _solve_ik(self, foot: np.ndarray) -> np.ndarray:
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

    def _observe_link_contacts(self, contacts):
        self.link_metrics.observe(contacts)
        if self.attitude_control:
            self.min_knee_position = min(
                self.min_knee_position, float(self.data.qpos[self._leg_positions][2::3].min())
            )

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        if action.shape != self.action_space.shape or not np.isfinite(action).all():
            raise ValueError("locomotion action has the wrong shape or a non-finite value")
        action = np.clip(action, -1, 1)
        self.adjustment = 0.75 * self.adjustment + 0.25 * action
        x0 = float(self.data.qpos[0])
        targets = self._targets(self.adjustment)
        self._apply_joint_targets(
            targets,
            observe_contacts=(
                self._observe_link_contacts if self.link_metrics is not None else None
            ),
        )
        if self.attitude_control:
            controller = self.body_motion
            self.attitude_peak = np.maximum(
                self.attitude_peak, np.abs((controller.roll, controller.pitch, controller.yaw))
            )
        self.steps += 1
        self.phase = (self.phase + 0.02 / self.period) % 1.0
        self.previous_action[:] = (targets - self.nominal) / 0.25
        self.previous_action[:] = self.previous_action.clip(-1.0, 1.0)
        x, y, z = self.data.qpos[:3]
        upright = 1 - 2 * float(self.data.qpos[4:6] @ self.data.qpos[4:6])
        soles = self.data.xpos[self._foot_ids].copy()
        contact = self.body_motion.contacts() if self.body_motion else self._contacts().astype(bool)
        rear_riser_contact = self.body_motion is not None and self.body_motion.rear_riser_contact()
        self.rear_riser_contact_steps += int(rear_riser_contact)
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
            and (not self.rubble or abs(y - self.course.goal_y) <= 0.12)
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
            - 0.07 * (self.edge_control and rear_riser_contact)
            + 5.0 * success
            - 3.0 * (fallen or out)
        )
        if self.link_metrics is not None:
            info_contact = self.link_metrics.info()
        else:
            info_contact = {}
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
            **info_contact,
        }
        if self.body_motion:
            self.min_body_z = min(self.min_body_z, float(z))
            self.max_body_z = max(self.max_body_z, float(z))
            self.max_lateral_com = max(
                self.max_lateral_com, abs(float(self.data.subtree_com[self.body_motion.base_id, 1]))
            )
            info.update(
                body_z_m=float(z),
                body_target_z_m=float(self.body_motion.body[2]),
                body_height_range_m=self.max_body_z - self.min_body_z,
                max_lateral_com_m=self.max_lateral_com,
                supporting_feet=self.body_motion.support_count,
            )
        if self.hybrid:
            info.update(
                gait=self.gait,
                approach_x_m=self.approach_x,
                approach_speed_m_s=self.approach_speed,
                precision_speed_m_s=(
                    float((x - self.approach_x) / max(0.02, (self.steps - self.switch_step) * 0.02))
                    if self.switch_step is not None
                    else 0.0
                ),
                rear_riser_contact_steps=self.rear_riser_contact_steps,
            )
        if self.attitude_control:
            info["min_knee_deg"] = math.degrees(self.min_knee_position)
            info["peak_target_rpy_deg"] = np.rad2deg(self.attitude_peak).tolist()
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
