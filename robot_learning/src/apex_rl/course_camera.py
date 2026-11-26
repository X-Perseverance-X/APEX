"""Record and exercise the simulated front RGB camera on stair courses."""

from __future__ import annotations

import base64
import io
import json
import math
from dataclasses import asdict
from pathlib import Path

import mujoco
import numpy as np

from .locomotion import Course, LocomotionEnv
from .stair_rgb import CameraCalibration, estimate_riser


class CourseCamera:
    tilt = math.radians(20)

    def __init__(self, env):
        self.env = env
        sensors = env.config["sensors"]
        axis = np.array(sensors["gimbal"]["tilt_axis_xyz_m"])
        dx, _, dz = np.array(sensors["camera"]["xyz_m"]) - axis
        c, s = math.cos(self.tilt), math.sin(self.tilt)
        self.offset_x = float(axis[0] + c * dx + s * dz)
        height = env.stand_height + float(axis[2] - s * dx + c * dz)
        self.calibration = CameraCalibration(
            height, self.tilt, math.radians(float(env.model.camera("front_camera").fovy[0]))
        )
        self.renderer = mujoco.Renderer(env.model, 180, 320)
        self.options = mujoco.MjvOption()
        self.options.sitegroup.fill(0)

    def aim(self):
        self.env.data.joint("camera_tilt_joint").qpos[0] = self.tilt
        self.env.data.actuator("camera_tilt_servo").ctrl[0] = self.tilt
        mujoco.mj_forward(self.env.model, self.env.data)

    def capture(self):
        self.renderer.update_scene(self.env.data, camera="front_camera", scene_option=self.options)
        return self.renderer.render()

    def close(self):
        self.renderer.close()


class CameraGatedCourse(LocomotionEnv):
    """Use only calibrated RGB for the approach decision in the onset experiment."""

    def __init__(self, course: str, **kwargs):
        super().__init__(course, **kwargs)
        if course != "hybrid-step-05":
            raise ValueError("RGB walking uses the single upward step experiment")
        self.edge_control = True
        self.camera = CourseCamera(self)
        self.riser = None
        self.camera_votes = 0
        self.body_motion.terrain = Course(0, 0)

    def reset(self, **kwargs):
        obs, info = super().reset(**kwargs)
        self.camera.aim()
        self.riser = None
        self.camera_votes = 0
        self.body_motion.terrain = Course(0, 0)
        return obs, info

    def _approach_complete(self, x):
        if self.steps % 10 == 0:
            estimate = estimate_riser(self.camera.capture(), self.camera.calibration)
            if estimate is None:
                self.camera_votes = 0
            elif (
                self.riser is not None
                and abs(estimate.height_m - self.riser.height_m) <= 0.03
                and self.camera.offset_x + estimate.range_m <= 0.68
            ):
                self.camera_votes += 1
            else:
                self.camera_votes = 0
            self.riser = estimate
            if self.camera_votes >= 2:
                edge = x + self.camera.offset_x + estimate.range_m
                self.body_motion.terrain = Course(estimate.height_m, 1, edge_x=edge)
        return self.camera_votes >= 2

    def _targets(self, residual):
        if self.gait == "tripod" and self.steps >= 250:
            # Hold the last joint targets and end the experiment if the
            # camera has not confirmed a riser during the bounded approach.
            return self.data.ctrl[self._legs]
        return super()._targets(residual)

    def step(self, action):
        obs, reward, done, truncated, info = super().step(action)
        timeout = self.gait == "tripod" and self.steps >= 251
        info["camera_gate"] = "confirmed" if self.switch_step is not None else "waiting"
        if timeout:
            info["camera_gate"] = "no-confirmed-riser"
        if self.riser is not None:
            info["camera_riser_m"] = self.riser.height_m
            info["camera_range_m"] = self.riser.range_m
        if self.switch_step is not None:
            info["camera_foothold_edge_x_m"] = self.body_motion.terrain.edge_x
        return obs, reward, done, truncated or timeout, info

    def close(self):
        self.camera.close()
        super().close()


def capture_course(course: str, output: Path):
    from PIL import Image, ImageDraw

    env = LocomotionEnv(course, randomize=False)
    camera = CourseCamera(env)
    output.mkdir(parents=True, exist_ok=False)
    samples = []
    panels = []
    try:
        env.reset(seed=42)
        for distance in (0.68, 0.56, 0.44, 0.35):
            env.data.qpos[0] = env.course.edge_x - camera.offset_x - distance
            camera.aim()
            rgb = camera.capture()
            estimate = estimate_riser(rgb, camera.calibration)
            truth = env.course.rise_m * (-1 if env.course.descending else 1)
            samples.append(
                {
                    "range_truth_m": distance,
                    "rise_truth_m": truth,
                    "estimate": asdict(estimate) if estimate else None,
                }
            )
            image = Image.fromarray(rgb)
            if estimate:
                draw = ImageDraw.Draw(image)
                draw.line((0, estimate.top_row, 319, estimate.top_row), fill="#ffb34d", width=2)
                draw.line((0, estimate.base_row, 319, estimate.base_row), fill="#2aeac4", width=2)
            encoded = io.BytesIO()
            image.save(encoded, format="PNG")
            source = base64.b64encode(encoded.getvalue()).decode("ascii")
            measurement = (
                f"Tahmin {100 * estimate.height_m:.1f} cm · Mesafe {100 * estimate.range_m:.0f} cm"
                if estimate
                else "Ölçüm yok"
            )
            panels.append(
                f'<figure><img src="data:image/png;base64,{source}" alt="Ön kamera karesi">'
                f"<figcaption>{measurement}<br>"
                f"Parkur yüksekliği {100 * truth:.0f} cm · Mesafe {100 * distance:.0f} cm"
                "</figcaption></figure>"
            )
    finally:
        camera.close()
        env.close()
    (output / "samples.json").write_text(json.dumps(samples, indent=2), encoding="utf-8")
    path = output / "report.html"
    path.write_text(
        '<!doctype html><html lang="tr"><meta charset="utf-8"><meta name="viewport" '
        'content="width=device-width,initial-scale=1"><title>META-1 RGB basamak ölçümü</title>'
        "<style>body{font:16px system-ui;background:#101822;color:#e8edf4;"
        "margin:2rem auto;padding:0 1rem;max-width:930px}main{display:grid;"
        "grid-template-columns:repeat(auto-fit,minmax(310px,1fr));gap:1rem}"
        "figure{margin:0;background:#202d3a;padding:1rem;border-radius:8px}"
        "img{width:100%;display:block;margin-bottom:.75rem}</style>"
        f"<h1>META-1 · {course}</h1><p>Turuncu üst kenar, yeşil zemin birleşimi. "
        "Parkur ölçüleri yalnızca karşılaştırma için kullanılıyor.</p>"
        f"<main>{''.join(panels)}</main>",
        encoding="utf-8",
    )
    print(path)
