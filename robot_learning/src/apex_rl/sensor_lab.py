"""Simulation-only gaze stabilization and active planar LiDAR inspection."""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass

import mujoco
import numpy as np

from .body_motion import support_target
from .env import _rotation_from_quaternion
from .locomotion import LocomotionEnv


def gaze_angles(local):
    x, y, z = local
    return np.array([math.atan2(y, x), -math.atan2(z, math.hypot(x, y))])


class GazeController:
    """Command both simulated servos from a delayed, noisy chassis attitude."""

    def __init__(self, env, seed, *, stabilize=True):
        self.env = env
        self.stabilize = stabilize
        self.rng = np.random.default_rng(seed)
        self.camera = env.data.camera("front_camera")
        self.target = self.camera.xpos + np.array([3.0, 0.0, 0.08])
        self.ids = [env.model.actuator(f"camera_{axis}_servo").id for axis in ("pan", "tilt")]
        self.bounds = env.model.actuator_ctrlrange[self.ids]
        self.fifo = deque([env.data.qpos[3:7].copy() for _ in range(3)], maxlen=3)
        self.command = gaze_angles(self.target - self.camera.xpos)
        self.filtered = self.command.copy()
        self.bias = self.rng.normal(0, math.radians(0.28), size=2)
        env.data.ctrl[self.ids] = self.command

    def update(self):
        data = self.env.data
        self.fifo.append(data.qpos[3:7].copy())
        if self.stabilize:
            attitude = _rotation_from_quaternion(self.fifo[0])
            aim = gaze_angles(attitude.T @ (self.target - self.camera.xpos))
            aim += self.bias + self.rng.normal(0, math.radians(0.48), size=2)
            self.filtered += 0.6 * (aim - self.filtered)
            delta = math.radians(210) * 0.02
            self.command += np.clip(self.filtered - self.command, -delta, delta)
            self.command = np.clip(self.command, self.bounds[:, 0], self.bounds[:, 1])
        data.ctrl[self.ids] = self.command

    def error(self):
        camera_frame = self.camera.xmat.reshape(3, 3)
        actual = gaze_angles(-camera_frame[:, 2])
        wanted = gaze_angles(self.target - self.camera.xpos)
        diff = (actual - wanted + math.pi) % (2 * math.pi) - math.pi
        return np.degrees(diff)


@dataclass(frozen=True)
class Ray:
    origin: np.ndarray
    direction: np.ndarray
    distance: float


def estimate_lip(rays, floor_z):
    """Fit a repeatedly observed vertical face and bracket its highest hit."""
    observed = [(r, r.origin + r.direction * r.distance) for r in rays if 0 < r.distance < 2.5]
    if len(observed) < 8:
        return None
    points = np.array([p for _, p in observed])
    # A vertical step produces repeat returns at the same forward coordinate,
    # at multiple scan heights; floor intersections spread in X.
    counts = np.count_nonzero(abs(points[:, None, 0] - points[None, :, 0]) <= 0.012, axis=1)
    for i in np.argsort(counts)[::-1]:
        if counts[i] < 7:
            break
        cluster = points[abs(points[:, 0] - points[i, 0]) <= 0.012]
        if np.ptp(cluster[:, 2]) < 0.018 or cluster[:, 2].max() < floor_z + 0.018:
            continue
        edge = float(np.median(cluster[:, 0]))
        last_hit = float(cluster[:, 2].max())
        clear = []
        for r in rays:
            run = (edge - r.origin[0]) / r.direction[0]
            if run <= 0 or (
                r.distance > 0 and r.distance * r.direction[0] + r.origin[0] < edge + 0.016
            ):
                continue
            z = float(r.origin[2] + run * r.direction[2])
            if z >= last_hit:
                clear.append(z)
        if clear:
            lower, upper = last_hit - floor_z, min(clear) - floor_z
            if 0 <= upper - lower <= 0.027:
                # Repeated returns from the horizontal surface immediately
                # behind the lip can tighten the bracket from the face alone.
                x, z = points[:, 0], points[:, 2] - floor_z
                top = z[
                    (x > edge + 0.022)
                    & (x < edge + 0.19)
                    & (z >= lower - 0.005)
                    & (z <= upper + 0.005)
                ]
                if len(top) >= 7 and np.ptp(top) <= 0.009:
                    centre = float(np.median(top))
                    refined_lower = max(lower, centre - 0.004)
                    refined_upper = min(upper, centre + 0.004)
                    if refined_lower <= refined_upper:
                        lower, upper = refined_lower, refined_upper
                return {
                    "edge_x_m": edge,
                    "lower_m": lower,
                    "upper_m": upper,
                    "rise_m": (lower + upper) / 2,
                }
    return None


def planted_pose(env, feet, origin, pitch):
    """Check an inspection pose against the real model joint limits and foot FK."""
    data = env.data
    data.qpos[:3] = origin
    data.qpos[3:7] = (math.cos(pitch / 2), 0, -math.sin(pitch / 2), 0)
    rotation = _rotation_from_quaternion(data.qpos[3:7])
    local = (feet - origin) @ rotation - env.hip
    c, s = np.cos(env.mount), np.sin(env.mount)
    targets = np.column_stack(
        (local[:, 0] * c + local[:, 1] * s, -local[:, 0] * s + local[:, 1] * c, local[:, 2])
    )
    data.qpos[env._leg_positions] = env._solve_ik(targets)
    mujoco.mj_forward(env.model, data)
    error = np.linalg.norm(data.xpos[env._foot_ids] - feet, axis=1).max()
    shell = np.isin(data.contact.geom, env._chassis_geoms).any()
    com = data.subtree_com[env.model.body("base").id, :2]
    projected = support_target(feet[:, :2], com)
    supported = projected is not None and np.linalg.norm(projected - com) < 0.001
    return error <= 0.003 and supported and not shell


def inspect_course(course: str):
    env = LocomotionEnv(course, randomize=False)
    try:
        env.reset(seed=1)
        env.data.qpos[0] = env.course.edge_x - 0.65
        mujoco.mj_forward(env.model, env.data)
        neutral = env.data.qpos[:3].copy()
        feet = env.data.xpos[env._foot_ids].copy()
        floor_z = float(np.median(feet[:, 2]) - env.config["foot_radius_m"])
        env.model.geom_group[[env.model.geom("floor").id, *env._terrain_ids]] = 5
        mask = np.array([0, 0, 0, 0, 0, 1], dtype=np.uint8)
        lidar = env.data.site("lidar_site")
        samples = []
        poses = []
        rng = np.random.default_rng(29)
        assisted_pitch = None
        target = env.data.camera("front_camera").xpos + np.array([0.25, 0.0, 0.39])
        neutral_tilt = math.degrees(gaze_angles(target - env.data.camera("front_camera").xpos)[1])
        for pitch_deg in range(-30, 21, 2):
            pitch = math.radians(pitch_deg)
            for lift in np.linspace(-0.040, 0.042, 16):
                origin = neutral + np.array([0.0, 0.0, lift])
                if not planted_pose(env, feet, origin, pitch):
                    continue
                poses.append((pitch_deg, float(lift)))
                rotation = lidar.xmat.reshape(3, 3)
                for yaw in np.deg2rad([-9, -4, 0, 4, 9]):
                    direction = rotation @ [math.cos(yaw), math.sin(yaw), 0.0]
                    distance = mujoco.mj_ray(
                        env.model, env.data, lidar.xpos, direction, mask, True, -1, None
                    )
                    if distance > 0:
                        distance += rng.normal(0, 0.0015)
                    samples.append(Ray(lidar.xpos.copy(), direction, distance))
                if pitch_deg > 0 and abs(lift) < 0.012:
                    local = rotation.T @ (target - env.data.camera("front_camera").xpos)
                    if abs(gaze_angles(local)[1]) <= math.radians(44):
                        assisted_pitch = (
                            pitch_deg if assisted_pitch is None else min(assisted_pitch, pitch_deg)
                        )
        measured = estimate_lip(samples, floor_z)
        # Authorization is a measured training envelope, independent of IK reach.
        qualified_ascent_m = 0.055
        if measured is None:
            decision = "BİLİNMİYOR · DUR"
        elif measured["upper_m"] > qualified_ascent_m:
            decision = "EĞİTİM SINIRI DIŞI · DUR"
        else:
            decision = "EĞİTİM ARALIĞINDA"
        return {
            "course": course,
            "truth_m": env.course.rise_m,
            "accepted_poses": len(poses),
            "lidar_rays": len(samples),
            "pitch_span_deg": [min(p[0] for p in poses), max(p[0] for p in poses)] if poses else [],
            "estimated": measured,
            "qualified_ascent_m": qualified_ascent_m,
            "decision": decision,
            "camera_body_assist_deg": assisted_pitch,
            "camera_neutral_request_deg": neutral_tilt,
        }
    finally:
        env.close()


def compare_gaze(course, seed, stabilized):
    env = LocomotionEnv(course)
    try:
        env.reset(seed=seed)
        gimbal = GazeController(env, seed, stabilize=stabilized)
        base = env.model.body("base").id
        trace = []
        for _ in range(env.episode_steps):
            t = env.steps * 0.02
            # Small periodic disturbance in the walking physics. The controller
            # sees its effect via the delayed attitude, not the forcing signal.
            env.data.xfrc_applied[base, 3:] = [
                0.55 * math.sin(2 * math.pi * 3.7 * t),
                1.1 * math.sin(2 * math.pi * 3.0 * t),
                0.52 * math.sin(2 * math.pi * 2.7 * t),
            ]
            gimbal.update()
            _, _, done, truncated, info = env.step(np.zeros(env.action_space.shape))
            if env.steps >= 45:
                trace.append(gimbal.error().tolist())
            if done or truncated:
                break
        rms = np.sqrt(np.mean(np.square(trace), axis=0))
        return {
            "samples": len(trace),
            "pitch_rms_deg": float(rms[1]),
            "yaw_rms_deg": float(rms[0]),
            "walk_success": info["success"],
            "trace_deg": trace,
        }
    finally:
        env.close()


def sensor_report(output):
    output.mkdir(parents=True, exist_ok=False)
    scans = [
        inspect_course(course) for course in ("hybrid-step-05", "hybrid-lab-up-07", "stairs-20")
    ]
    comparison = {
        key: compare_gaze("hybrid-down-05", 1006100, stabilized)
        for key, stabilized in (("fixed", False), ("stabilized", True))
    }
    result = {"scans": scans, "gaze": comparison}
    (output / "sensors.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    scan_rows = []
    for row in scans:
        estimate = row["estimated"]
        height = f"{100 * estimate['rise_m']:.1f} cm" if estimate else "Ölçüm yok"
        scan_rows.append(
            f"<tr><td>{100 * row['truth_m']:.0f} cm</td><td>{height}</td>"
            f"<td>{row['accepted_poses']}</td><td>{row['decision']}</td></tr>"
        )
    chart = []
    for key, color in (("fixed", "#d45b45"), ("stabilized", "#087c80")):
        trace = comparison[key]["trace_deg"]
        points = " ".join(
            f"{20 + 700 * i / len(trace):.1f},{85 - np.clip(pair[1], -18, 18) * 3.5:.1f}"
            for i, pair in enumerate(trace)
            if i % 4 == 0
        )
        chart.append(
            f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.5"/>'
        )
    fixed, stabil = comparison["fixed"], comparison["stabilized"]
    page = output / "report.html"
    page.write_text(
        '<!doctype html><html lang="tr"><meta charset="utf-8"><title>META-1 · Sensör ölçümü</title>'
        "<style>body{font:16px system-ui;background:#eff3f6;color:#132739;max-width:900px;"
        "margin:30px auto;padding:0 16px}table{border-collapse:collapse}td,th{padding:8px;"
        "text-align:left;border-bottom:1px solid #bbc9d4}svg{width:100%;background:white}</style>"
        "<h1>META-1 sensör deneyleri</h1><h2>Sabit hedefe bakış</h2>"
        f"<p>Dikey açı hatası (RMS). Sabit servo {fixed['pitch_rms_deg']:.2f}°, "
        f"dengelemeli servo {stabil['pitch_rms_deg']:.2f}°.</p>"
        '<svg viewBox="0 0 740 170" role="img" aria-label="Yürüyüşte dikey kamera hatası">'
        '<path d="M20 85H720" stroke="#aab9c5"/>'
        f"{''.join(chart)}</svg><p>Kırmızı sabit servo, yeşil dengelemeli servo.</p>"
        "<h2>Gövde konumu değişirken 2B LiDAR</h2><table><thead><tr><th>Gerçek</th>"
        "<th>Tahmin</th><th>Uygun duruşlar</th><th>Karar</th></tr></thead>"
        f"<tbody>{''.join(scan_rows)}</tbody></table>"
        "<p>Tarama testinde ayaklar sabitken kinematik olarak erişilebilir gövde "
        "pozlarından kesit alındı. Karar, ölçüm aralığının üst sınırına göre verildi.</p>"
        f"<p>Yukarı bakış deneyi. Gimbalın düz gövdedeki isteği "
        f"{abs(scans[0]['camera_neutral_request_deg']):.1f}°. "
        f"Servo sınırına dönmek için gövde {scans[0]['camera_body_assist_deg']}° "
        "öne yükseltildi.</p></html>",
        encoding="utf-8",
    )
    print(page)
