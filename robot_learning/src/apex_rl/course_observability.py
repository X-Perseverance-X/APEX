"""Report PPO iterations and replay recorded worker poses in a free camera."""

from __future__ import annotations

import html
import json
import time
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

import mujoco
import numpy as np

from .locomotion import COURSES, LocomotionEnv

POSE_FILE = "tracks.jsonl"
METRICS_FILE = "iterations.jsonl"
LANE_SPACING = 1.65


def training_recorder(stage: Path):
    from stable_baselines3.common.callbacks import BaseCallback

    class Recorder(BaseCallback):
        def __init__(self):
            super().__init__()
            self.latest = {}
            self.completed = deque(maxlen=100)
            self.iteration = 0
            self.started = time.monotonic()
            self.prev_steps = 0

        def _on_training_start(self):
            self.prev_steps = self.num_timesteps

        def _on_step(self):
            for worker, info in enumerate(self.locals["infos"]):
                if "pose" in info:
                    self.latest[worker] = {
                        key: info[key]
                        for key in (
                            "pose",
                            "episode_id",
                            "episode_step",
                            "progress_m",
                            "touchdowns",
                            "fallen",
                            "success",
                            "pack_voltage_proxy_v",
                            "body_z_m",
                            "body_target_z_m",
                            "supporting_feet",
                            "gait",
                            "approach_speed_m_s",
                            "precision_speed_m_s",
                            "min_knee_deg",
                            "peak_upper_contacts",
                            "forward_transfer_m",
                        )
                        if key in info
                    }
                if "episode" in info:
                    self.completed.append(info)
            if self.latest and (self.n_calls % 5 == 0):
                row = {"tick": self.n_calls, "workers": self.latest}
                with (stage / POSE_FILE).open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(row, separators=(",", ":")) + "\n")
            return True

        def _on_rollout_end(self):
            self.iteration += 1
            now = time.monotonic()
            row = {
                "iteration": self.iteration,
                "optimizer_epochs": self.model._n_updates,
                "transitions": self.num_timesteps,
                "transitions_s": (self.num_timesteps - self.prev_steps)
                / max(now - self.started, 1e-3),
                "episodes_in_window": len(self.completed),
            }
            self.started, self.prev_steps = now, self.num_timesteps
            if self.completed:
                for field in (
                    "success",
                    "fallen",
                    "progress_m",
                    "mean_forward_m_s",
                    "slip_m",
                    "mechanical_work_j",
                    "max_clearance_m",
                    "body_height_range_m",
                    "max_lateral_com_m",
                    "rear_riser_contact_steps",
                    "min_knee_deg",
                    "peak_upper_contacts",
                    "forward_transfer_m",
                ):
                    if field in self.completed[-1]:
                        row[field] = float(np.mean([episode[field] for episode in self.completed]))
                row["episode_reward"] = float(np.mean([i["episode"]["r"] for i in self.completed]))
            for name, value in row.items():
                self.logger.record(f"parkour/{name}", value)
            with (stage / METRICS_FILE).open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(row) + "\n")

    return Recorder()


def read_records(path: Path):
    if not path.exists():
        return
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.endswith("\n"):
                yield json.loads(line)


def fleet_model(course: str, lanes: int):
    env = LocomotionEnv(course, randomize=False)
    try:
        env.reset(seed=42)
        initial = env.data.qpos.copy()
        root = ET.fromstring(env._model_xml())
    finally:
        env.close()
    for tag in ("keyframe", "sensor", "actuator"):
        root.remove(root.find(tag))
    pairs = root.find("contact")
    if pairs is not None:
        root.remove(pairs)
    world = root.find("worldbody")
    shared = [node for node in world if node.tag == "light" or node.get("name") == "floor"]
    scenery = [ET.tostring(node) for node in world if node not in shared and node.tag != "camera"]
    for node in list(world):
        if node not in shared:
            world.remove(node)
    world.find("geom[@name='floor']").set("size", f"8 {max(4, lanes * LANE_SPACING)} 0.1")
    for i in range(lanes):
        frame = ET.SubElement(world, "frame", pos=f"0 {i * LANE_SPACING} 0")
        for encoded in scenery:
            component = ET.fromstring(encoded)
            for item in component.iter():
                if "name" in item.attrib:
                    item.set("name", f"lane{i}_{item.attrib['name']}")
            frame.append(component)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    return model, initial


def watch_stage(stage: Path, *, live: bool):
    from mujoco import viewer

    metadata = json.loads((stage / "manifest.json").read_text(encoding="utf-8"))
    lanes = int(metadata["envs"])
    model, initial = fleet_model(metadata["course"], lanes)
    data = mujoco.MjData(model)
    lane_states = data.qpos.reshape(lanes, -1)
    lane_states[:] = initial
    offsets = np.arange(lanes) * LANE_SPACING
    lane_states[:, 1] += offsets
    mujoco.mj_forward(model, data)
    path = stage / POSE_FILE
    if not live and not path.exists():
        raise ValueError(f"No worker recordings found in {path}")
    key_events = deque()
    # Keep the last 120 simulated seconds for replay. The complete recording remains on disk.
    frames = deque(maxlen=1200)
    file_pos = 0
    index = -1
    selected = None
    follow = False
    paused = False
    latest = {}
    metric_path = stage / METRICS_FILE
    metric_stamp = None
    training_status = ""
    next_poll = 0.0
    next_metric_poll = 0.0
    next_frame = 0.0
    with viewer.launch_passive(model, data, key_callback=key_events.append) as window:
        with window.lock():
            window.cam.lookat[:] = [0.8, float(offsets.mean()), 0.10]
            window.cam.distance = max(2.6, lanes * 1.14)
            window.cam.elevation = -38
            window.cam.azimuth = 125
        while window.is_running():
            now = time.monotonic()
            if now >= next_metric_poll and metric_path.exists():
                stamp = metric_path.stat().st_mtime_ns
                if stamp != metric_stamp:
                    metric = deque(read_records(metric_path), maxlen=1)
                    if metric:
                        row = metric[0]
                        training_status = (
                            f"Iteration {row['iteration']} | transitions {row['transitions']:,}"
                            f" | {row['transitions_s']:.0f}/s"
                        )
                        if "optimizer_epochs" in row:
                            training_status += f" | PPO epochs {row['optimizer_epochs']}"
                    metric_stamp = stamp
                next_metric_poll = now + 1.0
            if (live or not frames) and now >= next_poll and path.exists():
                with path.open("rb") as stream:
                    stream.seek(file_pos)
                    for line in stream:
                        if not line.endswith(b"\n"):
                            break
                        frames.append(json.loads(line))
                        file_pos += len(line)
                next_poll = now + 0.20
            with window.lock():
                while key_events:
                    key = key_events.popleft()
                    if 49 <= key <= min(57, 48 + lanes) or key in (91, 93):
                        if key in (91, 93):
                            selected = ((selected or 0) + (1 if key == 93 else -1)) % lanes
                        else:
                            selected = key - 49
                        follow = True
                        window.cam.distance = 1.75
                    elif key == ord("F"):
                        selected = 0 if selected is None else selected
                        follow = not follow
                    elif key == ord("O"):
                        selected, follow = None, False
                        window.cam.lookat[:] = [0.8, float(offsets.mean()), 0.10]
                        window.cam.distance = max(2.6, lanes * 1.14)
                    elif key == ord("P"):
                        paused = not paused
                    elif key == ord("R") and not live:
                        index = -1
                if frames and not paused and now >= next_frame:
                    index = len(frames) - 1 if live else (index + 1) % len(frames)
                    latest = frames[index]["workers"]
                    for worker, row in latest.items():
                        i = int(worker)
                        lane_states[i] = row["pose"]
                        lane_states[i, 1] += offsets[i]
                    mujoco.mj_forward(model, data)
                    next_frame = now + 0.10
                if follow and selected is not None:
                    window.cam.lookat[:] = lane_states[selected, :3]
            lines = []
            for worker, row in latest.items():
                state = "PASS" if row["success"] else ("FALL" if row["fallen"] else "RUN")
                marker = ">" if selected == int(worker) else " "
                body_status = (
                    f"  Z {100 * row['body_z_m']:.1f}/{100 * row['body_target_z_m']:.1f} cm"
                    f"  support {row['supporting_feet']}/6"
                    if "body_z_m" in row
                    else ""
                )
                lines.append(
                    f"{marker}{int(worker) + 1}  {state}  ep {row['episode_id'] + 1}  "
                    f"{row['progress_m']:.2f} m  landings {row['touchdowns']}{body_status}"
                    + (f"  {row['gait'].upper()}" if "gait" in row else "")
                    + (f"  knee {row['min_knee_deg']:.1f} deg" if "min_knee_deg" in row else "")
                    + (
                        f"  upper {row['peak_upper_contacts']}/6"
                        if "peak_upper_contacts" in row
                        else ""
                    )
                )
            replay_help = "" if live else " | R replay"
            window.set_texts(
                (
                    None,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    "Workers",
                    "\n".join(([training_status] if training_status else []) + lines)
                    + "\n1-9 / [ ] select | F follow | O overview | P pause"
                    + replay_help,
                )
            )
            window.sync()
            time.sleep(0.025)


def _chart(rows: list[dict], key: str, title: str) -> str:
    points = [(row["iteration"], row[key]) for row in rows if key in row]
    if not points:
        return f"<section><h3>{title}</h3><p>Tamamlanan bölüm bekleniyor.</p></section>"
    lo, hi = min(y for _, y in points), max(y for _, y in points)
    if key in ("success", "fallen"):
        lo, hi = 0, 1
    elif hi == lo:
        lo, hi = lo - 0.01, hi + 0.01
    xmin, xmax = points[0][0], max(points[0][0] + 1, points[-1][0])
    xy = " ".join(
        f"{38 + 275 * (x - xmin) / (xmax - xmin):.1f},{97 - 83 * (y - lo) / (hi - lo):.1f}"
        for x, y in points
    )
    latest = f"{points[-1][1]:.0%}" if key in ("success", "fallen") else f"{points[-1][1]:.3g}"
    return (
        f"<section><h3>{title}<small>{latest}</small></h3>"
        '<svg viewBox="0 0 330 118" role="img">'
        '<path d="M38 14V97H313" fill="none" stroke="#65768a"/>'
        f'<text x="1" y="16">{hi:.3g}</text><text x="1" y="99">{lo:.3g}</text>'
        f'<polyline points="{xy}" fill="none" stroke="#0f9a86" stroke-width="2"/>'
        f'<text x="38" y="113">İterasyon {points[0][0]}–{points[-1][0]}</text></svg></section>'
    )


def write_report(run: Path) -> Path:
    blocks = []
    for stage in (run / name for name in COURSES if (run / name / "manifest.json").exists()):
        metadata = json.loads((stage / "manifest.json").read_text(encoding="utf-8"))
        body_control = metadata.get("body_control", False)
        rows = list(read_records(stage / METRICS_FILE))
        result = stage / "evaluation.json"
        title = html.escape(stage.name)
        text = "Eğitim / değerlendirme sürüyor."
        detail = ""
        if result.exists():
            data = json.loads(result.read_text(encoding="utf-8"))
            p, b = data["policy"], data["body_reference" if body_control else "tripod"]
            verdict = "GEÇTİ" if data["passed"] else "EŞİK GEÇİLMEDİ"
            text = (
                f"<b>{verdict}</b> · PPO {p['successes']}/{p['episodes']} · "
                f"referans {b['successes']}/{b['episodes']} · "
                f"düşme {p['falls']} · gövde teması {p['chassis_hits']} · "
                f"ortalama ilerleme {p['mean_progress_m']:.2f} m"
            )
            if body_control:
                text += (
                    f" · ort. gövde Z aralığı {100 * p['body_height_range_m']:.1f} cm"
                    f" · ort. azami yanal COM sapması {100 * p['max_lateral_com_m']:.1f} cm"
                )
            if stage.name.startswith("hybrid-"):
                approach = np.mean([row["approach_speed_m_s"] for row in p["outcomes"]])
                precision = np.mean([row["precision_speed_m_s"] for row in p["outcomes"]])
                text += (
                    f" · yaklaşma {100 * approach:.1f} cm/s"
                    f" · hassas yürüyüş {100 * precision:.1f} cm/s"
                )
                if "rear_riser_contact_steps" in p:
                    text += f" · arkada kenar temaslı örnek {p['rear_riser_contact_steps']:.1f}"
                if "interleg_peak_m" in p:
                    text += (
                        f" · bacak örtüşmesi {1000 * p['interleg_peak_m']:.2f} mm"
                        f" · arka bacak / basamak {1000 * p['rear_step_peak_m']:.2f} mm"
                    )
            if "min_knee_deg" in p:
                roll, pitch, yaw = p["peak_target_rpy_deg"]
                text += (
                    f" · en kapalı diz {p['min_knee_deg']:.1f}°"
                    f" · tepe R/P/Y hedefi {roll:.1f}° / {pitch:.1f}° / {yaw:.1f}°"
                )
            if "forward_transfer_m" in p:
                text += (
                    f" · üst temas en fazla {p['peak_upper_contacts']}/6"
                    f" · öne ağırlık merkezi hedefi {1000 * p['forward_transfer_m']:.0f} mm"
                )
            if "mean_riser_contact_s" in p:
                text += (
                    f" · dik yüz ortalama teması {p['mean_riser_contact_s']:.2f} s"
                    f" · sert uzuv/basamak {1000 * p['leg_stair_peak_m']:.2f} mm"
                    f" · yumuşak taban/üst yüzey {1000 * p['sole_top_peak_m']:.2f} mm"
                )
            body = "".join(
                "<tr>"
                f"<td>{row['seed']}</td>"
                f"<td>{'Geçti' if row['success'] else 'Düşme' if row['fallen'] else 'Tamamlanmadı'}</td>"
                f"<td>{row['progress_m']:.2f}</td><td>{row['mean_forward_m_s']:.2f}</td>"
                f"<td>{row['touchdowns']}</td><td>{100 * row['max_clearance_m']:.1f}</td>"
                "</tr>"
                for row in p["outcomes"]
            )
            detail = (
                "<details><summary>Ayrı değerlendirme bölümleri</summary>"
                "<table><thead><tr><th>Tohum</th><th>Sonuç</th><th>İlerleme, m</th>"
                "<th>Hız, m/s</th><th>Ayak teması</th><th>Açıklık, cm</th></tr></thead>"
                f"<tbody>{body}</tbody></table></details>"
            )
            recheck = stage / "recheck.json"
            if recheck.exists():
                verify = json.loads(recheck.read_text(encoding="utf-8"))
                text += (
                    f" · ayrı tekrar {verify['successes']}/{verify['episodes']}"
                    f", {verify['falls']} düşme"
                )
        charts = "".join(
            _chart(rows, key, label)
            for key, label in (
                ("success", "Başarı oranı"),
                ("fallen", "Düşme oranı"),
                ("episode_reward", "Bölüm ödülü"),
                ("transitions_s", "Geçiş / saniye"),
                ("progress_m", "İlerleme, m"),
                ("max_clearance_m", "Ayak açıklığı, m"),
                ("body_height_range_m", "Gövde Z aralığı, m"),
                ("max_lateral_com_m", "Azami yanal COM sapması, m"),
                ("optimizer_epochs", "Tamamlanan PPO epoch"),
                ("rear_riser_contact_steps", "Arka ayak kenar teması, örnek"),
                ("min_knee_deg", "En kapalı diz, °"),
                ("peak_upper_contacts", "Üst yüzeyde en çok ayak"),
                ("forward_transfer_m", "Öne ağırlık merkezi hedefi, m"),
            )
            if body_control or key not in ("body_height_range_m", "max_lateral_com_m")
            if key
            not in (
                "optimizer_epochs",
                "rear_riser_contact_steps",
                "min_knee_deg",
                "peak_upper_contacts",
                "forward_transfer_m",
            )
            or any(key in row for row in rows)
        )
        sample = (
            f" · {rows[-1]['iteration']} iterasyon · {rows[-1]['transitions']:,} toplam geçiş"
            if rows
            else ""
        )
        blocks.append(
            f"<article><h2>{title}</h2><p>{text}{sample}</p><div>{charts}</div>{detail}</article>"
        )
    stress = run / "stress.json"
    if stress.exists():
        results = json.loads(stress.read_text(encoding="utf-8"))
        rows = "".join(
            f"<tr><td>{html.escape(row['course'])}</td><td>{row['successes']}/{row['episodes']}</td>"
            f"<td>{row['falls']}</td><td>{row['mean_progress_m']:.2f}</td></tr>"
            for row in results
        )
        blocks.append(
            "<article><h2>Referans yürüyüşle zorlu parkur taraması</h2>"
            "<p>Bu koşular PPO müfredatına geçiş sağlamaz.</p><table>"
            "<thead><tr><th>Parkur</th><th>Başarı</th><th>Düşme</th><th>İlerleme, m</th></tr></thead>"
            f"<tbody>{rows}</tbody></table></article>"
        )
    path = run / "report.html"
    path.write_text(
        '<!doctype html><html lang="tr"><meta charset="utf-8"><meta name="viewport" '
        'content="width=device-width,initial-scale=1"><title>META-1 Eğitim Raporu</title>'
        "<style>body{font:16px system-ui;background:#f3f6f9;color:#182b41;margin:32px auto;"
        "padding:0 20px;max-width:1140px}article{margin:30px 0}article div{display:grid;"
        "grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:12px}section{background:white;"
        "padding:12px;border-radius:8px}h3{margin:0 0 12px;font-size:15px}small{float:right;"
        "color:#0b7668}svg text{font-size:10px;fill:#405568}"
        "details{margin:12px 0}td,th{padding:6px 12px;text-align:left}</style>"
        "<h1>META-1 · Eğitim raporu</h1><p>Grafikler son 100 bitmiş eğitim bölümünü izler. "
        "Aşama başlığındaki sonuçlar, eğitimden ayrı değerlendirme tohumlarına aittir.</p>"
        + "".join(blocks)
        + "</html>",
        encoding="utf-8",
    )
    return path
