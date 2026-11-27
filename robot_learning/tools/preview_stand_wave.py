"""Measure the five-foot stance and replay the right-front wave in MuJoCo."""

from __future__ import annotations

import argparse
import json
import math
from contextlib import nullcontext
from pathlib import Path

import mujoco
import numpy as np

from apex_rl.ascent_planner import hull_margin
from apex_rl.contract import LEG_ORDER
from apex_rl.env import Meta1Env


def run(output: Path | None = None, *, seed: int = 42, randomize: bool = False) -> dict:
    if output:
        from PIL import Image, ImageDraw

    env = Meta1Env(domain_randomization=randomize)
    try:
        env.reset(seed=seed)
        dt = env.control_steps * env.model.opt.timestep
        foot_ids = [env.model.body(f"{name.lower()}_foot").id for name in LEG_ORDER]
        base_id = env.model.body("base").id
        radius = env.config["foot_radius_m"]
        camera = mujoco.MjvCamera()
        camera.lookat[:] = (0.04, 0, 0.10)
        camera.distance, camera.azimuth, camera.elevation = 1.12, 128, -25
        scene = mujoco.Renderer(env.model, 300, 520) if output else nullcontext()
        frames, margins, gaps = [], [], []
        initial = None
        minimum_support = 5
        with scene as renderer:
            for step in range(round(8 / dt)):
                t = step * dt
                if step == round(1 / dt):
                    initial = env.data.xpos[foot_ids[0]].copy()
                u = min(1.0, max(0.0, min(t - 1, 7 - t)))
                lift = u * u * (3 - 2 * u)
                envelope = min(1.0, max(0.0, min(2 * (t - 2), 2 * (6 - t))))
                wave = envelope * math.sin(2 * math.pi * (t - 2) / 1.8)
                target = env.nominal.copy()
                target[:3] += np.deg2rad((4 * wave, 14 * lift, 0))
                env._apply_joint_targets(target)
                feet = env.data.xpos[foot_ids]
                contacts = env._contacts().astype(bool)
                if t >= 1:
                    minimum_support = min(minimum_support, int(contacts[1:].sum()))
                if 2 <= t < 6:
                    gaps.append(float(feet[0, 2] - radius))
                    margins.append(hull_margin(feet[1:, :2], env.data.subtree_com[base_id, :2]))
                if output and step % 3 == 0:
                    renderer.update_scene(env.data, camera=camera)
                    frame = Image.fromarray(renderer.render())
                    draw = ImageDraw.Draw(frame)
                    draw.rectangle((0, 0, 520, 22), fill="#14181c")
                    feet_label = " ".join(
                        name if touch else "--" for name, touch in zip(LEG_ORDER, contacts, strict=True)
                    )
                    draw.text((8, 5), f"{t:4.1f}s  |  {feet_label}", fill="white")
                    frames.append(frame)
        if output:
            output.parent.mkdir(parents=True, exist_ok=True)
            frames[0].save(output, save_all=True, append_images=frames[1:], duration=60, loop=0)
        return {
            "minimum_other_contacts": minimum_support,
            "minimum_swing_clearance_mm": round(1000 * min(gaps), 2),
            "minimum_support_margin_mm": round(1000 * min(margins), 2),
            "return_error_mm": round(1000 * float(np.linalg.norm(feet[0] - initial)), 2),
            "final_contacts": int(contacts.sum()),
        }
    finally:
        env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="write an animated GIF")
    print(json.dumps(run(parser.parse_args().output), indent=2))
