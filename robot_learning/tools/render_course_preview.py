"""Render the META-1 stairs course in the same scene the trainer uses."""

from pathlib import Path

import mujoco
from PIL import Image

from apex_rl.locomotion import LocomotionEnv


def main():
    env = LocomotionEnv("stairs-05", randomize=False)
    env.reset(seed=42)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.83, 0, 0.04]
    camera.azimuth = 102
    camera.elevation = -29
    camera.distance = 2.76
    out = Path(__file__).resolve().parents[1] / "generated/meta1-course.png"
    out.parent.mkdir(exist_ok=True)
    try:
        with mujoco.Renderer(env.model, height=720, width=1280) as renderer:
            renderer.update_scene(env.data, camera=camera)
            Image.fromarray(renderer.render()).save(out)
    finally:
        env.close()
    print(out)


if __name__ == "__main__":
    main()
