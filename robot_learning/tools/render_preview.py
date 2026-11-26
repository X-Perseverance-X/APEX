"""Render the assembled robot and its mounting details offline."""

from pathlib import Path

import mujoco
from PIL import Image

from apex_rl.model import build_xml


def main() -> None:
    model = mujoco.MjModel.from_xml_string(build_xml())
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    for _ in range(1000):
        mujoco.mj_step(model, data)
    output = Path(__file__).resolve().parents[1] / "generated"
    output.mkdir(parents=True, exist_ok=True)
    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        for camera, filename in (
            ("overview", "meta1-stand.png"),
            ("camera_detail", "meta1-camera.png"),
            ("rear_detail", "meta1-rear.png"),
            ("underside", "meta1-underside.png"),
        ):
            if camera == "underside":
                model.geom("floor").rgba[3] = 0
                # The regular scene is lit from above. Aim the inspection
                # light up at the model for the view from below.
                model.light_dir[0] = [-0.3, 0.4, 2]
                mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            path = output / filename
            Image.fromarray(renderer.render()).save(path)
            print(path)


if __name__ == "__main__":
    main()
