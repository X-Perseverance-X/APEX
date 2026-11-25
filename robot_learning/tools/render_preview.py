"""Render one offline stand frame for visual geometry inspection."""

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
    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        renderer.update_scene(data, camera="overview")
        image = renderer.render().copy()
    output = Path(__file__).resolve().parents[1] / "generated/meta1-stand.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image).save(output)
    print(output)


if __name__ == "__main__":
    main()
