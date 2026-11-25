import math
import re
from pathlib import Path

import mujoco
import numpy as np
import pytest

from apex_rl.contract import LEG_ORDER, load_contract, validate_contract
from apex_rl.env import Meta1Env
from apex_rl.model import build_xml, stand_joint_angles

ROOT = Path(__file__).resolve().parents[2]


def test_geometry_matches_current_firmware_constants():
    source = (ROOT / "firmware/v57_spine_safe_boot/v57_spine_safe_boot.ino").read_text(
        encoding="utf-8"
    )
    cfg = load_contract()
    for name, firmware_name in (("coxa", "l"), ("femur", "a"), ("tibia", "b")):
        value = float(re.search(rf"const float {firmware_name} = ([\d.]+);", source).group(1))
        assert cfg["links_m"][name] == pytest.approx(value / 1000)
    for axis in "XY":
        values = re.search(rf"const float bodyOffset{axis}\[6\] = \{{([^}}]+)\}};", source).group(1)
        firmware = [float(value) / 1000 for value in values.split(",")]
        assert [cfg["hip_xyz_m"][leg]["XYZ".index(axis)] for leg in LEG_ORDER] == firmware
    assert "stepLift = 65.0" in source  # highlights stale 50 mm ROS parameter file


def test_contract_rejects_wrong_units_or_mirror():
    cfg = load_contract()
    cfg["units"] = "mm"
    with pytest.raises(ValueError):
        validate_contract(cfg)
    cfg = load_contract()
    cfg["hip_xyz_m"]["RF"][1] = 0.098
    with pytest.raises(ValueError):
        validate_contract(cfg)


def test_mjcf_compiles_and_stands_on_six_contacts():
    model = mujoco.MjModel.from_xml_string(build_xml())
    assert model.nu == 18 and model.nq == 25 and model.nv == 24
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    assert data.ctrl == pytest.approx(stand_joint_angles())
    for _ in range(1000):
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all()
    assert data.qpos[2] == pytest.approx(0.118, abs=0.01)
    assert data.ncon >= 6


def test_gymnasium_step_and_deterministic_seed():
    env = Meta1Env(task="velocity", domain_randomization=True)
    try:
        obs_a, _ = env.reset(seed=123)
        friction_a = env.model.geom_friction[0, 0]
        obs_b, _ = env.reset(seed=123)
        assert np.array_equal(obs_a, obs_b)
        assert env.model.geom_friction[0, 0] == friction_a
        for _ in range(10):
            obs, reward, terminated, truncated, info = env.step(np.zeros(18, dtype=np.float32))
        assert env.observation_space.contains(obs)
        assert math.isfinite(reward)
        assert not terminated and not truncated
        assert info["contacts"] >= 4
    finally:
        env.close()
