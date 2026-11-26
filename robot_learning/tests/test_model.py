import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest

from apex_rl.body_visual import load_body_layout
from apex_rl.contract import LEG_ORDER, load_contract, validate_contract
from apex_rl.env import Meta1Env
from apex_rl.model import BODY_MESH_DIR, LEG_MESH_DIR, build_xml, stand_joint_angles

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
    assert (model.nu, model.nq, model.nv) == (20, 27, 26)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    assert data.ctrl == pytest.approx([*stand_joint_angles(), 0, 0])
    for _ in range(1000):
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all()
    assert data.qpos[2] == pytest.approx(0.118, abs=0.01)
    assert data.ncon >= 6


def test_mark2_mesh_alignment_and_conservative_limits():
    cfg = load_contract()
    cad = json.loads((LEG_MESH_DIR / "manifest.json").read_text(encoding="utf-8"))
    model = mujoco.MjModel.from_xml_string(build_xml())
    for side in ("left", "right"):
        for link in ("mount", "coxa", "femur", "tibia"):
            assert model.mesh(f"{link}_{side}").id >= 0
    assert mujoco.mj_getTotalmass(model) == pytest.approx(cfg["mass_profile"]["design_total_kg"])
    for leg in LEG_ORDER:
        for joint, length in cfg["links_m"].items():
            assert cad["cad_pivot_distances_mm"][joint] == pytest.approx(length * 1000, abs=1)
            geom = model.geom(f"{leg.lower()}_{joint}_cad")
            assert int(geom.contype[0]) == 0 and int(geom.conaffinity[0]) == 0
            torque = cfg["joint_torque_limit_nm"][joint]
            assert model.actuator(f"{leg.lower()}_{joint}_servo").forcerange == pytest.approx(
                [-torque, torque]
            )
    for axis in ("pan", "tilt"):
        servo_size = model.geom(f"camera_{axis}_SG90_case").size
        assert 2 * servo_size == pytest.approx(cfg["servo_case_size_m"]["SG90"])


def test_body_mesh_mounts_follow_correct_camera_joints():
    model = mujoco.MjModel.from_xml_string(build_xml())
    parts = load_body_layout()
    actual = json.loads((BODY_MESH_DIR / "manifest.json").read_text(encoding="utf-8"))
    assert actual["sources"]["Bottom_Plate.stl"]["dimensions_mm"] == [300.0, 110.0, 8.0]
    assert model.nmesh == 8 + len(actual["sources"])
    parents = {"base": "base", "pan": "camera_pan_body", "tilt": "camera_tilt_body"}
    for part in parts:
        geom = model.geom(part["name"])
        assert int(geom.bodyid[0]) == model.body(parents[part["parent"]]).id
        assert int(geom.contype[0]) == 0 and int(geom.conaffinity[0]) == 0
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    stationary = data.geom("pan_fixed_cradle").xpos.copy()
    arm = data.geom("pan_to_tilt_arm").xpos.copy()
    carrier = data.geom("tilt_camera_carrier").xpos.copy()
    data.joint("camera_tilt_joint").qpos[0] = -math.pi / 6
    mujoco.mj_forward(model, data)
    assert data.geom("pan_fixed_cradle").xpos == pytest.approx(stationary)
    assert data.geom("pan_to_tilt_arm").xpos == pytest.approx(arm)
    assert np.linalg.norm(data.geom("tilt_camera_carrier").xpos - carrier) > 0.01
    data.joint("camera_pan_joint").qpos[0] = math.pi / 6
    mujoco.mj_forward(model, data)
    assert data.geom("pan_fixed_cradle").xpos == pytest.approx(stationary)
    assert np.linalg.norm(data.geom("pan_to_tilt_arm").xpos - arm) > 0.01


def test_sensor_frames_and_camera_rotation():
    cfg = load_contract()["sensors"]
    model = mujoco.MjModel.from_xml_string(build_xml())
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    origin = data.body("base").xpos.copy()
    for name in ("imu", "lidar", "tof"):
        assert data.site(f"{name}_site").xpos - origin == pytest.approx(cfg[name]["xyz_m"])
    assert data.site("tof_site").xmat.reshape(3, 3)[:, 2] == pytest.approx([1, 0, 0])
    assert data.cam("front_camera").xpos - origin == pytest.approx(cfg["camera"]["xyz_m"])
    assert -data.cam("front_camera").xmat.reshape(3, 3)[:, 2] == pytest.approx([1, 0, 0])
    for axis in ("pan", "tilt"):
        assert data.body(f"camera_{axis}_body").xpos - origin == pytest.approx(
            cfg["gimbal"][f"{axis}_axis_xyz_m"]
        )
        assert model.jnt_range[model.joint(f"camera_{axis}_joint").id] == pytest.approx(
            np.deg2rad([-45, 45])
        )
    data.joint("camera_pan_joint").qpos[0] = math.pi / 4
    data.joint("camera_tilt_joint").qpos[0] = math.pi / 6
    mujoco.mj_forward(model, data)
    camera_forward = -data.cam("front_camera").xmat.reshape(3, 3)[:, 2]
    assert camera_forward == pytest.approx([math.sqrt(6) / 4, math.sqrt(6) / 4, -0.5])
    assert data.site("tof_site").xmat.reshape(3, 3)[:, 2] == pytest.approx(camera_forward)


def test_tof_measures_forward_obstacle():
    scene = ET.fromstring(build_xml())
    world = scene.find("worldbody")
    world.find("geom[@name='floor']").set("rgba", "0 0 0 0")
    target = ET.SubElement(world, "body", name="tof_test_target", mocap="true", pos="2 0 0")
    ET.SubElement(target, "geom", type="sphere", size="0.02")
    model = mujoco.MjModel.from_xml_string(ET.tostring(scene, encoding="unicode"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    for pan in np.deg2rad([-45, 0, 45]):
        for tilt in np.deg2rad([-45, 0, 45]):
            data.joint("camera_pan_joint").qpos[0] = pan
            data.joint("camera_tilt_joint").qpos[0] = tilt
            mujoco.mj_forward(model, data)
            site = data.site("tof_site")
            data.mocap_pos[0] = site.xpos + 1.02 * site.xmat.reshape(3, 3)[:, 2]
            mujoco.mj_forward(model, data)
            assert data.sensor("forward_tof").data[0] == pytest.approx(1.0)


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
        assert obs.shape == (72,)
        assert np.array_equal(env.data.ctrl[18:], [0, 0])
        assert env.observation_space.contains(obs)
        assert math.isfinite(reward)
        assert not terminated and not truncated
        assert info["contacts"] >= 4
    finally:
        env.close()
