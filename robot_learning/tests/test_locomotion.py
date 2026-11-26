import mujoco
import numpy as np
import pytest

from apex_rl.course_observability import LANE_SPACING, fleet_model
from apex_rl.locomotion import COURSES, LocomotionEnv
from apex_rl.stair_rgb import CameraCalibration, estimate_riser


def test_reference_walk_passes_flat_course():
    env = LocomotionEnv("flat", randomize=False)
    try:
        obs, _ = env.reset(seed=5)
        assert obs.shape == (85,)
        for _ in range(env.episode_steps):
            obs, reward, done, truncated, info = env.step(np.zeros(3))
            assert np.isfinite(obs).all() and np.isfinite(reward)
            if done or truncated:
                break
        assert info["success"]
        assert not info["fallen"]
        assert info["progress_m"] >= 1.2 and info["touchdowns"] >= 6
        assert info["max_clearance_m"] > 0.01
    finally:
        env.close()


@pytest.mark.parametrize("course_name", ["step-05", "stairs-05", "stairs-20", "stairs-down-20"])
def test_course_risers_are_visible_and_collidable(course_name):
    env = LocomotionEnv(course_name)
    try:
        obs, _ = env.reset(seed=23)
        course = COURSES[course_name]
        assert env.observation_space.contains(obs)
        assert env.data.qpos[2] == pytest.approx(env.stand_height + course.start_height)
        for i in range(course.risers):
            geom = env.model.geom(f"course_step_{i + 1}")
            assert geom.type[0] == mujoco.mjtGeom.mjGEOM_BOX and geom.contype[0] > 0
            assert geom.pos[2] + geom.size[2] == pytest.approx(course.height(geom.pos[0]))
        assert 6.8 <= env.pack_voltage_proxy_v <= 8.0
        assert env.model.actuator_forcerange[0, 1] == pytest.approx(
            env.config["joint_torque_limit_nm"]["coxa"] * env.supply_scale
        )
    finally:
        env.close()


def test_descent_reference_walk_and_lane_frames():
    env = LocomotionEnv("down-02", randomize=False)
    try:
        env.reset(seed=1)
        for _ in range(env.episode_steps):
            _, _, done, truncated, info = env.step(np.zeros(3))
            if done or truncated:
                break
        assert info["success"] and not info["fallen"]
        assert np.min(env.data.xpos[env._foot_ids, 0]) > env.course.last_edge
        model, initial = fleet_model("down-02", 3)
        assert model.nq == 3 * initial.size
        assert model.geom("lane2_course_step_1").pos[1] == pytest.approx(2 * LANE_SPACING)
    finally:
        env.close()


@pytest.mark.parametrize("course_name", ["step-05", "down-05"])
def test_body_control_tracks_step_with_independent_footholds(course_name):
    env = LocomotionEnv(course_name, randomize=False, body_control=True)
    try:
        obs, _ = env.reset(seed=5)
        assert obs.shape == (103,)
        mixed_footholds = False
        for _ in range(env.episode_steps):
            obs, reward, done, truncated, info = env.step(np.zeros(6))
            assert env.observation_space.contains(obs) and np.isfinite(reward)
            mixed_footholds |= np.ptp(env.body_motion.anchors[:, 2]) > 0.035
            if done or truncated:
                break
        assert info["success"] and not info["fallen"]
        assert np.min(env.data.xpos[env._foot_ids, 0]) > env.course.last_edge
        assert mixed_footholds and info["body_height_range_m"] > 0.035
    finally:
        env.close()


@pytest.mark.parametrize("course_name", ["hybrid-step-05", "hybrid-down-05"])
def test_tripod_handoff_occurs_on_planted_feet(course_name):
    env = LocomotionEnv(course_name, randomize=True)
    try:
        env.reset(seed=1006100)
        saw_handoff = False
        for _ in range(env.episode_steps):
            contacts = env.body_motion.contacts().sum()
            _, _, done, truncated, info = env.step(np.zeros(6))
            if not saw_handoff and info["gait"] == "wave":
                assert contacts >= 5
                saw_handoff = True
            if done or truncated:
                break
        assert saw_handoff and info["success"]
        assert info["approach_speed_m_s"] > 2 * info["precision_speed_m_s"]
        env.reset(seed=1006100)
        assert env.gait == "tripod" and env.switch_step is None
    finally:
        env.close()


def test_riser_estimate_uses_floor_seam_and_top_edge():
    calibration = CameraCalibration(0.1554, np.deg2rad(20), np.deg2rad(41.3))
    rgb = np.full((180, 320, 3), 120, dtype=np.uint8)
    assert estimate_riser(rgb, calibration) is None
    rgb[49:91] = 35
    result = estimate_riser(rgb, calibration)
    assert result is not None
    assert result.range_m == pytest.approx(0.41, abs=0.03)
    assert result.height_m == pytest.approx(0.076, abs=0.012)


def test_high_narrow_descent_places_rear_leg_first():
    env = LocomotionEnv("hybrid-narrow-down-10")
    try:
        env.reset(seed=1006500)
        for _ in range(500):
            _, _, done, truncated, _ = env.step(np.zeros(6))
            assert not done and not truncated
            if env.body_motion.leg is not None:
                assert env.body_motion.leg == 2
                break
        else:
            pytest.fail("wave gait did not start")
    finally:
        env.close()
