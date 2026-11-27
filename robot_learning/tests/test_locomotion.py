import mujoco
import numpy as np
import pytest

from apex_rl.ascent_pose import KNEE_CLEARANCE_LIMIT, KNEE_COMMAND_MIN, KNEE_STOP, attitude_matrix
from apex_rl.course_observability import LANE_SPACING, fleet_model
from apex_rl.leg_contacts import OVERLAP_LIMIT_M
from apex_rl.locomotion import COURSES, LocomotionEnv
from apex_rl.sensor_lab import inspect_course
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


@pytest.mark.parametrize(
    ("course", "rise", "allowed"),
    [("hybrid-step-05", 0.05, True), ("hybrid-lab-up-07", 0.07, False), ("stairs-20", 0.20, False)],
)
def test_tilted_lidar_estimates_lip_and_rejects_unqualified_climb(course, rise, allowed):
    result = inspect_course(course)
    assert result["estimated"]["rise_m"] == pytest.approx(rise, abs=0.005)
    assert (result["decision"] == "EĞİTİM ARALIĞINDA") is allowed
    assert abs(result["camera_neutral_request_deg"]) > 45
    assert result["camera_body_assist_deg"] == 10


@pytest.mark.parametrize("course", ["hybrid-contact-down-10", "hybrid-contact-down-15"])
def test_descent_with_solid_cad_links_clears_stair(course):
    env = LocomotionEnv(course)
    try:
        env.reset(seed=1007470)
        assert env.model.npair > 0
        for _ in range(env.episode_steps):
            _, _, done, truncated, info = env.step(np.zeros(6))
            if done or truncated:
                break
        assert info["success"] and not info["fallen"]
        assert 0 < info["interleg_peak_m"] <= OVERLAP_LIMIT_M
        assert info["rear_step_peak_m"] <= OVERLAP_LIMIT_M
        assert info["interleg_overlap_samples"] == info["rear_step_overlap_samples"] == 0
        assert np.min(env.data.xpos[env._foot_ids, 0]) > env.course.last_edge
    finally:
        env.close()


def test_playback_loads_course_with_named_contact_pairs():
    model, initial = fleet_model("hybrid-contact-down-15", 2)
    assert model.nq == 2 * initial.size
    assert model.npair == 0


def test_five_centimetre_ascent_finishes_with_all_feet_over_riser():
    env = LocomotionEnv("hybrid-contact-up-05")
    try:
        env.reset(seed=1007750)
        for _ in range(env.episode_steps):
            _, _, done, truncated, info = env.step(np.zeros(6))
            if done or truncated:
                break
        assert info["success"] and not info["fallen"]
        assert np.min(env.data.xpos[env._foot_ids, 0]) > env.course.last_edge
        assert max(info["interleg_peak_m"], info["rear_step_peak_m"]) <= OVERLAP_LIMIT_M
    finally:
        env.close()


@pytest.mark.parametrize("course_name", ["hybrid-rubble-01", "hybrid-rubble-02"])
def test_rubble_height_map_matches_collision_geometry_and_offset_goal(course_name):
    env = LocomotionEnv(course_name)
    try:
        obs, _ = env.reset(seed=1)
        assert env.observation_space.contains(obs)
        assert len(env._terrain_ids) == len(env.course.blocks)
        for geom_id in env._terrain_ids:
            geom = env.model.geom(geom_id)
            x, y, z = geom.pos
            assert env.course.height(x, y) == pytest.approx(z + geom.size[2])
        assert abs(env.course.goal_y) > 0.01
        assert env.course.route_y(env.course.goal + 0.01) == pytest.approx(env.course.goal_y)
        assert env.model.geom("goal_marker").pos[1] == pytest.approx(env.course.goal_y)
    finally:
        env.close()


def test_attitude_matrix_positive_pitch_raises_robot_nose():
    pose = attitude_matrix(0.08, 0.20, -0.11)
    np.testing.assert_allclose(pose.T @ pose, np.eye(3), atol=1e-12)
    assert np.linalg.det(pose) == pytest.approx(1.0)
    assert pose[2, 0] > 0 and pose[2, 1] > 0 and pose[1, 0] < 0


def test_knee_guard_retains_clearance_during_five_centimetre_ascent():
    env = LocomotionEnv("hybrid-pose-up-05")
    try:
        obs, _ = env.reset(seed=1008170)
        assert obs.shape == (105,) and env.action_space.shape == (8,)
        assert env.model.joint("rf_tibia_joint").range[0] == pytest.approx(KNEE_STOP)
        assert env.joint_low[2] == pytest.approx(KNEE_COMMAND_MIN)
        for _ in range(env.episode_steps):
            _, _, done, truncated, info = env.step(np.zeros(8))
            if done or truncated:
                break
        assert info["success"] and np.deg2rad(info["min_knee_deg"]) > KNEE_CLEARANCE_LIMIT
        assert np.min(env.data.xpos[env._foot_ids, 0]) > env.course.last_edge
    finally:
        env.close()


def test_attitude_policy_controls_roll_and_yaw_independently():
    env = LocomotionEnv("hybrid-steep-up-10")
    try:
        env.reset(seed=1008210)
        env.gait = "wave"
        action = np.zeros(8)
        action[6], action[7] = 1, -1
        for _ in range(15):
            obs, _, _, _, _ = env.step(action)
        assert env.body_motion.roll > 0 and env.body_motion.yaw < 0
        np.testing.assert_allclose(obs[-2:], [env.body_motion.roll, env.body_motion.yaw])
    finally:
        env.close()
