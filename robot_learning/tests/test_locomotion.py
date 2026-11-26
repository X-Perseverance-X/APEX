import mujoco
import numpy as np
import pytest

from apex_rl.course_observability import LANE_SPACING, fleet_model
from apex_rl.locomotion import COURSES, LocomotionEnv


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
