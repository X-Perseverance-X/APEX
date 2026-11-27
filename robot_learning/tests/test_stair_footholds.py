import numpy as np
import pytest

from apex_rl.ascent_pose import attitude_matrix
from apex_rl.locomotion import LocomotionEnv
from apex_rl.stair_footholds import MIN_PAD_GAP_M, select_foothold


def test_landing_search_spreads_a_crowded_tread_within_reach():
    env = LocomotionEnv("hybrid-spread-up-13")
    try:
        env.reset(seed=5)
        hip = env.body_motion
        z = env.course.rise_m + hip.radius - 0.0015
        nominal = np.r_[hip.neutral_xy[0] + [1.10, 0.0], z]
        root = np.array([1.10, 0.0, z - (env._neutral_foot[0, 2] + env.hip[0, 2])])
        occupied = np.array([nominal - [0.040, 0.0, 0.0]])
        result = select_foothold(env, 0, nominal, occupied, root, attitude_matrix(0, 0, 0))
        assert result is not None
        assert result[0] > nominal[0] and result[1] < nominal[1]
        assert np.linalg.norm(result[:2] - occupied[0, :2]) >= MIN_PAD_GAP_M
        pad = hip.radius + 0.007
        for x, y in ((pad, 0), (-pad, 0), (0, pad), (0, -pad)):
            assert env.course.height(result[0] + x, result[1] + y) == pytest.approx(0.13)
        assert (
            select_foothold(
                env, 0, nominal, occupied, root + [0, 0, 0.75], attitude_matrix(0, 0, 0)
            )
            is None
        )
    finally:
        env.close()


def test_per_leg_ik_matches_the_normal_controller():
    env = LocomotionEnv("hybrid-spread-up-13")
    try:
        feet = env._neutral_foot + np.array([0.015, -0.02, 0.01])
        expected = env._solve_ik(feet).reshape(6, 3)
        result = np.vstack([env._solve_ik(foot[None, :], leg=i) for i, foot in enumerate(feet)])
        np.testing.assert_allclose(result, expected)
    finally:
        env.close()
