import json
from pathlib import Path

import numpy as np
import pytest

from apex_rl.ascent_planner import LegGeometry, check_stance, hull_margin, vertical_loads
from apex_rl.curriculum import _fingerprint
from apex_rl.gait_lab import PhaseTripod, ReferenceTripod
from apex_rl.locomotion import COURSES, LocomotionEnv
from apex_rl.staged_ascent import STAGED, StagedEnv, capture_state, make_env

EXPERIMENTS = Path(__file__).resolve().parents[1] / "experiments"


def test_staged_courses_reuse_the_brace_transfer_geometry_and_limits():
    for height in range(13, 21):
        assert (
            COURSES[f"hybrid-staged-up-{height}"] is COURSES[f"hybrid-brace-transfer-up-{height}"]
        )
    assert set(STAGED) == {f"hybrid-staged-up-{h}" for h in range(13, 21)} | {
        "hybrid-wide-rear-up-20"
    }
    assert COURSES["hybrid-wide-rear-up-20"] is COURSES["hybrid-staged-up-20"]
    assert type(make_env("hybrid-brace-transfer-up-13")) is LocomotionEnv


def test_existing_policy_hashes_do_not_change():
    manifest = json.loads(
        (EXPERIMENTS / "stair-spread/hybrid-spread-up-13/manifest.json").read_text(encoding="utf-8")
    )
    env = LocomotionEnv("hybrid-spread-up-13")
    try:
        assert _fingerprint(env) == manifest["model_sha256"]
    finally:
        env.close()


def test_static_loads_balance_the_weight_and_stand_pose_is_feasible():
    points = np.array([[0.2, 0.2], [0.2, -0.2], [-0.2, 0.2], [-0.2, -0.2]])
    loads = vertical_loads(points, np.array([0.05, 0.0]), 50.0)
    assert loads.sum() == pytest.approx(50.0)
    assert (loads * (points[:, 0] - 0.05)).sum() == pytest.approx(0.0, abs=1e-9)
    assert vertical_loads(points, np.array([0.3, 0.0]), 50.0) is None
    assert hull_margin(points, (0.0, 0.0)) == pytest.approx(0.2)
    env = StagedEnv("hybrid-staged-up-13")
    try:
        env.reset(seed=3)
        for _ in range(20):
            env._apply_joint_targets(env.nominal)
        data = env.data
        com = data.subtree_com[1] - data.qpos[:3]
        feet = data.xpos[env._foot_ids].copy()
        weight = float(env.model.body_mass.sum() * 9.81)
        geom = LegGeometry.from_env(env)
        limits = env.model.actuator_forcerange[:18, 1]
        check = check_stance(geom, limits, feet, np.ones(6, bool), data.qpos[:3], 0.0, com, weight)
        assert check.feasible and check.loads.sum() == pytest.approx(weight)
        tripod = np.array([True, False, True, False, True, False])
        assert check_stance(geom, limits, feet, tripod, data.qpos[:3], 0.0, com, weight).feasible
    finally:
        env.close()


def test_flat_reference_matches_the_existing_tripod_and_phase_gait_stops():
    env = LocomotionEnv("flat", randomize=False)
    try:
        env.reset(seed=1)
        gait = ReferenceTripod(env)
        gait.reset()
        for _ in range(40):
            np.testing.assert_allclose(
                env._tripod_targets(np.zeros(3), flat=True), gait.targets((0.13, 0, 0)), atol=1e-9
            )
            env.steps += 1
        phase = PhaseTripod(env)
        phase.reset()
        for _ in range(100):
            env._apply_joint_targets(phase.targets((0.1, 0, 0)))
        for _ in range(150):
            env._apply_joint_targets(phase.targets((0, 0, 0)))
        assert phase.standing
    finally:
        env.close()


def test_staged_env_keeps_the_policy_interface_and_resumes_a_snapshot_exactly():
    env = StagedEnv("hybrid-staged-up-13")
    try:
        obs, _ = env.reset(seed=500001)
        assert obs.shape == (127,) and env.action_space.shape == (12,)
        assert env.action_mask.tolist() == [0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0]
        for _ in range(120):
            env.step(np.zeros(12))
        snapshot = capture_state(env)
        for _ in range(60):
            env.step(np.zeros(12))
        expected = env.data.qpos.copy()
    finally:
        env.close()
    resumed = StagedEnv("hybrid-staged-up-13", start_states=[snapshot], start_fraction=1.0)
    try:
        resumed.reset(seed=7)
        assert resumed.steps == 120
        for _ in range(60):
            resumed.step(np.zeros(12))
        np.testing.assert_allclose(resumed.data.qpos, expected, atol=1e-9)
    finally:
        resumed.close()


def test_last_rear_sole_is_lifted_after_tilting_the_body_on_the_20_cm_tilt_course():
    env = make_env("hybrid-wide-rear-up-20")
    try:
        env.episode_steps = 4000
        env.reset(seed=1013201)
        action = np.zeros(env.action_space.shape, dtype=np.float32)
        lifted = []
        done = False
        while not done:
            _, _, terminated, truncated, info = env.step(action)
            plan = env.body_motion.plan
            if plan is not None and plan.get("post_body") is not None:
                lifted.append((plan["pitch"], plan["post_pitch"]))
            done = terminated or truncated
        # All six soles reached the tread, with the body tilted forward of the
        # start pose while the last rear sole hung, and nothing fell.
        assert info["peak_upper_contacts"] == 6
        assert not info["fallen"] and not info["body_impact"]
        assert any(after < before - 0.01 for before, after in lifted)
    finally:
        env.close()


def test_recorded_staged_ppo_policies_still_match_the_environment():
    # A change to the staged controller sources must keep the recorded PPO
    # policies loadable, or bind the new build only after a replay check.
    for height in (13,):
        run = EXPERIMENTS / f"staged-ascent/ppo-{height}/hybrid-staged-up-{height}"
        manifest = json.loads((run / "manifest.json").read_text(encoding="utf-8"))
        env = make_env(f"hybrid-staged-up-{height}")
        try:
            assert _fingerprint(env) == manifest["model_sha256"]
        finally:
            env.close()
