import json

import numpy as np
import pytest

from apex_rl import box_obstacle as box
from apex_rl.ascent_planner import check_stance
from apex_rl.curriculum import STAGES, _summarize
from apex_rl.locomotion import COURSES


def test_box_course_is_16_cm_high_60_cm_wide_and_long():
    course = COURSES["hybrid-box-16"]
    assert course.rise_m == pytest.approx(0.16)
    assert course.height(course.edge_x + 0.01) == pytest.approx(0.16)
    assert course.height(course.edge_x + 0.59, 0.29) == pytest.approx(0.16)
    assert course.height(course.edge_x + 0.61) == 0.0
    assert course.height(course.edge_x + 0.30, 0.31) == 0.0
    assert course.last_edge == pytest.approx(course.edge_x + 0.60)
    assert "hybrid-box-16" not in STAGES
    assert box.GUARDED_BOX_COURSE not in STAGES


def test_mirrored_descent_checks_equal_the_direct_checks():
    env = box.BoxEnv("hybrid-box-16")
    try:
        env.reset(seed=3)
        c = env.body_motion
        rng = np.random.default_rng(0)
        torque = c._torque_limit()
        mirrored_torque = torque.reshape(6, 3)[box.MIRROR].reshape(-1)
        for _ in range(200):
            feet = np.zeros((6, 3))
            feet[:, :2] = c.neutral_xy + rng.normal(0, 0.04, (6, 2))
            feet[:, 2] = c.radius
            body = np.array([rng.normal(0, 0.03), rng.normal(0, 0.02), 0.10 + 0.08 * rng.random()])
            pitch = rng.normal(0, 0.15)
            stance = np.ones(6, dtype=bool)
            stance[rng.integers(6)] = rng.random() < 0.5
            margin = np.full(6, np.radians(4))
            com = c._com_body()
            direct = check_stance(c.geom, torque, feet, stance, body, pitch, com, c._weight(), (),
                                  required_margin=margin)  # fmt: skip
            mf, mb, mp = c._mirrored(feet, body, pitch)
            mirrored = check_stance(c.geom, mirrored_torque, mf, stance[box.MIRROR], mb, mp,
                                    com * np.array([-1.0, 1.0, 1.0]), c._weight(), (),
                                    required_margin=margin)  # fmt: skip
            assert direct.feasible == mirrored.feasible
            if direct.feasible:
                assert direct.torque_ratio == pytest.approx(mirrored.torque_ratio, abs=1e-9)
    finally:
        env.close()


def test_box_traverse_climbs_walks_on_top_and_steps_down():
    env = box.BoxEnv("hybrid-box-16")
    try:
        env.reset(seed=1013200)
        action = np.zeros(env.action_space.shape, dtype=np.float32)
        phases = set()
        done = False
        while not done:
            _, _, terminated, truncated, info = env.step(action)
            phases.add(info["staged_phase"])
            done = terminated or truncated
        assert info["success"] and not info["fallen"] and not info["body_impact"]
        assert {"top", "down-front", "down-middle", "down-rear", "floor"} <= phases
        assert info["riser_contact_substeps"] * env.model.opt.timestep <= 0.25
        # A completed box run with excessive side contact fails the same
        # report check as a stair run, even though BoxCourse has no home stairs.
        scraped = dict(info, riser_contact_substeps=150)
        report = _summarize(env, [scraped])
        assert report["successes"] == 1
        assert report["clean_completions"] == 0
        assert report["max_riser_contact_s"] == pytest.approx(0.30)
        assert not report["riser_clearance_passed"]
        json.dumps(report)
    finally:
        env.close()
