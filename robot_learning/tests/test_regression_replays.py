"""Recorded 13 cm climbs that every later controller change must still reproduce."""

from pathlib import Path

import pytest

from apex_rl.curriculum import _read_policy, _run_episodes
from apex_rl.locomotion import LocomotionEnv

EXPERIMENTS = Path(__file__).resolve().parents[1] / "experiments"
RESIDUAL_QUARTER = EXPERIMENTS / "body-support-13/residual-quarter/hybrid-brace-transfer-up-13"


def _replay(seed, run=None):
    env = LocomotionEnv("hybrid-brace-transfer-up-13")
    try:
        policy = _read_policy(run, env, exact_course=True) if run else None
        return _run_episodes(env, policy, 1, seed)[0], env.model.opt.timestep
    finally:
        env.close()


def test_reference_1013208_climbs_without_riser_scrape_or_rigid_overlap():
    outcome, dt = _replay(1013208)
    assert outcome["success"] and not outcome["fallen"] and not outcome["body_impact"]
    assert outcome["peak_upper_contacts"] == 6
    assert outcome["riser_contact_substeps"] * dt == 0.0
    assert outcome["leg_stair_peak_m"] == 0.0
    assert outcome["interleg_peak_m"] <= 0.002
    assert outcome["min_knee_deg"] > -152


@pytest.mark.skipif(not RESIDUAL_QUARTER.exists(), reason="archived policy missing")
def test_residual_quarter_1013213_climbs_with_recorded_riser_time():
    pytest.importorskip("stable_baselines3")
    outcome, dt = _replay(1013213, RESIDUAL_QUARTER)
    assert outcome["success"] and not outcome["fallen"] and not outcome["body_impact"]
    assert outcome["peak_upper_contacts"] == 6
    # The archived result: 0.40 s of riser contact and 0.44 mm peak inter-leg overlap.
    assert outcome["riser_contact_substeps"] * dt == pytest.approx(0.40, abs=0.02)
    assert outcome["interleg_peak_m"] == pytest.approx(0.00044, abs=0.0002)
