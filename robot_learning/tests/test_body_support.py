import mujoco
import numpy as np

from apex_rl.body_support import BodySupport
from apex_rl.curriculum import _passed_stage, benchmark
from apex_rl.locomotion import LocomotionEnv


def _support_fixture(start):
    model = mujoco.MjModel.from_xml_string(f"""
    <mujoco>
      <option timestep="0.002"/>
      <worldbody>
        <geom name="tread" type="box" pos="0.2 0 0.075" size="0.2 0.4 0.075"/>
        <body pos="{start}">
          <freejoint/>
          <geom name="chassis" type="box" size="0.07 0.06 0.04" mass="6"/>
        </body>
      </worldbody>
    </mujoco>
    """)
    return (
        model,
        mujoco.MjData(model),
        BodySupport(model, [model.geom("chassis").id], [model.geom("tread").id]),
    )


def test_vertical_load_counts_as_support_without_an_impact():
    model, data, monitor = _support_fixture("0.20 0 0.190")
    for _ in range(200):
        mujoco.mj_step(model, data)
        monitor.observe(model, data, data.contact)
    assert monitor.support_steps > 100
    assert 0.95 < monitor.support_load < 1.05
    np.testing.assert_allclose(monitor.xy, [0.2, 0.0], atol=1e-3)
    assert not monitor.hard_impact


def test_striking_a_riser_does_not_provide_support():
    model, data, monitor = _support_fixture("-0.09 0 0.08")
    data.qvel[0] = 2.0
    for _ in range(35):
        mujoco.mj_step(model, data)
        monitor.observe(model, data, data.contact)
    assert monitor.support_steps == 0
    assert monitor.side_steps > 0 and monitor.hard_impact


def test_brace_policy_observes_load_and_keeps_derated_servos():
    env = LocomotionEnv("hybrid-brace-up-13", train_near_step=True)
    try:
        obs, _ = env.reset(seed=1011020)
        assert env.observation_space.contains(obs) and obs.shape == (127,)
        assert env.data.qpos[0] > 0
        assert np.all(env._max_target_rate < env._leg_target_rate)
        assert np.all(env.model.actuator_forcerange[:18, 1] <= env._leg_force_range[:, 1])
        _, _, _, _, info = env.step(np.zeros(12))
        assert info["body_support_s"] == 0 and 0 <= info["servo_saturation_fraction"] <= 1
    finally:
        env.close()


def test_brace_curriculum_rejects_small_or_unsafe_evaluations():
    clean = {"episodes": 20, "successes": 18, "falls": 1, "body_impacts": 0}
    assert _passed_stage(clean, body_support=True)
    assert not _passed_stage({**clean, "body_impacts": 1}, body_support=True)
    assert not _passed_stage({**clean, "riser_clearance_passed": False}, body_support=True)
    assert not _passed_stage({"episodes": 10, "successes": 10, "falls": 0}, body_support=True)


def test_rear_transfer_records_support_and_all_six_upper_contacts():
    env = LocomotionEnv("hybrid-brace-transfer-up-13")
    try:
        result = benchmark(env, None, 1, seed=1012211)
        assert result["peak_upper_contacts"] == 6
        assert result["falls"] == 0
        assert result["mean_body_support_s"] > 0.25
    finally:
        env.close()
