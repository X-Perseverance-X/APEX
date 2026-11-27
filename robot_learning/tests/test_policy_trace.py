import json

import numpy as np
import pytest

from apex_rl.locomotion import LocomotionEnv
from apex_rl.policy_trace import ActorTrace, observation_names, write_trace


def test_policy_trace_uses_the_real_actor_and_applied_command(tmp_path):
    sb3 = pytest.importorskip("stable_baselines3")
    env = LocomotionEnv("hybrid-brace-transfer-up-13", episode_steps=4)
    try:
        env.action_scale = 0.25
        model = sb3.PPO(
            "MlpPolicy",
            env,
            n_steps=4,
            batch_size=4,
            seed=32,
            policy_kwargs={"net_arch": {"pi": [8, 8], "vf": [8, 8]}},
        )
        observation, _ = env.reset(seed=32)
        assert len(observation_names()) == observation.size == 127
        action, _ = model.predict(observation, deterministic=True)
        actor = ActorTrace(model.policy)
        hidden, unbounded, _ = actor.sample(observation, action)
        assert [len(layer) for layer in hidden] == [8, 8]
        np.testing.assert_allclose(np.clip(unbounded, -1, 1), action, atol=5e-5)

        path = tmp_path / "trace.html"
        write_trace(env, model, seed=32, sample_every=1, output=path, fragment=True)
        html = path.read_text(encoding="utf-8")
        data = json.loads(html.split('id="mpt-data">', 1)[1].split("</script>", 1)[0])
        assert data["scale"] == 0.25 and len(data["outputs"]) == 12
        np.testing.assert_allclose(data["policy"]["samples"][0]["applied"], action / 16, atol=5e-5)
        assert "layers" not in data["reference"]["samples"][0]
    finally:
        env.close()
