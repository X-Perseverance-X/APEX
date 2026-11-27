"""Record the actual deterministic PPO actor during an offline MuJoCo replay."""

from __future__ import annotations

import json
from importlib.resources import files

import numpy as np

from .contract import LEG_ORDER

ACTION_NAMES = (
    "İlerleme temposu",
    "Ayak kaldırma",
    "Gövde ileri / geri",
    "Gövde yüksekliği",
    "Gövde yana kayma",
    "Öne eğim",
    "Yana eğim",
    "Gövde sapması",
    "Ayak hedefi X",
    "Ayak hedefi Y",
    "Bacağın tarafı",
    "Gövdeyi dayama",
)


def observation_names():
    names = [
        f"{name} {axis}" for name in ("Doğrusal hız", "Açısal hız", "Yerçekimi") for axis in "XYZ"
    ]
    for group in ("Nötrden açı farkı", "Eklem hızı", "Önceki eklem hedefi"):
        names.extend(
            f"{group} · {leg} {joint}" for leg in LEG_ORDER for joint in ("coxa", "femur", "tibia")
        )
    names.extend(("İleri hız isteği", "Yanal hız isteği", "Dönüş isteği"))
    names.extend(f"Ayak teması · {leg}" for leg in LEG_ORDER)
    names.extend(("Yürüyüş fazı sin", "Yürüyüş fazı cos"))
    names.extend(f"Zemin yükseklik farkı · {cm} cm ileride" for cm in (15, 25, 35, 45, 55, 70))
    names.extend(("Besleme oranı", "Yanal konum", "Yön sin", "Yön cos", "Bitişe uzaklık"))
    names.extend(
        (
            "Gövde hedef hatası X",
            "Gövde hedef hatası Y",
            "Gövde hedef hatası Z",
            "Öne eğim hedefi",
            "Destek ayak oranı",
            "Adım süresi",
        )
    )
    names.extend(f"Hareket eden ayak · {leg}" for leg in LEG_ORDER)
    names.extend(f"Ayak / zemin aralığı · {leg}" for leg in LEG_ORDER)
    names.extend(
        (
            "Yana eğim hedefi",
            "Sapma hedefi",
            "Gövde destek yükü",
            "Dik yüz baskı süresi",
            "Destek noktası X",
            "Destek noktası Y",
        )
    )
    names.extend(
        f"Servo yük oranı · {leg} {joint}"
        for leg in LEG_ORDER
        for joint in ("coxa", "femur", "tibia")
    )
    return names


def _numbers(tensor):
    return np.round(tensor.detach().cpu().numpy().astype(float), 4).tolist()


class ActorTrace:
    def __init__(self, policy):
        from torch import nn

        self.policy = policy
        net = list(policy.mlp_extractor.policy_net)
        if len(net) != 4 or any(
            not isinstance(layer, kind)
            for layer, kind in zip(net, (nn.Linear, nn.Tanh, nn.Linear, nn.Tanh), strict=True)
        ):
            raise ValueError("the actor trace expects two fully connected tanh hidden layers")
        self.hidden = ((net[0], net[1]), (net[2], net[3]))
        self.weights = [
            {"weight": _numbers(layer.weight), "bias": _numbers(layer.bias)}
            for layer in (net[0], net[2], policy.action_net)
        ]

    def sample(self, observation, action):
        import torch

        with torch.inference_mode():
            tensor, _ = self.policy.obs_to_tensor(observation)
            latent = self.policy.extract_features(tensor)
            hidden = []
            for linear, activation in self.hidden:
                latent = activation(linear(latent))
                hidden.append(_numbers(latent[0]))
            mean = self.policy.action_net(latent)[0]
            if not np.allclose(mean.cpu().numpy().clip(-1, 1), action, atol=1e-5):
                raise ValueError("captured actor output does not match the control action")
            return (
                hidden,
                _numbers(mean),
                round(float(self.policy.predict_values(tensor).item()), 4),
            )


def _rollout(env, seed, model, actor, sample_every):
    obs, _ = env.reset(seed=seed)
    rows = []
    dt = env.control_steps * env.model.opt.timestep
    while True:
        action = model.predict(obs, deterministic=True)[0] if model is not None else np.zeros(12)
        sample = env.steps % sample_every == 0
        if sample:
            x, _, z = env.data.qpos[:3]
            row = {"t": round(env.steps * dt, 3), "x": round(float(x), 4)}
            if actor is not None:
                layers, mean, value = actor.sample(obs, action)
                correction = env.action_scale * action
                filtered = 0.75 * env.adjustment + 0.25 * correction
                row.update(
                    height=round(float(z), 4),
                    layers=layers,
                    mean=mean,
                    value=value,
                    inputs=np.round(obs.astype(float), 4).tolist(),
                    applied=np.round(filtered, 4).tolist(),
                    phase=env.gait,
                    leg=env.body_motion.leg,
                )
            rows.append(row)
        obs, _, done, truncated, info = env.step(action)
        if done or truncated:
            return {
                "samples": rows,
                "success": info["success"],
                "fallen": info["fallen"],
                "elapsed": round(env.steps * dt, 2),
                "progress": round(info["progress_m"], 4),
            }


def write_trace(env, model, *, seed, sample_every, output, fragment=False):
    if env.observation_space.shape != (127,) or env.action_space.shape != (12,):
        raise ValueError("the actor trace currently supports the 127-input body-support courses")
    actor = ActorTrace(model.policy)
    source = {
        "course": env.course_name,
        "seed": seed,
        "scale": env.action_scale,
        "rise": env.course.rise_m,
        "edge": env.course.edge_x,
        "goal": env.course.goal,
        "inputs": observation_names(),
        "outputs": ACTION_NAMES,
        "legs": LEG_ORDER,
        "weights": actor.weights,
        "policy": _rollout(env, seed, model, actor, sample_every),
        "reference": _rollout(env, seed, None, None, sample_every),
    }
    payload = json.dumps(source, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    template = files("apex_rl").joinpath("assets/policy-trace.html").read_text(encoding="utf-8")
    page = template.replace("__TRACE_DATA__", payload.replace("<", "\\u003c"))
    if not fragment:
        page = (
            '<!doctype html><html lang="tr"><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width, initial-scale=1">'
            "<title>META-1 · Politika izleme</title><style>"
            "body{max-width:1050px;margin:24px auto;padding:0 16px;font:15px system-ui;"
            "color:CanvasText;background:Canvas;color-scheme:light dark} "
            "h2,p{margin:0} input[type=range]{accent-color:Highlight} "
            "button,input,select{font:inherit} .tabular-nums{font-variant-numeric:tabular-nums}</style>"
            f"<body>{page}</body></html>"
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(page, encoding="utf-8")
