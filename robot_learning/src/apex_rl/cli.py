"""Offline META-1 model, smoke-test and RL training commands."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import shutil
from pathlib import Path

import mujoco
import numpy as np

from .contract import load_contract
from .env import Meta1Env
from .model import LEG_MESH_DIR, build_xml

PROJECT = Path(__file__).resolve().parents[2]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    build = sub.add_parser("build", help="write physics MJCF; does not start robot hardware")
    build.add_argument("--output", type=Path, default=PROJECT / "generated/meta1.xml")
    sub.add_parser("view", help="open the MuJoCo viewer in the stand pose")
    smoke = sub.add_parser("smoke", help="compile and step a fixed-stand physics episode")
    smoke.add_argument("--steps", type=int, default=100)
    train = sub.add_parser("train", help="train an experimental simulation-only PPO policy")
    train.add_argument("--task", choices=("stand", "velocity"), default="stand")
    train.add_argument("--timesteps", type=int, default=10_000)
    train.add_argument("--seed", type=int, default=42)
    train.add_argument("--domain-randomization", action="store_true")
    train.add_argument("--output", type=Path, default=PROJECT / "runs/pilot")
    evaluate = sub.add_parser(
        "evaluate", help="compare a saved policy against zero action in simulation"
    )
    evaluate.add_argument("--run", type=Path, required=True)
    evaluate.add_argument("--episodes", type=int, default=5)
    args = parser.parse_args()

    if args.command == "build":
        args.output.parent.mkdir(parents=True, exist_ok=True)
        mesh_dir = args.output.parent / LEG_MESH_DIR.name
        if mesh_dir.resolve() != LEG_MESH_DIR:
            shutil.copytree(LEG_MESH_DIR, mesh_dir, dirs_exist_ok=True)
        args.output.write_text(build_xml(mesh_dir=mesh_dir.name), encoding="utf-8")
        model = mujoco.MjModel.from_xml_path(str(args.output))
        print(
            json.dumps(
                {
                    "model": str(args.output),
                    "joints": model.njnt,
                    "actuators": model.nu,
                    "state_variables": model.nq,
                }
            )
        )
        return

    if args.command == "view":
        from mujoco import viewer

        model = mujoco.MjModel.from_xml_string(build_xml())
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, model.key("stand").id)
        mujoco.mj_forward(model, data)
        viewer.launch(model, data)
        return

    if args.command == "smoke":
        if not 1 <= args.steps <= 20_000:
            parser.error("--steps must be between 1 and 20000")
        env = Meta1Env()
        try:
            obs, _ = env.reset(seed=42)
            rewards = []
            info = {}
            for step in range(args.steps):
                obs, reward, terminated, truncated, info = env.step(np.zeros(18, dtype=np.float32))
                rewards.append(reward)
                if terminated or truncated:
                    break
            print(
                json.dumps(
                    {
                        "steps": step + 1,
                        "terminated": terminated,
                        "height_m": info["height_m"],
                        "contacts": info["contacts"],
                        "mean_reward": float(np.mean(rewards)),
                        "finite": bool(np.isfinite(obs).all()),
                    }
                )
            )
            if terminated or info["contacts"] < 4 or not np.isfinite(obs).all():
                raise SystemExit("stand smoke failed")
        finally:
            env.close()
        return

    if args.command == "evaluate":
        if not 1 <= args.episodes <= 100:
            parser.error("--episodes must be between 1 and 100")
        try:
            from stable_baselines3 import PPO
        except ImportError as exc:
            raise SystemExit(
                "Install training extras: pip install -e 'robot_learning[train]'"
            ) from exc
        run_dir = args.run.resolve()
        manifest = json.loads((run_dir / "manifest.json").read_text(encoding="utf-8"))
        current_hash = hashlib.sha256(
            json.dumps(load_contract(), sort_keys=True).encode()
        ).hexdigest()
        if current_hash != manifest["geometry_sha256"]:
            raise SystemExit("Geometry contract differs from the training run")
        policy = PPO.load(str(run_dir / "policy_sim_only.zip"), device="cpu")
        results = {}
        for candidate in ("zero", "policy"):
            env = Meta1Env(
                task=manifest["task"], domain_randomization=manifest["domain_randomization"]
            )
            scores, lengths, falls = [], [], 0
            try:
                for episode in range(args.episodes):
                    obs, _ = env.reset(seed=int(manifest["seed"]) + 10_000 + episode)
                    total = 0.0
                    for step in range(env.episode_steps):
                        action = (
                            np.zeros(18, dtype=np.float32)
                            if candidate == "zero"
                            else policy.predict(obs, deterministic=True)[0]
                        )
                        obs, reward, terminated, truncated, _ = env.step(action)
                        total += reward
                        if terminated or truncated:
                            falls += int(terminated)
                            break
                    scores.append(total)
                    lengths.append(step + 1)
            finally:
                env.close()
            results[candidate] = {
                "mean_reward": float(np.mean(scores)),
                "mean_length": float(np.mean(lengths)),
                "falls": falls,
            }
        print(
            json.dumps(
                {"episodes": args.episodes, "results": results, "hardware_validated": False},
                indent=2,
            )
        )
        return

    if args.timesteps < 1_000:
        parser.error("--timesteps must be at least 1000")
    try:
        import stable_baselines3 as sb3
        from stable_baselines3 import PPO
        from stable_baselines3.common.env_checker import check_env
        from stable_baselines3.common.monitor import Monitor
    except ImportError as exc:
        raise SystemExit("Install training extras: pip install -e 'robot_learning[train]'") from exc
    check = Meta1Env(task=args.task, domain_randomization=args.domain_randomization)
    try:
        check_env(check, warn=True)
    finally:
        check.close()
    run_dir = args.output.resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    config = load_contract()
    manifest = {
        "schema": 1,
        "task": args.task,
        "timesteps": args.timesteps,
        "seed": args.seed,
        "domain_randomization": args.domain_randomization,
        "geometry_sha256": hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest(),
        "python": platform.python_version(),
        "mujoco": mujoco.__version__,
        "stable_baselines3": sb3.__version__,
        "policy_export_allowed": False,
        "reason": "physical mass, torque, lag and contact parameters are not identified",
    }
    (run_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    env = Monitor(
        Meta1Env(task=args.task, domain_randomization=args.domain_randomization),
        filename=str(run_dir / "monitor.csv"),
    )
    try:
        model = PPO(
            "MlpPolicy",
            env,
            seed=args.seed,
            n_steps=1024,
            batch_size=256,
            learning_rate=3e-4,
            gamma=0.99,
            verbose=1,
            tensorboard_log=str(run_dir / "tensorboard"),
        )
        model.learn(total_timesteps=args.timesteps)
        model.save(str(run_dir / "policy_sim_only.zip"))
    finally:
        env.close()
    print(json.dumps({"run_dir": str(run_dir), "policy_export_allowed": False}))


if __name__ == "__main__":
    main()
