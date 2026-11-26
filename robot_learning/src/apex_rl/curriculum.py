"""Train, measure and watch the offline META-1 locomotion policy."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import time
from functools import partial
from pathlib import Path

import mujoco
import numpy as np

from .course_observability import training_recorder, watch_stage, write_report
from .locomotion import COURSES, LocomotionEnv

STAGES = tuple(COURSES)
SCHEMA = 1
EVALUATION_SEED = 1_000_000
MIN_SUCCESS = 0.80
MAX_FALLS = 0.10
POLICY_FILE = "policy_sim_only.zip"
REPLAY_MODELS = {
    "b48bc1e3ddb0f1be1a92dd8c41a2e9f467070306f8796b9c6738f248368c7ed2": {
        "flat",
        "step-02",
        "step-05",
    },
    "ea069691f75242c07b5cb6d25121bf3aecbdcaef17284dec6bafdc7e78b67c6c": {"down-02", "step-03"},
    "e6528336f60c4e91413b94485cfc12590a7421cbbae6fc342735c7bedfa6f155": {
        "step-03",
        "down-03",
        "step-04",
    },
}


def add_commands(sub: argparse._SubParsersAction, project: Path) -> None:
    view = sub.add_parser("course-view", help="watch the tripod reference or a saved policy")
    view.add_argument("--course", choices=STAGES)
    view.add_argument("--run", type=Path, help="stage folder containing policy and manifest")
    view.add_argument("--static", action="store_true", help="inspect the parkour at rest")
    view.add_argument("--seed", type=int, default=42)
    bench = sub.add_parser("course-bench", help="evaluate the tripod reference or policy")
    bench.add_argument("--course", choices=STAGES)
    bench.add_argument("--run", type=Path)
    bench.add_argument("--episodes", type=int, default=10)
    bench.add_argument("--seed", type=int, default=EVALUATION_SEED)
    watch = sub.add_parser(
        "course-watch", help="watch the recorded training workers on separate lanes"
    )
    watch.add_argument(
        "--run", type=Path, required=True, help="stage folder, not the experiment root"
    )
    watch.add_argument("--live", action="store_true", help="tail the recordings during training")
    report = sub.add_parser("course-report", help="generate an offline HTML report")
    report.add_argument("--run", type=Path, required=True, help="experiment root")
    train = sub.add_parser("course-train", help="train PPO with evaluation gates between courses")
    train.add_argument("--start", choices=STAGES, default=STAGES[0])
    train.add_argument("--through", choices=STAGES, default=STAGES[-1])
    train.add_argument("--timesteps", type=int, default=200_000, help="new transitions per stage")
    train.add_argument("--envs", type=int, default=4)
    train.add_argument("--eval-episodes", type=int, default=10)
    train.add_argument("--eval-seed", type=int, default=EVALUATION_SEED)
    train.add_argument("--seed", type=int, default=42)
    train.add_argument("--resume", type=Path, help="load a previous stage policy and optimizer")
    train.add_argument("--output", type=Path, default=project / "runs/parkour")
    for command in (view, bench, train):
        command.add_argument(
            "--body-control",
            action="store_true",
            help="use terrain-aware wave gait and body control",
        )


def _fingerprint(env: LocomotionEnv) -> str:
    # Include the geometry and implementation that define policy observations and actions.
    source = [
        Path(__file__).with_name(name).read_text(encoding="utf-8")
        for name in ("locomotion.py", "env.py", "model.py", "body_motion.py")
    ]
    return hashlib.sha256(
        json.dumps([env.config, env.body_motion is not None, *source], sort_keys=True).encode()
    ).hexdigest()


def _read_policy(run: Path, env: LocomotionEnv, *, exact_course: bool, vector=None):
    from stable_baselines3 import PPO

    manifest = json.loads((run / "manifest.json").read_text(encoding="utf-8"))
    match = manifest["model_sha256"] == _fingerprint(env)
    legacy = manifest["course"] in REPLAY_MODELS.get(manifest["model_sha256"], ())
    if (
        manifest["schema"] != SCHEMA
        or manifest.get("body_control", False) != (env.body_motion is not None)
        or not (match or legacy)
    ):
        raise ValueError("run uses an incompatible locomotion environment")
    if exact_course and manifest["course"] != env.course_name:
        raise ValueError("run course does not match requested course")
    return PPO.load(run / POLICY_FILE, env=vector, device="cpu")


def _action(env: LocomotionEnv, policy, obs: np.ndarray) -> np.ndarray:
    return (
        np.zeros(env.action_space.shape, dtype=np.float32)
        if policy is None
        else policy.predict(obs, deterministic=True)[0]
    )


def benchmark(env: LocomotionEnv, policy, episodes: int, *, seed: int = EVALUATION_SEED) -> dict:
    outcomes = []
    for episode in range(episodes):
        obs, _ = env.reset(seed=seed + episode)
        while True:
            obs, _, done, truncated, info = env.step(_action(env, policy, obs))
            if done or truncated:
                outcomes.append({"seed": seed + episode, "steps": env.steps, **info})
                break
    body_stats = {}
    if env.body_motion is not None:
        for key in ("body_height_range_m", "max_lateral_com_m"):
            body_stats[key] = float(np.mean([item[key] for item in outcomes]))
    return {
        "episodes": episodes,
        "successes": sum(item["success"] for item in outcomes),
        "falls": sum(item["fallen"] for item in outcomes),
        "chassis_hits": sum(item["chassis_hit"] for item in outcomes),
        "out_of_lane": sum(item["out"] for item in outcomes),
        "mean_progress_m": float(np.mean([item["progress_m"] for item in outcomes])),
        "mean_forward_m_s": float(np.mean([item["mean_forward_m_s"] for item in outcomes])),
        "mean_touchdowns": float(np.mean([item["touchdowns"] for item in outcomes])),
        "mean_slip_m": float(np.mean([item["slip_m"] for item in outcomes])),
        "mean_mechanical_work_j": float(np.mean([item["mechanical_work_j"] for item in outcomes])),
        "max_clearance_m": float(max(item["max_clearance_m"] for item in outcomes)),
        **body_stats,
        "outcomes": outcomes,
    }


def _inspect(args) -> None:
    course = args.course
    body_control = args.body_control
    if args.run is not None:
        manifest = json.loads((args.run / "manifest.json").read_text(encoding="utf-8"))
        course = course or manifest["course"]
        body_control = body_control or manifest.get("body_control", False)
    env = LocomotionEnv(
        course or "flat", randomize=args.command == "course-bench", body_control=body_control
    )
    try:
        policy = _read_policy(args.run, env, exact_course=True) if args.run else None
        if args.command == "course-bench":
            report = benchmark(env, policy, args.episodes, seed=args.seed)
            print(json.dumps({"course": env.course_name, **report}, indent=2))
        else:
            _view(env, policy, args)
    finally:
        env.close()


def _view(env, policy, args) -> None:
    from mujoco import viewer

    obs, _ = env.reset(seed=args.seed)
    if args.static:
        viewer.launch(env.model, env.data)
        return
    with viewer.launch_passive(env.model, env.data) as window:
        with window.lock():
            window.cam.distance = 1.65
            window.cam.azimuth = 125
            window.cam.elevation = -28
        while window.is_running():
            start = time.monotonic()
            with window.lock():
                obs, _, done, truncated, info = env.step(_action(env, policy, obs))
                window.cam.lookat[:] = (env.data.qpos[0] + 0.20, 0, env.data.qpos[2])
            window.sync()
            if done or truncated:
                print(json.dumps(info), flush=True)
                with window.lock():
                    obs, _ = env.reset(seed=args.seed)
            time.sleep(max(0.0, 0.02 - (time.monotonic() - start)))


def _make_env(course: str, monitor_path: str, body_control: bool):
    from stable_baselines3.common.monitor import Monitor

    return Monitor(
        LocomotionEnv(course, record_pose=True, body_control=body_control),
        filename=monitor_path,
        info_keywords=("success", "fallen", "progress_m", "touchdowns", "pack_voltage_proxy_v"),
    )


def _train(args) -> None:
    import stable_baselines3 as sb3
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

    torch.set_num_threads(1)
    args.output.mkdir(parents=True, exist_ok=False)
    policy = None
    summary = []
    for course in STAGES[STAGES.index(args.start) : STAGES.index(args.through) + 1]:
        stage = args.output / course
        stage.mkdir()
        eval_env = LocomotionEnv(course, body_control=args.body_control)
        factories = [
            partial(_make_env, course, str(stage / f"env-{i}.csv"), args.body_control)
            for i in range(args.envs)
        ]
        vector = (
            DummyVecEnv(factories)
            if args.envs == 1
            else SubprocVecEnv(factories, start_method="spawn")
        )
        vector.seed(args.seed)
        try:
            manifest = {
                "schema": SCHEMA,
                "course": course,
                "seed": args.seed,
                "eval_seed": args.eval_seed,
                "envs": args.envs,
                "body_control": args.body_control,
                "total_transitions": None,
                "model_sha256": _fingerprint(eval_env),
                "python": platform.python_version(),
                "mujoco": mujoco.__version__,
                "stable_baselines3": sb3.__version__,
                "policy_export_allowed": False,
            }
            manifest_path = stage / "manifest.json"
            manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
            if policy is None and args.resume:
                policy = _read_policy(args.resume, eval_env, exact_course=False, vector=vector)
            if policy is None:
                policy_kwargs = {"net_arch": {"pi": [64, 64], "vf": [64, 64]}}
                if args.body_control:
                    policy_kwargs["log_std_init"] = -1.7
                policy = PPO(
                    "MlpPolicy",
                    vector,
                    device="cpu",
                    seed=args.seed,
                    n_steps=512,
                    batch_size=128,
                    learning_rate=3e-4,
                    policy_kwargs=policy_kwargs,
                    verbose=1,
                    tensorboard_log=str(args.output / "tensorboard"),
                )
            else:
                policy.set_env(vector)
            policy.tensorboard_log = str(args.output / "tensorboard")
            policy.learn(
                total_timesteps=args.timesteps,
                reset_num_timesteps=False,
                tb_log_name=course,
                callback=training_recorder(stage),
            )
            policy.save(stage / POLICY_FILE)
            manifest["total_transitions"] = policy.num_timesteps
            updated = stage / "manifest.tmp"
            updated.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
            updated.replace(manifest_path)
            evaluation = benchmark(eval_env, policy, args.eval_episodes, seed=args.eval_seed)
            baseline = benchmark(eval_env, None, args.eval_episodes, seed=args.eval_seed)
        finally:
            vector.close()
            eval_env.close()
        passed = (
            evaluation["successes"] / args.eval_episodes >= MIN_SUCCESS
            and evaluation["falls"] / args.eval_episodes <= MAX_FALLS
        )
        baseline_key = "body_reference" if args.body_control else "tripod"
        report = {"course": course, "passed": passed, "policy": evaluation, baseline_key: baseline}
        (stage / "evaluation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        result = {
            key: evaluation[key] for key in ("episodes", "successes", "falls", "mean_progress_m")
        }
        summary.append({"course": course, "passed": passed, **result})
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        write_report(args.output)
        print(json.dumps(summary[-1]), flush=True)
        if not passed:
            break


def run_command(args, parser: argparse.ArgumentParser) -> None:
    if args.command == "course-watch":
        watch_stage(args.run, live=args.live)
    elif args.command == "course-report":
        print(write_report(args.run))
    elif args.command == "course-train":
        if not 1 <= args.envs <= 32:
            parser.error("--envs must be between 1 and 32")
        if args.timesteps < 512 or not 10 <= args.eval_episodes <= 100:
            parser.error("use at least 512 training transitions and 10–100 evaluation episodes")
        if STAGES.index(args.start) > STAGES.index(args.through):
            parser.error("--through must be at or after --start")
        if not 0 <= args.seed < EVALUATION_SEED - 32:
            parser.error("--seed must be below the evaluation seed range")
        if args.eval_seed < EVALUATION_SEED:
            parser.error("--eval-seed must be at or above 1000000")
        _train(args)
    else:
        if args.command == "course-bench" and not 1 <= args.episodes <= 100:
            parser.error("--episodes must be between 1 and 100")
        if args.seed < 0:
            parser.error("--seed must be nonnegative")
        if args.command == "course-view" and args.static and args.run:
            parser.error("--static cannot replay a learned policy")
        _inspect(args)
