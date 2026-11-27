"""Residual PPO around the staged ascent controller, with checkpoint selection.

Simulation only; the saved policies are marked ``policy_export_allowed: false``.

1. Successful deterministic climbs on ``--start-seeds`` provide intermediate
   states; ``--start-fraction`` of the training episodes begin from one of them
   (front, middle or rear stage), so the rear transfer is practised often.
2. PPO trains small corrections (``--action-scale``) to the planned body pose
   and landing point. A checkpoint is written every ``--chunk`` transitions.
3. Every checkpoint and the zero-correction controller are evaluated from the
   course start on a separate validation group (``--val-seed``).
4. The best checkpoint by validation completions, falls, body impacts and
   riser contact is copied to ``<output>/<course>/`` and evaluated once on the
   held-out group (``--final-seed``) next to the controller alone.

Example (repository root):

    python robot_learning/tools/staged_train.py --course hybrid-staged-up-13 \\
        --timesteps 196608 --output robot_learning/runs/staged-ppo-13
"""

from __future__ import annotations

import argparse
import json
import pickle
import platform
import shutil
import time
from functools import partial
from pathlib import Path

import mujoco

from apex_rl.course_observability import training_recorder
from apex_rl.curriculum import POLICY_FILE, SCHEMA, _benchmark_parallel, _fingerprint, _passed_stage
from apex_rl.locomotion import COURSES
from apex_rl.staged_ascent import STAGED, StagedEnv, record_start_states

SUMMARY_KEYS = (
    "episodes",
    "successes",
    "falls",
    "body_impacts",
    "max_riser_contact_s",
    "mean_riser_contact_s",
    "interleg_peak_m",
    "leg_stair_peak_m",
    "min_knee_deg",
    "min_upper_foot_gap_m",
    "mean_servo_saturation_fraction",
    "mean_progress_m",
    "passed",
)


def _summary(report, course):
    report["passed"] = _passed_stage(report, body_support=COURSES[course].body_support)
    return {k: report.get(k) for k in SUMMARY_KEYS}


def _within_limits(s):
    """Contact limits of the stage gate: no body impact, riser <= 0.25 s, links, knee."""
    return bool(
        not s["body_impacts"]
        and s["max_riser_contact_s"] <= 0.25
        and max(s["interleg_peak_m"] or 0.0, s["leg_stair_peak_m"] or 0.0) <= 0.002
        and s["min_knee_deg"] > -152.0
    )


def _score(s):
    """Validation ranking: contact limits first, then completions, falls, riser time."""
    return (
        _within_limits(s),
        s["successes"],
        -s["falls"],
        -s["max_riser_contact_s"],
        -s["mean_riser_contact_s"],
    )


def _make(course, states, fraction, monitor_path, action_scale):
    from stable_baselines3.common.monitor import Monitor

    env = StagedEnv(
        course, record_pose=True, train_near_step=True, start_states=states, start_fraction=fraction
    )
    env.action_scale = action_scale
    return Monitor(env, filename=monitor_path, info_keywords=("success", "fallen", "progress_m"))


def _manifest(args, env, policy, transitions):
    import stable_baselines3 as sb3

    return {
        "schema": SCHEMA,
        "course": args.course,
        "seed": args.seed,
        "eval_seed": args.val_seed,
        "final_seed": args.final_seed,
        "envs": args.envs,
        "body_control": True,
        "attitude_control": env.attitude_control,
        "body_support": env.course.body_support,
        "total_transitions": transitions,
        "model_sha256": _fingerprint(env),
        "python": platform.python_version(),
        "mujoco": mujoco.__version__,
        "stable_baselines3": sb3.__version__,
        "policy_export_allowed": False,
        "action_scale": args.action_scale,
        "controller": "staged-ascent",
        "start_fraction": args.start_fraction,
        "start_seeds": args.start_seeds,
        "ppo_epochs": policy.n_epochs,
        "ppo_gamma": policy.gamma,
        "ppo_gae_lambda": policy.gae_lambda,
        "steps_per_worker": policy.n_steps,
        "net_arch": policy.policy.net_arch,
    }


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--course", choices=sorted(STAGED), required=True)
    parser.add_argument("--timesteps", type=int, default=196_608)
    parser.add_argument("--chunk", type=int, default=32_768)
    parser.add_argument("--envs", type=int, default=2)
    parser.add_argument("--seed", type=int, default=6301)
    parser.add_argument("--action-scale", type=float, default=0.5)
    parser.add_argument("--start-seeds", default="500000-500011")
    parser.add_argument("--start-fraction", type=float, default=0.5)
    parser.add_argument("--val-seed", type=int, default=1016100)
    parser.add_argument("--val-episodes", type=int, default=10)
    parser.add_argument("--final-seed", type=int, default=1017300)
    parser.add_argument("--final-episodes", type=int, default=20)
    parser.add_argument("--resume", type=Path, help="stage folder of an earlier staged PPO policy")
    parser.add_argument("--start-states", type=Path, help="reuse a recorded start_states.pkl")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--select-only",
        action="store_true",
        help="rank the checkpoints already validated in --output and run the held-out evaluation",
    )
    args = parser.parse_args()
    if args.select_only:
        select(args)
        return

    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

    torch.set_num_threads(1)
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.time()
    log = args.output / "progress.jsonl"

    def note(**entry):
        entry["wall_s"] = round(time.time() - started, 1)
        with log.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(entry, default=float) + "\n")
        print(json.dumps(entry, default=float), flush=True)

    if args.start_states:
        states = pickle.loads(args.start_states.read_bytes())
    else:
        first, last = (int(v) for v in args.start_seeds.split("-"))
        states = record_start_states(args.course, range(first, last + 1))
    (args.output / "start_states.pkl").write_bytes(pickle.dumps(states))
    note(stage="start_states", count=len(states), phases=sorted({s["phase_name"] for s in states}))

    eval_env = StagedEnv(args.course)
    eval_env.action_scale = args.action_scale
    baseline = _summary(
        _benchmark_parallel(eval_env, None, args.val_episodes, args.val_seed, 2), args.course
    )
    note(stage="validation", checkpoint="controller", **baseline)

    factories = [
        partial(
            _make,
            args.course,
            states,
            args.start_fraction,
            str(args.output / f"env-{i}.csv"),
            args.action_scale,
        )
        for i in range(args.envs)
    ]
    vector = (
        DummyVecEnv(factories) if args.envs == 1 else SubprocVecEnv(factories, start_method="spawn")
    )
    vector.seed(args.seed)
    if args.resume:
        policy = PPO.load(args.resume / POLICY_FILE, env=vector, device="cpu")
        policy.tensorboard_log = str(args.output / "tensorboard")
    else:
        policy = PPO(
            "MlpPolicy",
            vector,
            device="cpu",
            seed=args.seed,
            n_steps=512,
            batch_size=128,
            learning_rate=3e-4,
            policy_kwargs={"net_arch": {"pi": [128, 128], "vf": [128, 128]}, "log_std_init": -1.7},
            verbose=0,
            tensorboard_log=str(args.output / "tensorboard"),
        )
    recorder_stage = args.output / "training"
    recorder_stage.mkdir()
    (recorder_stage / "manifest.json").write_text(
        json.dumps(_manifest(args, eval_env, policy, 0), indent=2), encoding="utf-8"
    )
    checkpoints = []
    best = ("controller", baseline)
    try:
        # ``--timesteps`` counts new transitions, also when continuing a policy.
        target = policy.num_timesteps + args.timesteps
        while policy.num_timesteps < target:
            policy.learn(
                total_timesteps=args.chunk,
                reset_num_timesteps=False,
                tb_log_name=args.course,
                callback=training_recorder(recorder_stage),
            )
            stage = args.output / "checkpoints" / f"{policy.num_timesteps:07d}"
            stage.mkdir(parents=True)
            policy.save(stage / POLICY_FILE)
            (stage / "manifest.json").write_text(
                json.dumps(_manifest(args, eval_env, policy, policy.num_timesteps), indent=2),
                encoding="utf-8",
            )
            summary = _summary(
                _benchmark_parallel(eval_env, stage, args.val_episodes, args.val_seed, 2),
                args.course,
            )
            checkpoints.append((stage, summary))
            note(
                stage="validation",
                checkpoint=stage.name,
                transitions=policy.num_timesteps,
                **summary,
            )
            if _score(summary) > _score(best[1]):
                best = (stage, summary)
    finally:
        vector.close()

    result = {"course": args.course, "validation_seed": args.val_seed, "selected": None}
    final_base = _benchmark_parallel(eval_env, None, args.final_episodes, args.final_seed, 2)
    result["controller_final"] = _summary(final_base, args.course)
    if best[0] != "controller":
        chosen = args.output / args.course
        shutil.copytree(best[0], chosen)
        final = _benchmark_parallel(eval_env, chosen, args.final_episodes, args.final_seed, 2)
        result["selected"] = best[0].name
        result["policy_final"] = _summary(final, args.course)
        (chosen / "evaluation.json").write_text(
            json.dumps(
                {
                    "course": args.course,
                    "passed": result["policy_final"]["passed"],
                    "policy": final,
                    "body_reference": final_base,
                },
                indent=1,
                default=float,
            ),
            encoding="utf-8",
        )
    result["validation"] = [{"checkpoint": "controller", **baseline}] + [
        {"checkpoint": s.name, **summary} for s, summary in checkpoints
    ]
    (args.output / "summary.json").write_text(
        json.dumps(result, indent=2, default=float), encoding="utf-8"
    )
    note(stage="final", **{k: v for k, v in result.items() if k != "validation"})
    eval_env.close()


def select(args):
    rows = [
        json.loads(line)
        for line in (args.output / "progress.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    validation = [r for r in rows if r.get("stage") == "validation"]
    best = max(validation, key=_score)
    eval_env = StagedEnv(args.course)
    result = {
        "course": args.course,
        "validation_seed": args.val_seed,
        "selection_rule": "contact limits, completions, falls, max and mean riser contact",
        "selected": best["checkpoint"],
        "validation": validation,
    }
    final_base = _benchmark_parallel(eval_env, None, args.final_episodes, args.final_seed, 2)
    result["controller_final"] = _summary(final_base, args.course)
    if best["checkpoint"] != "controller":
        chosen = args.output / args.course
        if chosen.exists():
            shutil.rmtree(chosen)
        shutil.copytree(args.output / "checkpoints" / best["checkpoint"], chosen)
        final = _benchmark_parallel(eval_env, chosen, args.final_episodes, args.final_seed, 2)
        result["policy_final"] = _summary(final, args.course)
        (chosen / "evaluation.json").write_text(
            json.dumps(
                {
                    "course": args.course,
                    "passed": result["policy_final"]["passed"],
                    "policy": final,
                    "body_reference": final_base,
                },
                indent=1,
                default=float,
            ),
            encoding="utf-8",
        )
    (args.output / "summary.json").write_text(
        json.dumps(result, indent=2, default=float), encoding="utf-8"
    )
    print(json.dumps({k: v for k, v in result.items() if k != "validation"}, default=float))
    eval_env.close()


if __name__ == "__main__":
    main()
