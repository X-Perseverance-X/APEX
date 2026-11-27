"""Train, measure and watch the offline META-1 locomotion policy."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
import time
from functools import partial
from pathlib import Path

import mujoco
import numpy as np

from .ascent_pose import KNEE_CLEARANCE_LIMIT
from .course_observability import training_recorder, watch_stage, write_report
from .leg_contacts import OVERLAP_LIMIT_M
from .locomotion import COURSES, LocomotionEnv

STAGES = tuple(name for name in COURSES if name != "home-corner-preview")
SCHEMA = 1
EVALUATION_SEED = 1_000_000
MIN_SUCCESS = 0.80
MAX_FALLS = 0.10
MAX_RISER_CONTACT_S = 0.25
BRACE_MIN_SUCCESS = 0.90
BRACE_MAX_FALLS = 0.05
BRACE_EVAL_EPISODES = 20
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
    "46a7718e7241d72350e03127c90e4fafba0a82df923c3b7066fa52c32ee9ba95": {
        "step-05",
        "down-05",
        "stairs-05",
        "stairs-down-05",
    },
    "fae96baa03d52d3e7a41e838c65d901a4e209a2278876d93396bb90f86a79dec": {
        "hybrid-step-05",
        "hybrid-down-05",
        "hybrid-narrow-05",
        "hybrid-narrow-08",
    },
    "9d1139c9179d441749299a0f92313a9a6acba40f1933a05739e33a6554384a98": {
        "hybrid-narrow-down-08",
        "hybrid-narrow-down-10",
    },
    "4d5bb4fc244dda3d7dbee77a5188edb3c89f7e0d94213bdf4229a0a8d37bcb63": {
        "hybrid-lab-up-07",
        "hybrid-lab-down-10",
        "hybrid-lab-down-12",
    },
    "ecd307904ecf17c432e4cd87bcc2d3bd545f339dbe5a0e3d507fe5ea6fbbb2b8": {
        "hybrid-lab-down-15-single",
        "hybrid-lab-down-15-wide",
    },
    "2d3a53cd3acb46b46327d7c10d337f920dc716e58fd52f8dbee4ae953fb757b4": {
        "hybrid-lab-down-15-wide",
    },
    "c5b74395ef52f20fbc1ae7791436a99fc4ed8b07b7f4fa0c632c43e8aeb2b4b3": {
        "hybrid-lab-up-07-slow",
    },
    "52ae7e0e3813d47545ccb18e992928da7d9c87f9d5766a4207573a0903fc8d89": {
        "hybrid-contact-up-05",
    },
    "b1458877e84c1232ffb8a979077825708fcd56004038f9482e0c84805594f350": {
        "hybrid-contact-up-05",
        "hybrid-contact-up-07",
        "hybrid-contact-up-15",
        "hybrid-contact-stairs-up-10",
        "hybrid-rubble-01",
    },
    "8d294a8593263dad55f8d9547abe018058ad2686e58db1ab2c08c5abe97153b1": {
        "hybrid-pose-up-05",
        "hybrid-pose-up-07",
        "hybrid-pose-up-10",
    },
    "cc008c71630c095ea590864c619c166e9ae36d483316a8a7cff3d7552008daae": {
        "hybrid-steep-up-08",
        "hybrid-steep-up-09",
        "hybrid-steep-up-10",
        "hybrid-steep-up-12",
        "hybrid-steep-up-15",
    },
    "1500ef8a5ddebfe6b1481e1ab9112cd807ba7eda85e74fa914a6737d70fb96b8": {
        "hybrid-diagonal-up-12",
    },
    "5938ddbb3a55a04eb745ada63e55bde6df5524496b780654d9967f500dc971c7": {
        "hybrid-transfer-up-12",
    },
    "2e47318eaf206cb9ba4ad37f8fd6e07f2d7cd20d1d8c277c71bee7ec314b2565": {
        "hybrid-probe-up-11",
        "hybrid-probe-up-12",
        "hybrid-probe-up-14",
    },
    "c80e1481f30dc92c8010f7c1d3626579e81b17dad632a90fa4c66cc9bcdc1218": {
        "hybrid-home-up-10",
        "hybrid-home-down-10",
    },
    "e316d00e667d3d171d203b402e14d3bd459759703c68ee2ab8dd1621ea1dba26": {
        "hybrid-home-down-20-single",
    },
    "e14016b3b23185a4613947451c641cf25ad869aeb24d2651444821eb83330d26": {
        "hybrid-home-up-20-single",
    },
    "de3c16e2749fcd9c0182d4e10be6b4872ecf64a5ce1b7a0e3fcbc5a4eb21e065": {
        "hybrid-brace-up-13",
    },
}


def add_commands(sub: argparse._SubParsersAction, project: Path) -> None:
    view = sub.add_parser("course-view", help="watch the tripod reference or a saved policy")
    view.add_argument("--course", choices=COURSES)
    view.add_argument("--run", type=Path, help="stage folder containing policy and manifest")
    view.add_argument("--static", action="store_true", help="inspect the parkour at rest")
    view.add_argument("--seed", type=int, default=42)
    view.add_argument("--randomize", action="store_true", help="replay a benchmark seed")
    view.add_argument("--stabilized-gaze", action="store_true", help="drive the simulated gimbal")
    view.add_argument("--front-view", action="store_true", help="look through the robot camera")
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
    train.add_argument("--through", choices=STAGES)
    train.add_argument("--timesteps", type=int, default=200_000, help="new transitions per stage")
    train.add_argument("--envs", type=int, default=4)
    train.add_argument(
        "--gamma", type=float, default=0.99, help="PPO reward discount per control step"
    )
    train.add_argument("--gae-lambda", type=float, default=0.95, help="PPO advantage trace decay")
    train.add_argument("--eval-episodes", type=int, default=10)
    train.add_argument("--eval-seed", type=int, default=EVALUATION_SEED)
    train.add_argument("--seed", type=int, default=42)
    train.add_argument("--resume", type=Path, help="load a previous stage policy and optimizer")
    train.add_argument("--output", type=Path, default=project / "runs/parkour")
    camera = sub.add_parser("course-camera", help="measure synthetic stair risers from RGB")
    camera.add_argument("--course", choices=STAGES, required=True)
    camera.add_argument("--output", type=Path, required=True)
    sensors = sub.add_parser("course-sensors", help="measure gaze correction and planar LiDAR")
    sensors.add_argument("--output", type=Path, required=True)
    for command in (view, bench, train):
        command.add_argument(
            "--body-control",
            action="store_true",
            help="use terrain-aware wave gait and body control",
        )
    for command in (view, bench):
        command.add_argument("--camera-gate", action="store_true", help="start wave gait from RGB")


def _fingerprint(env: LocomotionEnv) -> str:
    # Include the geometry and implementation that define policy observations and actions.
    source = [
        Path(__file__).with_name(name).read_text(encoding="utf-8")
        for name in (
            "locomotion.py",
            "env.py",
            "model.py",
            "body_motion.py",
            "leg_contacts.py",
            "ascent_pose.py",
            "home_stairs.py",
            "body_support.py",
        )
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
        or manifest.get("attitude_control", False) != env.attitude_control
        or manifest.get("body_control", False) != (env.body_motion is not None)
        or manifest.get("body_support", False) != env.course.body_support
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
    if env.hybrid:
        for key in ("approach_speed_m_s", "precision_speed_m_s", "rear_riser_contact_steps"):
            body_stats[key] = float(np.mean([item[key] for item in outcomes]))
    if env.course.link_contacts:
        for key in ("interleg_peak_m", "rear_step_peak_m"):
            body_stats[key] = max(item[key] for item in outcomes)
        body_stats["link_overlap_passed"] = (
            max(body_stats["interleg_peak_m"], body_stats["rear_step_peak_m"]) <= OVERLAP_LIMIT_M
        )
    if env.attitude_control:
        body_stats["min_knee_deg"] = min(item["min_knee_deg"] for item in outcomes)
        knee_limit = (
            math.radians(-152) if env.course.home_stairs is not None else KNEE_CLEARANCE_LIMIT
        )
        body_stats["knee_clearance_passed"] = bool(
            np.deg2rad(body_stats["min_knee_deg"]) > knee_limit
        )
        body_stats["peak_target_rpy_deg"] = np.max(
            [item["peak_target_rpy_deg"] for item in outcomes], axis=0
        ).tolist()
    if env.course.ledge_transfer:
        body_stats["peak_upper_contacts"] = max(item["peak_upper_contacts"] for item in outcomes)
        body_stats["forward_transfer_m"] = max(item["forward_transfer_m"] for item in outcomes)
    if env.course.home_stairs is not None:
        body_stats["leg_stair_peak_m"] = max(item["leg_stair_peak_m"] for item in outcomes)
        side_times = env.model.opt.timestep * np.array(
            [item["riser_contact_substeps"] for item in outcomes]
        )
        body_stats["mean_riser_contact_s"] = float(side_times.mean())
        body_stats["max_riser_contact_s"] = float(side_times.max())
        body_stats["riser_clearance_passed"] = (
            body_stats["max_riser_contact_s"] <= MAX_RISER_CONTACT_S
        )
        body_stats["sole_top_peak_m"] = max(item["sole_top_peak_m"] for item in outcomes)
        body_stats["link_overlap_passed"] &= body_stats["leg_stair_peak_m"] <= OVERLAP_LIMIT_M
    if env.course.body_support:
        body_stats.update(
            body_impacts=sum(item["body_impact"] for item in outcomes),
            mean_body_support_s=float(np.mean([item["body_support_s"] for item in outcomes])),
            max_body_support_weight=max(item["body_support_weight"] for item in outcomes),
            max_body_impact_weight=max(item["body_impact_weight"] for item in outcomes),
            mean_servo_saturation_fraction=float(
                np.mean([item["servo_saturation_fraction"] for item in outcomes])
            ),
        )
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
    env_type = LocomotionEnv
    if course == "home-corner-preview":
        args.static = True
    if args.camera_gate:
        from .course_camera import CameraGatedCourse

        env_type = CameraGatedCourse
    randomized = args.command == "course-bench" or getattr(args, "randomize", False)
    env = env_type(course or "flat", randomize=randomized, body_control=body_control)
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

    from .sensor_lab import GazeController

    obs, _ = env.reset(seed=args.seed)
    if args.static:
        viewer.launch(env.model, env.data)
        return
    gaze = GazeController(env, args.seed) if args.stabilized_gaze else None
    with viewer.launch_passive(env.model, env.data) as window:
        with window.lock():
            window.cam.distance = 1.65
            window.cam.azimuth = 125
            window.cam.elevation = -28
            if args.front_view:
                window.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                window.cam.fixedcamid = env.model.camera("front_camera").id
        while window.is_running():
            start = time.monotonic()
            with window.lock():
                if gaze is not None:
                    gaze.update()
                obs, _, done, truncated, info = env.step(_action(env, policy, obs))
                window.cam.lookat[:] = (env.data.qpos[0] + 0.20, 0, env.data.qpos[2])
            if env.hybrid:
                details = (
                    "TRIPOD  |  FAST APPROACH"
                    if env.gait == "tripod"
                    else "WAVE  |  PRECISE FOOT PLACEMENT"
                )
                if "camera_gate" in info:
                    height = info.get("camera_riser_m")
                    details += "\nRGB " + (
                        f"{100 * height:.1f} cm" if height is not None else "searching"
                    )
                if gaze is not None:
                    yaw_error, pitch_error = gaze.error()
                    details += f"\nGimbal yaw {yaw_error:+.1f} deg | pitch {pitch_error:+.1f} deg"
                if "interleg_peak_m" in info:
                    details += (
                        f"\nLink overlap peak {info['interleg_peak_m'] * 1000:.1f} mm"
                        f" | rear / stair {info['rear_step_peak_m'] * 1000:.1f} mm"
                    )
                if env.attitude_control:
                    c = env.body_motion
                    roll, pitch, yaw = np.rad2deg([c.roll, c.pitch, c.yaw])
                    rotation = env.data.xmat[c.base_id].reshape(3, 3)
                    measured = np.rad2deg(
                        [
                            np.arctan2(rotation[2, 1], rotation[2, 2]),
                            np.arctan2(rotation[2, 0], np.hypot(rotation[2, 1], rotation[2, 2])),
                            np.arctan2(rotation[1, 0], rotation[0, 0]),
                        ]
                    )
                    details += f"\nRPY targets {roll:+.1f} / {pitch:+.1f} / {yaw:+.1f} deg"
                    details += "\nRPY measured " + " / ".join(f"{axis:+.1f}" for axis in measured)
                    details += f"\nKnee minimum {info['min_knee_deg']:.1f} deg"
                if env.course.ledge_transfer:
                    c = env.body_motion
                    phase = (
                        "rear transfer"
                        if c.rear_transfer
                        else "front setup"
                        if c.fore_probe
                        else "wave"
                    )
                    details += f"\n{phase} | upper contacts peak {info['peak_upper_contacts']}/6"
                    details += f" | CoM shift {1000 * info['forward_transfer_m']:.0f} mm"
                if env.course.home_stairs is not None:
                    side_time = info["riser_contact_substeps"] * env.model.opt.timestep
                    details += f"\nRiser contact {side_time:.2f} s"
                    details += f" | rigid links {1000 * info['leg_stair_peak_m']:.2f} mm"
                if env.brace is not None:
                    details += f"\nBody support {env.brace.support_load:.0%} of weight"
                    details += f" | time {info['body_support_s']:.2f} s"
                    details += f" | servo saturation {info['servo_saturation_fraction']:.0%}"
                window.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, "Gait", details))
            window.sync()
            if done or truncated:
                print(json.dumps(info), flush=True)
                with window.lock():
                    obs, _ = env.reset(seed=args.seed)
                    if gaze is not None:
                        gaze = GazeController(env, args.seed)
            time.sleep(max(0.0, 0.02 - (time.monotonic() - start)))


def _make_env(course: str, monitor_path: str, body_control: bool):
    from stable_baselines3.common.monitor import Monitor

    return Monitor(
        LocomotionEnv(course, record_pose=True, body_control=body_control, train_near_step=True),
        filename=monitor_path,
        info_keywords=("success", "fallen", "progress_m", "touchdowns", "pack_voltage_proxy_v"),
    )


def _passed_stage(evaluation, *, body_support):
    count = evaluation["episodes"]
    return (
        (not body_support or count >= BRACE_EVAL_EPISODES)
        and evaluation["successes"] / count >= (BRACE_MIN_SUCCESS if body_support else MIN_SUCCESS)
        and evaluation["falls"] / count <= (BRACE_MAX_FALLS if body_support else MAX_FALLS)
        and not evaluation.get("body_impacts", 0)
        and evaluation.get("link_overlap_passed", True)
        and evaluation.get("knee_clearance_passed", True)
        and evaluation.get("riser_clearance_passed", True)
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
                "body_control": eval_env.body_motion is not None,
                "attitude_control": eval_env.attitude_control,
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
                layers = [128, 128] if eval_env.course.body_support else [64, 64]
                policy_kwargs = {"net_arch": {"pi": layers, "vf": layers}}
                if eval_env.body_motion is not None:
                    policy_kwargs["log_std_init"] = -1.7
                policy = PPO(
                    "MlpPolicy",
                    vector,
                    device="cpu",
                    seed=args.seed,
                    n_steps=512,
                    batch_size=128,
                    learning_rate=3e-4,
                    gamma=args.gamma,
                    gae_lambda=args.gae_lambda,
                    policy_kwargs=policy_kwargs,
                    verbose=1,
                    tensorboard_log=str(args.output / "tensorboard"),
                )
            else:
                policy.set_env(vector)
            policy.tensorboard_log = str(args.output / "tensorboard")
            manifest.update(
                ppo_epochs=policy.n_epochs,
                ppo_gamma=policy.gamma,
                ppo_gae_lambda=policy.gae_lambda,
                steps_per_worker=policy.n_steps,
                net_arch=policy.policy.net_arch,
                body_support=eval_env.course.body_support,
            )
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
        passed = _passed_stage(evaluation, body_support=COURSES[course].body_support)
        baseline_key = "body_reference" if eval_env.body_motion is not None else "tripod"
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
    if args.command == "course-sensors":
        from .sensor_lab import sensor_report

        sensor_report(args.output)
    elif args.command == "course-camera":
        from .course_camera import capture_course

        capture_course(args.course, args.output)
    elif args.command == "course-watch":
        watch_stage(args.run, live=args.live)
    elif args.command == "course-report":
        print(write_report(args.run))
    elif args.command == "course-train":
        if not 1 <= args.envs <= 32:
            parser.error("--envs must be between 1 and 32")
        if not (0 < args.gamma < 1 and 0 <= args.gae_lambda <= 1):
            parser.error("gamma must be in (0, 1), and GAE lambda must be in [0, 1]")
        if args.timesteps < 512 or not 10 <= args.eval_episodes <= 100:
            parser.error("use at least 512 training transitions and 10–100 evaluation episodes")
        hybrid = args.start.startswith("hybrid-")
        if args.through is None:
            if COURSES[args.start].brace_rear_placement:
                args.through = args.start
            elif hybrid:
                brace = COURSES[args.start].body_support
                args.through = next(
                    name
                    for name in reversed(STAGES)
                    if COURSES[name].body_support == brace
                    and not COURSES[name].brace_rear_placement
                )
            else:
                args.through = "stairs-down-20"
        if hybrid != args.through.startswith("hybrid-"):
            parser.error("use a single controller family in each curriculum run")
        if COURSES[args.start].body_support != COURSES[args.through].body_support:
            parser.error("body-support courses use a separate policy and curriculum")
        if COURSES[args.start].brace_rear_placement != COURSES[args.through].brace_rear_placement:
            parser.error("the rear-placement experiment runs as a standalone course")
        if COURSES[args.start].body_support and args.eval_episodes < BRACE_EVAL_EPISODES:
            parser.error(
                f"body-support courses require at least {BRACE_EVAL_EPISODES} evaluation episodes"
            )
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
        if args.camera_gate:
            if args.course != "hybrid-step-05" or args.run:
                parser.error(
                    "--camera-gate uses --course hybrid-step-05 without a policy checkpoint"
                )
            if args.command == "course-view" and args.static:
                parser.error("--camera-gate requires a running viewer")
            if args.command == "course-view" and args.stabilized_gaze:
                parser.error("the RGB approach has its own camera angle calibration")
        _inspect(args)
