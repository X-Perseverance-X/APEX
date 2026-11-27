"""Reproduce the staged-ascent measurements: flat gaits, climbs, phase traces.

Simulation only. Examples, from the repository root:

    python robot_learning/tools/staged_experiments.py flat --seeds 10 --output robot_learning/experiments/staged-ascent/flat.json
    python robot_learning/tools/staged_experiments.py climb --course hybrid-staged-up-13 --seed 1017300 --episodes 20 --output out.json
    python robot_learning/tools/staged_experiments.py phases --course hybrid-spread-up-20 --seeds 1013208,1013210,1013213 --output out.json
"""

from __future__ import annotations

import argparse
import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from apex_rl.climb_trace import deficit_attribution, phase_summary, trace_episode
from apex_rl.curriculum import _benchmark_parallel, _passed_stage
from apex_rl.gait_lab import COMMANDS, run_flat_trial
from apex_rl.locomotion import COURSES
from apex_rl.staged_ascent import make_env

FLAT_KEYS = (
    "mean_vx_m_s",
    "mean_vy_m_s",
    "mean_wz_rad_s",
    "loaded_sole_slide_mm",
    "sole_slide_mm",
    "recontacts",
    "roll_rms_deg",
    "pitch_rms_deg",
    "height_std_mm",
    "touchdown_speed_p95_m_s",
    "touchdown_peak_p95_weight",
    "hard_touchdowns",
    "saturated_any_s",
    "saturated_joint_s",
)


def _flat_one(job):
    controller, mode, seed = job
    return run_flat_trial(controller, mode, seed)


def flat(args):
    jobs = [
        (controller, mode, args.seed + k)
        for controller in ("reference", "phase")
        for mode in COMMANDS
        for k in range(args.seeds)
    ]
    with ProcessPoolExecutor(args.workers) as pool:
        trials = list(pool.map(_flat_one, jobs))
    summary = {}
    for controller in ("reference", "phase"):
        for mode in COMMANDS:
            rows = [t for t in trials if t["controller"] == controller and t["mode"] == mode]
            entry = {"trials": len(rows), "falls": sum(t["fallen"] for t in rows)}
            for key in FLAT_KEYS:
                entry[key] = float(np.mean([t[key] for t in rows]))
            joints = rows[0]["saturated_s_by_joint"].keys()
            entry["saturated_s_by_joint"] = {
                j: float(np.mean([t["saturated_s_by_joint"][j] for t in rows])) for j in joints
            }
            if mode == "stop":
                for key in ("stop_foot_centre_travel_m", "stop_overshoot_m"):
                    entry[key] = float(np.mean([t[key] for t in rows]))
                settle = [t["settle_s"] for t in rows if t["settle_s"] is not None]
                entry["settled_trials"] = len(settle)
                entry["mean_settle_s"] = float(np.mean(settle)) if settle else None
                entry["standing_at_end"] = sum(t["standing"] for t in rows)
            summary[f"{controller}/{mode}"] = entry
    return {"seeds": [args.seed, args.seed + args.seeds - 1], "summary": summary, "trials": trials}


def climb(args):
    env = make_env(args.course)
    try:
        run = Path(args.run) if args.run else None
        report = _benchmark_parallel(env, run, args.episodes, args.seed, args.workers)
    finally:
        env.close()
    report["passed_gate"] = _passed_stage(report, body_support=COURSES[args.course].body_support)
    return {"course": args.course, "run": args.run, "first_seed": args.seed, **report}


def phases(args):
    out = []
    for seed in (int(s) for s in args.seeds.split(",")):
        env = make_env(args.course)
        policy = None
        if args.run:
            from apex_rl.curriculum import _read_policy

            policy = _read_policy(Path(args.run), env, exact_course=True)
        rows, info = trace_episode(env, policy, seed)
        out.append(
            {
                "seed": seed,
                "success": info["success"],
                "fallen": info["fallen"],
                "peak_upper_contacts": info.get("peak_upper_contacts"),
                "phases": phase_summary(rows, env.stand_height),
                "front_deficit": deficit_attribution(rows, "front"),
                "transfer_deficit": deficit_attribution(rows, "transfer"),
                "middle_deficit": deficit_attribution(rows, "middle"),
                "rear_deficit": deficit_attribution(rows, "rear"),
                "plan_log": [
                    {k: v for k, v in e.items() if k != "reasons"}
                    for e in getattr(env.body_motion, "log", [])
                    if "failed" not in e
                ][:80],
                "blocked_reasons": next(
                    (
                        e["reasons"]
                        for e in reversed(getattr(env.body_motion, "log", []))
                        if "failed" in e
                    ),
                    None,
                ),
            }
        )
        env.close()
    return {"course": args.course, "run": args.run, "episodes": out}


def summarize(args):
    """Stage table from the climb result files in ``--folder``."""
    rows = []
    for path in sorted(Path(args.folder).glob("*.json")):
        d = json.loads(path.read_text(encoding="utf-8"))
        rows.append(
            {
                "file": path.name,
                "course": d["course"],
                "run": d.get("run"),
                "first_seed": d["first_seed"],
                "episodes": d["episodes"],
                "successes": d["successes"],
                "falls": d["falls"],
                "body_impacts": d.get("body_impacts"),
                "max_riser_contact_s": d.get("max_riser_contact_s"),
                "mean_riser_contact_s": d.get("mean_riser_contact_s"),
                "interleg_peak_m": d.get("interleg_peak_m"),
                "leg_stair_peak_m": d.get("leg_stair_peak_m"),
                "min_knee_deg": d.get("min_knee_deg"),
                "peak_upper_contacts": d.get("peak_upper_contacts"),
                "min_upper_foot_gap_m": d.get("min_upper_foot_gap_m"),
                "mean_servo_saturation_fraction": d.get("mean_servo_saturation_fraction"),
                "upper_contacts_per_episode": [o["peak_upper_contacts"] for o in d["outcomes"]],
                "failed_in_phase": [
                    o.get("staged_phase") for o in d["outcomes"] if not o["success"]
                ],
                "passed_gate": d["passed_gate"],
            }
        )
    return {"rows": rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    f = sub.add_parser("flat")
    f.add_argument("--seeds", type=int, default=10)
    f.add_argument("--seed", type=int, default=2026001)
    c = sub.add_parser("climb")
    c.add_argument("--course", required=True, choices=COURSES)
    c.add_argument("--run")
    c.add_argument("--seed", type=int, required=True)
    c.add_argument("--episodes", type=int, default=20)
    p = sub.add_parser("phases")
    p.add_argument("--course", required=True, choices=COURSES)
    p.add_argument("--run")
    p.add_argument("--seeds", required=True)
    s = sub.add_parser("summarize")
    s.add_argument("--folder", type=Path, required=True)
    for command in (f, c, p, s):
        command.add_argument("--workers", type=int, default=2)
        command.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    start = time.time()
    commands = {"flat": flat, "climb": climb, "phases": phases, "summarize": summarize}
    result = commands[args.command](args)
    result["wall_s"] = round(time.time() - start, 1)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=1, default=float), encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
