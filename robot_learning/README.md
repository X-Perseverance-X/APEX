# META-1 offline robot learning

This is a **separate physics and RL workspace**. It imports no Pi, ESP, USB,
PWM, SSH, or live command module. It cannot move the physical robot.

## Current scope

- 18-DOF MuJoCo physics proxy: six coxa, femur and tibia chains, floating body,
  ground contacts, IMU site, camera and LiDAR reference sites.
- `+X` forward, `+Y` left, `+Z` up; leg order `RF RM RR LF LM LR`.
- Measured link lengths and hip positions come from firmware v57.33. A test
  compares those values to the current firmware source.
- Body mass, inertias, foot friction, servo torque and response are **provisional**.
  The model is suitable for software experiments, not yet for physical policy
  transfer. The simple body box is a collision proxy, not a CAD mesh.
- Gymnasium stand and velocity-command tasks with a 50 Hz joint-target policy
  interface, joint-rate limits, contact observations, reproducible seeds, and
  narrow optional dynamics randomization. PPO is an optional training extra.
- Checkpoints are explicitly marked `policy_export_allowed: false`.

## Layout

```text
robot_learning/
  pyproject.toml                  installable Python package and tool configuration
  src/apex_rl/assets/meta1.json  versioned robot geometry/dynamics contract
  src/apex_rl/contract.py         units, frame, leg order, symmetry validation
  src/apex_rl/model.py            MJCF generator (physics proxy)
  src/apex_rl/env.py              Gymnasium tasks and rewards
  src/apex_rl/cli.py              build, smoke, train, evaluate entry points
  tests/                          geometry and physics contract tests
  tools/render_preview.py         offline visualization snapshot
  config/physical_measurements.template.json
  docs/WORKPLAN.md                staged system-identification and RL program
  generated/                      derived MJCF/PNG, ignored by Git
  runs/                           local checkpoints and logs, ignored by Git
  experiments/pilot-smoke/         curated small example run, tracked in Git
```

## Use on this Windows workstation

The isolated environment prepared here is `.runtime/sim-rl-env`. From the
repository root in PowerShell:

```powershell
.runtime\sim-rl-env\Scripts\python.exe -m pytest robot_learning\tests -q
.runtime\sim-rl-env\Scripts\python.exe -m apex_rl.cli build
.runtime\sim-rl-env\Scripts\python.exe -m apex_rl.cli smoke --steps 100
.runtime\sim-rl-env\Scripts\python.exe robot_learning\tools\render_preview.py
.runtime\sim-rl-env\Scripts\python.exe -m mujoco.viewer --mjcf=robot_learning\generated\meta1.xml
```

For a new machine, create a Python 3.12 virtual environment and install the
package from `robot_learning` with `python -m pip install -e ".[dev,train]"`.
Train and compare one **simulation-only** pilot:

```powershell
.runtime\sim-rl-env\Scripts\python.exe -m apex_rl.cli train --task stand --timesteps 10000 --seed 42 --output robot_learning\runs\stand-pilot
.runtime\sim-rl-env\Scripts\python.exe -m apex_rl.cli evaluate --run robot_learning\runs\stand-pilot --episodes 5
```

The 2,048-step setup check completed, but its deterministic stand score was
essentially the same as the zero-action baseline (2498.31 vs 2498.49 over
three episodes). That checks the training pipeline; it is **not a learned gait**.
Do not transfer `policy_sim_only.zip` to the Pi or ESP.
The small pilot checkpoint, manifest and evaluation are retained under
`experiments/pilot-smoke/` so this first software check is backed up in Git.

## Stack decision

| Layer | Choice now | Why |
| --- | --- | --- |
| Physics | MuJoCo MJCF | Compiled dynamics, contacts, Windows CPU setup; fast model iteration. |
| Task interface | Gymnasium | Standard `reset/step`, seed and environment validation. |
| Initial learner | Python + Stable-Baselines3 PPO | Reproducible baseline and mature tooling. Physics and tensor math run in compiled code; Python is the orchestration layer. |
| High-throughput later | Isaac Lab/Newton | Only after GPU/driver check and physical model validation; optional, not the source of truth. |
| ROS integration | Existing ROS 2/Gazebo workspace | Keep for sensors, navigation and software-in-the-loop. Avoid mixing it into the training loop. |
| Rust | Possible future typed low-latency transport | Changing to Rust alone would not speed MuJoCo/PyTorch kernels or solve sim-to-real mismatch. |

MuJoCo model/Python API: [official documentation](https://mujoco.readthedocs.io/en/stable/python.html).
Gymnasium environment contract: [official guide](https://gymnasium.farama.org/tutorials/gymnasium_basics/environment_creation/).
PPO/custom environment checks: [Stable-Baselines3 documentation](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html).
Isaac Lab system requirements: [NVIDIA documentation](https://docs.nvidia.com/isaaclab/isaac-lab/installation).

Read [the staged work plan](docs/WORKPLAN.md) before extending the model.
