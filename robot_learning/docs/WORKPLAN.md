# META-1 simulation-to-reality work plan

## Why the pipeline is staged

An attractive 3D mesh is only a visual twin. Locomotion RL needs a **dynamics
twin** with correct joint direction, collision geometry, masses, center of mass,
actuator lag/limits and foot contact. The current primitive model is the first
physics proxy. No real-robot deployment is enabled.

## Program and exit gates

| Stage | Work | Deliverable and evidence | Exit gate |
| --- | --- | --- | --- |
| 0. Source contract, 1–2 days | Reconcile firmware v57.33, ROS description and kinematics lab; fix frame/leg convention; mark unknown dynamics. | Versioned geometry contract, generated 18-DOF MJCF, CI checks. | All 18 joint signs, ranges and neutral foot positions reviewed against physical robot. |
| 1. System identification, 3–7 days | Weigh body/links; measure COM; record loaded servo PWM-angle and step response, supply droop, foot friction/contact delay. | Filled measurement template, CSV traces and calibration report. | Predicted vs measured foot position and loaded servo timing within agreed tolerances. |
| 2. Physics validation, 2–4 days | Drop/stand, individual joint, isolated leg lift, friction and current proxy checks; compare video/IMU/contact logs. | Reproducible tests and uncertainty ranges. | Stable stand, no unexplained penetration/contact, model error reported quantitatively. |
| 3. RL curriculum, 4–10 days | Stand recovery, in-place rotation, forward/back/side velocity, then slopes and uneven terrain. Start proprioceptive; randomized mass/friction/lag/latency/noise only within measured ranges. | Seeded PPO runs, TensorBoard logs, checkpoints, zero-action and firmware-gait baselines. | Held-out seeds and randomized environments beat both baselines without excessive falls or torque. |
| 4. Policy interface, 2–4 days | Freeze observation order/units and 50 Hz action contract; export CPU inference candidate; add command timeout and safety clamp outside policy. | Golden replay vectors from trainer and Pi, version/hash matching. | Identical policy outputs on workstation and Pi; inference fits measured CPU/power budget. |
| 5. Controlled physical transfer, 3–7 days | PWM cut dry run, supported single leg, six legs suspended, short platform walk, then floor at low speed. Human kill switch throughout. | Time-aligned commanded/observed telemetry, abort logs. | No joint limit breach; controlled recovery from dropped commands, low voltage and IMU fault. |
| 6. Perception/navigation, separate track | Keep SLAM, route planner and object tracking outside the first locomotion policy. Add terrain observations after the walking policy is stable; validate camera/LiDAR transforms. | Simulated sensor feeds and real replay benchmark. | Consistent route and obstacle decisions across replay and simulation. |

These are **engineering estimates**, not guaranteed calendar dates. Stage 1 can
take longer if there is no encoder, current sensor or calibrated scale.

## Learning contract

- Initial observation (72 float32): local body linear/angular velocity (6),
  projected gravity (3), 18 joint position offsets, 18 joint velocities,
  18 previous actions, commanded planar/yaw velocity (3), 6 foot contacts.
- Initial action (18 float32): normalized residual target around safe stand,
  clamped to ±0.25 rad and joint limits; a joint-rate limiter feeds the model.
- Physics step 2 ms; policy step 20 ms (50 Hz). Align this with measured ESP
  cadence and servo response before transfer.
- Tasks `stand` and `velocity` are software experiments. The current reward
  weights have not been tuned for natural walking or safety; compare against
  baselines, monitor each reward term and inspect video, not just mean reward.
- Critical limits, output disable, deadman and thermal/power guards must remain
  in firmware/control software; a policy never bypasses them.

## Existing repository drift to resolve

- Firmware v57.33 has `stepLift=65 mm`; the ROS model parameter file still says
  50 mm and describes older firmware. It was intentionally left untouched so
  the existing Gazebo launch stays stable until parity tests are in place.
- ROS/Gazebo xacro currently initializes all coxa joints at 0 rad. Firmware
  stand has front/rear coxa fan offsets of ±45°; the new MJCF uses those.
- The kinematics lab body mass 8 kg and body dimensions 430×260×80 mm are
  provisional. Actual batteries, AI HAT, wiring and head mass affect COM.
- The hobby servos provide commands but no measured absolute joint angles in
  the current telemetry. Observation channels that require joint position
  feedback need an estimator or new encoders before real policy transfer.
- `nvidia-smi` on this workstation did not return GPU/driver data (permission
  error), so local Isaac Lab acceleration is unverified. The Windows MuJoCo CPU
  path is verified; Pi training is intentionally excluded.

## Engineering practices

1. Keep source assets/config, generated MJCF and training outputs separate.
2. Pin a dependency lock after the first reproducible run; record simulator,
   Python and learner versions plus contract hash with every experiment.
3. Run model compile, stand physics, Gym API and geometry parity tests in CI.
4. Evaluate on held-out seeds and several measured parameter distributions;
   report fall rate, velocity error, foot slip, energy and control latency.
5. Review every policy change against the firmware safety contract before a
   hardware-in-the-loop run. Physical testing needs a supported robot, a human
   at the kill switch and short bounded commands.

## Sources used for architecture

- [MuJoCo model and Python API](https://mujoco.readthedocs.io/en/stable/python.html)
- [Gymnasium custom environment contract](https://gymnasium.farama.org/tutorials/gymnasium_basics/environment_creation/)
- [NVIDIA Isaac Lab requirements and backends](https://docs.nvidia.com/isaaclab/isaac-lab/installation)
- [ETH legged-gym sim-to-real ingredients and Isaac Lab migration](https://github.com/leggedrobotics/legged_gym/blob/master/README.md)
- [ROS 2 Gazebo control integration](https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html)
- [Burn Rust machine-learning framework](https://burn.dev/docs/burn/)
