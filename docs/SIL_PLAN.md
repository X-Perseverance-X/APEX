# Software-in-the-loop plan

## Supported stack

- Host: Ubuntu 22.04 (bu makinede WSL2 + WSLg).
- ROS: ROS 2 Humble Hawksbill.
- Simulator: Gazebo Fortress.
- Control integration: `ros2_control` + `gz_ros2_control`.
- Transport/sensors: `ros_gz_bridge` and `ros_gz_image` where appropriate.

The simulator should not run on the Raspberry Pi. The Pi remains a gateway and
perception computer; the development workstation runs physics and visualization.

## ROS workspace layout

```text
ros2_ws/src/
  apex_description/   Xacro, meshes, joint limits, ros2_control tags
  apex_gazebo/        worlds, contacts, IMU/range/camera plugins, launch files
  apex_interfaces/    typed ROS messages/services for APEX state and faults
  apex_control/       gait source, command arbiter, safety monitor
  apex_transport/     protocol codec, SIL/nRF/Wi-Fi adapters
  apex_bringup/       one-command SIL, HIL, and robot launch profiles
```

## Required simulation fidelity stages

1. **Kinematic smoke test:** joint directions, zero positions, limits, leg order.
2. **Rigid-body test:** masses, inertias, collision geometry, ground contacts.
3. **Actuator test:** rate/acceleration/effort limits and first-order servo lag.
4. **Sensor test:** IMU noise/bias/drift, ToF limits, camera intrinsics, contacts.
5. **Network test:** delay, loss, duplication, reorder, CRC corruption, reboot and
   session change.
6. **Scenario test:** joystick release, UI crash, Pi crash, radio loss during
   swing, low battery, one stuck servo, one missing sensor.

## Acceptance gates before powered walking

- Joint-direction test passes for all 22 actuators.
- No command survives longer than the configured link timeout.
- Duplicated and old-session commands never actuate the model.
- Output-disable remains latched across subsequent normal commands.
- Failsafe lowers a lifted leg and converges to a stable support polygon.
- Replay of a recorded command stream produces deterministic state transitions.
- HIL logic-only test passes before servo power is connected.
