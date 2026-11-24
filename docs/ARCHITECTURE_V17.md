# APEX META-1 v17 Architecture

## Objective

The same control contract must run against three interchangeable targets:

1. Gazebo SIL, with no robot hardware.
2. Hardware-in-the-loop, with the real ESP32 and simulated or disconnected actuators.
3. The physical robot over nRF24, with local IP retained for high-bandwidth media.

The browser must never talk directly to an actuator transport. It sends typed
commands to the Command Center, which owns command authority, validation,
rate-limiting, recording, and the operator lease.

## Target topology

```text
Operator UI / ROS 2 / autonomy
              |
       Command arbiter
              |
     APEX protocol v1 (32 B)
       /          |          \
  SIL loopback  Wi-Fi HTTP  nRF24 gateway
      |             |             |
 Gazebo model   compatibility   ESP32-S3 safety core
      |           adapter            |
 simulated sensors             gait / IK / PWM
```

Camera images, LiDAR scans, maps, and logs stay on local Ethernet/Wi-Fi/USB.
They do not require internet access. nRF24 carries only bounded control and
compact health/telemetry frames.

## Responsibility boundaries

### ESP32-S3 safety core

- Own the final command timeout and output state.
- Reject malformed, stale, duplicated, wrong-session, or out-of-range frames.
- Accept commands from exactly one active transport/authority at a time.
- On link timeout: zero velocity, leave walking mode, and converge to stand.
- Keep output-disable latched until a neutral, explicitly authorized reset.
- Never depend on the browser or Raspberry Pi to stop motion safely.

### Raspberry Pi Command Center

- Own operator lease, source arbitration, API authentication, audit log, and
  conversion between browser/ROS messages and APEX protocol frames.
- Implement `HttpTransport`, `Nrf24Transport`, and `SilTransport` behind one
  interface.
- Publish freshness and link state; never label stale cached telemetry online.
- Keep camera/LiDAR services separate from the deterministic command loop.

### ROS 2 / Gazebo workstation

- Bu makine için Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress.
- URDF/Xacro is the source of geometry, joints, limits, and sensor mounting.
- `gz_ros2_control` drives the 18 leg joints and 4 arm joints.
- `ros_gz_bridge` exposes IMU, range, camera, joint states, contacts, and clock.
- The SIL adapter consumes the exact bytes used by nRF24 and returns exact
  telemetry frames, enabling packet loss, latency, duplication, and corruption
  injection before hardware tests.

## Safety state machine

```text
BOOT -> DISARMED -> ARMING -> ACTIVE
                       |        |
                       +--------+--> FAILSAFE_HOLD
                                      |
                         neutral + explicit clear
                                      |
                                   DISARMED

Any state --OUTPUT_DISABLE--> OUTPUT_DISABLED (latched)
Any internal invariant fault --> FAULT (latched)
```

Initial timeout targets:

- Command period: 20 ms (50 Hz).
- Soft link timeout: 200 ms. Set command to neutral and enter stand transition.
- Stale telemetry threshold at UI: 300 ms.
- Explicit output-disable is separate from link loss because dropping PWM on a
  standing hexapod can cause a collapse.

These are commissioning defaults, not final certified limits. They must be
confirmed on a suspended robot and in SIL fault-injection tests.

## Migration order

1. Fix current Wi-Fi path: zero-on-release, firmware deadman, persistent output
   mask, strict bounds, stale telemetry status.
2. Route Wi-Fi commands through the protocol/arbiter while preserving existing
   endpoints as compatibility adapters.
3. Bring up Gazebo SIL and replay the protocol test vectors.
4. Add Pi nRF24 gateway and ESP32 radio driver with radio disabled by default.
5. Confirm exact ESP32-S3 board and allocate SPI/CE/CSN/IRQ pins.
6. Bench HIL: logic power only, then one servo, then suspended full robot.
7. Switch command authority from Wi-Fi to radio; retain Wi-Fi read-only/debug.
