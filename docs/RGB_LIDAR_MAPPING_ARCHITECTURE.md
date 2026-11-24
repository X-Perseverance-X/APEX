# META-1 RGB + planar LiDAR mapping architecture

## Goal

Create a repeatable, actuator-free capture pipeline that can export metric,
colored point clouds now and can grow into a room-scale pose graph after the
IMU is rigidly mounted. The coordinate convention is always right handed:
`+X forward`, `+Y left`, `+Z up`.

## Current physical rig

The LiDAR is the reference origin. User measurements are intentionally stored
in one editable file, `object_scan/sensor_rig.json`:

| Sensor | X forward | Y left | Z up | Orientation | Fusion state |
|---|---:|---:|---:|---|---|
| LiDAR | 0.00 m | 0.00 m | 0.00 m | forward | reference |
| Camera | +0.06 m | 0.00 m | -0.02 m | forward | enabled |
| ESP32 IMU | +0.06 m | 0.00 m | -0.10 m | forward | disabled until rigidly mounted |

Changing a sensor later requires only editing its translation and roll/pitch/
yaw. The conversion from robot axes to OpenCV optical axes is calculated and
tested; it is not duplicated in capture code.

## Data path

```text
Pi camera owner ── raw JPEG + Pi monotonic timestamp ─┐
                                                     ├─ sync gate ─ visual pose
RPLIDAR owner ───── full scan + Pi monotonic timestamp┘               │
                              │                                       │
                              └─ metric scan plane ─ ICP refinement ──┤
                                                                      ▼
                                                     bounded colored voxel map
                                                                      │
                                                        PLY + JSON + raw frames
```

No camera or LiDAR device is opened a second time. The capture process consumes
immutable snapshots from the existing APEX server. Camera/LiDAR pairs with a
timestamp difference above 180 ms are rejected. Visual or ICP failures are
also rejected instead of being written as smeared geometry.

## Quality gates

1. **Power:** `get_throttled=0x0`, no undervoltage kernel event during a loaded
   camera+LiDAR+AI test, and stable USB topology.
2. **Camera intrinsics:** checkerboard calibration RMS <= 2.0 px. The current
   FOV fallback is explicitly marked approximate.
3. **Extrinsics:** measured XYZ entered in `sensor_rig.json`; final production
   use should refine camera/LiDAR rotation and translation with a calibration
   target.
4. **Time:** camera/LiDAR skew <= 180 ms; later target <= 50 ms after a combined
   snapshot endpoint is added.
5. **Frame acceptance:** sufficient LiDAR points, >=32 visual pose inliers, and
   bounded ICP update. Weak frames never mutate the cloud.
6. **Repeatability:** scan a static target twice; aligned-cloud median error
   target <= 30 mm for the present webcam+A1 prototype.

## Operating modes

### Mode A — current desk test

Camera and LiDAR are rigid relative to each other. The IMU remains disabled.
Move the complete camera+LiDAR head slowly around a static, textured target.
This produces a sparse colored metric cloud. If the head remains stationary,
only one horizontal LiDAR plane can be reconstructed.

### Mode B — pivot sweep

Rotate/tilt the complete rigid camera+LiDAR head around an approximately fixed
point. Use `--motion-mode pivot`. Translation is deliberately held at zero to
avoid monocular scale inventions.

### Mode C — room-scale mapping after IMU mounting

Rigidly attach the IMU, measure its XYZ/RPY, set `imu.rigidly_mounted=true`,
and perform six-position accelerometer plus gyro-bias calibration. Then add IMU
preintegration as the short-term orientation prior, camera visual odometry as
the 6-DoF motion proposal, planar LiDAR ICP as metric horizontal correction,
and pose-graph loop closures as global drift correction.

## Honest geometry boundary

An RPLIDAR A1 measures a single plane. Camera RGB can color those metric hits
and help estimate motion, but a monocular webcam does not directly measure
dense depth. iPhone-class dense surfaces require either:

- sweeping the rigid 2D LiDAR through different elevations while tracking the
  full 6-DoF head pose, or
- adding a depth/stereo/3D LiDAR sensor.

Monocular AI depth can later fill visualization gaps, but it must be labelled
estimated and must not overwrite measured LiDAR occupancy used for navigation.

## Commands

Power audit on the Pi:

```bash
cd ~/APEX_META1_RADIO_SIL
bash tools/pi_power_audit.sh
```

Camera calibration while moving a 9x6 inner-corner checkerboard:

```powershell
.\.venv\Scripts\python.exe tools\calibrate_camera_intrinsics.py --base-url http://apex:8080 --square-mm 25
```

Safe capture, with no actuator endpoints:

```powershell
.\.venv\Scripts\python.exe tools\object_scan_capture.py --base-url http://apex:8080 --seconds 60 --rate 4
```

Output contains raw frames, timestamp/skew records, the effective sensor-rig
snapshot, trajectory diagnostics and `object-cloud.ply`.
