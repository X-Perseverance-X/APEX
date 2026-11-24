# APEX camera + planar LiDAR reconstruction

The sensor geometry lives in `sensor_rig.json`. All physical measurements use
the robot convention `+X forward, +Y left, +Z up`, in metres. Moving a sensor
normally requires editing only its `translation_m` and `rotation_rpy_deg`.

Current measured values:

- camera: `x=+0.06`, `y=0`, `z=-0.02` relative to the LiDAR origin;
- IMU: `x=+0.06`, `y=0`, `z=-0.10`, but it is not rigidly mounted and must not
  be used as rig-motion evidence yet.

The capture tool never calls actuator endpoints. It reads the already-open
camera and LiDAR snapshots from the APEX server, rejects stale/asynchronous
pairs, estimates camera motion, refines it against metric LiDAR geometry, and
exports a colored PLY plus the effective rig configuration.

```powershell
.\.venv\Scripts\python.exe tools\object_scan_capture.py --base-url http://apex:8080 --seconds 60
```

Temporary measurement overrides do not edit the saved configuration:

```powershell
.\.venv\Scripts\python.exe tools\object_scan_capture.py --camera-x 0.065 --camera-z -0.022
```

For accurate color projection, first calibrate the camera with a flat 9x6
inner-corner checkerboard. Move the board across the image and tilt it while
keeping every corner visible:

```powershell
.\.venv\Scripts\python.exe tools\calibrate_camera_intrinsics.py --square-mm 25
```

## Capability boundary

A stationary 2D LiDAR produces one metric plane, not a room-volume scan. A
single camera supplies RGB and visual motion but not reliable absolute depth
by itself. iPhone-like room reconstruction requires the camera, LiDAR and IMU
to be one rigid calibrated head and that head to move with sufficient visual
texture and scan overlap. Until the IMU is rigidly mounted, capture is limited
to camera+LiDAR visual/ICP estimation and weak frames are rejected rather than
smeared into the map.
