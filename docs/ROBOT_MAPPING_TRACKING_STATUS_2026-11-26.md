# META-1 tracking and mapping status (2026-11-26)

## Verified on Pi 5

- The camera service delivered 1280 x 720 video at about 30 FPS after the
  tracking change. With tracking idle, the pan/tilt command returns to 90°/90°;
  idle IMU yaw drift no longer steers the camera.
- A bad first LiDAR scan could establish a map containing only 26 occupied
  cells. Subsequent scans were rejected as low-correlation, so mapping appeared
  frozen. Startup now requires at least 40 distinct endpoint cells; a map made
  from one bad provisional scan can recover automatically. A scan-only seed is
  always tried, and IMU yaw jumps greater than 8° are ignored by scan matching.
- The post-deploy live read showed TRACKING, 23 accepted matches, zero rejected
  matches, and 129 confirmed map cells. These counts reflect a short stationary
  test, not a completed room survey.
- PWM was DISARMED/CUT. No servo movement was commanded during this check.

## Dense color prototype

- `tools/live_dense_rgbd_worker.py` compared a camera wall-clock timestamp with
  LiDAR monotonic time. It now compares source frame ages at the same instant.
  The corrected worker has **not** been installed on the Pi 5.
- `tools/capture_static_rgbd_ap_once.py` captured one centered camera/LiDAR
  pair. `tools/preview_static_rgbd_capture.py` made a local point-cloud preview.
  The example pair had 140 LiDAR returns and about 188 ms frame skew; the
  output has about 3,800 colored points. Depth Anything V2 estimates most
  depths, scaled against a few LiDAR returns. The original frame was very dark;
  gamma enhancement helps visibility but cannot recover true surface colors.
- Camera intrinsics are still approximate. A centered camera is required by
  the current static LiDAR-to-camera transform. Pan/tilt and body motion need
  synchronized joint/IMU poses and calibrated transforms for multi-view fusion.
- The point-cloud preview is **not** measured dense 3D geometry, a complete
  room map, or a trained Gaussian splat. A 2D LiDAR alone cannot directly
  measure surfaces above and below its plane.

## Power and motion gate

- During the check, Pi 5 reported `get_throttled=0x50005`; EXT5V input measured
  4.6766–4.7101 V. This is measured voltage droop, regardless of the power
  supply's advertised current. Dense Pi-side inference and autonomous physical
  movement remain unverified.
- Route execution still uses the existing 10.2 V dual-3S battery guard and
  12 mm lift. A proposed automatic change to 2S guard/35 mm lift was rejected
  by approval review under the present undervoltage and was reverted locally;
  it was not deployed. That path must be assessed with stable voltage and
  controlled physical testing before changing motion thresholds.
- Physical target loss/body handoff, dynamic object-following stability,
  autonomous room exploration, and gait quality still require measured tests.

## Next validation sequence

1. Measure Pi EXT5V at idle and under camera + LiDAR load, and measure the
   source voltage at the USB-C plug. Fix the drop if the current flag persists.
2. Calibrate camera intrinsics with a checkerboard and pan/tilt zero offsets;
   capture synchronized camera, LiDAR, IMU, and gimbal samples.
3. Walk the sensor rig through a short loop with PWM cut. Check closed-loop
   drift and camera/LiDAR registration before retaining a permanent 3D map.
4. Test target handoff and each gait axis on the platform in short, bounded
   steps. Record servo supply voltage and foot-contact state with the video.
