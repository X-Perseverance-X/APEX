# Stand coxa + USB body IK: deployment state

2026-11-25: Source is committed on `codex/backup-2026-11-24` at `5ff5f07`.
ESP firmware `v57.31-stand-splay-body-ik` compiled locally for ESP32-S3;
26 firmware contract tests passed. **Not deployed to Pi/ESP.** The PC is on
`192.168.1.10/24` home Wi-Fi, Ethernet is disconnected, `apex.local` does not
resolve, and SSH to the Pi Ethernet address `192.168.50.1` times out. The last
verified deployed firmware is `v57.30-usb-arc-gait`.

## Changes awaiting deployment

- Front/rear stand feet splay individually after the vertical rising stage.
  Foot tips follow a 150 mm horizontal radius: RF=135°, RR=45°, LF=45°,
  LR=135° nominal servo coxa targets; middle coxas remain 90°.
- Each splay step lifts one foot 25 mm, sweeps it around the coxa, then lowers
  it. The previous HOME capture and vertical rise remain.
- `/bodyIk` forwards bounded pitch/roll/yaw through the USB bridge. Firmware
  accepts it only while outputs are armed, STAND/WALK is reached, and the
  current motion mode is IK capable.

## Physical test gate

Before flashing, support the chassis and make the kill switch accessible.
Firmware upload resets ESP and cuts servo PWM. Back up current Pi files, stop
`apex-ps3-gateway` and `apex-spine-usb`, flash the compiled ESP binary, install
`apex_server.py`, `apex_ui.html`, and `tools/apex_spine_usb_bridge.py`, restart
services, then verify firmware version and zero motion. Observe one commanded
stand on the platform. Test pitch/roll first at small angles. Do not infer
physical servo position from command angles alone.
