# Stand coxa + USB body IK: deployment state

2026-11-25: Deployed to Pi and ESP over home Wi-Fi. The Pi joined
`TURKNET_AB0AF`, and `apex.local` SSH worked without Ethernet. The PS3 gateway
has `APEX_HOME_PROFILE=TURKNET_AB0AF`. Pi application files were backed up to
`/opt/apex/backups/2026-11-25-130355-stand-body-ik` before installation.
ESP flash verified its data hash and now reports `v57.31-stand-splay-body-ik`.
All three services returned `active`. Telemetry initially showed `DISARMED`
and `outputs_armed=false`. Twenty-six firmware contract tests passed locally.

With the robot supported on a platform and the operator at the kill switch,
one STAND command reached `STAND` after about 20 seconds. The operator confirmed
front/rear coxas splayed correctly with no collision or strain. Bounded +3°
pitch and +3° roll commands each received `BODY_OK`, were returned to zero,
and were physically confirmed smooth and correct by the operator. No walking
command was sent during this deployment.

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
