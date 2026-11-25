#!/usr/bin/env python3
"""Flash the tested body-frame gait firmware while PS3 motion stays disabled."""
import getpass
import json
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / ".runtime" / "pydeps"))
from bootstrap_pi_home_wifi_once import remote, verified_transport  # noqa: E402
import paramiko  # noqa: E402

BIN = ROOT / ".runtime/esp-build-v57.32/v57_spine_safe_boot.ino.bin"
OLD_BIN = ROOT / ".runtime/esp-build-current/v57_spine_safe_boot.ino.bin"
if not BIN.is_file() or not OLD_BIN.is_file():
    raise RuntimeError("Compiled or rollback firmware binary is missing")

password = getpass.getpass("Pi SSH/sudo password: ")
transport = verified_transport("apex.local", password)
services_stopped = False


def run(command, timeout=20):
    return remote(transport, command, timeout=timeout, sudo_password=password)


try:
    ps3_state = run("systemctl show apex-ps3-gateway -p ActiveState").strip()
    if ps3_state != "ActiveState=inactive":
        raise RuntimeError(f"PS3 gateway must remain stopped: {ps3_state}")
    stage = "/tmp/apex-v57.32-gait"
    backup = "/opt/apex/backups/2026-11-25-gait-v57.32"
    run(f"mkdir -p {stage}")
    with paramiko.SFTPClient.from_transport(transport) as sftp:
        sftp.put(str(BIN), f"{stage}/new.bin")
        sftp.put(str(OLD_BIN), f"{stage}/rollback-v57.31.bin")
    run(f"sudo -n mkdir -p {backup}")
    run(f"sudo -n install -m 600 {stage}/rollback-v57.31.bin {backup}/rollback-v57.31.bin")
    run("sudo -n systemctl stop --no-block apex-command-center apex-spine-usb")
    services_stopped = True
    time.sleep(3)
    print("Flashing ESP; PS3 service remains stopped.", flush=True)
    result = run(f"sudo -n esptool --chip esp32s3 --no-stub --port /dev/ttyUSB0 "
                 f"--baud 115200 write_flash 0x10000 {stage}/new.bin", timeout=240)
    if "Hash of data verified" not in result:
        raise RuntimeError("ESP flash returned without data hash verification")
    print("\n".join(result.strip().splitlines()[-6:]), flush=True)
    run("sudo -n systemctl start --no-block apex-spine-usb apex-command-center")
    services_stopped = False
    time.sleep(5)
    state = json.loads(run("curl -fsS --max-time 5 http://127.0.0.1:8091/telemetry"))
    spine = state.get("spine", {})
    checked = {key: spine.get(key) for key in
               ("firmware_version", "spine_state", "outputs_armed", "pca_right_online", "pca_left_online")}
    print("Post-flash:", checked, flush=True)
    if not (spine.get("firmware_version") == "v57.32-body-frame-gait"
            and spine.get("spine_state") == "DISARMED"
            and not spine.get("outputs_armed")):
        raise RuntimeError("Post-flash safe state/version check failed")
    print(f"Rollback binary on Pi: {backup}/rollback-v57.31.bin", flush=True)
finally:
    if services_stopped:
        try:
            run("sudo -n systemctl start --no-block apex-spine-usb apex-command-center")
        except Exception as exc:
            print(f"Service restart needs attention: {exc}", file=sys.stderr, flush=True)
    transport.close()
