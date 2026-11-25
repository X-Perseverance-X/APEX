#!/usr/bin/env python3
"""Deploy the tested v57.31 stand/body IK repair without moving servos."""
from __future__ import annotations

import getpass
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / ".runtime" / "pydeps"))
from bootstrap_pi_home_wifi_once import verified_transport, remote  # noqa: E402
import paramiko  # noqa: E402

PI = "apex.local"
PROJECT = ROOT / ".github-handshake"
FILES = {
    PROJECT / "apex_server.py": "apex_server.py",
    PROJECT / "apex_ui.html": "apex_ui.html",
    PROJECT / "tools" / "apex_spine_usb_bridge.py": "tools/apex_spine_usb_bridge.py",
}
BIN = ROOT / ".runtime" / "esp-build-current" / "v57_spine_safe_boot.ino.bin"
STAGE = "/tmp/apex-v57.31-deploy"
DEST = "/opt/apex/control_station"
SERVICES = "apex-ps3-gateway apex-spine-usb apex-command-center"


def run(transport: paramiko.Transport, command: str, password: str,
        timeout: float = 20) -> str:
    return remote(transport, command, timeout=timeout, sudo_password=password)


def main() -> int:
    for path in [*FILES, BIN]:
        if not path.is_file():
            raise RuntimeError(f"Missing deploy input: {path}")
    password = getpass.getpass("Pi SSH/sudo password: ")
    transport = verified_transport(PI, password)
    stopped = False
    flashed = False
    installed = False
    backup = f"/opt/apex/backups/{time.strftime('%Y-%m-%d-%H%M%S')}-stand-body-ik"
    try:
        print(run(transport, "hostname; nmcli -t -f NAME,DEVICE connection show --active", password).strip(), flush=True)
        print(run(transport, "sudo -n true", password).strip(), flush=True)
        run(transport, f"mkdir -p {STAGE}/tools", password)
        with paramiko.SFTPClient.from_transport(transport) as sftp:
            for source, relative in FILES.items():
                sftp.put(str(source), f"{STAGE}/{relative}")
            sftp.put(str(BIN), f"{STAGE}/firmware.bin")
        run(transport, f"sudo -n mkdir -p {backup}/tools", password)
        for relative in FILES.values():
            run(transport, f"sudo -n cp -a {DEST}/{relative} {backup}/{relative}", password)
        print(f"Pi backup: {backup}", flush=True)
        stopped = True
        run(transport, f"sudo -n systemctl stop --no-block {SERVICES}", password)
        time.sleep(3)
        print(run(transport, f"systemctl show -p ActiveState {SERVICES}", password).strip(), flush=True)
        print("Services stopped; servo rail must remain off.", flush=True)
        for relative in FILES.values():
            run(transport, f"sudo -n install -m 644 {STAGE}/{relative} {DEST}/{relative}", password)
        installed = True
        print("Application and USB bridge installed; flashing ESP...", flush=True)
        flash = run(transport,
            f"sudo -n esptool --chip esp32s3 --no-stub --port /dev/ttyUSB0 "
            f"--baud 115200 write_flash 0x10000 {STAGE}/firmware.bin",
            password, timeout=240)
        print("\n".join(flash.strip().splitlines()[-8:]), flush=True)
        flashed = True
        run(transport, f"sudo -n systemctl start --no-block {SERVICES}", password, timeout=40)
        stopped = False
        time.sleep(5)
        print(run(transport, f"systemctl --no-pager --plain is-active {SERVICES}", password).strip(), flush=True)
        print("Deployment complete; no arm or motion command sent.", flush=True)
        return 0
    except Exception:
        if installed and not flashed:
            for relative in FILES.values():
                try:
                    run(transport, f"sudo -n cp -a {backup}/{relative} {DEST}/{relative}", password)
                except Exception:
                    pass
        raise
    finally:
        if stopped:
            try:
                run(transport, f"sudo -n systemctl start --no-block {SERVICES}", password, timeout=40)
            except Exception as exc:
                print(f"SERVICE RESTART FAILED: {exc}", file=sys.stderr, flush=True)
        transport.close()


if __name__ == "__main__":
    raise SystemExit(main())
