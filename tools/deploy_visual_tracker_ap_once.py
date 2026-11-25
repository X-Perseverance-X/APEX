"""Verified AP-only vision update with hash guard, rollback copy and Wi-Fi restore."""
import getpass
import hashlib
import pathlib
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / ".runtime" / "pydeps"))
import paramiko
from bootstrap_pi_home_wifi_once import AP_PROFILE, HOME_PROFILE, remote, verified_transport, wait_port, windows_connect


def connect_ap():
    try:
        windows_connect(AP_PROFILE)
    except RuntimeError:
        subprocess.run(["netsh", "wlan", "disconnect", "interface=Wi-Fi"],
                       capture_output=True, text=True, timeout=8, check=False)
        time.sleep(2)
        windows_connect(AP_PROFILE)
    if not wait_port("10.42.0.1", 25):
        raise RuntimeError("Robot AP SSH unavailable")


def main():
    password = getpass.getpass("Pi SSH password: ")
    transport = None
    try:
        connect_ap()
        transport = verified_transport("10.42.0.1", password)
        source = ROOT / "vision/detection_hud.py"
        expected = ROOT / ".github-handshake/vision/detection_hud.py"
        remote_path = "/opt/apex/control_station/vision/detection_hud.py"
        backup = "/opt/apex/backups/visual-tracker-" + time.strftime("%Y%m%d-%H%M%S") + ".py"
        with paramiko.SFTPClient.from_transport(transport) as sftp:
            with sftp.file(remote_path, "rb") as stream:
                actual_hash = hashlib.sha256(stream.read()).hexdigest()
            expected_hash = hashlib.sha256(expected.read_bytes()).hexdigest()
            if actual_hash != expected_hash:
                raise RuntimeError("Live camera source differs from committed baseline")
            sftp.put(str(source), "/tmp/apex-visual-tracker.py")
        remote(transport, f"sudo -n cp -p {remote_path} {backup}", sudo_password=password)
        remote(transport, f"sudo -n install -m 644 /tmp/apex-visual-tracker.py {remote_path}", sudo_password=password)
        remote(transport, "sudo -n systemctl restart apex-vision", timeout=30, sudo_password=password)
        time.sleep(4)
        print(remote(transport, "systemctl is-active apex-vision; curl -fsS --max-time 5 http://127.0.0.1:8888/health; curl -fsS --max-time 5 http://127.0.0.1:8888/tracking/status"), flush=True)
        print("Rollback:", backup, flush=True)
    finally:
        if transport is not None:
            transport.close()
        windows_connect(HOME_PROFILE)
        print("Laptop restored to home Wi-Fi", flush=True)


if __name__ == "__main__":
    main()
