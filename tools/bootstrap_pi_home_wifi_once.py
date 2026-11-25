#!/usr/bin/env python3
"""One-time Pi home Wi-Fi bootstrap through the robot AP.

The Windows Wi-Fi interface is returned to HOME in a finally block. A Pi-side
systemd timer returns the Pi to its AP if the new connection is not confirmed.
Passwords are prompted at runtime and are never written to this source file.
"""

from __future__ import annotations

import getpass
import io
import pathlib
import socket
import subprocess
import sys
import time
import uuid


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / ".runtime" / "pydeps"))
import paramiko  # noqa: E402

AP_PROFILE = "APEX-META1"
HOME_PROFILE = "TURKNET_AB0AF"
PI_AP_IP = "10.42.0.1"
PI_USER = "perseverance"
REFERENCE_HOST = "192.168.50.1"


def windows_connect(profile: str) -> None:
    for attempt in range(3):
        result = subprocess.run(
            ["netsh", "wlan", "connect", f"name={profile}", "interface=Wi-Fi"],
            capture_output=True, text=True, timeout=15, check=False,
        )
        if result.returncode == 0:
            return
        if attempt < 2:
            time.sleep(2)
    detail = (result.stdout + " " + result.stderr).strip().replace("\n", " ")[:250]
    raise RuntimeError(f"Windows Wi-Fi profile {profile} could not start: {result.returncode}: {detail}")


def wait_port(host: str, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, 22), timeout=1.0):
                return True
        except OSError:
            time.sleep(1)
    return False


def verified_transport(host: str, password: str) -> paramiko.Transport:
    known = paramiko.HostKeys()
    known.load(str(pathlib.Path.home() / ".ssh" / "known_hosts"))
    reference = known.lookup(REFERENCE_HOST)
    if reference is None or "ssh-ed25519" not in reference:
        raise RuntimeError("Previously verified Pi host key is missing")
    transport = paramiko.Transport((host, 22))
    transport.start_client(timeout=7)
    if transport.get_remote_server_key().asbytes() != reference["ssh-ed25519"].asbytes():
        transport.close()
        raise RuntimeError("Pi SSH host key differs from the verified Ethernet key")
    transport.auth_password(PI_USER, password)
    return transport


def remote(transport: paramiko.Transport, command: str, timeout: float = 10,
           sudo_password: str | None = None) -> str:
    channel = transport.open_session(timeout=5)
    channel.settimeout(timeout)
    if command.startswith("sudo -n ") and sudo_password is not None:
        command = "sudo -S -p '' " + command[len("sudo -n "):]
    channel.exec_command(command)
    if sudo_password is not None and command.startswith("sudo -S "):
        channel.sendall(sudo_password + "\n")
    output = channel.makefile("r").read().decode("utf-8", "replace")
    error = channel.makefile_stderr("r").read().decode("utf-8", "replace")
    code = channel.recv_exit_status()
    channel.close()
    if code:
        raise RuntimeError(f"Pi command failed ({code}): {command.split()[0]}: {error[:250]}")
    return output


def upload_private(transport: paramiko.Transport, content: str, path: str) -> None:
    with paramiko.SFTPClient.from_transport(transport) as sftp:
        with sftp.file(path, "wb") as target:
            target.write(content.encode("utf-8"))
        sftp.chmod(path, 0o600)


def main() -> int:
    if sys.argv[1:] == ["--probe"]:
        try:
            windows_connect(AP_PROFILE)
            time.sleep(3)
            config = subprocess.run(["ipconfig"], capture_output=True, text=True,
                                    timeout=5, check=False).stdout
            wifi = config.split("Wireless LAN adapter Wi-Fi:", 1)[-1].split("\n\n", 1)[0]
            print("Wi-Fi IP details: " + " | ".join(line.strip() for line in wifi.splitlines()
                  if "IPv4 Address" in line or "Default Gateway" in line), flush=True)
            for candidate in (PI_AP_IP, "192.168.7.1", "192.168.4.1", "apex.local"):
                reachable = wait_port(candidate, 3)
                print(f"Pi AP SSH {candidate}: {reachable}", flush=True)
                if reachable:
                    return 0
            return 2
        finally:
            windows_connect(HOME_PROFILE)
    if sys.argv[1:] == ["--inspect"]:
        pi_password = getpass.getpass("Pi SSH password: ")
        try:
            windows_connect(AP_PROFILE)
            if not wait_port(PI_AP_IP, 20):
                raise RuntimeError("Pi AP SSH did not appear")
            transport = verified_transport(PI_AP_IP, pi_password)
            try:
                print(remote(transport, "nmcli -t -f NAME,UUID,DEVICE connection show --active"))
                print(remote(transport, "nmcli -t -f NAME,UUID connection show"))
            finally:
                transport.close()
        finally:
            windows_connect(HOME_PROFILE)
        return 0
    pi_password = getpass.getpass("Pi SSH password: ")
    home_password = getpass.getpass("Home Wi-Fi password: ")
    if len(home_password) < 8:
        raise RuntimeError("Wi-Fi password too short for WPA-PSK")
    transport = None
    try:
        print("Switching laptop to robot AP; local setup will continue offline...", flush=True)
        windows_connect(AP_PROFILE)
        if not wait_port(PI_AP_IP, 25):
            raise RuntimeError("Pi AP SSH did not appear at 10.42.0.1")
        transport = verified_transport(PI_AP_IP, pi_password)
        def admin(command: str) -> str:
            return remote(transport, command, sudo_password=pi_password)
        admin("sudo -n true")
        profile = "\n".join([
            "[connection]", f"id={HOME_PROFILE}", f"uuid={uuid.uuid4()}",
            "type=wifi", "interface-name=wlan0", "autoconnect=false", "",
            "[wifi]", "mode=infrastructure", f"ssid={HOME_PROFILE}", "",
            "[wifi-security]", "key-mgmt=wpa-psk", f"psk={home_password}", "",
            "[ipv4]", "method=auto", "", "[ipv6]", "method=auto", "",
        ])
        upload_private(transport, profile, "/tmp/apex-home-once.nmconnection")
        admin("sudo -n install -m 600 -o root -g root /tmp/apex-home-once.nmconnection "
                          "/etc/NetworkManager/system-connections/TURKNET_AB0AF.nmconnection")
        remote(transport, "rm -f /tmp/apex-home-once.nmconnection")
        admin("sudo -n nmcli connection reload")
        names = remote(transport, "nmcli -g NAME connection show").splitlines()
        if HOME_PROFILE not in names or AP_PROFILE not in names:
            raise RuntimeError("Robot/home NetworkManager profiles not both present")
        gateway = "[Service]\nEnvironment=APEX_HOME_PROFILE=TURKNET_AB0AF\n"
        upload_private(transport, gateway, "/tmp/apex-ps3-home.conf")
        admin("sudo -n mkdir -p /etc/systemd/system/apex-ps3-gateway.service.d")
        admin("sudo -n install -m 644 /tmp/apex-ps3-home.conf "
                          "/etc/systemd/system/apex-ps3-gateway.service.d/home.conf")
        remote(transport, "rm -f /tmp/apex-ps3-home.conf")
        admin("sudo -n systemctl daemon-reload")
        admin("sudo -n systemctl restart apex-ps3-gateway")
        admin("sudo -n systemd-run --unit=apex-wifi-rollback --on-active=60s "
                          "/usr/bin/nmcli connection up APEX-META1")
        print("Home profile saved; Pi rollback timer armed. Starting home Wi-Fi...", flush=True)
        channel = transport.open_session(timeout=5)
        channel.exec_command("sudo -S -p '' nmcli connection up TURKNET_AB0AF")
        channel.sendall(pi_password + "\n")
        time.sleep(3)
    finally:
        if transport is not None:
            transport.close()
        windows_connect(HOME_PROFILE)
        print("Laptop returned to home Wi-Fi.", flush=True)

    if not wait_port("apex.local", 35):
        print("Pi not seen on home Wi-Fi; its 60-second rollback will restore APEX-META1.", flush=True)
        return 2
    transport = verified_transport("apex.local", pi_password)
    try:
        remote(transport, "sudo -n systemctl stop apex-wifi-rollback.timer",
               sudo_password=pi_password)
        print(remote(transport, "nmcli -t -f NAME,DEVICE connection show --active").strip(), flush=True)
        print("Pi joined home Wi-Fi and PS3 network button has a saved target.", flush=True)
    finally:
        transport.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
