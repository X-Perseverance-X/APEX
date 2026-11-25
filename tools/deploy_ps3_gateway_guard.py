#!/usr/bin/env python3
"""Install the PS3 input health guard without enabling motion service."""
import getpass
import pathlib
import sys

ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/".runtime"/"pydeps"))
from bootstrap_pi_home_wifi_once import remote,verified_transport  # noqa: E402
import paramiko  # noqa: E402

source=ROOT/".github-handshake/tools/apex_ps3_gateway.py"
password=getpass.getpass("Pi SSH/sudo password: ")
transport=verified_transport("apex.local",password)
try:
 def run(command): return remote(transport,command,sudo_password=password)
 if run("systemctl show apex-ps3-gateway -p ActiveState").strip()!="ActiveState=inactive":
  raise RuntimeError("PS3 gateway must be inactive before install")
 with paramiko.SFTPClient.from_transport(transport) as sftp:
  sftp.put(str(source),"/tmp/apex-ps3-gateway-guard.py")
 run("python3 -m py_compile /tmp/apex-ps3-gateway-guard.py")
 run("sudo -n mkdir -p /opt/apex/backups/2026-11-25-ps3-input-guard")
 run("sudo -n cp -a /opt/apex/control_station/tools/apex_ps3_gateway.py /opt/apex/backups/2026-11-25-ps3-input-guard/")
 run("sudo -n install -m 644 /tmp/apex-ps3-gateway-guard.py /opt/apex/control_station/tools/apex_ps3_gateway.py")
 print(run("systemctl is-enabled apex-ps3-gateway; systemctl show apex-ps3-gateway -p ActiveState"))
 print("PS3 guard installed; service remains disabled/inactive.")
finally: transport.close()
