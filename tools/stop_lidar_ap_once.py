"""Return the LiDAR motor to its idle state after a read-only mapping check."""
import getpass

from deploy_visual_tracker_ap_once import connect_ap
from bootstrap_pi_home_wifi_once import HOME_PROFILE, remote, verified_transport, windows_connect


def main():
    password = getpass.getpass("Pi SSH password: ")
    transport = None
    try:
        connect_ap()
        transport = verified_transport("10.42.0.1", password)
        command = """python3 - <<'PY'
import json,urllib.request,subprocess
base='http://127.0.0.1:8080'
def get(path):
    return json.load(urllib.request.urlopen(base+path,timeout=5))
print('before',get('/api/lidar/snapshot').get('enabled'),flush=True)
print('motor',get('/lidar/motor?state=0'),flush=True)
print('after',get('/api/lidar/snapshot').get('enabled'),flush=True)
print('throttled',subprocess.check_output(['vcgencmd','get_throttled'],text=True).strip(),flush=True)
PY"""
        print(remote(transport, command, timeout=12), flush=True)
    finally:
        if transport is not None:
            transport.close()
        windows_connect(HOME_PROFILE)
        print("Laptop restored to home Wi-Fi", flush=True)


if __name__ == "__main__":
    main()
