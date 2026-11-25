"""Capture one synchronized camera/LiDAR pair for laptop-side 3D preview."""
import getpass
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / ".runtime" / "pydeps"))
import paramiko
from deploy_visual_tracker_ap_once import connect_ap
from bootstrap_pi_home_wifi_once import HOME_PROFILE, remote, verified_transport, windows_connect


def main():
    password = getpass.getpass("Pi SSH password: ")
    transport = None
    try:
        connect_ap()
        transport = verified_transport("10.42.0.1", password)
        command = '''python3 - <<'PY'
import json,urllib.request,time
base='http://127.0.0.1:8080'
def get(path):
  with urllib.request.urlopen(base+path,timeout=5) as r:
    return r.read(),dict(r.headers)
for attempt in range(10):
  lidar,_=get('/api/lidar/snapshot'); scan=json.loads(lidar)
  scan_received=time.monotonic()
  camera,headers=get('/camera/raw-snapshot')
  telemetry,_=get('/telemetry'); state=json.loads(telemetry)
  gimbal=(float(state.get('cameraPanDeg',90)),float(state.get('cameraTiltDeg',90)))
  # Camera X-Apex-Frame-Ts is wall-clock while LiDAR uses Pi monotonic.
  # Compare ages at one instant, including time elapsed between responses.
  lidar_age=float(scan.get('scan_age_s') or 999)+time.monotonic()-scan_received
  frame_age = headers.get('x-apex-frame-age', headers.get('X-Apex-Frame-Age'))
  skew=abs(float(frame_age if frame_age is not None else 999)-lidar_age)
  print('attempt',attempt,'fresh',scan.get('fresh'),'points',len(scan.get('points') or []),'skew',round(skew,3),'gimbal',gimbal,flush=True)
  if scan.get('fresh') and len(scan.get('points') or [])>=80 and skew<0.35 and all(abs(a-90)<3 for a in gimbal):
    open('/tmp/apex-static-rgbd.jpg','wb').write(camera)
    json.dump({'lidar':scan,'telemetry':state,'camera_headers':headers,'skew_s':skew},open('/tmp/apex-static-rgbd.json','w'))
    print('captured',len(scan['points']),'lidar points','skew',round(skew,3),'gimbal',gimbal,flush=True)
    break
  time.sleep(0.15)
else: print('fresh centered camera+LiDAR pair unavailable',flush=True)
PY'''
        detail = remote(transport, command, timeout=22)
        print(detail, flush=True)
        if 'captured ' not in detail:
            return
        folder = ROOT / "artifacts" / "static_rgbd_latest"
        folder.mkdir(parents=True, exist_ok=True)
        with paramiko.SFTPClient.from_transport(transport) as sftp:
            sftp.get("/tmp/apex-static-rgbd.jpg", str(folder / "camera.jpg"))
            sftp.get("/tmp/apex-static-rgbd.json", str(folder / "sensors.json"))
        print("Saved", folder, flush=True)
    finally:
        if transport is not None:
            transport.close()
        windows_connect(HOME_PROFILE)
        print("Laptop restored to home Wi-Fi", flush=True)


if __name__ == "__main__":
    main()
