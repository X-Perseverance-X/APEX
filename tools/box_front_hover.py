"""Repeat the supervised right-front hover verified with the 16 cm box."""
import json
import math
import sys
import time
from urllib.parse import urlencode
from urllib.request import urlopen

HOST = "http://127.0.0.1:8091/"
BASE_PARAMS = "-100.0_100.0_65.0_1000.0"
HEIGHT_MM = 35
PITCH_DEG = -15
# 7 cm measured from the forward tip, plus approximately 18 mm to the
# simulated sole centre. The last waypoint hovers 25 mm over the tread.
WAYPOINTS = ((0, 0, 0), (45, -35, 26), (45, -35, 185), (128, -35, 185))
DURATIONS = (3.0, 6.0, 4.2)

def inverse(offset):
    dx, dy, dz = offset
    pitch = math.radians(PITCH_DEG)
    cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)
    x, y, z = 231.066 + dx, -204.066 + dy, -135 + dz
    hip_x = cos_pitch*x - sin_pitch*z - 125
    outward = -y - 98
    hip_z = sin_pitch*x + cos_pitch*z
    radial = math.hypot(hip_x, outward) - 69.66
    reach = math.hypot(radial, hip_z)
    a,b = 103.24,168.41
    if not abs(a-b) < reach < a+b:
        raise RuntimeError("Right-front target outside reach")
    angles = (
        180-math.degrees(math.atan2(outward,hip_x)),
        math.degrees(math.atan2(radial,-hip_z)+math.acos((a*a+reach*reach-b*b)/(2*a*reach))),
        180-math.degrees(math.acos((a*a+b*b-reach*reach)/(2*a*b))),
    )
    if not all(8 <= angle <= 172 for angle in angles):
        raise RuntimeError(f"Right-front target outside bench range: {angles}")
    return angles

def eased(a,b,t):
    t=max(0,min(1,t))
    blend=t*t*(3-2*t)
    return tuple(x+blend*(y-x) for x,y in zip(a,b))

def get(path):
    with urlopen(HOST+path,timeout=1.5) as reply:
        return json.load(reply)

def command(device,action,value=""):
    query={"device":device,"action":action}
    if value: query["value"]=value
    reply=get("command?"+urlencode(query))
    if not reply.get("ok"): raise RuntimeError(reply)
    return reply["message"]

def attitude(mode):
    d=get("telemetry")
    s=d["spine"]
    if not (d["connected"] and d["motion_writes"] and d["spine_age_s"]<.5
            and d["imu_age_s"]<.3 and d["imu"]["online"] and s["outputs_armed"]
            and s["mode"]==mode and s["spine_state"]=="STAND"
            and s["pca_right_online"] and s["pca_left_online"] and s["config_valid"]
            and s["firmware_version"]=="v57.33-gait-clearance-camera"
            and 7.0<=s["battery_right_v"]<=8.45):
        raise RuntimeError("STAND/IMU precondition changed")
    return d["imu"]["rpy_deg"][:2]

def body(level):
    command("SPINE","PARAMS",f"{-100-HEIGHT_MM*level:.2f},100,65,1000")
    command("SPINE","BODY",f"{PITCH_DEG*level:.2f},0,0")

def pose(offset):
    angles=inverse(offset)
    command("LEG_LAB","POSE","0,"+",".join(f"{angle:.2f}" for angle in angles))

def validate():
    for a,b in zip(WAYPOINTS,WAYPOINTS[1:]):
        for i in range(101):
            inverse(eased(a,b,i/100))

def run():
    validate()
    init_roll,init_pitch=attitude(0)
    if max(abs(init_roll),abs(init_pitch))>5 or command("SPINE","PARAMS")!="PARAMS_"+BASE_PARAMS:
        raise RuntimeError("Neutral STAND verification failed")
    level=0
    peak=0
    manual=False
    rf_home=True
    outside_motion=False
    try:
        for i in range(1,21):
            roll,pitch=attitude(0)
            if abs(roll-init_roll)>4 or not -20 <= pitch-init_pitch <= 3:
                raise RuntimeError("Body preparation drift")
            level=i
            body(level/20)
            time.sleep(.22)
        time.sleep(.7)
        roll,pitch=attitude(0)
        if abs(roll-init_roll)>3 or not -18 <= pitch-init_pitch <= -10:
            raise RuntimeError("Raised body pose was not confirmed")
        print("BODY_READY",round(pitch-init_pitch,2),flush=True)
        pose(WAYPOINTS[0])
        manual=True
        until=time.monotonic()+.8
        while get("telemetry")["spine"]["mode"]!=2:
            if time.monotonic()>until:raise RuntimeError("Manual hold not confirmed")
            time.sleep(.05)
        attitude(2)
        route=[(a,b,d) for a,b,d in zip(WAYPOINTS,WAYPOINTS[1:],DURATIONS)]
        route.append((WAYPOINTS[-1],WAYPOINTS[-1],2.0))
        route.extend((b,a,d) for a,b,d in list(route[:-1])[::-1])
        rf_home=False
        outside_motion=True
        for index,(a,b,d) in enumerate(route):
            print("RF_STAGE",index,flush=True)
            begin=time.monotonic()
            while True:
                measured_roll,measured_pitch=attitude(2)
                drift=max(abs(measured_roll-roll),abs(measured_pitch-pitch))
                peak=max(peak,drift)
                if drift>3.5:
                    raise RuntimeError("Support attitude deviated; pose held")
                u=min(1,(time.monotonic()-begin)/d)
                pose(eased(a,b,u))
                if u>=1:break
                time.sleep(.08)
        time.sleep(.5)
        rf_home=True
    finally:
        state=get("telemetry")["spine"]
        if state["outputs_armed"] and state["spine_state"]=="STAND" and rf_home:
            if manual and state["mode"]==2:
                command("SPINE","MODE","0")
                time.sleep(.35)
            for i in range(level-1,-1,-1):
                attitude(0)
                body(i/20)
                time.sleep(.18)
        after=get("telemetry")
        print(json.dumps({"max_roll_pitch_delta_deg":round(peak,2),"motion_stage_started":outside_motion,
                          "returned":rf_home,"state":after["spine"]["spine_state"],"mode":after["spine"]["mode"]}),flush=True)

def main():
    import argparse
    import getpass
    from pathlib import Path
    import shlex

    parser = argparse.ArgumentParser(description="Replay the verified, non-contact 16 cm box hover.")
    parser.add_argument("--execute", metavar="PI_IP")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if args.dry_run and not args.execute:
        validate()
        print([tuple(round(value, 2) for value in inverse(point)) for point in WAYPOINTS])
        return 0
    if not args.execute or args.dry_run:
        parser.error("Use --dry-run or --execute PI_IP.")
    from bootstrap_pi_home_wifi_once import verified_transport

    validate()
    print("Setup: 16 cm box, same level surface, centered, 7 cm from the right-front toe.")
    print("Watch the robot through the full motion. Keep the physical power switch within reach.")
    password = getpass.getpass("Pi SSH password: ")
    source = Path(__file__).read_text(encoding="utf-8")
    transport = verified_transport(args.execute, password)
    try:
        with transport.open_session(timeout=5) as channel:
            channel.settimeout(65)
            channel.set_combine_stderr(True)
            channel.exec_command("python3 -u -c " + shlex.quote(source) + " --on-pi")
            for line in channel.makefile():
                print(line.rstrip(), flush=True)
            return channel.recv_exit_status()
    finally:
        transport.close()


if __name__ == "__main__":
    if sys.argv[1:] == ["--on-pi"]:
        run()
    else:
        raise SystemExit(main())
