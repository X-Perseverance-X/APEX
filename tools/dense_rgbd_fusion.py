#!/usr/bin/env python3
"""Fuse real camera pixels into a metric RGB-D cloud using LiDAR scale + IMU pose.

The Depth Anything model provides dense *relative* depth.  Every accepted frame
is independently metrized against projected LiDAR returns.  Consecutive dense
clouds are then registered with an IMU rotation seed and visual-odometry
translation direction.  No actuator, GPIO or PWM API is imported or called.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import struct
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from object_scan import ColoredVoxelCloud, camera_from_lidar, camera_matrix, load_sensor_rig
from object_scan.core import icp_register, lidar_polar_to_camera, transform_points


def rpy_matrix(rpy_deg: list[float]) -> np.ndarray:
    roll, pitch, yaw = np.radians(np.asarray(rpy_deg, dtype=np.float64))
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    rx = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=np.float64)
    ry = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=np.float64)
    rz = np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=np.float64)
    return rz @ ry @ rx


def robust_line(x: np.ndarray, y: np.ndarray) -> tuple[float, float, np.ndarray]:
    keep = np.isfinite(x) & np.isfinite(y)
    for _ in range(6):
        if int(keep.sum()) < 10:
            break
        a, b = np.linalg.lstsq(np.c_[x[keep], np.ones(int(keep.sum()))], y[keep], rcond=None)[0]
        residual = y - (a * x + b)
        center = np.median(residual[keep])
        mad = 1.4826 * np.median(np.abs(residual[keep] - center))
        keep = keep & (np.abs(residual - center) <= max(0.025, 3.0 * mad))
    if int(keep.sum()) < 8:
        raise ValueError("not enough LiDAR/depth correspondences")
    a, b = np.linalg.lstsq(np.c_[x[keep], np.ones(int(keep.sum()))], y[keep], rcond=None)[0]
    return float(a), float(b), keep


def metric_depth_from_lidar(
    relative: np.ndarray,
    lidar_camera: np.ndarray,
    intrinsics: np.ndarray,
    min_depth: float,
    max_depth: float,
    allowed_modes: tuple[str, ...] = ("inverse", "direct"),
) -> tuple[np.ndarray, dict]:
    z = lidar_camera[:, 2]
    valid = z > 0.08
    points = lidar_camera[valid]
    z = z[valid]
    u = np.rint(intrinsics[0, 0] * points[:, 0] / z + intrinsics[0, 2]).astype(int)
    v = np.rint(intrinsics[1, 1] * points[:, 1] / z + intrinsics[1, 2]).astype(int)
    h, w = relative.shape
    inside = (u >= 3) & (u < w - 3) & (v >= 3) & (v < h - 3) & (z <= max_depth)
    u, v, z = u[inside], v[inside], z[inside]
    if len(z) < 8:
        raise ValueError("too few projected LiDAR samples")
    raw = np.asarray([np.median(relative[y-2:y+3, x-2:x+3]) for x, y in zip(u, v)], dtype=np.float64)

    candidates: list[tuple[float, str, float, float, np.ndarray]] = []
    mode_targets = {"inverse": 1.0 / z, "direct": z}
    if not allowed_modes or any(mode not in mode_targets for mode in allowed_modes):
        raise ValueError("allowed_modes must contain inverse and/or direct")
    for mode in allowed_modes:
        target = mode_targets[mode]
        try:
            a, b, keep = robust_line(raw, target)
        except ValueError:
            continue
        prediction = a * raw + b
        predicted_z = 1.0 / prediction if mode == "inverse" else prediction
        good = keep & np.isfinite(predicted_z) & (predicted_z > 0)
        if int(good.sum()) < 6:
            continue
        error = float(np.median(np.abs(predicted_z[good] - z[good]) / z[good]))
        candidates.append((error, mode, a, b, good))
    if not candidates:
        raise ValueError("LiDAR metric fit failed")
    error, mode, a, b, keep = min(candidates, key=lambda item: item[0])
    fitted = a * relative.astype(np.float64) + b
    metric = 1.0 / fitted if mode == "inverse" else fitted
    metric[~np.isfinite(metric)] = np.nan
    metric[(metric < min_depth) | (metric > max_depth)] = np.nan
    return metric.astype(np.float32), {
        "mode": mode, "a": a, "b": b, "samples": int(keep.sum()),
        "median_relative_error": error,
    }


def cloud_from_depth(image: np.ndarray, depth: np.ndarray, k: np.ndarray, stride: int) -> tuple[np.ndarray, np.ndarray]:
    v, u = np.mgrid[0:image.shape[0]:stride, 0:image.shape[1]:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z)
    u, v, z = u[valid].astype(np.float64), v[valid].astype(np.float64), z[valid].astype(np.float64)
    x = (u - k[0, 2]) * z / k[0, 0]
    y = (v - k[1, 2]) * z / k[1, 1]
    xyz = np.c_[x, y, z]
    rgb = image[::stride, ::stride][valid][:, ::-1]
    return xyz, rgb


def compact_cloud(xyz: np.ndarray, rgb: np.ndarray, voxel: float = 0.018, limit: int = 7000) -> tuple[np.ndarray, np.ndarray]:
    keys = np.floor(xyz / voxel).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    if len(indices) > limit:
        indices = indices[np.linspace(0, len(indices) - 1, limit, dtype=int)]
    return xyz[indices], rgb[indices]


def visual_relative(previous: np.ndarray, current: np.ndarray, k: np.ndarray) -> tuple[np.ndarray | None, float]:
    detector = cv2.SIFT_create(nfeatures=3000, contrastThreshold=0.025)
    kp0, d0 = detector.detectAndCompute(cv2.cvtColor(previous, cv2.COLOR_BGR2GRAY), None)
    kp1, d1 = detector.detectAndCompute(cv2.cvtColor(current, cv2.COLOR_BGR2GRAY), None)
    if d0 is None or d1 is None:
        return None, math.inf
    pairs = cv2.BFMatcher(cv2.NORM_L2).knnMatch(d0, d1, k=2)
    good = [a for a, b in pairs if a.distance < 0.72 * b.distance]
    if len(good) < 30:
        return None, math.inf
    p0 = np.float32([kp0[m.queryIdx].pt for m in good])
    p1 = np.float32([kp1[m.trainIdx].pt for m in good])
    parallax = float(np.median(np.linalg.norm(p1 - p0, axis=1)))
    essential, mask = cv2.findEssentialMat(p0, p1, k, cv2.RANSAC, 0.999, 1.2)
    if essential is None:
        return None, parallax
    inliers, rotation, translation, _ = cv2.recoverPose(essential, p0, p1, k, mask=mask)
    if int(inliers) < 25:
        return None, parallax
    current_from_previous = np.eye(4)
    current_from_previous[:3, :3] = rotation
    current_from_previous[:3, 3] = translation[:, 0]
    return current_from_previous, parallax


def imu_prev_from_current(previous_rpy: list[float], current_rpy: list[float]) -> np.ndarray:
    world_from_previous = rpy_matrix(previous_rpy)
    world_from_current = rpy_matrix(current_rpy)
    result = np.eye(4)
    result[:3, :3] = world_from_previous.T @ world_from_current
    return result


def odometry_bridge(
    visual_current_from_previous: np.ndarray | None,
    imu_seed: np.ndarray,
    reference_from_camera: np.ndarray,
    translation_step_m: float,
) -> np.ndarray:
    """Advance tracking through a rejected frame without fusing its geometry."""
    bridge = imu_seed.copy()
    if visual_current_from_previous is None:
        return bridge
    previous_from_current_camera = np.linalg.inv(visual_current_from_previous)
    previous_from_current_reference = reference_from_camera @ previous_from_current_camera @ np.linalg.inv(reference_from_camera)
    direction = previous_from_current_reference[:3, 3]
    length = float(np.linalg.norm(direction))
    if length > 1e-8:
        bridge[:3, 3] = direction * (translation_step_m / length)
    return bridge


def register_pair(
    current: np.ndarray,
    previous: np.ndarray,
    visual_current_from_previous: np.ndarray | None,
    imu_seed: np.ndarray,
    reference_from_camera: np.ndarray,
) -> tuple[np.ndarray | None, dict]:
    direction = np.zeros(3)
    if visual_current_from_previous is not None:
        previous_from_current_camera = np.linalg.inv(visual_current_from_previous)
        previous_from_current_reference = reference_from_camera @ previous_from_current_camera @ np.linalg.inv(reference_from_camera)
        direction = previous_from_current_reference[:3, 3]
        norm = np.linalg.norm(direction)
        if norm > 1e-8:
            direction /= norm
    best = None
    for scale in np.linspace(0.0, 0.30, 11):
        initial = imu_seed.copy()
        initial[:3, 3] = direction * scale
        result = icp_register(
            current, previous, initial,
            max_correspondence_m=0.30,
            max_iterations=24,
            min_correspondences=100,
            max_step_translation_m=0.24,
            max_step_rotation_deg=18.0,
        )
        finite_rmse = result.rmse_m if math.isfinite(result.rmse_m) else 10.0
        score = (result.fitness - 1.5 * finite_rmse, result.correspondences)
        if best is None or score > best[0]:
            best = (score, scale, result)
    assert best is not None
    _, scale, result = best
    accepted = result.accepted or (
        result.correspondences >= 140 and math.isfinite(result.rmse_m) and (
            (result.fitness >= 0.15 and result.rmse_m <= 0.12)
            or (result.fitness >= 0.60 and result.rmse_m <= 0.16)
        )
    )
    info = {"accepted": accepted, "fitness": result.fitness, "rmse_m": result.rmse_m,
            "correspondences": result.correspondences, "translation_seed_m": scale, "reason": result.reason}
    return result.transform, info


def write_viewer(path: Path, xyz: np.ndarray, rgb: np.ndarray, title: str) -> None:
    if len(xyz) > 180_000:
        take = np.linspace(0, len(xyz) - 1, 180_000, dtype=int)
        xyz, rgb = xyz[take], rgb[take]
    center = (np.nanmin(xyz, axis=0) + np.nanmax(xyz, axis=0)) * 0.5
    scale = max(1e-6, float(np.max(np.ptp(xyz, axis=0))) * 0.55)
    normalized = ((xyz - center) / scale).astype("<f4")
    blob = bytearray()
    for point, color in zip(normalized, rgb.astype(np.uint8)):
        blob.extend(struct.pack("<fffBBB", *point, *color))
    encoded = base64.b64encode(blob).decode("ascii")
    html = '''<!doctype html><html lang="tr"><meta charset="utf-8"><title>__TITLE__</title>
<style>
html,body,canvas{margin:0;width:100%;height:100%;overflow:hidden;background:#050708;color:#00ffd0}canvas{display:block;touch-action:none}
#hud{position:fixed;left:14px;top:12px;max-width:720px;background:#07100de8;border:1px solid #176b5b;padding:10px 12px;font:12px Consolas;user-select:none}
#bar{display:flex;flex-wrap:wrap;gap:5px;margin-top:8px}button{border:1px solid #176b5b;background:#101817;color:#d7e6e3;padding:5px 8px;font:11px Consolas;cursor:pointer}button:hover,button.active{border-color:#ff9d00;color:#ff9d00}b{color:#ff9d00}.hint{color:#9bb3ae;line-height:1.5;margin-top:7px}
</style>
<canvas id="c"></canvas><div id="hud"><b>APEX TRUE RGB-D CLOUD</b> · __COUNT__ gerçek kamera pikseli<br>
<span>X ileri · Y sol · Z yukarı</span><div id="bar">
<button data-view="front">ÖN</button><button data-view="back">ARKA</button><button data-view="left">SOL</button><button data-view="right">SAĞ</button><button data-view="top">ÜST</button><button data-view="bottom">ALT</button><button data-view="iso">İZOMETRİK</button><button id="mode">SERBEST UÇUŞ</button><button id="reset">MERKEZLE</button>
</div><div class="hint" id="help">Orbit: sol sürükle döndür · sağ/Shift+sürükle kaydır · tekerlek yakınlaştır</div></div>
<script>
const raw=Uint8Array.from(atob('__DATA__'),c=>c.charCodeAt(0)),n=__N__,pos=new Float32Array(n*3),col=new Uint8Array(n*3),dv=new DataView(raw.buffer);
for(let i=0;i<n;i++){const o=i*15;pos.set([dv.getFloat32(o,true),dv.getFloat32(o+4,true),dv.getFloat32(o+8,true)],i*3);col.set([raw[o+12],raw[o+13],raw[o+14]],i*3)}
const c=document.getElementById('c'),g=c.getContext('webgl',{antialias:true});if(!g)throw Error('WebGL desteklenmiyor');
const vs=`attribute vec3 p;attribute vec3 q;uniform mat4 mvp;uniform float size;varying vec3 color;void main(){gl_Position=mvp*vec4(p,1.);gl_PointSize=size;color=q;}`;
const fs=`precision mediump float;varying vec3 color;void main(){gl_FragColor=vec4(color,1.);}`;
function shader(t,s){const x=g.createShader(t);g.shaderSource(x,s);g.compileShader(x);if(!g.getShaderParameter(x,g.COMPILE_STATUS))throw Error(g.getShaderInfoLog(x));return x}
const pr=g.createProgram();g.attachShader(pr,shader(g.VERTEX_SHADER,vs));g.attachShader(pr,shader(g.FRAGMENT_SHADER,fs));g.linkProgram(pr);g.useProgram(pr);
function buffer(data,name,size,norm){const b=g.createBuffer();g.bindBuffer(g.ARRAY_BUFFER,b);g.bufferData(g.ARRAY_BUFFER,data,g.STATIC_DRAW);const a=g.getAttribLocation(pr,name);g.enableVertexAttribArray(a);g.vertexAttribPointer(a,size,data instanceof Uint8Array?g.UNSIGNED_BYTE:g.FLOAT,norm,0,0)}buffer(pos,'p',3,false);buffer(col,'q',3,true);g.enable(g.DEPTH_TEST);
const add=(a,b)=>a.map((v,i)=>v+b[i]),sub=(a,b)=>a.map((v,i)=>v-b[i]),mul=(a,s)=>a.map(v=>v*s),dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],norm=a=>{const l=Math.hypot(...a)||1;return a.map(v=>v/l)};
function matMul(a,b){const o=new Float32Array(16);for(let r=0;r<4;r++)for(let k=0;k<4;k++)for(let cc=0;cc<4;cc++)o[cc*4+r]+=a[k*4+r]*b[cc*4+k];return o}
function perspective(fov,aspect,near,far){const f=1/Math.tan(fov/2),nf=1/(near-far);return new Float32Array([f/aspect,0,0,0,0,f,0,0,0,0,(far+near)*nf,-1,0,0,2*far*near*nf,0])}
function lookAt(eye,target,up){const f=norm(sub(target,eye)),s=norm(cross(f,up)),u=cross(s,f);return new Float32Array([s[0],u[0],-f[0],0,s[1],u[1],-f[1],0,s[2],u[2],-f[2],0,-dot(s,eye),-dot(u,eye),dot(f,eye),1])}
let mode='orbit',target=[0,0,0],az=Math.PI,el=.08,dist=3.0,flyEye=[-3,0,0],flyYaw=0,flyPitch=0,drag=false,pan=false,lx=0,ly=0,last=performance.now();const keys=new Set();
function orbitEye(){return add(target,[dist*Math.cos(el)*Math.cos(az),dist*Math.cos(el)*Math.sin(az),dist*Math.sin(el)])}
function flyForward(){return [Math.cos(flyPitch)*Math.cos(flyYaw),Math.cos(flyPitch)*Math.sin(flyYaw),Math.sin(flyPitch)]}
function camera(){if(mode==='fly')return [flyEye,add(flyEye,flyForward())];return [orbitEye(),target]}
function setView(name){mode='orbit';target=[0,0,0];dist=3.0;const v={front:[Math.PI,0],back:[0,0],left:[Math.PI/2,0],right:[-Math.PI/2,0],top:[Math.PI,Math.PI/2-.001],bottom:[Math.PI,-Math.PI/2+.001],iso:[-2.35,.55]}[name];az=v[0];el=v[1];syncMode()}
function syncMode(){const b=document.getElementById('mode'),h=document.getElementById('help');b.classList.toggle('active',mode==='fly');b.textContent=mode==='fly'?'ORBIT MODUNA DÖN':'SERBEST UÇUŞ';h.textContent=mode==='fly'?'Serbest: sürükle bak · W/S ileri-geri · A/D sağ-sol · Q/E aşağı-yukarı · Shift hızlı':'Orbit: sol sürükle döndür · sağ/Shift+sürükle kaydır · tekerlek yakınlaştır'}
document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>setView(b.dataset.view));document.getElementById('reset').onclick=()=>setView('front');document.getElementById('mode').onclick=()=>{if(mode==='orbit'){flyEye=orbitEye();const f=norm(sub(target,flyEye));flyYaw=Math.atan2(f[1],f[0]);flyPitch=Math.asin(f[2]);mode='fly'}else{target=add(flyEye,mul(flyForward(),dist));mode='orbit'}syncMode()};
c.oncontextmenu=e=>e.preventDefault();c.onpointerdown=e=>{drag=true;pan=e.button===2||e.button===1||e.shiftKey;lx=e.clientX;ly=e.clientY;c.setPointerCapture(e.pointerId)};c.onpointerup=()=>drag=false;
c.onpointermove=e=>{if(!drag)return;const dx=e.clientX-lx,dy=e.clientY-ly;lx=e.clientX;ly=e.clientY;if(mode==='fly'){flyYaw-=dx*.005;flyPitch=Math.max(-1.54,Math.min(1.54,flyPitch-dy*.005))}else if(pan){const eye=orbitEye(),f=norm(sub(target,eye)),up=Math.abs(f[2])>.97?[1,0,0]:[0,0,1],right=norm(cross(f,up)),screenUp=norm(cross(right,f));target=add(target,add(mul(right,-dx*dist*.0013),mul(screenUp,dy*dist*.0013)))}else{az-=dx*.006;el=Math.max(-1.56,Math.min(1.56,el+dy*.006))}};
c.onwheel=e=>{e.preventDefault();if(mode==='orbit')dist=Math.max(.08,Math.min(30,dist*Math.exp(e.deltaY*.001)));else flyEye=add(flyEye,mul(flyForward(),e.deltaY*.001))};onkeydown=e=>keys.add(e.key.toLowerCase());onkeyup=e=>keys.delete(e.key.toLowerCase());
function frame(now){const dt=Math.min(.05,(now-last)/1000);last=now;if(mode==='fly'){const f=flyForward(),right=norm(cross(f,[0,0,1])),up=[0,0,1],speed=(keys.has('shift')?2.0:.55)*dt;let d=[0,0,0];if(keys.has('w'))d=add(d,f);if(keys.has('s'))d=sub(d,f);if(keys.has('d'))d=add(d,right);if(keys.has('a'))d=sub(d,right);if(keys.has('e'))d=add(d,up);if(keys.has('q'))d=sub(d,up);if(Math.hypot(...d)>0)flyEye=add(flyEye,mul(norm(d),speed))}draw();requestAnimationFrame(frame)}
function draw(){const d=devicePixelRatio||1,w=Math.max(1,innerWidth*d),h=Math.max(1,innerHeight*d);if(c.width!==w||c.height!==h){c.width=w;c.height=h;g.viewport(0,0,w,h)}g.clearColor(.02,.03,.035,1);g.clear(g.COLOR_BUFFER_BIT|g.DEPTH_BUFFER_BIT);const [eye,at]=camera(),forward=norm(sub(at,eye)),up=Math.abs(forward[2])>.995?[1,0,0]:[0,0,1],mvp=matMul(perspective(Math.PI/3,w/h,.01,100),lookAt(eye,at,up));g.uniformMatrix4fv(g.getUniformLocation(pr,'mvp'),false,mvp);g.uniform1f(g.getUniformLocation(pr,'size'),Math.max(1.4,2*d));g.drawArrays(g.POINTS,0,n)}
syncMode();requestAnimationFrame(frame);
</script></html>'''
    html = html.replace("__TITLE__", title).replace("__COUNT__", f"{len(xyz):,}")
    html = html.replace("__DATA__", encoded).replace("__N__", str(len(xyz)))
    path.write_text(html, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset")
    parser.add_argument("--rig", default=str(ROOT / "object_scan" / "sensor_rig.json"))
    parser.add_argument("--model-repo", default=str(ROOT / ".runtime" / "vendor" / "Depth-Anything-V2"))
    parser.add_argument("--weights", default=str(ROOT / ".runtime" / "models" / "depth_anything_v2_vits.pth"))
    parser.add_argument("--output", default=None)
    parser.add_argument("--input-size", type=int, default=420)
    parser.add_argument("--stride", type=int, default=4)
    parser.add_argument("--max-depth", type=float, default=6.0)
    parser.add_argument("--voxel-size", type=float, default=0.020)
    parser.add_argument("--max-frames", type=int, default=0, help="0 processes every captured frame")
    args = parser.parse_args()

    dataset = Path(args.dataset)
    output = Path(args.output) if args.output else dataset / "dense-result"
    output.mkdir(parents=True, exist_ok=True)
    (output / "depth-preview").mkdir(exist_ok=True)
    records = json.loads((dataset / "capture.json").read_text(encoding="utf-8"))
    if args.max_frames > 0:
        records = records[:args.max_frames]
    if not records:
        raise RuntimeError("empty capture")

    import torch
    sys.path.insert(0, str(Path(args.model_repo)))
    from depth_anything_v2.dpt import DepthAnythingV2
    model = DepthAnythingV2(encoder="vits", features=64, out_channels=[48, 96, 192, 384])
    model.load_state_dict(torch.load(args.weights, map_location="cpu", weights_only=True))
    model.eval()

    rig = load_sensor_rig(args.rig)
    camera_from_reference = camera_from_lidar(rig)
    reference_from_camera = np.linalg.inv(camera_from_reference)
    cloud = ColoredVoxelCloud(voxel_size_m=args.voxel_size, max_points=1_500_000)
    diagnostics = []
    previous_image = previous_local = previous_rpy = None
    world_from_reference = np.eye(4)
    accepted = 0
    recent_translation_steps: list[float] = []

    for record in records:
        image = cv2.imread(str(dataset / record["frame"]))
        if image is None:
            continue
        h, w = image.shape[:2]
        k = camera_matrix(rig, w, h)
        if k is None:
            f = w / (2 * math.tan(math.radians(float(rig["camera"]["horizontal_fov_deg"])) / 2))
            k = np.array(((f, 0, w/2), (0, f, h/2), (0, 0, 1)), dtype=np.float64)
        relative = model.infer_image(image, args.input_size)
        lidar_camera = lidar_polar_to_camera(record["lidar_points"], camera_from_reference, max_range_m=args.max_depth, forward_half_angle_deg=100)
        try:
            depth, calibration = metric_depth_from_lidar(relative, lidar_camera, k, 0.18, args.max_depth)
        except ValueError as exc:
            diagnostics.append({"index": record["index"], "accepted": False, "reason": str(exc)})
            print(f"[REJECT] {record['index']:03d} {exc}", flush=True)
            continue
        if calibration["median_relative_error"] > 0.40:
            diagnostics.append({"index": record["index"], "accepted": False, "reason": "weak metric fit", "calibration": calibration})
            print(f"[REJECT] {record['index']:03d} metric error={calibration['median_relative_error']:.2f}", flush=True)
            continue
        local_camera, colors = cloud_from_depth(image, depth, k, args.stride)
        dense_reference = transform_points(local_camera, reference_from_camera)
        local_reference, _registration_colors = compact_cloud(dense_reference, colors)
        imu = record.get("imu") or {}
        rpy = list(imu.get("rpy_deg", [0.0, 0.0, 0.0]))
        registration = {"accepted": True, "reason": "first frame"}
        if previous_local is not None:
            seed = imu_prev_from_current(previous_rpy, rpy)
            visual, parallax_px = visual_relative(previous_image, image, k)
            imu_angle_deg = math.degrees(math.acos(np.clip((np.trace(seed[:3, :3]) - 1.0) * 0.5, -1.0, 1.0)))
            if parallax_px < 1.5 and imu_angle_deg < 0.75:
                diagnostics.append({"index": record["index"], "accepted": True, "fused": False,
                                    "reason": "stationary duplicate", "calibration": calibration,
                                    "registration": {"accepted": True, "reason": "stationary duplicate",
                                                     "parallax_px": parallax_px, "imu_angle_deg": imu_angle_deg}})
                print(f"[HOLD] {record['index']:03d} stationary duplicate", flush=True)
                continue
            relative_pose, registration = register_pair(local_reference, previous_local, visual, seed, reference_from_camera)
            registration["parallax_px"] = parallax_px
            registration["imu_angle_deg"] = imu_angle_deg
            if not registration["accepted"]:
                step = float(np.median(recent_translation_steps[-8:])) if recent_translation_steps else 0.025
                use_icp_bridge = (
                    math.isfinite(registration["rmse_m"]) and registration["rmse_m"] <= 0.30
                    and registration["fitness"] >= 0.10 and np.isfinite(relative_pose).all()
                )
                bridge = relative_pose if use_icp_bridge else odometry_bridge(visual, seed, reference_from_camera, step)
                world_from_reference = world_from_reference @ bridge
                previous_image, previous_local, previous_rpy = image, local_reference, rpy
                registration["tracking_bridge"] = "coarse_icp" if use_icp_bridge else "imu_visual"
                diagnostics.append({"index": record["index"], "accepted": False, "fused": False,
                                    "reason": "tracking bridge", "calibration": calibration,
                                    "registration": registration,
                                    "world_from_reference": world_from_reference.tolist()})
                print(f"[BRIDGE] {record['index']:03d} {registration['tracking_bridge']} {registration['fitness']:.2f}/{registration['rmse_m']:.3f}m", flush=True)
                continue
            world_from_reference = world_from_reference @ relative_pose
            step_length = float(np.linalg.norm(relative_pose[:3, 3]))
            if 0.002 <= step_length <= 0.30:
                recent_translation_steps.append(step_length)
        world = transform_points(dense_reference, world_from_reference)
        cloud.integrate(world, colors)
        accepted += 1
        previous_image, previous_local, previous_rpy = image, local_reference, rpy
        finite = depth[np.isfinite(depth)]
        preview = np.zeros_like(depth, dtype=np.uint8)
        if len(finite):
            lo, hi = np.percentile(finite, [2, 98])
            scaled = np.where(np.isfinite(depth), (depth - lo) * 255 / max(1e-6, hi - lo), 0.0)
            preview = np.clip(scaled, 0, 255).astype(np.uint8)
        cv2.imwrite(str(output / "depth-preview" / f"{record['index']:05d}.png"), cv2.applyColorMap(255-preview, cv2.COLORMAP_TURBO))
        diagnostics.append({"index": record["index"], "accepted": True, "calibration": calibration, "registration": registration, "points": len(local_reference), "world_from_reference": world_from_reference.tolist()})
        print(f"[OK] {record['index']:03d} dense={len(local_reference)} fit={calibration['median_relative_error']:.2f} cloud={len(cloud)}", flush=True)

    if accepted == 0:
        raise RuntimeError("all RGB-D frames were rejected")
    ply = cloud.export_ply(output / "dense-rgb-cloud.ply", minimum_observations=1)
    xyz, rgb, _ = cloud.arrays()
    write_viewer(output / "dense-rgb-viewer.html", xyz, rgb, "APEX Dense RGB-D Cloud")
    summary = {"input_frames": len(records), "accepted_frames": accepted, "points": len(xyz), "ply": str(ply), "viewer": str(output / "dense-rgb-viewer.html"), "diagnostics": diagnostics}
    (output / "fusion-report.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in summary.items() if k != "diagnostics"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
