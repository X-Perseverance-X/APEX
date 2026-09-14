#!/usr/bin/env python3
"""
APEX Hexapod — RPi5 Ana Kontrol Sunucusu
FastAPI + WebSocket — Browser UI ile ESP32 (192.168.7.50) arasında köprü

Çalıştır:
    uvicorn apex_server:app --host 0.0.0.0 --port 8080
"""

import asyncio
import hashlib
import hmac
import json
import logging
import os
import queue
import threading
import time
from contextlib import asynccontextmanager
from html import escape
from typing import Optional
from urllib.parse import parse_qs

import httpx
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, FileResponse, StreamingResponse, HTMLResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

try:
    from rplidar import RPLidar
    RPLIDAR_AVAILABLE = True
except ImportError:
    RPLIDAR_AVAILABLE = False

try:
    import numpy as np
    from scipy.spatial import cKDTree
    SLAM_AVAILABLE = True
except ImportError:
    SLAM_AVAILABLE = False

import math

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("apex")

try:
    from mapping_v2 import ImuSample, LidarScan, MapArchive, MappingEngine, TofSample
    MAPPING_V2_AVAILABLE = True
except ImportError as mapping_import_error:
    MAPPING_V2_AVAILABLE = False
    MappingEngine = None
    MapArchive = None
    log.warning(f"[MAPPING-V2] Yüklenemedi: {mapping_import_error}")

# ─── Konfigürasyon ───────────────────────────────────────────────────────────
# ESP32_URL ortam değişkeninden okunur; kod düzenlemeden değiştirilebilir:
#   ESP32_URL=http://192.168.1.42 uvicorn apex_server:app --host 0.0.0.0 --port 8080

ESP32_URL    = os.environ.get("ESP32_URL", "http://192.168.7.50")
TELEMETRY_HZ = 5                        # Telemetri poll hızı (saniyede)
HTTP_TIMEOUT = 0.8                      # ESP32 HTTP timeout (saniye)
SIM_MODE     = os.environ.get("APEX_SIM_MODE", "0").strip().lower() in {"1", "true", "yes", "on"}
AUTH_ENABLED = os.environ.get("APEX_AUTH_ENABLED", "0").strip().lower() in {"1", "true", "yes", "on"}
AUTH_USER = os.environ.get("APEX_AUTH_USER", "apex")
AUTH_PASSWORD_HASH = os.environ.get("APEX_AUTH_PASSWORD_HASH", "")
AUTH_SECRET = os.environ.get("APEX_AUTH_SECRET", "")
AUTH_COOKIE = "apex_session"
AUTH_MAX_AGE_S = 12 * 60 * 60

LIDAR_PORT     = os.environ.get("LIDAR_PORT", "/dev/ttyLIDAR")
# NOT: eskiden /dev/ttyUSB1 sabit yazılıydı — ama gerçek kernel logunda
# doğrulandı (over-current olayı), USB yeniden bağlanınca numara değişip
# ttyUSB2 olabiliyor. /dev/ttyLIDAR, aşağıdaki udev kuralıyla LiDAR'ın
# SERİ NUMARASINA bağlı sabit bir isim — hangi ttyUSBx olursa olsun hep
# doğru cihazı gösterir.
LIDAR_BAUDRATE = 115200
LIDAR_BROADCAST_HZ = 10                 # Tarayıcıya yayın hızı (saniyede)
MAPPING_SENSOR_POLL_HZ = 20             # Yeni tam LiDAR turunu düşük faz gecikmesiyle yakala
NAVIGATION_BROADCAST_MAX_HZ = 10        # Yalnız durum değiştiğinde; boş tekrar paketi gönderilmez

# Tarayıcının her zaten yaptığı /telemetry isteğinin sonucu burada önbelleğe
# alınır — LiDAR thread'i ESP32'ye EK bir istek atmadan en güncel eğim/yükseklik
# değerine ulaşır. Bu hem ESP32 yükünü artırmaz hem de gecikmeyi azaltır.
_telemetry_cache_lock = threading.Lock()
_telemetry_cache = {
    "mpuP": 0.0, "mpuR": 0.0, "mpuY": 0.0, "mpuOnline": False,
    "tofOnline": False, "tofMm": None, "z": -100.0,
}

# /telemetry için KISA SÜRELİ (200ms) TAM YANIT önbelleği. Birden fazla
# bağımsız poller (tarayıcı ~400ms, detection_hud.py ~500ms) AYNI ESP32'ye
# ayrı ayrı istek atıyordu — ESP32'nin tek-iş-parçacıklı web sunucusuna
# saniyede ~4.5 istek biniyordu, bu da ESP32 reset sıklığının artmasıyla
# zamanlama olarak örtüştü (detection_hud.py'nin server çökmesi düzelene
# kadar bu yükü hiç oluşturamadığını fark ettik). HUD'ların 400/500ms poll
# aralığından biraz uzun pencere, iki tüketicinin ESP32'nin tek iş parçacıklı
# WebServer'ını sırayla yüklemesini engeller ve kokpiti 2Hz üstünde tutar.
_telemetry_resp_cache_lock = threading.Lock()
_telemetry_resp_cache = {"data": None, "ts": 0.0}
TELEMETRY_CACHE_TTL_S = 0.45

# apex_ui.html bu script ile AYNI klasörde olmalı
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
UI_FILE  = os.path.join(THIS_DIR, "apex_ui.html")
CAMERA_UI_FILE = os.path.join(THIS_DIR, "camera_console.html")
MAP_ARCHIVE_DIR = os.environ.get("APEX_MAP_DIR", os.path.join(THIS_DIR, ".runtime", "maps"))
_map_archive = MapArchive(MAP_ARCHIVE_DIR) if MAPPING_V2_AVAILABLE else None
NAVIGATION_UI_FILE = os.path.join(THIS_DIR, "navigation_console.html")
THREE_JS_FILE = os.path.join(THIS_DIR, "apex_kinematics_lab", "static", "three.min.js")


# ─── Robot kol / güvenli kalibrasyon ──────────────────────────────────────
# Kullanıcının net mekanik tanımı:
#   base:    90° başlangıç/ön merkez; 5..175° güvenli sınır.
#   joint1: 5° ≈ gövde üstüne katlı, 175° ≈ öne doğru açılma.
#   joint2: 5° ≈ bekleme/katlı mod, 175° ≈ dışarı doğru açılma.
#   gripper: rack-and-pinion; 90° açık, kapatmak için 90° üstüne/180° yönüne gider.
#            kapalı açı = kullanıcının bulup kaydettiği gripper MAX değeri.
#
# Bu bölüm mevcut bacak/joy/bodyIK endpointlerine dokunmaz. Kol için sadece
# /armPose veya /armServo endpoint'i kullanılır. Böylece yürüyüş sistemi korunur.
ARM_BASE_H_MM = float(os.environ.get("ARM_BASE_H_MM", "70.0"))
ARM_L1_MM     = float(os.environ.get("ARM_L1_MM", "200.0"))
ARM_L2_MM     = float(os.environ.get("ARM_L2_MM", "270.0"))

ARM_DRIVER_MODE = os.environ.get("ARM_DRIVER_MODE", "esp32_batch").lower()
ARM_POSE_ENDPOINT  = os.environ.get("ARM_POSE_ENDPOINT", f"{ESP32_URL}/armPose")
ARM_SERVO_ENDPOINT = os.environ.get("ARM_SERVO_ENDPOINT", f"{ESP32_URL}/armServo")
ARM_OFF_ENDPOINT   = os.environ.get("ARM_OFF_ENDPOINT",   f"{ESP32_URL}/armOff")
ARM_COMMAND_TIMEOUT_S = float(os.environ.get("ARM_COMMAND_TIMEOUT_S", "1.2"))

ARM_SERVO_MIN_DEG = 5.0
ARM_SERVO_MAX_DEG = 175.0
ARM_JOINTS = ("base", "joint1", "joint2", "gripper")
ARM_CALIB_FILE = os.environ.get("ARM_CALIB_FILE", os.path.join(THIS_DIR, "apex_arm_calibration.json"))

DEFAULT_ARM_CALIBRATION = {
    "limits": {
        "base":    {"min": ARM_SERVO_MIN_DEG, "max": ARM_SERVO_MAX_DEG},
        "joint1":  {"min": ARM_SERVO_MIN_DEG, "max": ARM_SERVO_MAX_DEG},
        "joint2":  {"min": ARM_SERVO_MIN_DEG, "max": ARM_SERVO_MAX_DEG},
        # Rack-and-pinion gripper: 90° açık sabit; kapatma yönü 90° -> 180°.
        # MAX, kullanıcının dişliyi zorlamadan bulduğu kapalı açı olarak kullanılır.
        "gripper": {"min": 90.0, "max": 120.0},
    },
    # Servo horn takarken kullanılacak elektriksel orta poz: hepsi 90°.
    "calibration_pose": {"base": 90.0, "joint1": 90.0, "joint2": 90.0, "gripper": 90.0},
    # Mekanik bekleme: base öne, joint1 gövde üstüne katlı, joint2 katlı/bekleme.
    "home_pose":        {"base": 90.0, "joint1": 5.0, "joint2": 5.0,  "gripper": 90.0},
    # IK matematik açılarını senin servo anlamlarına çeviren güvenli varsayılan.
    # base 90 merkez; joint1 artık ters takılı: 5° katlı, 175° öne açık. Bu yüzden
    # matematiksel q1 servo tarafına TERS işaretle çevrilir. joint2 aynı kaldı.
    "ik_map": {
        "base_center": 90.0, "base_dir": 1.0,
        "joint1_center": 90.0, "joint1_dir": -1.0,
        "joint2_zero": 5.0, "joint2_dir": 1.0,
    },
    # Gripper: açık poz her zaman 90°. Kapalı poz = MAX; kullanıcı dişliye zarar
    # vermeyecek gerçek kapalı açıyı bulup MAX olarak kaydeder.
    "gripper": {"open": 90.0, "close": 120.0, "current": 90.0},
}

_arm_calib_lock = threading.Lock()
_arm_calibration = None
_arm_last_result_lock = threading.Lock()
_arm_last_result = {"ok": False, "state": "idle", "ts": 0.0, "msg": "hazır"}

# Kamera/HUD MJPEG yayını ayrı process'te 8888 portunda çalışıyor. Tarayıcı
# artık doğrudan :8888'e gitmek zorunda değil; /camera/stream aynı 8080
# origin'i üzerinden proxy eder. Bu özellikle APEX-HUB'a başka PC/telefon
# bağlandığında port/firewall/localhost karışıklığını bitirir.
CAMERA_STREAM_URL = os.environ.get("CAMERA_STREAM_URL", "http://127.0.0.1:8888")


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _deepcopy_json(obj: dict) -> dict:
    return json.loads(json.dumps(obj))


def _merge_dict(base: dict, incoming: dict) -> dict:
    out = _deepcopy_json(base)
    for k, v in (incoming or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge_dict(out[k], v)
        else:
            out[k] = v
    return out


def _normalize_arm_mechanics(data: dict) -> dict:
    """Eski kalibrasyon dosyası varsa bile yeni mekanik gerçek korunur.

    Kullanıcı mekanik yönü değiştirdi:
    - joint1: 5° gövde üstüne katlı, 175° öne açılır.
    - gripper rack-and-pinion: 90° açık sabit, kapatma 90° -> 180° yönünde;
      kapalı açı her zaman gripper MAX değeridir.
    """
    data = _merge_dict(DEFAULT_ARM_CALIBRATION, data or {})
    data.setdefault("home_pose", {})["joint1"] = 5.0
    data.setdefault("ik_map", {})["joint1_dir"] = -1.0
    data["ik_map"]["joint1_center"] = 90.0

    limits = data.setdefault("limits", {}).setdefault("gripper", {})
    # Açık 90 olduğu için gripper minimumu artık 90'dan aşağı inmez.
    gmax = float(limits.get("max", DEFAULT_ARM_CALIBRATION["limits"]["gripper"]["max"]))
    gmax = _clamp(max(90.0, gmax), 90.0, ARM_SERVO_MAX_DEG)
    limits["min"] = 90.0
    limits["max"] = round(gmax, 2)

    grip = data.setdefault("gripper", {})
    grip["open"] = 90.0
    grip["close"] = round(gmax, 2)
    grip["current"] = round(_clamp(float(grip.get("current", 90.0)), 90.0, gmax), 2)
    return data


def _load_arm_calibration() -> dict:
    global _arm_calibration
    with _arm_calib_lock:
        if _arm_calibration is not None:
            return _deepcopy_json(_arm_calibration)
        data = _deepcopy_json(DEFAULT_ARM_CALIBRATION)
        try:
            if os.path.exists(ARM_CALIB_FILE):
                with open(ARM_CALIB_FILE, "r", encoding="utf-8") as f:
                    data = _merge_dict(data, json.load(f))
        except Exception as e:
            log.warning(f"[ARM-CAL] Kalibrasyon dosyası okunamadı, varsayılan kullanılacak: {e}")
        _arm_calibration = _normalize_arm_mechanics(data)
        return _deepcopy_json(_arm_calibration)


def _save_arm_calibration(calib: dict) -> None:
    global _arm_calibration
    with _arm_calib_lock:
        _arm_calibration = _normalize_arm_mechanics(calib)
        try:
            tmp = ARM_CALIB_FILE + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(_arm_calibration, f, ensure_ascii=False, indent=2)
            os.replace(tmp, ARM_CALIB_FILE)
        except Exception as e:
            log.warning(f"[ARM-CAL] Kalibrasyon dosyası yazılamadı: {e}")


def _arm_limits(joint: str) -> tuple[float, float]:
    calib = _load_arm_calibration()
    lim = calib.get("limits", {}).get(joint, {"min": ARM_SERVO_MIN_DEG, "max": ARM_SERVO_MAX_DEG})
    lo = float(lim.get("min", ARM_SERVO_MIN_DEG))
    hi = float(lim.get("max", ARM_SERVO_MAX_DEG))
    if hi < lo:
        lo, hi = hi, lo
    return lo, hi


def _arm_clamp_joint(joint: str, angle: float) -> float:
    lo, hi = _arm_limits(joint)
    return round(_clamp(float(angle), lo, hi), 2)


def _arm_normalize_payload(payload: dict) -> dict:
    """Sadece kullanıcının tanımladığı 4 kanal kalır: base, joint1, joint2, gripper."""
    out = {}
    for j in ARM_JOINTS:
        if j in payload and payload[j] is not None:
            out[j] = _arm_clamp_joint(j, float(payload[j]))
    return out


def _arm_esp_config_params() -> dict:
    """Python UI'da ayarlanan gripper limitlerini ESP32 firmware'e de yollar.
    Böylece ESP tarafındaki son güvenlik katmanı, UI'da seçilen dar aralığı bilir."""
    calib = _load_arm_calibration()
    g_lim = calib.get("limits", {}).get("gripper", {})
    g_cfg = calib.get("gripper", {})
    return {
        "gmin": 90.0,
        "gmax": float(g_lim.get("max", 120.0)),
        "gopen": 90.0,
        "gclose": float(g_lim.get("max", 120.0)),
    }


def _arm_ik_xyz(target: dict) -> dict:
    """Base + joint1 + joint2 için analitik IK.

    Çıktı matematiksel derecedir:
      base   = sağ/sol yaw, 0° kamera önü.
      joint1 = link-1 açısı; servo anlamı kalibrasyon katmanında uygulanır.
      joint2 = elbow açısı; senin tanımına göre 5° bekleme, açı arttıkça dışarı açılır.
    """
    x = float(target["x"])
    y = float(target["y"])
    z = float(target["z"])
    base = math.degrees(math.atan2(x, max(1e-6, y)))
    r = math.hypot(x, y)
    s = z - ARM_BASE_H_MM
    d = math.hypot(r, s)
    max_reach = ARM_L1_MM + ARM_L2_MM - 2.0
    min_reach = abs(ARM_L1_MM - ARM_L2_MM) + 2.0
    reachable = min_reach <= d <= max_reach
    if d > max_reach:
        scale = max_reach / d
        r *= scale
        s *= scale
        d = max_reach
    elif d < min_reach:
        scale = min_reach / max(d, 1e-6)
        r *= scale
        s *= scale
        d = min_reach

    c2 = _clamp((d*d - ARM_L1_MM*ARM_L1_MM - ARM_L2_MM*ARM_L2_MM) / (2.0 * ARM_L1_MM * ARM_L2_MM), -1.0, 1.0)
    q2 = math.atan2(math.sqrt(max(0.0, 1.0 - c2*c2)), c2)
    q1 = math.atan2(s, r) - math.atan2(ARM_L2_MM * math.sin(q2), ARM_L1_MM + ARM_L2_MM * math.cos(q2))
    joint1 = math.degrees(q1)
    joint2 = math.degrees(q2)
    return {
        "base": base,
        "joint1": joint1,
        "joint2": joint2,
        "reachable": reachable,
        "target_used": {"x": float(target["x"]), "y": float(target["y"]), "z": float(target["z"])},
        "target_solved": {"r": r, "z_rel": s, "d": d},
    }


def _arm_pose_to_payload(joints: dict, gripper_deg: float) -> dict:
    calib = _load_arm_calibration()
    m = calib.get("ik_map", {})
    payload = {
        "base":    float(m.get("base_center", 90.0)) + float(m.get("base_dir", 1.0)) * float(joints["base"]),
        "joint1":  float(m.get("joint1_center", 90.0)) + float(m.get("joint1_dir", 1.0)) * float(joints["joint1"]),
        "joint2":  float(m.get("joint2_zero", 5.0)) + float(m.get("joint2_dir", 1.0)) * float(joints["joint2"]),
        "gripper": float(gripper_deg),
    }
    return _arm_normalize_payload(payload)


async def _arm_send_payload(payload: dict, label: str) -> dict:
    """Hazır servo derecelerini gönderir. Bacak/joy endpointlerine dokunmaz."""
    payload = _arm_normalize_payload(payload)
    if not payload:
        return {"ok": False, "error": "Boş kol payload'u"}
    send_payload = dict(payload)
    send_payload.update(_arm_esp_config_params())
    send_payload["label"] = label

    if ARM_DRIVER_MODE in {"dry", "dryrun", "sim", "simulation"}:
        log.info(f"[ARM] DRYRUN {label}: {send_payload}")
        return {"ok": True, "dryrun": True, "payload": send_payload}

    try:
        async with esp32_lock:
            async with httpx.AsyncClient(timeout=ARM_COMMAND_TIMEOUT_S) as client:
                if ARM_DRIVER_MODE in {"esp32_batch", "batch", "pose"}:
                    r = await client.get(ARM_POSE_ENDPOINT, params=send_payload)
                    return {"ok": 200 <= r.status_code < 300, "status": r.status_code, "body": r.text[:160], "payload": send_payload}
                if ARM_DRIVER_MODE in {"esp32_servo", "servo"}:
                    for k in ARM_JOINTS:
                        if k not in payload:
                            continue
                        servo_params = {"id": k, "angle": payload[k]}
                        servo_params.update(_arm_esp_config_params())
                        r = await client.get(ARM_SERVO_ENDPOINT, params=servo_params)
                        if not (200 <= r.status_code < 300):
                            return {"ok": False, "status": r.status_code, "body": r.text[:160], "payload": send_payload, "failed": k}
                        await asyncio.sleep(0.035)
                    return {"ok": True, "payload": send_payload}
        return {"ok": False, "error": f"Bilinmeyen ARM_DRIVER_MODE={ARM_DRIVER_MODE}"}
    except Exception as e:
        return {"ok": False, "error": str(e), "payload": send_payload}


async def _arm_send_pose(joints: dict, gripper_deg: float, label: str) -> dict:
    return await _arm_send_payload(_arm_pose_to_payload(joints, gripper_deg), label)


async def _vision_tracking_status() -> dict:
    try:
        async with httpx.AsyncClient(timeout=0.45) as client:
            r = await client.get("http://127.0.0.1:8888/tracking/status")
        if r.status_code != 200:
            return {"state": "offline", "selected": False, "visible": False, "error": f"HUD HTTP {r.status_code}"}
        return r.json()
    except Exception as e:
        return {"state": "offline", "selected": False, "visible": False, "error": str(e)}


def _arm_set_last(ok: bool, state: str, msg: str, extra: Optional[dict] = None) -> None:
    global _arm_last_result
    data = {"ok": ok, "state": state, "ts": time.time(), "msg": msg}
    if extra:
        data.update(extra)
    with _arm_last_result_lock:
        _arm_last_result = data


# ─── Bağlantı yöneticisi ─────────────────────────────────────────────────────

class ConnectionManager:
    def __init__(self):
        self.clients: set[WebSocket] = set()

    async def connect(self, ws: WebSocket):
        await ws.accept()
        self.clients.add(ws)
        log.info(f"[WS] Client bağlandı — toplam: {len(self.clients)}")

    def disconnect(self, ws: WebSocket):
        self.clients.discard(ws)
        log.info(f"[WS] Client ayrıldı — toplam: {len(self.clients)}")

    async def broadcast(self, msg: dict):
        if not self.clients:
            return
        data = json.dumps(msg, ensure_ascii=False)
        dead: set[WebSocket] = set()
        for ws in self.clients:
            try:
                await ws.send_text(data)
            except Exception:
                dead.add(ws)
        self.clients -= dead


mgr        = ConnectionManager()
esp_online = False   # ESP32 erişilebilirlik durumu

# ─── LiDAR arka plan thread'i ────────────────────────────────────────────────
# rplidar kütüphanesi senkron/blocking olduğu için asyncio event loop'unu
# bloke etmemesi için ayrı bir thread'de çalışır. En son taramayı paylaşılan
# bir listede tutar; WebSocket istemcileri bunu periyodik olarak okur.

_lidar_lock   = threading.Lock()
_lidar_points: list[tuple[float, float]] = []   # [(açı_derece, mesafe_mm), ...]
_lidar_online = False
_lidar_thread_started = False
_lidar_enabled = False  # güvenli/enerji tasarruflu açılış: kullanıcı veya OTO mod istemeden motor dönmez
_lidar_last_scan_ts = 0.0
_lidar_scan_seq = 0
_lidar_last_error = ""
_lidar_motor_command_lock = asyncio.Lock()
_mapping_archive_lock = asyncio.Lock()

# mapping_v2 LiDAR/kamera aygıtı açmaz ve hiçbir aktüatör endpoint'i bilmez.
# Mevcut LiDAR sahibinin paylaşılan son taramasını salt okunur tüketir.
_mapping_engine = MappingEngine() if MAPPING_V2_AVAILABLE else None
_mapping_thread_started = False

# Yerel harita rotası bütün waypoint parçalarını düşük hızda yürütür. Konum
# odometrisi henüz güvenilir olmadığı için süre, açıkça ayarlanabilir ölçülmüş
# hız tahmininden hesaplanır; katı uzunluk/süre ve engel sınırları korunur.
NAV_ROUTE_COMMAND_HZ = 10.0
NAV_PLATFORM_TEST_REQUEST_BUDGET_S = 0.30
NAV_ROUTE_JOY_MAG = 0.20
NAV_ROUTE_FULL_SPEED_MPS = float(os.environ.get("APEX_NAV_FULL_SPEED_MPS", "0.18"))
NAV_ROUTE_MAX_LENGTH_M = float(os.environ.get("APEX_NAV_MAX_LENGTH_M", "4.0"))
NAV_ROUTE_MAX_DURATION_S = float(os.environ.get("APEX_NAV_MAX_DURATION_S", "120"))
NAV_ROUTE_OBSTACLE_STOP_M = float(os.environ.get("APEX_NAV_OBSTACLE_STOP_M", "0.35"))
NAV_PLATFORM_TEST_MIN_BATTERY_V = 10.2
_navigation_platform_task: asyncio.Task | None = None
_navigation_platform_status = {
    "state": "IDLE", "active": False, "message": "Rota yürütücüsü hazır",
    "max_duration_s": NAV_ROUTE_MAX_DURATION_S,
}

# AI TAKİP (kişi takibi) — VARSAYILAN KAPALI, güvenlik için. detection_hud.py
# bu bayrak True olmadan ASLA hareket komutu (/joy) göndermez. Kullanıcı
# arayüzden bilerek açmadıkça robot kendiliğinden hareket etmez.
_ai_tracking_enabled = False

# Hedef SEÇİMİ (tıkla-takip-et) — kullanıcı arayüzde bir kişiye tıkladığında
# normalize edilmiş (0-1) koordinatları burada saklanır. detection_hud.py
# bunu /telemetry üzerinden okuyup HANGİ track_id'nin o noktada olduğunu
# bulup kilitler. "ts" (zaman damgası) her tıklamada artar — detection_hud.py
# bunu görüp "yeni bir seçim var" diye anlar (aynı seçimi tekrar işlemez).
_target_selection = {"pending": False, "x": 0.0, "y": 0.0, "ts": 0.0, "clear": False}
_target_selection_lock = threading.Lock()

# ─── LiDAR OTONOM YÜRÜYÜŞ (koridor ortalama + duvar kaçınma) ────────────────
# VARSAYILAN KAPALI — güvenlik. /lidar_nav?state=1 ile açılır.
# ⚠️ AÇI KURALI DOĞRULANMADI: LiDAR'ın 0° okumasının robotun GERÇEKTEN
# "ileri" yönüne denk geldiği bu projede HİÇ test edilmedi (önceden sadece
# görselleştirme için kullanıldı, gerçek hareket kararı için ilk kez
# kullanılıyor). Yanlışsa LIDAR_FORWARD_ANGLE_DEG'i ayarlaman gerekir.
_lidar_nav_enabled = False
LIDAR_FORWARD_ANGLE_DEG = 0     # "ileri" — DOĞRULA
LIDAR_LEFT_ANGLE_DEG    = 90    # "sol" — DOĞRULA
LIDAR_RIGHT_ANGLE_DEG   = 270   # "sağ" — DOĞRULA
NAV_WALL_TURN_DIST_M    = 0.5   # bu mesafede ön duvar varsa dur + dön
NAV_SIDE_DANGER_DIST_M  = 0.3   # dönerken bile yan taraf bu kadar yakınsa tam dur (köşe)
NAV_FORWARD_SPEED       = 0.40
NAV_TURN_SPEED          = 0.40  # duvar kaçınmada sabit dönüş hızı
NAV_CENTERING_GAIN      = 0.50  # koridor ortalama kazancı
NAV_MAX_CENTERING_TURN  = 0.30
NAV_COMMAND_HZ          = 5

# SLAM işlemesi (ICP, harita büyüdükçe yavaşlar) LiDAR okuma thread'inden
# AYRI bir thread'de çalışır — yoksa ICP gecikmesi seri port okumasını
# bloke edip "yavaşladı" hissine yol açıyordu. maxsize=1: kuyrukta her zaman
# sadece EN TAZE tarama tutulur, eski/işlenmemiş taramalar atılır (backlog
# büyümesin — SLAM için en güncel veri yeterli, eskileri işlemek gereksiz).
_slam_queue: "queue.Queue" = queue.Queue(maxsize=1)

# ─── SLAM (ICP tabanlı scan-matching) ────────────────────────────────────────
# Odometri kaynağımız yok (MPU6050 sadece eğim verir, konum vermez). Bunun yerine
# art arda gelen LiDAR taramalarını birbirine hizalayarak (ICP) robotun göreli
# hareketini geometrik olarak kestiriyoruz. Loop-closure YOK — uzun süre
# çalıştırılırsa küçük sapmalar birikebilir, ama oda ölçeğinde haritalama için
# yeterli ve gerçek bir konum/harita üretir (egocentric + decay yöntemi yerine).

_slam_lock  = threading.Lock()
_slam_pose  = {"x": 0.0, "y": 0.0, "theta": 0.0}   # metre, metre, radyan — DÜNYA çerçevesi
_voxel_map: dict[tuple[int, int], tuple[float, float, float]] = {}  # (wx,wy,height) — seyrekleştirilmiş harita

# Kalıcı SLAM haritalama KAPALI — kararsız/sürüklenen davranış nedeniyle kullanıcı
# isteğiyle devre dışı bırakıldı. Bunun yerine arayüz ANLIK (egocentric, ~8s
# eskime) taramayı gösteriyor — sürüklenme/spiral riski yok, her tarama anında
# güncellenir. Tekrar açmak için: True yap.
SLAM_MAPPING_ENABLED = False

VOXEL_SIZE         = 0.05    # metre — harita noktalarını seyrekleştirme hücre boyutu
MAX_MAP_POINTS     = 8000
ICP_MAX_ITERS      = 15
ICP_MAX_CORR_DIST  = 0.3     # metre — bu mesafeden uzak eşleşmeler reddedilir
ICP_MIN_POINTS     = 10

# ── Sapma/spiral düzeltmeleri (gerçek veri ile teşhis edildi) ───────────────
FIT_THRESHOLD       = 0.6                # eski: 0.3 — çok gevşekti, zayıf eşleşmeleri kabul ediyordu
MAX_TRANS_PER_SCAN  = 0.05               # metre — bir taramada bundan büyük düzeltme fiziksel olarak anlamsız, reddedilir
MAX_ROT_PER_SCAN    = math.radians(15)   # bir taramada bundan büyük dönüş düzeltmesi reddedilir
MIN_MAP_FOR_ROTATION = 150               # harita bu kadar nokta toplayana kadar ROTASYON düzeltmesi UYGULANMAZ
                                          # (bootstrap kilitlenmesi: küçük haritada ICP'nin döndürme tahmini
                                          # güvenilmez, erken küçük bir hata kalıcı olarak haritaya yazılıp
                                          # bir daha düzelmiyor — uzun oturumda spirale dönüşüyor)
MAP_BLEND_ALPHA     = 0.3                # mevcut hücre + yeni nokta ortalaması (overwrite yerine), gürültü smear'ini azaltır

def _scan_to_xy(scan_points):
    """[(açı_derece, mesafe_mm), ...] -> Nx2 numpy array, METRE, robot lokal çerçevesi."""
    pts = []
    for angle_deg, dist_mm in scan_points:
        if not (dist_mm > 0) or dist_mm > 6000:
            continue
        rad = math.radians(angle_deg)
        d_m = dist_mm / 1000.0
        pts.append((d_m * math.cos(rad), d_m * math.sin(rad)))
    return np.array(pts) if pts else np.empty((0, 2))

def _icp(src, dst, max_iters=ICP_MAX_ITERS, max_corr=ICP_MAX_CORR_DIST):
    """src noktalarını dst'ye hizalayan rigid transformu (R, t) tahmin eder.
    src, dst: Nx2 / Mx2 numpy array (metre). Dönüş: (R 2x2, t uzunluk-2, fit_ratio)."""
    if len(src) < ICP_MIN_POINTS or len(dst) < ICP_MIN_POINTS:
        return np.eye(2), np.zeros(2), 0.0

    src_cur  = src.copy()
    R_total  = np.eye(2)
    t_total  = np.zeros(2)
    tree     = cKDTree(dst)
    mask     = np.zeros(len(src), dtype=bool)

    for _ in range(max_iters):
        dists, idx = tree.query(src_cur)
        mask = dists < max_corr
        if mask.sum() < ICP_MIN_POINTS:
            break
        matched_src = src_cur[mask]
        matched_dst = dst[idx[mask]]

        src_mean = matched_src.mean(axis=0)
        dst_mean = matched_dst.mean(axis=0)
        H = (matched_src - src_mean).T @ (matched_dst - dst_mean)
        U, _S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        t = dst_mean - R @ src_mean

        src_cur = (R @ src_cur.T).T + t
        R_total = R @ R_total
        t_total = R @ t_total + t

    fit_ratio = float(mask.sum()) / len(src) if len(src) else 0.0
    return R_total, t_total, fit_ratio

def _slam_update(scan_points):
    """Yeni bir tarama geldiğinde çağrılır: pozu günceller, dünya haritasına ekler.

    SCAN-TO-MAP eşleştirme: her yeni tarama sadece BİR ÖNCEKİ taramayla değil,
    o ana kadar BİRİKMİŞ TÜM HARİTA ile karşılaştırılır. Bu, scan-to-scan'e göre
    çok daha az sapma biriktirir — her adımın hatası bir öncekine eklenmek yerine
    her zaman global haritaya göre düzeltilir (tur attıkça duvarların "ikiye"
    görünmesi/birbirine girmesi büyük ölçüde önlenir).
    """
    global _slam_pose

    scan_xy = _scan_to_xy(scan_points)
    if len(scan_xy) < ICP_MIN_POINTS:
        return

    with _telemetry_cache_lock:
        pitch_deg = _telemetry_cache["mpuP"]
        roll_deg  = _telemetry_cache["mpuR"]
        z_mm      = _telemetry_cache["z"]

    with _slam_lock:
        # 1) Mevcut poz tahminiyle taramayı DÜNYA çerçevesine taşı (ilk tahmin)
        cos_g, sin_g = math.cos(_slam_pose["theta"]), math.sin(_slam_pose["theta"])
        guess_world = np.column_stack([
            scan_xy[:, 0] * cos_g - scan_xy[:, 1] * sin_g + _slam_pose["x"],
            scan_xy[:, 0] * sin_g + scan_xy[:, 1] * cos_g + _slam_pose["y"],
        ])

        # 2) İlk tahmini, BİRİKMİŞ HARİTAYA göre düzelt (scan-to-map)
        if len(_voxel_map) > ICP_MIN_POINTS:
            map_xy = np.array([(v[0], v[1]) for v in _voxel_map.values()])
            R, t, fit = _icp(guess_world, map_xy)
            dtheta = math.atan2(R[1, 0], R[0, 0])

            accept = fit > FIT_THRESHOLD
            # ÖNEMLİ: |t| DOĞRUDAN kontrol edilemez — t dünya orijinine göre
            # tanımlı, robot orijinden uzaktaysa küçük bir dönüş bile büyük
            # |t| üretir (yarıçap × açı). Doğru ölçüm: ROBOTUN KENDİ POZİSYONU
            # bu düzeltmeyle ne kadar değişiyor (ilk denemede bunu atlayıp
            # ham |t|'yi kontrol etmiştim — gerçek dönüş hareketini de
            # reddediyordu, test sırasında yakalandı).
            old_pos  = np.array([_slam_pose["x"], _slam_pose["y"]])
            new_pos  = R @ old_pos + t
            disp_mag = float(np.linalg.norm(new_pos - old_pos))
            if accept and (disp_mag > MAX_TRANS_PER_SCAN or abs(dtheta) > MAX_ROT_PER_SCAN):
                accept = False
            # Bootstrap kilitlenmesi koruması: harita henüz yeterince büyük/çeşitli
            # değilse ROTASYON bileşenini uygulama (sadece konum düzelt). Küçük
            # haritada dönüş tahmini güvenilmez ve kalıcı spiral hatasına yol açar.
            apply_rotation = accept and len(_voxel_map) >= MIN_MAP_FOR_ROTATION

            if accept:
                old_x, old_y = _slam_pose["x"], _slam_pose["y"]
                if apply_rotation:
                    _slam_pose["x"]      = R[0, 0] * old_x + R[0, 1] * old_y + t[0]
                    _slam_pose["y"]      = R[1, 0] * old_x + R[1, 1] * old_y + t[1]
                    _slam_pose["theta"] += dtheta
                else:
                    # Sadece öteleme uygula, dönüşü atla — R≈I varsayımıyla
                    _slam_pose["x"] = old_x + t[0]
                    _slam_pose["y"] = old_y + t[1]

        # 3) DÜZELTİLMİŞ poz ile taramayı yeniden dünya çerçevesine taşı, haritaya ekle
        cos_p, sin_p = math.cos(_slam_pose["theta"]), math.sin(_slam_pose["theta"])
        pitch_rad = math.radians(pitch_deg)
        roll_rad  = math.radians(roll_deg)
        base_h    = -(z_mm) / 1000.0   # metre

        for lx, ly in scan_xy:
            wx = lx * cos_p - ly * sin_p + _slam_pose["x"]
            wy = lx * sin_p + ly * cos_p + _slam_pose["y"]
            height = base_h + lx * math.sin(roll_rad) + ly * math.sin(pitch_rad)
            key = (round(wx / VOXEL_SIZE), round(wy / VOXEL_SIZE))
            if key in _voxel_map:
                ox, oy, oh = _voxel_map[key]
                wx     = ox * (1 - MAP_BLEND_ALPHA) + wx * MAP_BLEND_ALPHA
                wy     = oy * (1 - MAP_BLEND_ALPHA) + wy * MAP_BLEND_ALPHA
                height = oh * (1 - MAP_BLEND_ALPHA) + height * MAP_BLEND_ALPHA
            _voxel_map[key] = (wx, wy, height)

        if len(_voxel_map) > MAX_MAP_POINTS:
            excess = len(_voxel_map) - MAX_MAP_POINTS
            for k in list(_voxel_map.keys())[:excess]:
                del _voxel_map[k]

def _slam_worker():
    """SLAM (ICP) işlemesini LiDAR okuma thread'inden ayrı yapar — büyüyen
    haritada ICP yavaşladığında seri port okumasını bloke etmemesi için."""
    while True:
        pts = _slam_queue.get()  # yeni veri gelene kadar bloke olur (CPU harcamaz)
        try:
            _slam_update(pts)
        except Exception as slam_err:
            log.error(f"[SLAM] Hata: {slam_err}")

def _lidar_worker():
    global _lidar_points, _lidar_online, _lidar_last_scan_ts, _lidar_scan_seq, _lidar_last_error
    # A1M8 donanımı saniyede ~8000 örnek alabiliyor (~1450 nokta/devir) ama
    # 'normal' modda kütüphane bunun ÇOK küçük bir kısmını kullanıyor (gerçek
    # donanımda ölçüldü: ~130-140 nokta/tur). 'express' modu aynı seri bağlantı
    # üzerinden ~2x örnekleme hızı kullanır (Slamtec dokümanları + kütüphane
    # kaynağı doğrulandı). Bazı eski firmware'lerde desteklenmeyebilir — ilk
    # denemede hata verirse otomatik 'normal'a düşülür, elle uğraşmana gerek yok.
    # GERİ ALINDI: 'express' modu ~2x sürekli USB veri trafiği üretiyordu —
    # bu, LiDAR çalıştığı HER AN aktif (kullanıcı hiçbir şey yapmasa bile).
    # USB hattının zaten sınırda olduğu gerçek kernel kanıtıyla (over-current
    # olayı) doğrulanmıştı — üstüne sürekli %100 fazla veri bindirmek somut,
    # tanımlanabilir bir ek yük. Nokta yoğunluğundan ödün veriliyor ama
    # kararlılık önce gelir.
    scan_type = "normal"
    lidar = None  # ÖNEMLİ: artık fonksiyon kapsamında kalıcı — manuel
                  # duraklatmada disconnect EDİLMİYOR (aşağıda neden açıklanıyor).
    motor_stopped = False
    while True:
        if not _lidar_enabled:
            if lidar is None:
                # A1 motoru DTR ile sürülür. USB aygıtı açılışta kendi başına
                # dönebileceği için portu bir kez açıp DTR'yi STOP durumunda
                # tutmak gerekir; bağlantıyı kapatmak motoru yeniden başlatabilir.
                try:
                    lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
                    lidar.stop()
                    lidar.stop_motor()
                    lidar.clean_input()
                    _lidar_last_error = ""
                    motor_stopped = True
                    log.info("[LIDAR] Hazır, motor başlangıçta KAPALI")
                except Exception as e:
                    _lidar_last_error = str(e) or type(e).__name__
                    log.warning(f"[LIDAR] Başlangıçta motoru kapatma denemesi başarısız: {e}")
                    if lidar is not None:
                        try:
                            lidar.disconnect()
                        except Exception:
                            pass
                        lidar = None
            if lidar is not None and not motor_stopped:
                # Motoru durdur ama BAĞLANTIYI KAPATMA. RPLidar A1 serisinde
                # motor, seri DTR hattıyla kontrol edilir (komut protokolü
                # değil) — ve bilinen bir donanım/sürücü davranışı: portu
                # kapatmak (disconnect/close) DTR hattını sıfırlayıp motoru
                # KENDİLİĞİNDEN YENİDEN BAŞLATABİLİYOR. Bu yüzden "durdur"
                # dediğinde durmuyordu — stop_motor() çalışıyordu ama hemen
                # sonra disconnect() motoru geri tetikliyordu. Bağlantıyı
                # açık tutarak DTR durumu korunuyor, motor gerçekten kapalı kalıyor.
                try:
                    lidar.stop()
                    lidar.stop_motor()
                except Exception as e:
                    log.warning(f"[LIDAR] Motor durdurma hatası: {e}")
                motor_stopped = True
            _lidar_online = False
            time.sleep(0.5)
            continue
        try:
            motor_stopped = False
            if lidar is None:
                lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
                # Önceki sunucu süreci sıcak kapandıysa A1M8 seri tamponunda
                # yarım kalmış tarama baytları bulunabilir. Doğrudan get_info /
                # iter_scans çağrısı bu durumda descriptor beklerken takılır.
                # Cihazı kapatmadan protokolü idle'a alıp tamponu temizle; sonra
                # iter_scans motoru ve taramayı temiz bir oturumda başlatsın.
                try:
                    lidar.stop()
                    lidar.stop_motor()
                    lidar.clean_input()
                except Exception as reset_err:
                    log.warning(f"[LIDAR] Başlangıç temizliği uyarısı: {reset_err}")
                lidar.get_info()  # bağlantıyı doğrula
                _lidar_last_error = ""
                log.info(f"[LIDAR] Bağlandı: {LIDAR_PORT} (mod: {scan_type})")
            else:
                # Bağlantı zaten açık (manuel kapatmadan sonra tekrar açıldı)
                # — motoru yeniden başlat, taramaya devam et.
                lidar.start_motor()
                log.info("[LIDAR] Motor manuel olarak yeniden başlatıldı")
            _lidar_online = True
            for scan in lidar.iter_scans(scan_type=scan_type):
                if not _lidar_enabled:
                    # Taranırken manuel kapatıldı — motoru durdurup döngüden çık
                    # (bağlantı YİNE kapatılmıyor, üstteki NOT'a bak).
                    log.info("[LIDAR] Manuel olarak kapatıldı, motor durduruluyor")
                    _lidar_online = False
                    break
                pts = [(angle, dist) for (_quality, angle, dist) in scan]
                with _lidar_lock:
                    _lidar_points = pts
                    _lidar_last_scan_ts = time.monotonic()
                    _lidar_scan_seq += 1
                if SLAM_AVAILABLE and SLAM_MAPPING_ENABLED:
                    try:
                        _slam_queue.put_nowait(pts)
                    except queue.Full:
                        # SLAM henüz öncekini işlemedi — eskiyi at, en taze
                        # taramayla değiştir (backlog büyüyüp gecikme yaratmasın).
                        try:
                            _slam_queue.get_nowait()
                        except queue.Empty:
                            pass
                        try:
                            _slam_queue.put_nowait(pts)
                        except queue.Full:
                            pass
        except Exception as e:
            _lidar_online = False
            _lidar_last_error = str(e) or type(e).__name__
            log.error(f"[LIDAR] Hata: {e} — 3 saniye sonra yeniden denenecek")
            if scan_type == "express":
                # 'express' bu donanım/firmware ile uyumsuz olabilir —
                # kalıcı olarak 'normal'a düş, böylece sonsuz döngüde
                # aynı hatayı tekrar tekrar almak yerine bir kerede düzelir.
                log.warning("[LIDAR] 'express' modu başarısız oldu — 'normal' moda düşülüyor")
                scan_type = "normal"
            # Gerçek bir HATA durumunda (manuel kapatma değil) bağlantıyı
            # tamamen kapatıp baştan kurmak güvenli — DTR sorunu sadece
            # "iyi çalışırken kasıtlı kapatma" senaryosunda riskli.
            if lidar is not None:
                try:
                    lidar.stop()
                    lidar.stop_motor()
                    lidar.disconnect()
                except Exception:
                    pass
                lidar = None
            time.sleep(3)

def _get_lidar_distance_at_angle(pts, center_angle, window_deg):
    """Belirli bir açı etrafında (±window_deg) EN YAKIN mesafeyi döner (metre)
    — min kullanılır, en kötü/en tehlikeli durumu yakalamak için. İzole
    test edildi (sentetik koridor taramalarıyla, 5 senaryo, hepsi doğru)."""
    candidates = []
    for angle, dist in pts:
        if dist <= 0:
            continue
        diff = abs(((angle - center_angle + 180) % 360) - 180)
        if diff <= window_deg:
            candidates.append(dist)
    if not candidates:
        return None
    return min(candidates) / 1000.0  # mm -> metre

def _lidar_nav_decide(pts):
    """Saf karar fonksiyonu — (fwd, turn) döner. +turn=SAĞA (joyR ile AYNI,
    AI TAKİP özelliğinde zaten doğrulanmış kural). İzole test edilip
    sentetik koridor senaryolarıyla doğrulandı, ondan SONRA buraya taşındı."""
    front = _get_lidar_distance_at_angle(pts, LIDAR_FORWARD_ANGLE_DEG, 20)
    left  = _get_lidar_distance_at_angle(pts, LIDAR_LEFT_ANGLE_DEG, 25)
    right = _get_lidar_distance_at_angle(pts, LIDAR_RIGHT_ANGLE_DEG, 25)

    if front is None:
        return 0.0, 0.0

    if front is not None and front < NAV_WALL_TURN_DIST_M:
        if left is not None and right is not None and min(left, right) < NAV_SIDE_DANGER_DIST_M:
            return 0.0, 0.0  # köşeye sıkışmış — tam dur, ileri/dönüş yapma
        if left is not None and right is not None:
            turn = -NAV_TURN_SPEED if left > right else NAV_TURN_SPEED
            return 0.0, turn
        return 0.0, -NAV_TURN_SPEED  # yan bilgi yok, varsayılan yöne dön

    if left is not None and right is not None:
        diff = right - left  # sağ duvara yakınsa (right küçük) negatif -> SOLA dön
        turn = max(-NAV_MAX_CENTERING_TURN, min(NAV_MAX_CENTERING_TURN, diff * NAV_CENTERING_GAIN))
        return NAV_FORWARD_SPEED, turn

    return NAV_FORWARD_SPEED, 0.0  # yan bilgi yok, düz ileri

def _lidar_nav_worker():
    """LiDAR FPS'inden bağımsız, sabit hızda çalışır. _lidar_nav_enabled
    False olduğu sürece (varsayılan) HİÇBİR hareket komutu göndermez."""
    global _lidar_nav_enabled
    consecutive_failures = 0
    last_error_log = 0.0
    with httpx.Client(timeout=0.6) as client:
        while True:
            time.sleep(1.0 / NAV_COMMAND_HZ)
            if not _lidar_nav_enabled:
                continue
            with _lidar_lock:
                pts = list(_lidar_points)
                fresh = _lidar_online and time.monotonic() - _lidar_last_scan_ts < 0.6
            fwd, turn = _lidar_nav_decide(pts) if pts and fresh else (0.0, 0.0)
            try:
                response = client.get(f"http://localhost:8080/joy?x=0&y={fwd:.2f}&r={turn:.2f}&source=lidar")
                response.raise_for_status()
                consecutive_failures = 0
            except Exception as e:
                if not _lidar_nav_enabled:
                    continue  # An explicit mode change already stopped this source.
                # Tek bir WiFi/HTTP gecikmesi otonomiyi kalıcı kapatmasın.
                # Firmware deadman cevapsızlıkta bağımsız olarak robotu durdurur;
                # döngü burada açık kalıp bağlantı gelince kendini toparlar.
                consecutive_failures += 1
                now = time.monotonic()
                if now - last_error_log >= 2.0:
                    log.warning(f"[LIDAR-NAV] Komut gecikti ({consecutive_failures}); yeniden deneniyor: {e}")
                    last_error_log = now

def start_lidar_thread():
    global _lidar_thread_started
    if not RPLIDAR_AVAILABLE:
        log.warning("[LIDAR] 'rplidar' kütüphanesi kurulu değil — LiDAR devre dışı")
        return
    if not SLAM_AVAILABLE:
        log.warning("[SLAM] 'numpy'/'scipy' kurulu değil — sadece ham (egocentric) LiDAR verisi yayınlanacak")
    if _lidar_thread_started:
        return
    t = threading.Thread(target=_lidar_worker, daemon=True)
    t.start()
    if SLAM_AVAILABLE and SLAM_MAPPING_ENABLED:
        s = threading.Thread(target=_slam_worker, daemon=True)
        s.start()
    n = threading.Thread(target=_lidar_nav_worker, daemon=True)
    n.start()
    _lidar_thread_started = True
    log.info(f"[LIDAR] Arka plan thread başladı — port: {LIDAR_PORT}")


def _mapping_worker():
    """Feed mapping_v2 from shared sensor snapshots; never contact hardware."""
    last_sequence = -1
    while True:
        with _lidar_lock:
            points = list(_lidar_points)
            sequence = int(_lidar_scan_seq)
            scan_timestamp_ns = int(_lidar_last_scan_ts * 1_000_000_000)
            online = bool(_lidar_online)
        if _mapping_engine is not None and online and points and sequence != last_sequence:
            with _telemetry_cache_lock:
                telemetry = dict(_telemetry_cache)
            now_ns = time.monotonic_ns()
            _mapping_engine.ingest_imu(ImuSample(
                timestamp_ns=now_ns,
                roll_deg=float(telemetry.get("mpuR") or 0.0),
                pitch_deg=float(telemetry.get("mpuP") or 0.0),
                yaw_deg=float(telemetry.get("mpuY") or 0.0),
                valid=bool(telemetry.get("mpuOnline", False)),
            ))
            tof_mm = telemetry.get("tofMm")
            _mapping_engine.ingest_lidar(LidarScan(
                timestamp_ns=scan_timestamp_ns or now_ns,
                sequence=sequence,
                points=points,
            ))
            # LiDAR scan matching first updates the current map pose. Fuse the
            # forward ToF return afterwards so it lands in that same pose,
            # rather than the previous scan's frame.
            _mapping_engine.ingest_tof(TofSample(
                timestamp_ns=now_ns,
                distance_mm=float(tof_mm) if isinstance(tof_mm, (int, float)) else 0.0,
                valid=bool(telemetry.get("tofOnline", False)),
            ))
            last_sequence = sequence
        # A1M8 normal modda yaklaşık 5 tam tur/s üretir. 20 Hz salt-okunur
        # kontrol, yeni turu ortalama 25 ms içinde yakalar; aynı sequence ağır
        # occupancy hesabına ikinci kez girmez.
        time.sleep(1.0 / MAPPING_SENSOR_POLL_HZ)


def start_mapping_thread():
    global _mapping_thread_started
    if _mapping_engine is None or _mapping_thread_started:
        return
    threading.Thread(target=_mapping_worker, daemon=True, name="mapping-v2").start()
    _mapping_thread_started = True
    log.info("[MAPPING-V2] Salt-okunur LOCAL_ONLY haritalama başladı; hareket yürütme kilitli")


async def _dummy_lidar_loop():
    """Generate a deterministic room scan when no physical LiDAR is present."""
    global _lidar_points, _lidar_online, _lidar_last_scan_ts, _lidar_scan_seq
    while True:
        now = time.monotonic()
        pts: list[tuple[float, float]] = []
        for angle in range(0, 360, 2):
            rad = math.radians(angle)
            wall = 2400.0 / max(0.35, max(abs(math.cos(rad)), abs(math.sin(rad))))
            obstacle_center = 8.0 * math.sin(now * 0.35)
            delta = ((angle - obstacle_center + 180.0) % 360.0) - 180.0
            distance = 700.0 + 60.0 * math.sin(now) if abs(delta) < 9.0 else wall
            pts.append((float(angle), float(distance)))
        with _lidar_lock:
            _lidar_points = pts
            _lidar_online = _lidar_enabled
            _lidar_last_scan_ts = time.monotonic()
            _lidar_scan_seq += 1
        await asyncio.sleep(0.1)


# ─── Telemetri loop ──────────────────────────────────────────────────────────

async def telemetry_loop():
    """ESP32'den /telemetry'yi poll eder; tüm WS istemcilerine broadcast eder."""
    global esp_online
    interval = 1.0 / TELEMETRY_HZ

    async with httpx.AsyncClient(timeout=HTTP_TIMEOUT) as client:
        while True:
            try:
                r = await client.get(f"{ESP32_URL}/telemetry")
                r.raise_for_status()
                data = r.json()

                if not esp_online:
                    log.info("[ESP32] Online")
                    esp_online = True
                    await mgr.broadcast({"type": "status", "esp32": True})

                await mgr.broadcast({"type": "telemetry", "data": data})

            except httpx.RequestError:
                if esp_online:
                    log.warning("[ESP32] Offline — bağlantı yok")
                    esp_online = False
                await mgr.broadcast({"type": "status", "esp32": False})

            except Exception as e:
                log.error(f"[Telemetry] Poll hatası: {e}")

            await asyncio.sleep(interval)


# ─── Lifespan ────────────────────────────────────────────────────────────────

@asynccontextmanager
async def lifespan(app: FastAPI):
    # NOT: apex_ui.html şu an /telemetry'yi DOĞRUDAN REST proxy üzerinden
    # çekiyor (WebSocket kullanmıyor). Bu arka plan döngüsü WS yayını için
    # yazılmıştı; aynı anda ikisi de ESP32'yi yoklarsa ESP32'nin tek-thread'li
    # WebServer'ı tıkanıp 500/boş cevap veriyor. React SPA + WS akışına
    # geçilince aşağıdaki satırı tekrar aç.
    # task = asyncio.create_task(telemetry_loop())
    log.info(f"[APEX] Sunucu başladı — ESP32: {ESP32_URL} (arka plan telemetri döngüsü KAPALI)")

    async def _delayed_lidar_start():
        # ANLIK GÜÇ YÜKÜNÜ YAYMAK İÇİN KASITLI GECİKME: LiDAR motoru
        # sunucu açılır açılmaz ANINDA dönmeye başlamasın — ESP32 WiFi
        # bağlantısı/diğer ilk açılış olayları biraz stabilize olsun.
        # HTTP sunucusunu BLOKLAMIYOR (launcher'ın hazır-mı kontrolü hemen
        # başarılı olur, motor arka planda biraz sonra başlar).
        await asyncio.sleep(3)
        start_lidar_thread()

    background_tasks: list[asyncio.Task] = []
    start_mapping_thread()
    if SIM_MODE:
        log.info("[APEX] SIM_MODE aktif — fiziksel LiDAR yerine dummy scan kullanılıyor")
        background_tasks.append(asyncio.create_task(_dummy_lidar_loop()))
    else:
        background_tasks.append(asyncio.create_task(_delayed_lidar_start()))
    try:
        yield
    finally:
        for task in background_tasks:
            task.cancel()
        await asyncio.gather(*background_tasks, return_exceptions=True)
        log.info("[APEX] Sunucu kapatılıyor")


# ─── FastAPI app ─────────────────────────────────────────────────────────────

app = FastAPI(lifespan=lifespan, title="APEX Control Server", version="1.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


def _password_matches(password: str) -> bool:
    """Validate PBKDF2-SHA256 encoded as iterations$salt_hex$digest_hex."""
    try:
        iterations_text, salt_hex, expected_hex = AUTH_PASSWORD_HASH.split("$", 2)
        digest = hashlib.pbkdf2_hmac("sha256", password.encode(), bytes.fromhex(salt_hex), int(iterations_text))
        return hmac.compare_digest(digest.hex(), expected_hex)
    except (ValueError, TypeError):
        return False


def _session_value(user: str, expires: int) -> str:
    payload = f"{user}|{expires}"
    signature = hmac.new(AUTH_SECRET.encode(), payload.encode(), hashlib.sha256).hexdigest()
    return f"{payload}|{signature}"


def _session_valid(value: str | None) -> bool:
    if not AUTH_ENABLED:
        return True
    if not value or not AUTH_SECRET:
        return False
    try:
        user, expires_text, signature = value.rsplit("|", 2)
        expires = int(expires_text)
    except (ValueError, TypeError):
        return False
    expected = _session_value(user, expires).rsplit("|", 1)[1]
    return user == AUTH_USER and expires >= int(time.time()) and hmac.compare_digest(signature, expected)


LOGIN_HTML = """<!doctype html><html lang=\"tr\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><title>APEX GİRİŞ</title><style>body{margin:0;min-height:100vh;display:grid;place-items:center;background:#050505;color:#ddd;font-family:monospace}.box{width:min(360px,calc(100vw - 42px));border:1px solid #333;border-top:3px solid #e60000;padding:26px;background:#101010;box-shadow:0 20px 60px #000}h1{margin:0 0 22px;color:#e60000;letter-spacing:3px;font-size:21px}label{display:block;margin:12px 0 5px;color:#00ffd0}input,button{box-sizing:border-box;width:100%;padding:12px;background:#050505;color:#fff;border:1px solid #555;font:inherit}button{margin-top:20px;border-color:#e60000;background:#e60000;font-weight:bold;cursor:pointer}.err{color:#ff9b00;min-height:18px}</style></head><body><form class=\"box\" method=\"post\" action=\"/login\"><h1>APEX ACCESS</h1><div class=\"err\">{error}</div><label>KULLANICI ADI</label><input name=\"username\" autocomplete=\"username\" required autofocus><label>ŞİFRE</label><input type=\"password\" name=\"password\" autocomplete=\"current-password\" required><button type=\"submit\">SİSTEME GİR</button></form></body></html>"""


@app.middleware("http")
async def require_login(request: Request, call_next):
    public = {"/login", "/healthz", "/generate_204", "/hotspot-detect.html", "/connecttest.txt", "/ncsi.txt"}
    # Kamera/Hailo ve diğer yerel yardımcılar bu sunucuyla loopback üzerinden
    # konuşur. Dış istemci oturumu zorunlu kalırken bu iç trafik giriş sayfasına
    # yönlendirilmemeli; aksi halde HUD telemetrisi ve AI takip sessizce kopar.
    client_host = request.client.host if request.client else ""
    if not AUTH_ENABLED or request.url.path in public or client_host in {"127.0.0.1", "::1"}:
        return await call_next(request)
    if _session_valid(request.cookies.get(AUTH_COOKIE)):
        return await call_next(request)
    return RedirectResponse("/login", status_code=303)


@app.get("/login", response_class=HTMLResponse)
async def login_page():
    if not AUTH_ENABLED:
        return RedirectResponse("/", status_code=303)
    return HTMLResponse(LOGIN_HTML.replace("{error}", ""))


@app.post("/login")
async def login_submit(request: Request):
    fields = parse_qs((await request.body()).decode("utf-8", "replace"), keep_blank_values=True)
    username = fields.get("username", [""])[0]
    password = fields.get("password", [""])[0]
    if not (hmac.compare_digest(username, AUTH_USER) and _password_matches(password)):
        await asyncio.sleep(0.35)
        return HTMLResponse(LOGIN_HTML.replace("{error}", escape("Kullanıcı adı veya şifre hatalı.")), status_code=401)
    response = RedirectResponse("/", status_code=303)
    response.set_cookie(AUTH_COOKIE, _session_value(username, int(time.time()) + AUTH_MAX_AGE_S),
                        max_age=AUTH_MAX_AGE_S, httponly=True, samesite="strict", secure=False)
    return response


@app.get("/logout")
async def logout():
    response = RedirectResponse("/login", status_code=303)
    response.delete_cookie(AUTH_COOKIE)
    return response


@app.get("/healthz")
async def healthz():
    return JSONResponse({"ok": True})


@app.get("/generate_204", status_code=204)
async def android_connectivity_check():
    return Response(status_code=204)


@app.get("/hotspot-detect.html")
@app.get("/connecttest.txt")
@app.get("/ncsi.txt")
async def captive_portal_hint():
    return RedirectResponse("/login", status_code=302)


async def _require_ws_login(ws: WebSocket) -> bool:
    if _session_valid(ws.cookies.get(AUTH_COOKIE)):
        return True
    await ws.close(code=4401, reason="Login required")
    return False

# ─── Komut tablosu ───────────────────────────────────────────────────────────
# format: "cmd": ("/esp32/endpoint", [forwarded_param_keys])
# Yüksek frekanslı "joy" komutu özel: ack gönderilmez (bandwidth tasarrufu)

CMD_MAP: dict[str, tuple[str, list[str]]] = {
    "joy":             ("/joy",             ["x", "y", "r"]),
    "bodyIk":          ("/bodyIk",          ["p", "r", "y"]),
    "mode":            ("/mode",            ["m"]),
    "params":          ("/params",          ["z", "len", "lift", "spd"]),
    "setGimbal":       ("/setGimbal",       ["state"]),
    "setGimbalFilter": ("/setGimbalFilter", ["v"]),
    "audio":           ("/audio",           ["cmd", "t", "v"]),
    "manual":          ("/manual",          ["leg", "j", "v"]),
    "homePreview":     ("/homePreview",     ["leg", "j", "v"]),
    "goHome":          ("/goHome",          ["confirm"]),
}

SILENT_CMDS = {"joy"}  # Bu komutlar için ack gönderilmez

# ─── WebSocket endpoint ───────────────────────────────────────────────────────

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    if not await _require_ws_login(ws):
        return
    await mgr.connect(ws)
    # Bağlantıda mevcut durumu gönder
    await ws.send_json({"type": "status", "esp32": esp_online})

    async with httpx.AsyncClient(timeout=1.0) as client:
        try:
            while True:
                raw = await ws.receive_text()

                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    await ws.send_json({"type": "error", "msg": "Geçersiz JSON"})
                    continue

                cmd = msg.get("cmd")
                if cmd not in CMD_MAP:
                    await ws.send_json({
                        "type": "error",
                        "msg": f"Bilinmeyen komut: {cmd}",
                    })
                    continue

                path, param_keys = CMD_MAP[cmd]
                params = {k: msg[k] for k in param_keys if k in msg}

                try:
                    r = await _operator_proxy(path, params)
                    if cmd not in SILENT_CMDS:
                        await ws.send_json({
                            "type": "ack",
                            "cmd":  cmd,
                            "ok":   r.status_code == 200,
                        })
                except httpx.RequestError:
                    await ws.send_json({
                        "type": "error",
                        "cmd":  cmd,
                        "msg":  "ESP32 erişilemiyor",
                    })
                except Exception as e:
                    await ws.send_json({"type": "error", "cmd": cmd, "msg": str(e)})

        except WebSocketDisconnect:
            mgr.disconnect(ws)

# ─── LiDAR motoru manuel aç/kapa — OTO modda olmasan da kullanılabilir ───────

async def _set_lidar_motor(enabled: bool) -> dict:
    global _lidar_enabled
    async with _lidar_motor_command_lock:
        _lidar_enabled = bool(enabled)
        log.info(f"[LIDAR] Motor manuel olarak {'AÇILDI' if _lidar_enabled else 'KAPATILDI'}")

        # En fazla 3 saniye tarama thread'inin gerçek duruma geçmesini bekle ve
        # sonucu kanıtıyla dön. Bu yardımcı hem HTTP hem de canlı WS kontrol
        # hattında kullanılır; iki yolun davranışı böylece birebir aynıdır.
        deadline = time.monotonic() + 3.0
        applied = False
        scan_age = None
        while time.monotonic() < deadline:
            with _lidar_lock:
                scan_age = (time.monotonic() - _lidar_last_scan_ts) if _lidar_last_scan_ts else None
            applied = (not _lidar_online) if not _lidar_enabled else (_lidar_online and scan_age is not None and scan_age < 1.0)
            if applied:
                break
            await asyncio.sleep(0.05)

        return {
            "enabled": _lidar_enabled,
            "online": _lidar_online,
            "applied": applied,
            "scan_age_s": round(scan_age, 3) if scan_age is not None else None,
            "scan_seq": _lidar_scan_seq,
            "error": _lidar_last_error or None,
        }


@app.get("/lidar/motor")
async def lidar_motor(state: str | None = None):
    # A parameterless GET is a read-only status request.  The old default of
    # state="1" meant that a health/status probe could unexpectedly start the
    # physical motor.  Require an explicit state for every mutation.
    if state is None:
        with _lidar_lock:
            scan_age = (time.monotonic() - _lidar_last_scan_ts) if _lidar_last_scan_ts else None
        return JSONResponse({
            "enabled": _lidar_enabled,
            "online": _lidar_online,
            "applied": (not _lidar_online) if not _lidar_enabled else (
                _lidar_online and scan_age is not None and scan_age < 1.0
            ),
            "scan_age_s": round(scan_age, 3) if scan_age is not None else None,
            "scan_seq": _lidar_scan_seq,
            "error": _lidar_last_error or None,
        })
    if state not in {"0", "1"}:
        return JSONResponse({"error": "state 0 veya 1 olmalı"}, status_code=400)
    return JSONResponse(await _set_lidar_motor(state == "1"))

# ─── AI TAKİP (kişi takibi) — detection_hud.py bu bayrağı /telemetry üzerinden
# okuyup hareket komutu gönderip göndermeyeceğine karar verir. VARSAYILAN
# KAPALI — kullanıcı bilerek açmadan robot hiç hareket etmez. ─────────────────

@app.get("/ai_tracking")
async def ai_tracking(state: str = "1"):
    global _ai_tracking_enabled, _lidar_nav_enabled
    requested = (state == "1")
    if requested:
        if _navigation_platform_status.get("active"):
            return JSONResponse(
                {"enabled": False, "error": "Rota yürütülürken AI takip başlatılamaz"},
                status_code=409,
            )
        tracking = await _vision_tracking_status()
        if tracking.get("state") != "locked" or not tracking.get("visible"):
            _ai_tracking_enabled = False
            return JSONResponse(
                {"enabled": False, "error": "Önce görüntüde görünen bir kişiyi seçip kilitle"},
                status_code=409,
            )
        if _lidar_nav_enabled:
            # İKİSİ BİRDEN /joy'a komut gönderirse çakışır — karşılıklı dışlayıcı.
            _lidar_nav_enabled = False
            log.info("[AI-TRACK] LiDAR-NAV ile çakışmayı önlemek için LiDAR-NAV kapatıldı")
        # BEZIER_JOY modunda değilse joy komutları IDLE modun decay mantığı
        # tarafından anında sıfırlanır — takip çalışmadan önce doğru moda
        # geçirilmesi GEREKİYOR, yoksa hiçbir hareket görünmez.
        try:
            telemetry = await _motion_start_telemetry()
        except RuntimeError as exc:
            _ai_tracking_enabled = False
            return JSONResponse({"enabled": False, "error": str(exc)}, status_code=409)
        safety_error = _navigation_telemetry_error(telemetry)
        if safety_error:
            _ai_tracking_enabled = False
            return JSONResponse({"enabled": False, "error": safety_error}, status_code=409)
        mode_response = await _proxy_to_esp32("/mode", {"m": "bezier_joy"})
        if mode_response.status_code >= 400:
            _ai_tracking_enabled = False
            return JSONResponse(
                {"enabled": False, "error": "Omurilik çevrimdışı; AI takip güvenle başlatılmadı"},
                status_code=503,
            )
        _ai_tracking_enabled = True
        log.info("[AI-TRACK] Açıldı — mod bezier_joy'a alındı")
    else:
        _ai_tracking_enabled = False
        # Tek sıfır joy WALK modunu kapatmaz ve 750ms sonra yanlış FAILSAFE
        # üretir. Tam stop sekansı sıfır komutlardan sonra IDLE/STAND'e geçer.
        asyncio.create_task(_navigation_send_stop())
        log.info("[AI-TRACK] Kapatıldı — sıfır joy + IDLE duruş sekansı gönderildi")
    return JSONResponse({"enabled": _ai_tracking_enabled})


@app.get("/api/vision/status")
async def vision_status():
    """Read-only state for the dedicated AI Camera Console."""
    with _target_selection_lock:
        selection = dict(_target_selection)
    tracking = await _vision_tracking_status()
    return JSONResponse({
        "ai_tracking": _ai_tracking_enabled,
        "selection": selection,
        "tracking": tracking,
    })


@app.get("/api/vision/control")
async def vision_control():
    """Fast Pi-local control state consumed by the Hailo process.

    This deliberately never contacts the ESP32, so target selection continues
    to work while the spine is powered off or reconnecting.
    """
    with _target_selection_lock:
        selection = dict(_target_selection)
    return JSONResponse({"ai_tracking": _ai_tracking_enabled, "selection": selection})

# ─── HEDEF SEÇİMİ (tıkla-takip-et) ───────────────────────────────────────────
# Kullanıcı arayüzde bir kişiye tıkladığında normalize (0-1) koordinat
# buraya gelir. detection_hud.py /telemetry üzerinden okuyup o noktadaki
# kişinin track_id'sini bulup kilitler — sadece O kişiyi takip eder,
# kareye giren başka insanları görmezden gelir.

@app.get("/select_target")
async def select_target(x: float = 0.0, y: float = 0.0, clear: str = "0"):
    global _target_selection, _ai_tracking_enabled
    with _target_selection_lock:
        if clear == "1":
            _ai_tracking_enabled = False
            _target_selection = {"pending": False, "x": 0.0, "y": 0.0,
                                  "ts": time.time(), "clear": True}
            log.info("[AI-TRACK] Hedef seçimi temizlendi")
        else:
            x = max(0.0, min(1.0, x))
            y = max(0.0, min(1.0, y))
            _target_selection = {"pending": True, "x": x, "y": y,
                                  "ts": time.time(), "clear": False}
            log.info(f"[AI-TRACK] Yeni hedef seçimi: x={x:.3f} y={y:.3f}")
    if clear == "1":
        asyncio.create_task(_navigation_send_stop())
    return JSONResponse({"ok": True})


@app.websocket("/ws/vision")
async def ws_vision_control(ws: WebSocket):
    """Dedicated camera-console control channel.

    MJPEG and long-lived dashboard connections can exhaust a browser's short
    HTTP connection pool. Target/AI commands use this already-open socket so a
    valid click cannot be aborted while waiting for a free HTTP connection.
    Existing HTTP endpoints remain available as a compatibility fallback.
    """
    if not await _require_ws_login(ws):
        return
    await ws.accept()
    try:
        while True:
            message = await ws.receive_json()
            request_id = message.get("id")
            command = message.get("cmd")
            try:
                if command == "select":
                    response = await select_target(
                        x=float(message.get("x", 0.0)),
                        y=float(message.get("y", 0.0)),
                    )
                elif command == "clear":
                    response = await select_target(clear="1")
                elif command == "ai":
                    response = await ai_tracking(state="1" if bool(message.get("state")) else "0")
                else:
                    await ws.send_json({"type": "vision_ack", "id": request_id, "ok": False, "error": "Bilinmeyen kamera komutu"})
                    continue

                payload = json.loads(response.body.decode("utf-8"))
                await ws.send_json({
                    "type": "vision_ack",
                    "id": request_id,
                    "ok": response.status_code < 400,
                    "status": response.status_code,
                    **payload,
                })
            except Exception as command_err:
                await ws.send_json({"type": "vision_ack", "id": request_id, "ok": False, "error": str(command_err) or type(command_err).__name__})
    except WebSocketDisconnect:
        pass

# ─── LiDAR OTONOM YÜRÜYÜŞ aç/kapa ─────────────────────────────────────────────

@app.get("/lidar_nav")
async def lidar_nav(state: str = "1"):
    global _lidar_nav_enabled, _ai_tracking_enabled
    if state == "1":
        if _navigation_platform_status.get("active"):
            return JSONResponse({"error": "Rota testi çalışıyor; önce durdurun"}, status_code=409)
        try:
            telemetry = await _motion_start_telemetry()
        except RuntimeError as exc:
            return JSONResponse({"error": str(exc), "enabled": False}, status_code=409)
        safety_error = _navigation_telemetry_error(telemetry)
        if safety_error:
            return JSONResponse({"error": safety_error, "enabled": False}, status_code=409)
        motor = await _set_lidar_motor(True)
        if not motor.get("applied"):
            return JSONResponse({
                "error": motor.get("error") or "LiDAR motoru/taraması başlatılamadı",
                "enabled": False,
            }, status_code=409)
        with _lidar_lock:
            fresh = _lidar_online and bool(_lidar_points) and time.monotonic() - _lidar_last_scan_ts < 0.6
        if not fresh:
            return JSONResponse({"error": "Güncel LiDAR taraması yok", "enabled": False}, status_code=409)
        response = await _proxy_to_esp32("/mode", {"m": "bezier_joy"})
        if response.status_code >= 400:
            return response
        if _ai_tracking_enabled:
            _ai_tracking_enabled = False
            log.info("[LIDAR-NAV] AI-TAKİP ile çakışmayı önlemek için AI-TAKİP kapatıldı")
        _lidar_nav_enabled = True
        log.info("[LIDAR-NAV] Açıldı — mod bezier_joy'a alındı")
    else:
        _lidar_nav_enabled = False
        await _navigation_send_stop()
        await _set_lidar_motor(False)
        log.info("[LIDAR-NAV] Kapatıldı — IDLE duruş sekansı ve LiDAR motor kapatma gönderildi")
    return JSONResponse({"enabled": _lidar_nav_enabled})


# ─── ROBOT KOL — güvenli kalibrasyon ve manuel/IK kontrol ─────────────────

@app.get("/arm/status")
async def arm_status():
    with _arm_last_result_lock:
        last = dict(_arm_last_result)
    calib = _load_arm_calibration()
    return JSONResponse({
        "driver": ARM_DRIVER_MODE,
        "pose_endpoint": ARM_POSE_ENDPOINT,
        "servo_endpoint": ARM_SERVO_ENDPOINT,
        "geometry": {"base_h": ARM_BASE_H_MM, "l1": ARM_L1_MM, "l2": ARM_L2_MM},
        "calibration_file": ARM_CALIB_FILE,
        "calibration": calib,
        "last": last,
    })

@app.get("/arm/calibration")
async def arm_calibration_get():
    return JSONResponse(_load_arm_calibration())

@app.get("/arm/calibration/set")
async def arm_calibration_set(
    gripper_min: Optional[float] = None,
    gripper_max: Optional[float] = None,
    gripper_open: Optional[float] = None,
    gripper_close: Optional[float] = None,
    gripper_current: Optional[float] = None,
):
    """UI'dan gripper MAX/kapalı açısını güvenli kaydet.

    Rack-and-pinion mekanik: 90° açık sabit, kapatma 90° -> 180° yönünde.
    Bu yüzden gripper_min ve gripper_open gelse bile 90'a sabitlenir;
    kapalı değer her zaman MAX değeridir.
    """
    calib = _load_arm_calibration()
    requested_hi = gripper_max if gripper_max is not None else gripper_close
    if requested_hi is None:
        requested_hi = calib["limits"]["gripper"].get("max", 120.0)
    hi = _clamp(float(requested_hi), 90.0, ARM_SERVO_MAX_DEG)
    calib["limits"]["gripper"] = {"min": 90.0, "max": round(hi, 2)}
    calib["gripper"]["open"] = 90.0
    calib["gripper"]["close"] = round(hi, 2)
    if gripper_current is not None:
        calib["gripper"]["current"] = round(_clamp(float(gripper_current), 90.0, hi), 2)
    else:
        calib["gripper"]["current"] = round(_clamp(float(calib["gripper"].get("current", 90.0)), 90.0, hi), 2)
    _save_arm_calibration(calib)
    return JSONResponse({"ok": True, "calibration": _load_arm_calibration()})

@app.get("/arm/calibrate_pose")
async def arm_calibrate_pose():
    """Horn takma pozu: bütün kol servoları elektriksel 90° konumuna gider."""
    calib = _load_arm_calibration()
    payload = dict(calib.get("calibration_pose", DEFAULT_ARM_CALIBRATION["calibration_pose"]))
    res = await _arm_send_payload(payload, "calibration/90")
    _arm_set_last(bool(res.get("ok")), "calibration_pose", "Kol 90° kalibrasyon pozuna gönderildi", {"result": res})
    return JSONResponse({"ok": bool(res.get("ok")), "result": res, "payload": payload}, status_code=200 if res.get("ok") else 502)

@app.get("/arm/home")
async def arm_home():
    """Mekanik bekleme pozu: base 90, joint1 gövde üstüne katlı, joint2 bekleme/katlı."""
    calib = _load_arm_calibration()
    payload = dict(calib.get("home_pose", DEFAULT_ARM_CALIBRATION["home_pose"]))
    payload["gripper"] = float(calib.get("gripper", {}).get("current", payload.get("gripper", 90.0)))
    res = await _arm_send_payload(payload, "home/standby")
    _arm_set_last(bool(res.get("ok")), "home", "Kol bekleme pozuna gönderildi", {"result": res})
    return JSONResponse({"ok": bool(res.get("ok")), "result": res, "payload": payload}, status_code=200 if res.get("ok") else 502)

@app.get("/arm/off")
async def arm_off():
    """Sadece robot kol için ayrılmış PCA kanallarında PWM'i keser; bacak/joy/LiDAR tarafına dokunmaz."""
    if ARM_DRIVER_MODE in {"dry", "dryrun", "sim", "simulation"}:
        _arm_set_last(True, "off", "Kol PWM DRYRUN kapatıldı")
        return JSONResponse({"ok": True, "dryrun": True})
    try:
        async with esp32_lock:
            async with httpx.AsyncClient(timeout=ARM_COMMAND_TIMEOUT_S) as client:
                r = await client.get(ARM_OFF_ENDPOINT)
        ok = 200 <= r.status_code < 300
        res = {"ok": ok, "status": r.status_code, "body": r.text[:160]}
        _arm_set_last(ok, "off", "Kol PWM kesildi" if ok else "Kol PWM kesilemedi", {"result": res})
        return JSONResponse(res, status_code=200 if ok else 502)
    except Exception as e:
        res = {"ok": False, "error": str(e)}
        _arm_set_last(False, "off_error", "Kol PWM kesme bağlantı hatası", {"result": res})
        return JSONResponse(res, status_code=502)

@app.get("/arm/servo")
async def arm_servo(joint: str, angle: float):
    """Kalibrasyon için tek servo hareketi. Sadece base/joint1/joint2/gripper kabul eder."""
    joint = joint.strip().lower()
    if joint not in ARM_JOINTS:
        return JSONResponse({"ok": False, "error": f"Bilinmeyen kol servosu: {joint}", "allowed": ARM_JOINTS}, status_code=400)
    payload = {joint: angle}
    res = await _arm_send_payload(payload, f"manual/{joint}")
    if joint == "gripper" and res.get("ok"):
        calib = _load_arm_calibration()
        calib["gripper"]["current"] = _arm_clamp_joint("gripper", angle)
        _save_arm_calibration(calib)
    _arm_set_last(bool(res.get("ok")), "manual_servo", f"{joint} → {angle:.1f}°", {"result": res})
    return JSONResponse({"ok": bool(res.get("ok")), "result": res}, status_code=200 if res.get("ok") else 502)

@app.get("/arm/pose")
async def arm_pose(
    base: Optional[float] = None,
    joint1: Optional[float] = None,
    joint2: Optional[float] = None,
    gripper: Optional[float] = None,
):
    """Kalibrasyon için hazır servo dereceleriyle 4 kanal poz gönderir."""
    payload = {"base": base, "joint1": joint1, "joint2": joint2, "gripper": gripper}
    payload = {k: v for k, v in payload.items() if v is not None}
    res = await _arm_send_payload(payload, "manual/pose")
    _arm_set_last(bool(res.get("ok")), "manual_pose", "Kol manuel poz gönderildi", {"result": res})
    return JSONResponse({"ok": bool(res.get("ok")), "result": res}, status_code=200 if res.get("ok") else 502)

@app.get("/arm/gripper")
async def arm_gripper(angle: float):
    """Gripper için ayrı güvenli kısa yol: UI slider burayı kullanır."""
    return await arm_servo("gripper", angle)

@app.get("/arm/ik")
async def arm_ik(x: float, y: float, z: float):
    target = {"x": x, "y": y, "z": z}
    joints = _arm_ik_xyz(target)
    calib = _load_arm_calibration()
    payload = _arm_pose_to_payload(joints, float(calib.get("gripper", {}).get("open", 90.0)))
    return JSONResponse({"target": target, "joints_math_deg": joints, "servo_payload_deg": payload, "calibration": calib})

# ─── LiDAR WebSocket — en son taramayı periyodik olarak tarayıcıya yayınlar ──

@app.websocket("/ws/lidar")
async def ws_lidar(ws: WebSocket):
    if not await _require_ws_login(ws):
        return
    await ws.accept()
    interval = 1.0 / LIDAR_BROADCAST_HZ
    receive_task = asyncio.create_task(ws.receive_text())
    try:
        while True:
            done, _ = await asyncio.wait({receive_task}, timeout=interval)
            if receive_task in done:
                raw = receive_task.result()
                receive_task = asyncio.create_task(ws.receive_text())
                try:
                    command = json.loads(raw)
                except json.JSONDecodeError:
                    command = {}
                if command.get("cmd") == "motor" and command.get("state") in (0, 1, False, True):
                    result = await _set_lidar_motor(bool(command["state"]))
                    await ws.send_json({"type": "lidar_motor_ack", **result})

            with _lidar_lock:
                pts = list(_lidar_points)
                scan_age = (time.monotonic() - _lidar_last_scan_ts) if _lidar_last_scan_ts else None
            with _slam_lock:
                pose     = dict(_slam_pose)
                map_pts  = list(_voxel_map.values())  # [(wx, wy, height), ...] metre
            await ws.send_json({
                "type": "lidar",
                "online": _lidar_online,
                "enabled": _lidar_enabled,
                "applied": (not _lidar_online) if not _lidar_enabled else (_lidar_online and scan_age is not None and scan_age < 1.0),
                "scan_age_s": round(scan_age, 3) if scan_age is not None else None,
                "scan_seq": _lidar_scan_seq,
                "error": _lidar_last_error or None,
                "slam": SLAM_AVAILABLE and SLAM_MAPPING_ENABLED,
                "points": pts,        # [[açı_derece, mesafe_mm], ...] — ham, robota göre
                "pose": pose,         # {"x":, "y":, "theta":} metre/radyan — DÜNYA çerçevesi
                "map_points": map_pts # [[wx, wy, height], ...] metre — DÜNYA çerçevesi, kalıcı harita
            })
    except WebSocketDisconnect:
        pass
    finally:
        if not receive_task.done():
            receive_task.cancel()


# ─── mapping_v2 / rota + süre sınırlı platform HIL testi ───────────────────

def _navigation_unavailable():
    return JSONResponse(
        {"error": "mapping_v2 kullanılamıyor", "execution_locked": True},
        status_code=503,
    )


@app.get("/api/navigation/status")
async def navigation_status():
    if _mapping_engine is None:
        return _navigation_unavailable()
    result = _mapping_engine.snapshot(include_grid=True)
    result["platform_test"] = dict(_navigation_platform_status)
    return JSONResponse(result)


@app.get("/api/navigation/maps")
async def navigation_maps():
    if _mapping_engine is None or _map_archive is None:
        return _navigation_unavailable()
    return JSONResponse({"ok": True, "maps": _map_archive.list_maps()})


@app.post("/api/navigation/mapping/start")
async def navigation_mapping_start(request: Request):
    if _mapping_engine is None:
        return _navigation_unavailable()
    if _navigation_platform_status.get("active"):
        return JSONResponse({"ok": False, "error": "Rota yürütülürken haritalama sıfırlanamaz"}, status_code=409)
    try:
        payload = await request.json()
    except Exception:
        payload = {}
    async with _mapping_archive_lock:
        motor = await _set_lidar_motor(True)
        if not motor.get("applied"):
            return JSONResponse({
                "ok": False,
                "error": motor.get("error") or "LiDAR taraması başlatılamadı; harita temizlenmedi",
                "motor": motor,
            }, status_code=409)
        status = _mapping_engine.start_mapping_session(str(payload.get("name") or "Ev Haritası"))
    return JSONResponse({"ok": True, "mapping_session": status, "motor": motor})


@app.post("/api/navigation/mapping/finish")
async def navigation_mapping_finish():
    if _mapping_engine is None or _map_archive is None:
        return _navigation_unavailable()
    try:
        async with _mapping_archive_lock:
            status = _mapping_engine.finish_mapping_session()
            if status.get("archive_id"):
                return JSONResponse({"ok": True, "mapping_session": status})
            metadata = await asyncio.to_thread(
                _map_archive.save,
                _mapping_engine.export_archive(),
                str(status.get("label") or "Ev Haritası"),
            )
            status = _mapping_engine.mark_archive_saved(metadata)
        return JSONResponse({"ok": True, "mapping_session": status, "map": metadata})
    except (ValueError, OSError, KeyError) as exc:
        return JSONResponse({"ok": False, "error": str(exc)}, status_code=409)


@app.post("/api/navigation/mapping/load")
async def navigation_mapping_load(request: Request):
    if _mapping_engine is None or _map_archive is None:
        return _navigation_unavailable()
    if _navigation_platform_status.get("active"):
        return JSONResponse({"ok": False, "error": "Rota yürütülürken harita değiştirilemez"}, status_code=409)
    try:
        payload = await request.json()
        map_id = str(payload["map_id"])
        async with _mapping_archive_lock:
            archive = await asyncio.to_thread(_map_archive.load, map_id)
            status = _mapping_engine.load_archive(archive)
        return JSONResponse({"ok": True, "mapping_session": status})
    except (KeyError, ValueError, OSError) as exc:
        return JSONResponse({"ok": False, "error": str(exc)}, status_code=400)


def _navigation_telemetry_error(data: dict, require_stand: bool = True) -> str | None:
    if not data.get("outputsArmed"):
        return "Bacak PWM çıkışları hazır değil"
    if not data.get("pcaRightOnline") or not data.get("pcaLeftOnline"):
        return "PCA9685 bacak sürücülerinden biri çevrimdışı"
    if not data.get("configValid") or not data.get("homeSet"):
        return "Servo/HOME yapılandırması geçersiz"
    if data.get("failsafe"):
        return "Omurilik failsafe durumunda"
    if require_stand and data.get("spineState") not in {"STAND", "WALK"}:
        return f"Omurilik STAND/WALK durumunda değil: {data.get('spineState')}"
    if not data.get("mpuOnline"):
        return "IMU çevrimdışı"
    if abs(float(data.get("mpuP", 0.0))) > 45.0 or abs(float(data.get("mpuR", 0.0))) > 45.0:
        return "IMU güvenli eğim sınırı aşıldı"
    if min(float(data.get("batR", 0.0)), float(data.get("batL", 0.0))) < NAV_PLATFORM_TEST_MIN_BATTERY_V:
        return "Batarya güvenli test sınırının altında"
    return None


async def _navigation_read_telemetry() -> dict:
    # Safety preflight must never accept a recently cached ARMED/STAND sample.
    response = await _proxy_to_esp32("/telemetry", {}, force_fresh=True)
    if response.status_code >= 400:
        raise RuntimeError("Omurilik telemetrisi okunamadı")
    return json.loads(bytes(response.body).decode("utf-8"))


async def _motion_start_telemetry() -> dict:
    """Return fresh motion preflight data, safely recovering a latched timeout.

    Recovery never arms outputs: it sends only zero joystick and IDLE, then
    verifies a second fresh sample. Any failed step leaves motion blocked.
    """
    telemetry = await _navigation_read_telemetry()
    if not telemetry.get("failsafe") and telemetry.get("spineState") != "FAILSAFE":
        return telemetry
    zero = await _proxy_to_esp32("/joy", {"x": "0", "y": "0", "r": "0"})
    idle = await _proxy_to_esp32("/mode", {"m": "idle"})
    if zero.status_code >= 400 or idle.status_code >= 400:
        raise RuntimeError("Omurilik failsafe güvenli şekilde temizlenemedi")
    await asyncio.sleep(0.12)
    telemetry = await _navigation_read_telemetry()
    if telemetry.get("failsafe") or telemetry.get("spineState") == "FAILSAFE":
        raise RuntimeError("Omurilik failsafe durumunda kaldı")
    return telemetry


async def _navigation_send_stop() -> None:
    # Deadman'e ek olarak iki açık sıfır komutu ve IDLE/STAND geçişi.
    await _proxy_to_esp32("/joy", {"x": "0", "y": "0", "r": "0"})
    await asyncio.sleep(0.06)
    await _proxy_to_esp32("/joy", {"x": "0", "y": "0", "r": "0"})
    await _proxy_to_esp32("/mode", {"m": "idle"})


async def _navigation_direct_request(
    client: httpx.AsyncClient,
    path: str,
    query: dict[str, str],
    *,
    timeout: float | None = None,
) -> httpx.Response:
    """Send one HIL command without the lossy joystick coalescer.

    During route execution, ordinary HUD telemetry is served from the last
    safe cache entry. Therefore this lock is normally acquired at
    once and every heartbeat actually reaches the ESP32 deadman.
    """
    async with esp32_lock:
        if timeout is None:
            return await client.get(f"{ESP32_URL}{path}", params=query)
        async with asyncio.timeout(timeout):
            return await client.get(f"{ESP32_URL}{path}", params=query, timeout=timeout)


async def _navigation_send_heartbeat(
    client: httpx.AsyncClient,
    joy_x: float,
    joy_y: float,
) -> httpx.Response:
    """Deliver the current state packet, retrying one transient HTTP stall.

    Two 300 ms attempts finish inside the firmware's independent 750 ms
    deadman. Repeating a joystick state is idempotent and does not queue steps.
    """
    for attempt in range(2):
        try:
            return await _navigation_direct_request(
                client, "/joy",
                {"x": f"{joy_x:.3f}", "y": f"{joy_y:.3f}", "r": "0"},
                timeout=NAV_PLATFORM_TEST_REQUEST_BUDGET_S,
            )
        except (TimeoutError, httpx.TimeoutException):
            if attempt == 1:
                raise
    raise RuntimeError("Hareket kalp atışı gönderilemedi")


def _navigation_require_ok(response: httpx.Response, message: str) -> None:
    if response.status_code < 400:
        return
    detail = response.text.strip().replace("\n", " ")[:160]
    suffix = f" (HTTP {response.status_code}: {detail})" if detail else f" (HTTP {response.status_code})"
    raise RuntimeError(message + suffix)


async def _cancel_navigation_platform_test(reason: str) -> None:
    global _navigation_platform_task
    task = _navigation_platform_task
    if task is not None and not task.done() and task is not asyncio.current_task():
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
    _navigation_platform_task = None
    await _navigation_send_stop()
    _navigation_platform_status.update(state="STOPPED", active=False, message=reason)


def _navigation_route_length(route: list[list[float]]) -> float:
    return sum(
        math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
        for a, b in zip(route, route[1:])
    )


def _navigation_segment_command(a: list[float], b: list[float]) -> tuple[float, float, float, float]:
    dx, dy = float(b[0]) - float(a[0]), float(b[1]) - float(a[1])
    distance = math.hypot(dx, dy)
    if distance < 0.01:
        return 0.0, 0.0, 0.0, distance
    # Harita +X ileri/+Y sol; firmware joystick: joyY ileri, joyX sağ.
    joy_x = -NAV_ROUTE_JOY_MAG * dy / distance
    joy_y = NAV_ROUTE_JOY_MAG * dx / distance
    speed_mps = max(0.01, NAV_ROUTE_FULL_SPEED_MPS * NAV_ROUTE_JOY_MAG)
    return joy_x, joy_y, distance / speed_mps, distance


async def _run_navigation_platform_test(route: list[list[float]], previous_params: dict) -> None:
    """Follow every planned segment with bounded command-time dead reckoning.

    The current hardware has no trusted wheel/visual odometry, so completion is
    explicitly ESTIMATED rather than falsely claiming a measured goal arrival.
    Fresh directional LiDAR remains a hard stop throughout execution.
    """
    global _navigation_platform_task, _ai_tracking_enabled, _lidar_nav_enabled
    try:
        route_length = _navigation_route_length(route)
        estimated_duration = route_length / max(0.01, NAV_ROUTE_FULL_SPEED_MPS * NAV_ROUTE_JOY_MAG)
        if route_length < 0.05:
            raise RuntimeError("Rota hareket yönü üretmiyor")
        if route_length > NAV_ROUTE_MAX_LENGTH_M:
            raise RuntimeError(f"Rota {route_length:.2f} m; güvenli sınır {NAV_ROUTE_MAX_LENGTH_M:.2f} m")
        if estimated_duration > NAV_ROUTE_MAX_DURATION_S:
            raise RuntimeError(f"Tahmini rota süresi {estimated_duration:.0f} sn; güvenli süre sınırı aşıldı")
        _ai_tracking_enabled = False
        _lidar_nav_enabled = False

        # ESP32 WebServer closes HTTP connections itself. Explicitly avoiding
        # stale keep-alive reuse and limiting connection churn to 10 Hz proved
        # more deterministic than the generic proxy's lossy coalescing path.
        timeout = httpx.Timeout(NAV_PLATFORM_TEST_REQUEST_BUDGET_S)
        limits = httpx.Limits(max_connections=1, max_keepalive_connections=0)
        async with httpx.AsyncClient(
            timeout=timeout,
            limits=limits,
            headers={"Connection": "close"},
        ) as motion_client:
            params = await _navigation_direct_request(
                motion_client, "/params",
                {"z": str(previous_params["z"]), "len": "40", "lift": "10", "spd": "1000"},
                timeout=1.0,
            )
            _navigation_require_ok(params, "Yavaş yürüyüş profili uygulanamadı")

            mode = await _navigation_direct_request(
                motion_client, "/mode", {"m": "bezier_joy"},
                timeout=NAV_PLATFORM_TEST_REQUEST_BUDGET_S,
            )
            _navigation_require_ok(mode, "Omurilik WALK moduna geçmedi")

            started = time.monotonic()
            _navigation_platform_status.update(
                state="RUNNING", active=True,
                message=f"Rota yürütülüyor · 0/{len(route)-1} parça",
                route_length_m=round(route_length, 3), progress=0.0,
                waypoint=0, waypoint_count=len(route) - 1,
                estimated_duration_s=round(estimated_duration, 1),
                heartbeat_hz=NAV_ROUTE_COMMAND_HZ,
            )
            travelled_before = 0.0
            for index, (a, b) in enumerate(zip(route, route[1:]), start=1):
                joy_x, joy_y, segment_duration, segment_distance = _navigation_segment_command(a, b)
                if segment_distance < 0.01:
                    continue
                segment_started = time.monotonic()
                next_command = segment_started
                while time.monotonic() - segment_started < segment_duration:
                    with _lidar_lock:
                        points = list(_lidar_points)
                        scan_fresh = _lidar_online and time.monotonic() - _lidar_last_scan_ts < 0.8
                    if not scan_fresh or not points:
                        raise RuntimeError("LiDAR taraması güncel değil; rota güvenli durduruldu")
                    direction_deg = math.degrees(math.atan2(-joy_x, joy_y)) % 360.0
                    obstacle_m = _get_lidar_distance_at_angle(points, direction_deg, 10.0)
                    if obstacle_m is not None and obstacle_m < NAV_ROUTE_OBSTACLE_STOP_M:
                        raise RuntimeError(f"Rota yönünde {obstacle_m:.2f} m engel algılandı")
                    command = await _navigation_send_heartbeat(motion_client, joy_x, joy_y)
                    _navigation_require_ok(command, "Omurilik hareket komutunu reddetti")
                    segment_fraction = min(1.0, (time.monotonic() - segment_started) / segment_duration)
                    progress = (travelled_before + segment_distance * segment_fraction) / route_length
                    _navigation_platform_status.update(
                        message=f"Rota yürütülüyor · {index}/{len(route)-1} parça · %{progress*100:.0f}",
                        joy_x=round(joy_x, 3), joy_y=round(joy_y, 3),
                        waypoint=index, progress=round(progress, 3),
                        elapsed_s=round(time.monotonic() - started, 1),
                        obstacle_m=None if obstacle_m is None else round(obstacle_m, 3),
                    )
                    next_command += 1.0 / NAV_ROUTE_COMMAND_HZ
                    await asyncio.sleep(max(0.0, next_command - time.monotonic()))
                travelled_before += segment_distance
        _navigation_platform_status.update(
            state="COMPLETED", active=True, progress=1.0,
            message="Rota tamamlandı (komut-zaman tahmini); robot durduruldu",
        )
    except asyncio.CancelledError:
        _navigation_platform_status.update(state="STOPPED", active=True, message="Rota operatör tarafından durduruldu")
        raise
    except Exception as exc:
        detail = str(exc).strip() or type(exc).__name__
        _navigation_platform_status.update(state="ABORTED", active=True, message=f"Güvenli duruş: {detail}")
        log.warning(f"[NAV-ROUTE] Yürütme durduruldu: {type(exc).__name__}: {detail}")
    finally:
        await _navigation_send_stop()
        restore = await _proxy_to_esp32("/params", {k: str(v) for k, v in previous_params.items()})
        if restore.status_code >= 400:
            _navigation_platform_status.update(state="ABORTED", active=False,
                message="Robot durduruldu; önceki yürüyüş ayarları geri yüklenemedi")
        _navigation_platform_status["active"] = False
        _navigation_platform_task = None


async def _begin_navigation_execution() -> dict:
    """Run the common safety preflight and start exactly one route task."""
    global _navigation_platform_task
    snapshot = _mapping_engine.snapshot(include_grid=False)
    route = snapshot.get("route") or []
    if not snapshot.get("route_ok") or len(route) < 2:
        raise ValueError("Yürütülebilir rota parçası yok")
    if _navigation_platform_task is not None and not _navigation_platform_task.done():
        raise ValueError("Rota zaten yürütülüyor")

    try:
        telemetry = await _motion_start_telemetry()
    except RuntimeError as exc:
        raise ValueError(str(exc)) from exc
    safety_error = _navigation_telemetry_error(telemetry)
    if safety_error:
        raise ValueError(safety_error)
    previous_params = telemetry.get("gaitParams")
    if not isinstance(previous_params, dict) or not all(
        isinstance(previous_params.get(key), (int, float)) and math.isfinite(previous_params[key])
        for key in ("z", "len", "lift", "spd")
    ):
        raise ValueError("Rota yürütme için v57.3+ omurilik telemetrisi gerekli")
    previous_params = {key: previous_params[key] for key in ("z", "len", "lift", "spd")}

    motor = await _set_lidar_motor(True)
    if not motor.get("applied"):
        raise ValueError(motor.get("error") or "LiDAR taraması başlatılamadı")

    # Motor hazırlanırken harita değişmiş olabilir. Daima en güncel güvenli
    # rotayı kullan; manuel modda değişmiş rota tekrar açık onay gerektirir.
    current = _mapping_engine.snapshot(include_grid=False)
    if current.get("goal") != snapshot.get("goal"):
        raise ValueError("Hedef değişti; güncel rotayı yeniden onaylayın")
    if current.get("route") != route:
        if current.get("approval_mode") != "auto":
            raise ValueError("Rota değişti; güncel rotayı yeniden onaylayın")
        route = current.get("route") or []
    if not current.get("route_ok") or len(route) < 2:
        raise ValueError("Güncel haritada yürütülebilir rota kalmadı")
    if _navigation_platform_task is not None and not _navigation_platform_task.done():
        raise ValueError("Rota zaten yürütülüyor")

    result = current if current.get("approved") else _mapping_engine.approve_preview()
    _navigation_platform_status.clear()
    _navigation_platform_status.update(
        state="STARTING", active=True, progress=0.0,
        message="LiDAR güvenlikli rota yürütme başlatılıyor",
        max_duration_s=NAV_ROUTE_MAX_DURATION_S,
    )
    _navigation_platform_task = asyncio.create_task(_run_navigation_platform_test(route, previous_params))
    return {**result, "platform_test": dict(_navigation_platform_status)}


@app.post("/api/navigation/goal")
async def navigation_goal(request: Request):
    if _mapping_engine is None:
        return _navigation_unavailable()
    try:
        if _navigation_platform_status.get("active"):
            await _cancel_navigation_platform_test("Yeni hedef seçildi")
        payload = await request.json()
        result = _mapping_engine.set_goal(float(payload["x_m"]), float(payload["y_m"]))
        _navigation_platform_status.clear()
        _navigation_platform_status.update(
            state="READY" if result.get("route_ok") else "NO_ROUTE", active=False,
            message="Yeni hedef seçildi; rota onayı bekleniyor" if result.get("approval_mode") == "manual"
                    else "Otonom karar değerlendiriliyor",
            max_duration_s=NAV_ROUTE_MAX_DURATION_S,
        )
        if result.get("approved") and result.get("approval_mode") == "auto":
            started = await _begin_navigation_execution()
            return JSONResponse({"ok": True, **started})
        return JSONResponse({"ok": True, **result, "platform_test": dict(_navigation_platform_status)})
    except (KeyError, TypeError, ValueError) as exc:
        return JSONResponse({"ok": False, "error": str(exc), "execution_locked": True}, status_code=400)


@app.post("/api/navigation/mode")
async def navigation_mode(request: Request):
    if _mapping_engine is None:
        return _navigation_unavailable()
    try:
        payload = await request.json()
        result = _mapping_engine.set_approval_mode(str(payload["mode"]))
        if result.get("approved") and result.get("approval_mode") == "auto":
            started = await _begin_navigation_execution()
            return JSONResponse({"ok": True, **started})
        return JSONResponse({"ok": True, **result})
    except (KeyError, TypeError, ValueError) as exc:
        return JSONResponse({"ok": False, "error": str(exc), "execution_locked": True}, status_code=400)


@app.post("/api/navigation/approve")
async def navigation_approve():
    if _mapping_engine is None:
        return _navigation_unavailable()
    try:
        return JSONResponse({"ok": True, **await _begin_navigation_execution()})
    except ValueError as exc:
        if not _navigation_platform_status.get("active"):
            _navigation_platform_status.update(state="BLOCKED", active=False, message=str(exc))
        return JSONResponse({"ok": False, "error": str(exc), "execution_locked": True}, status_code=409)


@app.post("/api/navigation/clear")
async def navigation_clear():
    if _mapping_engine is None:
        return _navigation_unavailable()
    await _cancel_navigation_platform_test("Hedef/rota temizlendi")
    return JSONResponse({"ok": True, **_mapping_engine.clear_goal(), "platform_test": dict(_navigation_platform_status)})


@app.websocket("/ws/navigation")
async def ws_navigation(ws: WebSocket):
    if not await _require_ws_login(ws):
        return
    await ws.accept()
    last_camera_probe = 0.0
    last_revision = -1
    last_sent = 0.0
    try:
        async with httpx.AsyncClient(timeout=0.8) as camera_client:
            while True:
                if _mapping_engine is None:
                    await ws.send_json({"type": "navigation", "error": "mapping_v2 kullanılamıyor", "execution_locked": True})
                else:
                    # Mevcut kamera sahibinin hafif /health durumunu oku; kamera
                    # aygıtı veya MJPEG akışı ikinci kez açılmaz.
                    if time.monotonic() - last_camera_probe >= 3.0:
                        try:
                            health = await camera_client.get(f"{CAMERA_STREAM_URL}/health")
                            _mapping_engine.set_camera_health(health.status_code == 200)
                        except httpx.RequestError:
                            _mapping_engine.set_camera_health(False)
                        last_camera_probe = time.monotonic()
                    # Harita üretimi salt-okunurdur. Kontrol mesajları bu WebSocket'te
                    # bilinçli olarak kabul edilmez; değişiklikler dar POST API'lerinden geçer.
                    # Yeni tarama/rota durumu geldiğinde hemen yayınla. Değişiklik yoksa
                    # 1 saniyelik heartbeat yeterlidir; 13-15 KB'lık grid'i boşuna çoğaltma.
                    now = time.monotonic()
                    revision = _mapping_engine.revision()
                    if revision != last_revision or now - last_sent >= 1.0:
                        snapshot = await asyncio.to_thread(_mapping_engine.snapshot, True)
                        snapshot["platform_test"] = dict(_navigation_platform_status)
                        await ws.send_json(snapshot)
                        last_revision = revision
                        last_sent = now
                await asyncio.sleep(1.0 / NAVIGATION_BROADCAST_MAX_HZ)
    except WebSocketDisconnect:
        pass


# ─── Kamera/HUD proxy — browser tek porttan (8080) çalışsın ─────────────────
# camera_stream.py veya detection_hud.py 8888'de MJPEG yayınlar. Uzak PC/telefon
# bazen sadece 8080 arayüzünü görebiliyor, 8888'e erişemiyor ya da "localhost"
# / farklı interface karışıyor. Bu proxy ile UI artık /camera/stream kullanır;
# yani kamera da, telemetri de, WebSocket de aynı host:8080 üzerinden akar.

async def _open_camera_upstream(path: str = "/stream"):
    timeout = httpx.Timeout(connect=2.0, read=None, write=2.0, pool=None)
    client = httpx.AsyncClient(timeout=timeout)
    try:
        req = client.build_request("GET", f"{CAMERA_STREAM_URL}{path}")
        resp = await client.send(req, stream=True)
        if resp.status_code != 200:
            body = await resp.aread()
            await resp.aclose()
            await client.aclose()
            return None, None, Response(content=body, status_code=resp.status_code)
        return client, resp, None
    except Exception as e:
        await client.aclose()
        return None, None, JSONResponse({"error": f"Kamera/HUD yayınına ulaşılamıyor: {e}"}, status_code=502)



# ─── ROBOT KOL UYUMLULUK ALIAS'LARI ──────────────────────────────────────────
# UI tarafı /arm/... kullanıyor; ESP32 firmware tarafı /armPose, /armServo gibi
# camelCase endpoint'ler kullanıyor. Bu alias'lar iki adlandırmayı da RPi'da
# geçerli yapar. Böylece eski/yenilenmiş UI karışsa bile 404 yerine aynı kol
# sürücü zinciri çalışır. Bacak, LiDAR, kamera, joystick endpoint'lerine dokunmaz.

@app.get("/armStatus")
async def arm_status_alias():
    return await arm_status()

@app.get("/armCalibration")
async def arm_calibration_alias():
    return await arm_calibration_get()

@app.get("/armCalibrate")
async def arm_calibrate_alias(immediate: Optional[int] = 1):
    # immediate parametresi ESP tarafına gönderilmiyor olabilir; RPi zaten tek poz
    # gönderdiği için davranış /arm/calibrate_pose ile aynı kalır.
    return await arm_calibrate_pose()

@app.get("/armHome")
async def arm_home_alias(immediate: Optional[int] = 1):
    return await arm_home()

@app.get("/armOff")
async def arm_off_alias():
    return await arm_off()

@app.get("/armServo")
async def arm_servo_alias(id: Optional[str] = None, joint: Optional[str] = None, angle: float = 90.0, immediate: Optional[int] = 1):
    j = (joint or id or "").strip().lower()
    return await arm_servo(j, angle)

@app.get("/armPose")
async def arm_pose_alias(
    base: Optional[float] = None,
    joint1: Optional[float] = None,
    joint2: Optional[float] = None,
    gripper: Optional[float] = None,
    immediate: Optional[int] = 1,
):
    return await arm_pose(base=base, joint1=joint1, joint2=joint2, gripper=gripper)

@app.get("/armGripper")
async def arm_gripper_alias(angle: float):
    return await arm_gripper(angle)

@app.get("/armOpen")
async def arm_open_alias():
    calib = _load_arm_calibration()
    angle = float(calib.get("gripper", {}).get("open", 90.0))
    return await arm_servo("gripper", angle)

@app.get("/armClose")
async def arm_close_alias():
    calib = _load_arm_calibration()
    angle = float(calib.get("gripper", {}).get("close", 90.0))
    return await arm_servo("gripper", angle)

@app.get("/armConfig")
async def arm_config_alias(
    gmin: Optional[float] = None,
    gmax: Optional[float] = None,
    gopen: Optional[float] = None,
    gclose: Optional[float] = None,
    save: Optional[int] = 1,
):
    return await arm_calibration_set(
        gripper_min=gmin,
        gripper_max=gmax,
        gripper_open=gopen,
        gripper_close=gclose,
        gripper_current=None,
    )

@app.get("/arm/diagnose")
async def arm_diagnose():
    """404 kaynağını ayırmak için hafif teşhis: RPi route mevcut mu, ESP firmware route mevcut mu?"""
    out = {
        "ok": True,
        "rpi_routes_loaded": True,
        "driver": ARM_DRIVER_MODE,
        "esp32_url": ESP32_URL,
        "pose_endpoint": ARM_POSE_ENDPOINT,
        "servo_endpoint": ARM_SERVO_ENDPOINT,
        "expected_pwm": {"pca": "0x40", "base": 12, "joint1": 13, "joint2": 14, "gripper": 15},
        "esp": {},
    }
    try:
        async with httpx.AsyncClient(timeout=1.2) as client:
            for name, url in {
                "armStatus": f"{ESP32_URL}/armStatus",
                "arm/status": f"{ESP32_URL}/arm/status",
                "telemetry": f"{ESP32_URL}/telemetry",
            }.items():
                try:
                    r = await client.get(url)
                    out["esp"][name] = {"status": r.status_code, "body": r.text[:180]}
                except Exception as e:
                    out["esp"][name] = {"error": str(e)}
    except Exception as e:
        out["esp_error"] = str(e)
    return JSONResponse(out)


@app.get("/camera/stream")
async def camera_stream_proxy():
    client, resp, error_response = await _open_camera_upstream("/stream")
    if error_response is not None:
        return error_response

    async def _body():
        try:
            async for chunk in resp.aiter_bytes():
                if chunk:
                    yield chunk
        finally:
            await resp.aclose()
            await client.aclose()

    media_type = resp.headers.get(
        "content-type",
        "multipart/x-mixed-replace; boundary=frame",
    )
    return StreamingResponse(
        _body(),
        media_type=media_type,
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "Pragma": "no-cache",
            "X-Accel-Buffering": "no",
        },
    )

@app.get("/camera/snapshot")
async def camera_snapshot_proxy():
    try:
        async with httpx.AsyncClient(timeout=2.0) as client:
            r = await client.get(f"{CAMERA_STREAM_URL}/snapshot")
        return Response(
            content=r.content,
            status_code=r.status_code,
            media_type=r.headers.get("content-type", "image/jpeg"),
            headers={"Cache-Control": "no-store"},
        )
    except Exception as e:
        return JSONResponse({"error": f"Kamera/HUD snapshot alınamadı: {e}"}, status_code=502)

@app.get("/camera/status")
async def camera_status_proxy():
    try:
        async with httpx.AsyncClient(timeout=1.0) as client:
            r = await client.get(f"{CAMERA_STREAM_URL}/snapshot")
        age_raw = r.headers.get("x-apex-frame-age")
        try:
            frame_age = float(age_raw) if age_raw is not None else None
        except ValueError:
            frame_age = None
        fresh = (r.status_code == 200) and (frame_age is None or frame_age <= 3.0)
        return JSONResponse({
            "camera": fresh,
            "fresh": fresh,
            "status": r.status_code,
            "frame_age": frame_age,
        })
    except Exception as e:
        return JSONResponse({"camera": False, "fresh": False, "error": str(e)}, status_code=200)

# ─── Kalibrasyon API (büyük payload — direkt proxy) ──────────────────────────

@app.get("/api/getCalib")
async def get_calib(leg: int):
    async with httpx.AsyncClient(timeout=2.0) as c:
        r = await c.get(f"{ESP32_URL}/getCalib", params={"leg": leg})
    return JSONResponse(r.json())

@app.post("/api/saveCalib")
async def save_calib(request: Request):
    data = await request.json()
    async with httpx.AsyncClient(timeout=2.0) as c:
        r = await c.get(f"{ESP32_URL}/saveCalib", params=data)
    return Response(content=r.text, status_code=r.status_code)

@app.get("/api/getHome")
async def get_home(leg: int):
    async with httpx.AsyncClient(timeout=2.0) as c:
        r = await c.get(f"{ESP32_URL}/getHome", params={"leg": leg})
    return JSONResponse(r.json())

@app.post("/api/saveHome")
async def save_home(request: Request):
    data = await request.json()
    async with httpx.AsyncClient(timeout=2.0) as c:
        r = await c.get(f"{ESP32_URL}/saveHome", params=data)
    return Response(content=r.text, status_code=r.status_code)

# ─── Durum endpoint ───────────────────────────────────────────────────────────

@app.get("/api/status")
async def status():
    return {"esp32": esp_online, "server": "apex_v54", "telemetry_hz": TELEMETRY_HZ}

@app.get("/api/workspaces")
async def workspaces(request: Request):
    """Ana kontrol ve Kinematics Lab URL'lerini aynı host adıyla döndürür."""
    host = request.url.hostname or "localhost"
    scheme = request.url.scheme or "http"
    return {
        "command_center": f"{scheme}://{host}:8080",
        "kinematics_lab": f"{scheme}://{host}:8090",
        "camera_console": f"{scheme}://{host}:8080/camera-console",
    }

# ─── DOĞRUDAN REST PROXY (apex_ui.html'in fetch() çağırdığı yollar) ─────────
# apex_ui.html relative path'lerle çağırıyor: /telemetry, /joy, /bodyIk, /mode,
# /params, /setGimbal, /setGimbalFilter, /audio, /manual, /homePreview, /goHome,
# /getCalib, /saveCalib, /getHome, /saveHome — burada ESP32'ye 1:1 proxy edilir.

PROXIED_GET_PATHS = [
    "/telemetry", "/joy", "/bodyIk", "/mode", "/params",
    "/setGimbal", "/setGimbalFilter", "/audio",
    "/manual", "/homePreview", "/goHome",
    "/getCalib", "/saveCalib", "/getHome", "/saveHome",
    # Leg Servo Lab
    "/legServo", "/legMountPose", "/legHomePose", "/legOff", "/legOn", "/legStatus",
]

esp32_lock = asyncio.Lock()  # ESP32 WebServer senkron/blocking — eşzamanlı istek kabul etmiyor

async def _proxy_to_esp32(path: str, query: dict, *, force_fresh: bool = False) -> Response:
    global esp_online
    cached_telemetry = None
    if path == "/telemetry" and not force_fresh:
        with _telemetry_resp_cache_lock:
            cached = _telemetry_resp_cache["data"]
            fresh = (time.time() - _telemetry_resp_cache["ts"]) < TELEMETRY_CACHE_TTL_S
            cached_telemetry = None if cached is None else dict(cached)
        if _navigation_platform_status.get("active") and cached_telemetry is not None:
            # The ESP32 has a single-threaded WebServer and a 750 ms motion
            # deadman. A large telemetry response must never delay the 20 Hz
            # platform-test heartbeat. Keep the HUD alive with an explicitly
            # marked cache sample for the test's bounded four-second window.
            cached_telemetry["proxyStale"] = True
            cached_telemetry["platformTestActive"] = True
            return JSONResponse(content=cached_telemetry, status_code=200)
        if cached is not None and fresh:
            # 200ms içinde başka bir poller (tarayıcı/HUD) zaten taze veri
            # aldı — ESP32'ye TEKRAR gitmek yerine onu paylaşıyoruz.
            return JSONResponse(content=cached, status_code=200)
        if esp32_lock.locked() and cached_telemetry is not None:
            # Birden fazla açık HUD/Kinematics sekmesi ESP32'nin tek iş parçacıklı
            # WebServer'ı önünde sınırsız telemetri kuyruğu oluşturmasın. Devam
            # eden komutu bekletmek yerine son örneği açıkça stale olarak döndür.
            cached_telemetry["proxyStale"] = True
            return JSONResponse(content=cached_telemetry, status_code=200)
    if path == "/legStatus" and esp32_lock.locked():
        return JSONResponse({"error": "ESP32 busy", "retry": True}, status_code=429)
    try:
        # A heartbeat is either delivered within the deadman budget or fails.
        # Telemetry must release the single ESP connection promptly as well.
        bounded = path in {"/joy", "/telemetry", "/legStatus"}
        async with asyncio.timeout(0.6 if path == "/joy" else (1.2 if bounded else 6.0)):
            async with esp32_lock:
                async with httpx.AsyncClient(timeout=0.3 if bounded else 3.0, headers={"Connection": "close"}) as client:
                    r = await client.get(f"{ESP32_URL}{path}", params=query)

        if path == "/telemetry" and r.status_code == 200:
            try:
                data = r.json()
                esp_online = True
                with _telemetry_cache_lock:
                    if "mpuP" in data: _telemetry_cache["mpuP"] = data["mpuP"]
                    if "mpuR" in data: _telemetry_cache["mpuR"] = data["mpuR"]
                    if "mpuY" in data: _telemetry_cache["mpuY"] = data["mpuY"]
                    if "mpuOnline" in data: _telemetry_cache["mpuOnline"] = data["mpuOnline"]
                    if "tofOnline" in data: _telemetry_cache["tofOnline"] = data["tofOnline"]
                    if "tofMm" in data: _telemetry_cache["tofMm"] = data["tofMm"]
                    if "z" in data:    _telemetry_cache["z"]    = data["z"]
                # ESP32'nin bilmediği, sunucu tarafı bir bayrağı (AI takip
                # açık/kapalı) JSON'a ekliyoruz — detection_hud.py bunu zaten
                # poll ettiği /telemetry üzerinden okuyacak, ayrı bir endpoint
                # gerekmiyor.
                data["aiTrack"] = _ai_tracking_enabled
                with _target_selection_lock:
                    data["sel"] = dict(_target_selection)
                with _telemetry_resp_cache_lock:
                    _telemetry_resp_cache["data"] = data
                    _telemetry_resp_cache["ts"] = time.time()
                return JSONResponse(content=data, status_code=r.status_code)
            except Exception:
                pass  # JSON bozuksa ham içerikle devam — aşağıdaki passthrough

        return Response(
            content=r.content,
            status_code=r.status_code,
            media_type=r.headers.get("content-type", "text/plain"),
        )
    except (httpx.RequestError, TimeoutError):
        # ConnectError, ConnectTimeout, ReadTimeout vb. TÜM ağ seviyesi
        # hataları kapsar — sadece ConnectError yakalamak yetmiyordu,
        # zaman aşımları (ör. APEX-HUB kapalıyken ESP32'ye ulaşılamaması)
        # buradan kaçıp 500 olarak görünüyordu.
        return JSONResponse({"error": "ESP32 erişilemiyor"}, status_code=502)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)

async def _operator_proxy(path: str, query: dict):
    global _lidar_nav_enabled, _ai_tracking_enabled
    query = dict(query)
    if path == "/joy":
        source = query.pop("source", "manual")
        if _navigation_platform_status.get("active"):
            return JSONResponse({"error": "Rota testi joystick kanalını kullanıyor"}, status_code=409)
        if source == "ai":
            if not _ai_tracking_enabled:
                return JSONResponse({"error": "AI takip hareket kanalı kapalı"}, status_code=409)
        elif source == "lidar":
            if not _lidar_nav_enabled:
                return JSONResponse({"error": "LiDAR yürüyüşü kapalı"}, status_code=409)
        elif _lidar_nav_enabled or _ai_tracking_enabled:
            owner = "LiDAR otonomi" if _lidar_nav_enabled else "AI takip"
            return JSONResponse({"error": f"Hareket kanalı {owner} tarafından kullanılıyor"}, status_code=409)
    if path == "/mode":
        if _navigation_platform_status.get("active"):
            if query.get("m") != "idle":
                return JSONResponse({"error": "Önce rota testini durdurun"}, status_code=409)
            await _cancel_navigation_platform_test("Operatör durdurdu")
        _lidar_nav_enabled = False
        _ai_tracking_enabled = False
    return await _proxy_to_esp32(path, query)


def _register_proxy_route(path: str):
    async def _handler(request: Request):
        return await _operator_proxy(path, dict(request.query_params))
    app.get(path)(_handler)

for _p in PROXIED_GET_PATHS:
    _register_proxy_route(_p)

# ─── Ana sayfa — apex_ui.html ────────────────────────────────────────────────
# apex_server.py ile AYNI klasöre apex_ui.html dosyasını koy; tarayıcı
# http://apex.local:8080 adresine gidince bu dosya servis edilir.

@app.get("/")
async def serve_ui():
    if not os.path.isfile(UI_FILE):
        return JSONResponse(
            {"error": f"apex_ui.html bulunamadı: {UI_FILE}"},
            status_code=404,
        )
    return FileResponse(UI_FILE, media_type="text/html", headers={"Cache-Control": "no-store, no-cache, must-revalidate, max-age=0", "Pragma": "no-cache"})


@app.get("/camera-console")
@app.get("/camera-console/")
async def serve_camera_console():
    if not os.path.isfile(CAMERA_UI_FILE):
        return JSONResponse(
            {"error": f"camera_console.html bulunamadı: {CAMERA_UI_FILE}"},
            status_code=404,
        )
    return FileResponse(
        CAMERA_UI_FILE,
        media_type="text/html",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate, max-age=0", "Pragma": "no-cache"},
    )


@app.get("/navigation-console")
@app.get("/navigation-console/")
async def serve_navigation_console():
    if not os.path.isfile(NAVIGATION_UI_FILE):
        return JSONResponse(
            {"error": f"navigation_console.html bulunamadı: {NAVIGATION_UI_FILE}"},
            status_code=404,
        )
    return FileResponse(
        NAVIGATION_UI_FILE,
        media_type="text/html",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate, max-age=0", "Pragma": "no-cache"},
    )


@app.get("/assets/three.min.js")
async def serve_three_js():
    """Serve Three.js locally so the LiDAR view works on the robot's offline AP."""
    if not os.path.isfile(THREE_JS_FILE):
        return JSONResponse({"error": "three.min.js bulunamadı"}, status_code=404)
    return FileResponse(
        THREE_JS_FILE,
        media_type="application/javascript",
        headers={"Cache-Control": "public, max-age=86400"},
    )

# ─── React SPA static dosyaları (ileride) ────────────────────────────────────
# React SPA build edilince aşağıdaki satırı aç (yukarıdaki "/" route'unu kaldır):
# app.mount("/", StaticFiles(directory="/home/apex/ui/dist", html=True), name="spa")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("apex_server:app", host="0.0.0.0", port=8080, reload=False)
