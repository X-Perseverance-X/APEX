#!/usr/bin/env python3
"""
APEX HUD — Hailo-10H Destekli AI Tespit Sistemi
==================================================
Kullanıcının önceki oturumda yazdığı apex_hud.py'nin GÖRSEL TASARIMI ve
FONKSİYONELLİĞİ korunmuştur (bracket/crosshair/hedef kilitleme). Web HUD
için dikkat dağıtan radar tarama dilimi kaldırılmıştır. Üç şey adapte edildi:

1. cv2.imshow (yerel monitör) → MJPEG HTTP yayını (port 8888). Proje baştan
   sona tarayıcı/telefon üzerinden uzaktan erişim mimarisinde (apex_ui.html),
   yerel monitör hiç kullanılmadı — bu yüzden web'e akıtmak tutarlı.
2. serial_reader() (eski, TERK EDİLMİŞ USB-seri mimarisi, /dev/ttyUSB0 @
   921600 "TEL," satırları) → gerçek mimariye (WiFi HTTP, apex_server.py
   üzerinden) uygun HTTP polling. Eski kod ASLA veri almayacaktı çünkü
   ESP32 hiçbir zaman bu protokolle seri üzerinden TEL satırı göndermedi
   (bu konuşmanın çok başında netleşmişti: ESP32'nin gerçek kontrol/telemetri
   yolu WiFi HTTP'dir, USB-seri komut protokolü hiç yoktu).
3. Sağ/sol 3S batarya ile ön TOF okumaları doğrudan omurilik telemetrisinden
   alınır; sensör çevrimdışıysa HUD sahte değer üretmez ve N/A gösterir.

ÇALIŞTIRMA — kritik, GStreamerDetectionApp CLI argümanlarını KENDİ İÇİNDE
argparse ile sys.argv'den okuyor, bu yüzden DOĞRUDAN komut satırı argümanı
olarak verilmeli (script içinde ayarlamak işe yaramaz):

    python3 detection_hud.py --input usb

--arch BİLEREK verilmedi: donanımdan otomatik algılanıyor
(detect_hailo_arch()) — yanlış yazılmış bir mimari string'i riski ortadan
kalkıyor.
"""
import cv2, numpy as np, threading, math, time, faulthandler, queue, os
import json as jsonlib
import urllib.request
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
faulthandler.enable()
# Raspberry Pi OS Trixie uses Python 3.13.  TAPPAS 5.1 ships its ARM Python
# metadata binding only up to CPython 3.11, while PyHailoRT itself is available
# for 3.13.  Keep the proven GStreamer path for older images, but allow the same
# HUD to run on the direct asynchronous HailoRT API on the new SD image.
try:
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
    import hailo
    from hailo_apps.python.core.gstreamer.gstreamer_app import (
        app_callback_class, get_caps_from_pad, get_numpy_from_buffer,
    )
    from hailo_apps.python.pipeline_apps.detection.detection_pipeline import GStreamerDetectionApp
    GST_HAILO_AVAILABLE = True
except (ImportError, OSError) as exc:
    GST_HAILO_AVAILABLE = False
    GST_HAILO_IMPORT_ERROR = exc
    hailo = None

    class app_callback_class:
        pass

# ─── Konfigürasyon ────────────────────────────────────────────────────────────
STREAM_PORT      = 8888   # camera_stream.py ile AYNI port — apex_ui.html'in
                           # <img> etiketi zaten buraya bağlı, değişiklik gerekmiyor.
JPEG_QUALITY     = int(os.environ.get("APEX_HUD_JPEG_QUALITY", "62"))
APEX_SERVER_URL  = "http://localhost:8080/telemetry"  # gerçek telemetri kaynağı
APEX_VISION_CONTROL_URL = "http://localhost:8080/api/vision/control"
APEX_GIMBAL_URL = "http://localhost:8080/cameraGimbal"

# Kamera fiziksel olarak yeni pan/tilt kafasına dik yerleştirildi. Eski montaj
# için kullanılan 180° dönüş (hflip+vflip) artık görüntüyü baş aşağı yapıyordu.
# Değerler systemd ortamından değiştirilebilir; varsayılan gerçek/non-mirror
# görüntüdür ve seçim koordinatlarıyla Hailo kutuları aynı piksel düzlemindedir.
CAMERA_HFLIP = os.environ.get("APEX_CAMERA_HFLIP", "0") == "1"
CAMERA_VFLIP = os.environ.get("APEX_CAMERA_VFLIP", "0") == "1"

# AI TAKİP parametreleri — GÜVENLİK İÇİN KASITLI OLARAK MUHAFAZAKAR ama artık
# İKİ FAKTÖRE göre ADAPTİF: (1) merkeze olan ofset büyüklüğü (çapraz çizginin
# uzunluğu) — büyükse çok daha hızlı dönsün, küçükse yumuşak/hassas kalsın;
# (2) bounding box boyutundan tahmini mesafe — hedef YAKINSA daha temkinli/
# yavaş (güvenlik + hassasiyet), UZAKSA daha hızlı/agresif dönebilir (zaman/
# alan daha fazla, çarpışma riski daha düşük). İkisi de standart visual-
# servoing + mesafeye göre kazanç ölçekleme (gain scheduling) pratiğine dayanır.
BASE_TURN_GAIN     = 0.65   # taban kazanç — eski sabit kazançtan (0.55) ZATEN yüksek,
                             # "her durumda en az eskisi kadar hızlı" garantisi için
DIST_BOOST_MAX     = 0.9    # hedef UZAKTAYKEN taban kazanca eklenecek max çarpan payı
                             # (uzakken kazanç BASE*(1+0.9)=BASE*1.9'a kadar çıkar)
NEAR_DIST_M        = 1.0    # bu mesafede/altında boost YOK (sadece taban hız — temkinli)
FAR_DIST_M         = 3.0    # bu mesafede/üstünde TAM boost (oda-içi tipik mesafe)
MAX_TRACK_TURN     = 0.85   # mutlak güvenlik tavanı — boost ne olursa olsun bunu geçemez
SEARCH_TURN_SPEED  = 0.25   # hedef kaybolunca arama dönüş hızı (sabit, küçük — değişmedi)
SEARCH_TIMEOUT_S   = 4.0    # kaybolduktan sonra en fazla bu kadar saniye ara, sonra dur
COMMAND_HZ         = 8      # hareket komutu gönderme sıklığı (video FPS'inden BAĞIMSIZ)

# Pan/tilt visual-servo kontrolü. Bacak hareketi üretmez. Mekanik yönler:
# PAN açısı artarken sola, TILT açısı artarken aşağı gider.
GIMBAL_COMMAND_HZ = 10.0
GIMBAL_MIN_DEG = 45.0
GIMBAL_MAX_DEG = 135.0
GIMBAL_CENTER_DEG = 90.0
GIMBAL_PAN_RATE_DPS = 110.0
GIMBAL_TILT_RATE_DPS = 90.0
GIMBAL_X_DEADBAND = 0.045
GIMBAL_Y_DEADBAND = 0.055

# İLERİ HAREKET (yaklaşma) — daha önce SADECE dönüş (joyR) gönderiliyordu,
# hiç ileri/geri yoktu ("ileri gelmiyor" şikayeti tam buradan). GÜVENLİK İÇİN
# KASITLI TEK YÖNLÜ: SADECE yaklaşır, ASLA geri gitmez (arkasını göremiyor).
# Fiziksel doğrulama: kamera ve LiDAR robot önüyle aynı yöndeyken +joyY ileri.
TARGET_FOLLOW_DIST_M = 0.85  # kullanıcı isteği: hedefe daha yakın, fakat temas bölgesine girme
FORWARD_FULL_SPEED_M = 2.80  # bu mesafede ve ötesinde tam ileri hız
MAX_FORWARD_SPEED    = 0.45  # askıda doğrulama öncesi güvenli tavan; uzak hedefte eğri hızla buna çıkar
FORWARD_DEADBAND_M   = 0.10  # bbox mesafe gürültüsünde ileri/stop çırpınmasını önler
TOF_HARD_STOP_MM     = 450   # merkezde herhangi bir engel varsa hedef tahmininden üstün güvenlik
TOF_SLOW_END_MM      = 1000  # bu aralıkta ileri komutu kademeli azalt
DISTANCE_EMA_ALPHA   = 0.28  # bbox yüksekliğinden gelen mesafe titreşimini süzer
FORWARD_ACCEL_PER_S  = 0.55  # komut ivme sınırı; servolara basamak komut göndermez
FORWARD_DECEL_PER_S  = 1.20  # duruş ivmeden daha hızlıdır
TURN_SLEW_PER_S      = 1.60
TRACK_FRAME_MAX_AGE_S = 0.45 # donmuş/kuyrukta kalmış son kare hareket üretemez
CONTROL_MAX_AGE_S     = 0.60 # Pi-local AI açık bilgisinin eskisiyle yürümeyi engeller
PERSON_DISTANCE_K_M   = float(os.environ.get("APEX_PERSON_DISTANCE_K_M", "1.65"))

GREEN=(41,255,0); AMBER=(0,185,255); RED=(0,0,255); WHITE=(255,255,255); DIM=(0,100,25)
LOCK_RADIUS=80
fps_state={'fps':0.0,'count':0,'last':time.time()}
telemetry={
    'battery_r_v': None, 'battery_l_v': None,
    'tof_mm': None, 'tof_online': False,
    'mode':'IDLE', 'connected':False, 'ai_track':False, 'sel':None,
    'mpu_online':False, 'mpu_pitch':0.0, 'mpu_yaw':0.0,
    'camera_pan_deg':90.0, 'camera_tilt_deg':90.0,
    'spine_state':'DISARMED', 'outputs_armed':False,
}
vision_control_seen_at = [0.0]
latest_frame=[None]
latest_frame_ts=[0.0]
latest_jpeg=[None]
latest_jpeg_ts=[0.0]
latest_raw_frame=[None]
latest_raw_frame_ts=[0.0]
process_started_ts=time.time()
frame_lock=threading.Lock()
frame_ready=threading.Condition(frame_lock)
stream_stats={
    'encoded_frames': 0, 'encode_ms': 0.0, 'jpeg_bytes': 0,
    'stream_clients': 0, 'stream_clients_peak': 0,
    'detections': 0, 'detection_labels': [], 'inference_ms': 0.0,
}

# Donma koruması: Hailo/GStreamer pipeline yeni kare üretmeyi keserse eski
# latest_frame bellekte kalır. MJPEG handler bunu sonsuza kadar tekrar ederse
# tarayıcı hata görmez ve arayüz son karede donar. Bu eşikler sadece kamera
# akışını etkiler; joystick/ESP/LiDAR tarafına dokunmaz.
STREAM_STALE_TIMEOUT_S = 2.5
PIPELINE_WATCHDOG_S    = 12.0
STARTUP_GRACE_S        = 45.0

# AĞIR draw_hud() İŞLEMİNİ GStreamer CALLBACK'İNDEN AYIRMAK İÇİN — bu
# projede zaten kanıtlanmış desen (LiDAR/SLAM ayrıştırmasıyla AYNI mantık).
# maxsize=1: kuyrukta hep SADECE en taze kare tutulur, GStreamer callback'i
# bekletilmesin (eski/işlenmemiş kareler atılır, backlog büyümesin).
_raw_frame_queue: "queue.Queue" = queue.Queue(maxsize=1)

# Takip durumu — hud_callback() (video iş parçacığı) yazar, tracking_commander()
# (ayrı, video hızından bağımsız bir thread) okuyup hareket komutu üretir.
# Bu ayrım BİLEREK yapıldı: HTTP isteği video callback'ini asla bloklamasın
# (aynı SLAM/LiDAR ayrıştırmasında kullanılan desen — bu projede zaten kanıtlanmış).
tracking_state = {
    'visible': False, 'bx': None, 'by': None, 'dm': None,
    'frame_w': 640, 'frame_h': 480, 'last_seen': 0.0,
}
tracking_lock = threading.Lock()
gimbal_control_lock = threading.Lock()
gimbal_control_state = {
    'mode': 'stabilize', 'pan': GIMBAL_CENTER_DEG, 'tilt': GIMBAL_CENTER_DEG,
    'last_error_x': 0.0, 'last_error_y': 0.0, 'last_command': 0.0,
}

# Hedef SEÇİMİ — kullanıcı arayüzde bir kişiye tıklayınca o kişinin Hailo
# track_id'si buraya kilitlenir. None = henüz kimse seçilmedi (bu durumda
# AI TAKİP açık olsa bile robot hareket ETMEZ — "her insanı değil, seçileni
# takip etsin" isteği tam burada karşılanıyor).
selected_track_id = None
selected_bbox = None
last_processed_sel_ts = 0.0
selection_lock = threading.Lock()
selection_status = {
    'state': 'none', 'track_id': None, 'visible': False,
    'processed_ts': 0.0, 'last_seen': 0.0, 'bbox': None,
}
visual_target = {'patch': None, 'center': None, 'last_seen': 0.0, 'label': None}


def visual_frame(frame):
    h, w = frame.shape[:2]
    return cv2.cvtColor(cv2.resize(frame, (320, max(1, round(h * 320 / w)))), cv2.COLOR_BGR2GRAY)


def initialize_visual_target(frame, x, y):
    """Lock a textured patch at any clicked image point without a detector box."""
    gray = visual_frame(frame)
    size = 32
    if gray.shape[0] < size or gray.shape[1] < size:
        return None
    cx = max(size // 2, min(gray.shape[1] - size // 2, round(x * gray.shape[1])))
    cy = max(size // 2, min(gray.shape[0] - size // 2, round(y * gray.shape[0])))
    patch = gray[cy-size//2:cy+size//2, cx-size//2:cx+size//2].copy()
    if patch.std() < 4.0:
        return None
    visual_target.update({'patch': patch, 'center': (cx, cy), 'last_seen': time.time()})
    return ((cx-size//2)/gray.shape[1], (cy-size//2)/gray.shape[0],
            (cx+size//2)/gray.shape[1], (cy+size//2)/gray.shape[0])


def follow_visual_target(frame):
    """Search near the last position; stop on poor match instead of drifting."""
    patch, center = visual_target['patch'], visual_target['center']
    if patch is None or center is None:
        return None
    gray = visual_frame(frame)
    size = patch.shape[0]
    score = -1.0
    position = (0, 0)
    x0 = y0 = 0
    # A brief camera shake can move the target outside the normal window.
    # Retry a wider region only after the cheap local match fails.
    for radius, threshold in ((72, 0.62), (120, 0.70)):
        x0 = max(0, center[0] - radius - size//2)
        y0 = max(0, center[1] - radius - size//2)
        x1 = min(gray.shape[1], center[0] + radius + size//2)
        y1 = min(gray.shape[0], center[1] + radius + size//2)
        search = gray[y0:y1, x0:x1]
        if search.shape[0] < size or search.shape[1] < size:
            return None
        _, score, _, position = cv2.minMaxLoc(cv2.matchTemplate(search, patch, cv2.TM_CCOEFF_NORMED))
        if score >= threshold:
            break
    else:
        return None
    left, top = x0 + position[0], y0 + position[1]
    if score > 0.85:
        observed = gray[top:top+size, left:left+size]
        visual_target['patch'] = cv2.addWeighted(patch, 0.92, observed, 0.08, 0)
    visual_target['center'] = (left + size//2, top + size//2)
    visual_target['last_seen'] = time.time()
    bbox = (left/gray.shape[1], top/gray.shape[0],
            (left+size)/gray.shape[1], (top+size)/gray.shape[0])
    return bbox, score

def telemetry_poller():
    """Eski serial_reader()'ın yerini alıyor — gerçek mimariye (WiFi HTTP,
    apex_server.py /telemetry) uygun. Sağ/sol batarya ve öne bakan TOF
    değerleri aynı örnekten alınır; eksik/çevrimdışı alanlar uydurulmaz."""
    last_error_log = 0.0
    was_connected = False
    while True:
        try:
            with urllib.request.urlopen(APEX_SERVER_URL, timeout=1.5) as resp:
                data = jsonlib.loads(resp.read().decode('utf-8'))
            telemetry['mode'] = data.get('mode', 'IDLE')
            telemetry['battery_r_v'] = data.get('batR') if isinstance(data.get('batR'), (int, float)) and data.get('batR') > 1.0 else None
            telemetry['battery_l_v'] = data.get('batL') if isinstance(data.get('batL'), (int, float)) and data.get('batL') > 1.0 else None
            telemetry['tof_online'] = bool(data.get('tofOnline', False))
            tof_mm = data.get('tofMm')
            telemetry['tof_mm'] = tof_mm if telemetry['tof_online'] and isinstance(tof_mm, (int, float)) and 0 <= tof_mm < 65535 else None
            telemetry['mpu_online'] = bool(data.get('mpuOnline', False))
            telemetry['mpu_pitch'] = float(data.get('mpuP', 0.0) or 0.0)
            telemetry['mpu_yaw'] = float(data.get('mpuY', 0.0) or 0.0)
            telemetry['camera_pan_deg'] = float(data.get('cameraPanDeg', GIMBAL_CENTER_DEG) or GIMBAL_CENTER_DEG)
            telemetry['camera_tilt_deg'] = float(data.get('cameraTiltDeg', GIMBAL_CENTER_DEG) or GIMBAL_CENTER_DEG)
            telemetry['spine_state'] = data.get('spineState', 'DISARMED')
            telemetry['outputs_armed'] = bool(data.get('outputsArmed', False))
            telemetry['connected'] = True
            if not was_connected:
                print('[HUD] Telemetri bağlantısı aktif.', flush=True)
            was_connected = True
        except Exception as e:
            telemetry['connected'] = False
            telemetry['outputs_armed'] = False
            now = time.monotonic()
            # Omurilik kapalıyken beklenen timeout'u her iki saniyede bir
            # diske yazmak kamera sürecine gereksiz I/O yükü bindiriyordu.
            if was_connected or now - last_error_log >= 30.0:
                print('[HUD] Telemetri hatasi:', e, flush=True)
                last_error_log = now
            was_connected = False
        time.sleep(0.5)

def vision_control_poller():
    """Selection and AI-control state must not depend on ESP32 telemetry.

    The spine may be powered off while vision is being configured. Polling a
    Pi-local endpoint keeps click-to-lock responsive in that safe state.
    """
    last_error_log = 0.0
    was_connected = False
    while True:
        try:
            with urllib.request.urlopen(APEX_VISION_CONTROL_URL, timeout=0.6) as resp:
                data = jsonlib.loads(resp.read().decode('utf-8'))
            telemetry['ai_track'] = bool(data.get('ai_tracking', False))
            telemetry['sel'] = data.get('selection', None)
            vision_control_seen_at[0] = time.monotonic()
            if not was_connected:
                print('[AI-TRACK] Kontrol bağlantısı aktif.', flush=True)
            was_connected = True
        except Exception as e:
            now = time.monotonic()
            if now - vision_control_seen_at[0] > CONTROL_MAX_AGE_S:
                telemetry['ai_track'] = False
            if was_connected or now - last_error_log >= 15.0:
                print('[AI-TRACK] Kontrol durumu okunamadi:', e, flush=True)
                last_error_log = now
            was_connected = False
        time.sleep(0.1)

def send_gimbal_target(pan, tilt):
    """Atomik pan/tilt hedefi; yalnız PCA 0x40 CH15/CH14 hareket eder."""
    try:
        pan = max(GIMBAL_MIN_DEG, min(GIMBAL_MAX_DEG, float(pan)))
        tilt = max(GIMBAL_MIN_DEG, min(GIMBAL_MAX_DEG, float(tilt)))
        url = f"{APEX_GIMBAL_URL}?pan={pan:.1f}&tilt={tilt:.1f}"
        urllib.request.urlopen(url, timeout=0.8).read()
    except Exception as e:
        print('[AI-TRACK] Gimbal hedefi gonderilemedi:', e)

def send_gimbal_mode(action):
    """CENTER/STABILIZE geçişi; bacak hareket kanalına erişmez."""
    try:
        urllib.request.urlopen(f"{APEX_GIMBAL_URL}?action={action}", timeout=0.8).read()
    except Exception as e:
        print('[AI-TRACK] Gimbal modu gonderilemedi:', e)


def send_body_target(pitch, yaw, ai=True):
    """Small body IK compensation, subject to the command center's fresh safety gate."""
    source = 'ai' if ai else 'manual'
    url = f'http://localhost:8080/bodyIk?p={pitch:.1f}&r=0&y={yaw:.1f}&source={source}'
    try:
        with urllib.request.urlopen(url, timeout=0.7) as response:
            return bool(jsonlib.loads(response.read().decode('utf-8')).get('ok'))
    except Exception:
        return False

def compute_forward(dm, err, tof_mm=None):
    """Mesafeye göre ileri hız — SADECE yaklaşma, ASLA geri gitme (robot
    arkasını göremiyor, güvenlik için tek yönlü). Hedef merkezden uzaktaysa
    (err büyük) ileri gitmeyi azaltır — önce dönüp hizalansın, sonra düz
    yaklaşsın (çapraz/yamuk hareket etmesin)."""
    if dm is None or dm <= 0:
        return 0.0
    distance_error = dm - TARGET_FOLLOW_DIST_M
    if distance_error <= FORWARD_DEADBAND_M:
        return 0.0
    span = max(0.01, FORWARD_FULL_SPEED_M - TARGET_FOLLOW_DIST_M - FORWARD_DEADBAND_M)
    phase = max(0.0, min(1.0, (distance_error - FORWARD_DEADBAND_M) / span))
    # Smoothstep: yakında hassas ve yavaş, uzakta hızla tam komuta çıkar;
    # sınırların iki yanında türev sıfır olduğu için hız sıçramaz.
    raw = MAX_FORWARD_SPEED * phase * phase * (3.0 - 2.0 * phase)
    align_factor = max(0.0, 1.0 - (abs(err) / 0.72) ** 2)
    if isinstance(tof_mm, (int, float)):
        if tof_mm <= TOF_HARD_STOP_MM:
            return 0.0
        if tof_mm < TOF_SLOW_END_MM:
            raw *= (tof_mm - TOF_HARD_STOP_MM) / (TOF_SLOW_END_MM - TOF_HARD_STOP_MM)
    return raw * align_factor

def slew_towards(current, target, rise_per_s, fall_per_s, dt):
    """Komut basamağını zaman tabanlı sınırlar; duruş her zaman hızlanmadan hızlıdır."""
    rate = rise_per_s if abs(target) > abs(current) else fall_per_s
    delta = max(-rate * dt, min(rate * dt, target - current))
    return current + delta

def compute_turn(err, dm):
    """Hem ofset büyüklüğüne (err, -1..1, çapraz çizginin uzunluğu/yönü) HEM
    tahmini mesafeye (dm, metre, bbox boyutundan) göre ADAPTİF dönüş hızı
    hesaplar. Saf fonksiyon — thread'den bağımsız, izole test edilebilir.

    Tasarım: TABAN kazanç (BASE_TURN_GAIN) zaten eski sabit kazançtan
    yüksek — yani hiçbir durumda eskisinden daha yavaş olmaz. Mesafe
    SADECE EK bir hız artışı sağlar (çarpan >=1.0) — hedef uzaklaştıkça
    ("içine aldığı dikdörtgen küçüldükçe") kazanç daha da büyür, hedef
    yakınken ekstra artış uygulanmaz (güvenlik/hassasiyet) ama TABANIN
    ALTINA hiç düşmez.
    """
    err = max(-1.0, min(1.0, err))
    if abs(err) < 0.04:
        return 0.0
    if dm is None or dm <= 0:
        dist_factor = 0.0   # mesafe bilinmiyorsa boost yok, sadece taban hız
    else:
        dist_factor = (dm - NEAR_DIST_M) / (FAR_DIST_M - NEAR_DIST_M)
        dist_factor = max(0.0, min(1.0, dist_factor))
    boost = 1.0 + dist_factor * DIST_BOOST_MAX
    turn = err * BASE_TURN_GAIN * boost
    return max(-MAX_TRACK_TURN, min(MAX_TRACK_TURN, turn))

def tracking_commander():
    """Seçili kişiyi pan/tilt ile görüntü merkezinde tutar.

    Bu iş parçacığı bilinçli olarak /joy kullanmaz; robot bacakları DISARMED
    durumda kalabilir. Takip kapalıyken kafa 90°/90° merkeze döner. MPU6050
    yaw sürüklenmesi hareketsiz robotta sahte sağ/sol bakış üretmemelidir.
    """
    command_pan = GIMBAL_CENTER_DEG
    command_tilt = GIMBAL_CENTER_DEG
    last_mode = None
    last_command_at = time.monotonic()
    body_pitch = body_yaw = 0.0
    body_owned = False
    last_body_command = 0.0
    last_seen_at = 0.0
    last_err_x = last_err_y = 0.0
    while True:
        time.sleep(1.0 / GIMBAL_COMMAND_HZ)
        tracking_enabled = bool(telemetry.get('ai_track', False))
        with tracking_lock:
            visible   = tracking_state['visible']
            bx        = tracking_state['bx']
            by        = tracking_state['by']
            frame_w   = tracking_state['frame_w']
            frame_h   = tracking_state['frame_h']
            last_seen = tracking_state['last_seen']
        frame_fresh = bool(last_seen and (time.time() - last_seen) <= TRACK_FRAME_MAX_AGE_S)
        control_fresh = (time.monotonic() - vision_control_seen_at[0]) <= CONTROL_MAX_AGE_S

        if tracking_enabled and visible and bx is not None and by is not None and frame_fresh and control_fresh:
            if last_mode != 'track':
                command_pan = max(GIMBAL_MIN_DEG, min(
                    GIMBAL_MAX_DEG, float(telemetry.get('camera_pan_deg', GIMBAL_CENTER_DEG))))
                command_tilt = max(GIMBAL_MIN_DEG, min(
                    GIMBAL_MAX_DEG, float(telemetry.get('camera_tilt_deg', GIMBAL_CENTER_DEG))))
            tick = time.monotonic()
            dt = max(0.02, min(0.20, tick - last_command_at))
            last_command_at = tick
            err_x = (bx - frame_w * 0.5) / max(1.0, frame_w * 0.5)
            err_y = (by - frame_h * 0.5) / max(1.0, frame_h * 0.5)
            if abs(err_x) < GIMBAL_X_DEADBAND:
                err_x = 0.0
            if abs(err_y) < GIMBAL_Y_DEADBAND:
                err_y = 0.0
            last_err_x, last_err_y = err_x, err_y
            last_seen_at = tick
            # Hedef görüntüde sağdaysa mekanizma sağa dönmeli: PAN açısı azalır.
            # Hedef aşağıdaysa TILT açısı artar.
            command_pan = max(GIMBAL_MIN_DEG, min(
                GIMBAL_MAX_DEG, command_pan - err_x * GIMBAL_PAN_RATE_DPS * dt))
            command_tilt = max(GIMBAL_MIN_DEG, min(
                GIMBAL_MAX_DEG, command_tilt + err_y * GIMBAL_TILT_RATE_DPS * dt))
            send_gimbal_target(command_pan, command_tilt)
            if (telemetry.get('connected') and telemetry.get('outputs_armed') and
                    telemetry.get('spine_state') == 'STAND' and tick - last_body_command >= 0.25):
                pan_actual = telemetry.get('camera_pan_deg', 90.0)
                tilt_actual = telemetry.get('camera_tilt_deg', 90.0)
                yaw_direction = (-1 if err_x > 0.08 and pan_actual <= 48 else
                                  1 if err_x < -0.08 and pan_actual >= 132 else 0)
                pitch_direction = (1 if err_y < -0.08 and tilt_actual <= 48 else
                                   -1 if err_y > 0.08 and tilt_actual >= 132 else 0)
                next_pitch = max(-8.0, min(8.0, body_pitch + pitch_direction * 3.0 * dt))
                next_yaw = max(-8.0, min(8.0, body_yaw + yaw_direction * 3.0 * dt))
                if (abs(next_pitch - body_pitch) >= 0.2 or abs(next_yaw - body_yaw) >= 0.2) and send_body_target(next_pitch, next_yaw):
                    body_pitch, body_yaw = next_pitch, next_yaw
                    body_owned = True
                    last_body_command = tick
            last_mode = 'track'
            with gimbal_control_lock:
                gimbal_control_state.update({
                    'mode': 'track', 'pan': command_pan, 'tilt': command_tilt,
                    'last_error_x': err_x, 'last_error_y': err_y,
                    'last_command': time.time(),
                })
        else:
            # Takip kapalı veya hedef kayıp: baş merkezde. Aynı komutu
            # sürekli yollama; MPU yaw driftine göre sağa/sola gezinme.
            # Kısa algılama boşluğunda son bakış yönünü koru; hemen merkeze
            # dönmek hedefi kadrajın kenarında tekrar bulmayı engelliyordu.
            with selection_lock:
                selected_recently = (selection_status.get('state') == 'lost' and
                                     time.time() - selection_status.get('last_seen', 0.0) < 1.2)
            if tracking_enabled and selected_recently and last_mode == 'track':
                tick = time.monotonic()
                if (tick - last_seen_at < 0.6 and tick - last_body_command >= 0.25 and
                        telemetry.get('outputs_armed') and telemetry.get('spine_state') == 'STAND'):
                    yaw_direction = -1 if last_err_x > 0.08 and command_pan <= 48 else 1 if last_err_x < -0.08 and command_pan >= 132 else 0
                    pitch_direction = 1 if last_err_y < -0.08 and command_tilt <= 48 else -1 if last_err_y > 0.08 and command_tilt >= 132 else 0
                    next_pitch = max(-8.0, min(8.0, body_pitch + pitch_direction * 0.6))
                    next_yaw = max(-8.0, min(8.0, body_yaw + yaw_direction * 0.6))
                    if (next_pitch != body_pitch or next_yaw != body_yaw) and send_body_target(next_pitch, next_yaw):
                        body_pitch, body_yaw = next_pitch, next_yaw
                        body_owned = True
                        last_body_command = tick
                continue
            if body_owned and tracking_enabled and time.monotonic() - last_body_command >= 0.25:
                if send_body_target(0.0, 0.0):
                    body_pitch = body_yaw = 0.0
                    body_owned = False
                    last_body_command = time.monotonic()
            if body_owned and not tracking_enabled:
                if send_body_target(0.0, 0.0, ai=False):
                    body_pitch = body_yaw = 0.0
                    body_owned = False
            if last_mode != 'center':
                send_gimbal_mode('center')
                command_pan = GIMBAL_CENTER_DEG
                command_tilt = GIMBAL_CENTER_DEG
                last_mode = 'center'
                with gimbal_control_lock:
                    gimbal_control_state.update({
                        'mode': 'center', 'pan': command_pan, 'tilt': command_tilt,
                        'last_error_x': 0.0, 'last_error_y': 0.0,
                        'last_command': time.time(),
                    })
            last_command_at = time.monotonic()

def bracket(img,x1,y1,x2,y2,col,ln=30,th=3):
    for bx,by,dx,dy in [(x1,y1,1,1),(x2,y1,-1,1),(x1,y2,1,-1),(x2,y2,-1,-1)]:
        cv2.line(img,(bx,by),(bx+dx*ln,by),col,th)
        cv2.line(img,(bx,by),(bx,by+dy*ln),col,th)

def crosshair(img,cx,cy,col,sz=42,gap=12):
    """Sade UAV tipi merkez nişangâhı; hareketli radar/tarama içermez."""
    cv2.line(img,(cx-sz,cy),(cx-gap,cy),col,2)
    cv2.line(img,(cx+gap,cy),(cx+sz,cy),col,2)
    cv2.line(img,(cx,cy-sz),(cx,cy-gap),col,2)
    cv2.line(img,(cx,cy+gap),(cx,cy+sz),col,2)
    cv2.circle(img,(cx,cy),gap+5,col,1)
    cv2.circle(img,(cx,cy),2,col,-1)
    corner=sz+15; arm=11
    for x,y,dx,dy in ((cx-corner,cy-corner,1,1),(cx+corner,cy-corner,-1,1),
                      (cx-corner,cy+corner,1,-1),(cx+corner,cy+corner,-1,-1)):
        cv2.line(img,(x,y),(x+dx*arm,y),col,1)
        cv2.line(img,(x,y),(x,y+dy*arm),col,1)

def battery_tile(img,x,y,v,label='BATTERY',w=165):
    # v=None → sensör/veri yok; dürüstçe N/A.
    if v is None:
        tile(img,x,y,w,40,label,'N/A',(90,90,90),label_col=(90,90,90))
        return
    pct=max(0.0,min(1.0,(v-9.0)/3.6)) if v>1 else 0.0
    bc=GREEN if pct>0.5 else(AMBER if pct>0.2 else RED)
    tile(img,x,y,w,40,label,f'{v:.1f}V  {int(pct*100)}%',bc)

def front_tof_readout(img,cx,cy):
    """Ön TOF, optik/robot ileri eksenindeki merkez mesafesidir (3B konum değil)."""
    tof_mm = telemetry.get('tof_mm')
    online = telemetry.get('tof_online', False)
    value = f'FRONT TOF  {int(tof_mm)} mm' if online and tof_mm is not None else 'FRONT TOF  N/A'
    col = AMBER if online and tof_mm is not None else (90,90,90)
    (tw,th),_=cv2.getTextSize(value,cv2.FONT_HERSHEY_SIMPLEX,0.48,1)
    tx=max(8,min(img.shape[1]-tw-8,cx-tw//2))
    ty=cy+72
    cv2.rectangle(img,(tx-6,ty-th-6),(tx+tw+6,ty+6),(0,0,0),-1)
    cv2.rectangle(img,(tx-6,ty-th-6),(tx+tw+6,ty+6),col,1)
    put(img,value,tx,ty,col,0.48,1)

def tile(img, x, y, w, h, label, value, val_col, label_col=(170,170,170)):
    """Endüstriyel telemetri kutusu — üstte küçük/soluk etiket, altta büyük/
    kalın/renkli değer. Tespit (person/tracking) etiketlerinden BAĞIMSIZ,
    sadece sensör/pil/sistem bilgisi alanları için kullanılır."""
    cv2.rectangle(img,(x,y),(x+w,y+h),(8,8,8),-1)
    cv2.rectangle(img,(x,y),(x+w,y+h),val_col,2)
    cv2.line(img,(x,y),(x+w,y),val_col,3)  # üst kenar vurgusu — panel hissi
    cv2.putText(img,label,(x+8,y+16),cv2.FONT_HERSHEY_SIMPLEX,0.38,label_col,1)
    cv2.putText(img,value,(x+7,y+h-9),cv2.FONT_HERSHEY_DUPLEX,0.62,(0,0,0),3)
    cv2.putText(img,value,(x+8,y+h-10),cv2.FONT_HERSHEY_DUPLEX,0.62,val_col,1)

def put(img,s,x,y,col,sz=0.6,th=1):
    cv2.putText(img,s,(x+1,y+1),cv2.FONT_HERSHEY_SIMPLEX,sz,(0,0,0),th+2)
    cv2.putText(img,s,(x,y),cv2.FONT_HERSHEY_SIMPLEX,sz,col,th)


def process_target_selection(dets, sel, frame=None):
    """Yeni bir tıklama/temizleme olayı var mı kontrol eder, varsa
    selected_track_id'yi günceller. Saf mantık dışarı çıkarıldı ki izole
    test edilebilsin (thread/global'den ayrı).

    dets: [{'label','track_id','bbox':(x1,y1,x2,y2) normalize 0-1}, ...]
    sel: telemetry'den gelen {'pending','x','y','ts','clear'} ya da None

    Döner: (yeni_selected_track_id, islendi_mi)
    """
    global selected_track_id, selected_bbox, last_processed_sel_ts
    if not sel or sel.get('ts', 0) <= last_processed_sel_ts:
        return selected_track_id, False
    last_processed_sel_ts = sel['ts']
    if sel.get('clear'):
        visual_target.update({'patch': None, 'center': None, 'last_seen': 0.0, 'label': None})
        with selection_lock:
            visual_target['label'] = None
            selected_track_id = None
            selected_bbox = None
            selection_status.update({
                'state': 'cleared', 'track_id': None, 'visible': False,
                'processed_ts': time.time(), 'last_seen': 0.0, 'bbox': None,
            })
        with tracking_lock:
            tracking_state.update({'visible': False, 'bx': None, 'by': None, 'dm': None, 'last_seen': 0.0})
        return None, True

    click_x, click_y = float(sel.get('x', 0.0)), float(sel.get('y', 0.0))
    candidates = []
    for d in dets:
        if d.get('confidence', 0.0) < 0.35:
            continue
        x1, y1, x2, y2 = d['bbox']
        # MJPEG görüntüsü ile en yeni inference karesi arasında küçük bir zaman
        # farkı olabilir. %4 tolerans, hareket eden kişiye yapılan kenar tıklarını
        # kabul eder; en yakın merkez seçildiği için başka kişiye rastgele atlamaz.
        pad = 0.04
        if x1-pad <= click_x <= x2+pad and y1-pad <= click_y <= y2+pad:
            cx, cy = (x1+x2)/2.0, (y1+y2)/2.0
            candidates.append(((cx-click_x)**2 + (cy-click_y)**2, d))

    with selection_lock:
        if not candidates:
            visual_bbox = initialize_visual_target(frame, click_x, click_y) if frame is not None else None
            selected_track_id = None
            selected_bbox = visual_bbox
            selection_status.update({
                'state': 'locked' if visual_bbox else 'missed', 'track_id': None,
                'visible': bool(visual_bbox), 'processed_ts': time.time(),
                'last_seen': time.time() if visual_bbox else 0.0,
                'bbox': list(visual_bbox) if visual_bbox else None,
            })
            with tracking_lock:
                tracking_state.update({'visible': False, 'bx': None, 'by': None, 'dm': None, 'last_seen': 0.0})
            return None, True
        candidate = min(candidates, key=lambda item: item[0])[1]
        stable_id = candidate.get('track_id') if candidate.get('label') == 'person' else None
        visual_bbox = (initialize_visual_target(frame, click_x, click_y)
                       if stable_id is None and frame is not None else None)
        if stable_id is not None or visual_bbox is None:
            visual_target.update({'patch': None, 'center': None, 'last_seen': 0.0, 'label': None})
        else:
            visual_target['label'] = candidate.get('label')
        selected_track_id = stable_id
        selected_bbox = visual_bbox or tuple(candidate['bbox'])
        selection_status.update({
            'state': 'locked', 'track_id': selected_track_id, 'visible': True,
            'processed_ts': time.time(), 'last_seen': time.time(),
            'bbox': list(selected_bbox),
        })
    return selected_track_id, True

def bbox_iou(a, b):
    if not a or not b:
        return 0.0
    ax1, ay1, ax2, ay2 = a; bx1, by1, bx2, by2 = b
    iw = max(0.0, min(ax2,bx2)-max(ax1,bx1))
    ih = max(0.0, min(ay2,by2)-max(ay1,by1))
    inter = iw*ih
    union = max(1e-9, (ax2-ax1)*(ay2-ay1)+(bx2-bx1)*(by2-by1)-inter)
    return inter/union

def draw_hud(frame,dets,fps):
    global selected_bbox
    h,w=frame.shape[:2]; cx,cy=w//2,h//2
    process_target_selection(dets, telemetry.get('sel'), frame)
    visual_match = follow_visual_target(frame) if visual_target['patch'] is not None else None
    if visual_match is None and visual_target['patch'] is not None and visual_target['label']:
        # Reacquire the SAME detector class near the last image location.
        # Reinitializing the template from an unrelated box caused target swaps.
        last_x, last_y = visual_target['center']
        gray_h = max(1, round(frame.shape[0] * 320 / frame.shape[1]))
        candidates = [d for d in dets if d.get('label') == visual_target['label']
                      and d.get('confidence', 0.0) >= 0.45
                      and abs((d['bbox'][0] + d['bbox'][2]) * 160 - last_x) < 48
                      and abs((d['bbox'][1] + d['bbox'][3]) * gray_h / 2 - last_y) < 36]
        if len(candidates) == 1:
            box = candidates[0]['bbox']
            target_x, target_y = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            recovered = initialize_visual_target(frame, target_x, target_y)
            if recovered is not None:
                visual_target['label'] = candidates[0]['label']
                visual_match = (recovered, candidates[0]['confidence'])
    cv2.rectangle(frame,(6,6),(w-6,h-6),DIM,1)
    crosshair(frame,cx,cy,AMBER)
    front_tof_readout(frame,cx,cy)
    cv2.rectangle(frame,(0,0),(w,48),(0,0,0),-1)
    cv2.line(frame,(0,48),(w,48),DIM,1)
    put(frame,'APEX ROBOTICS',18,34,GREEN,0.72,2)
    put(frame,'TARGETING SYSTEM',215,34,DIM,0.42,1)
    put(frame,datetime.now().strftime('%H:%M:%S'),365,34,WHITE,0.62,1)
    put(frame,f'FPS: {fps:.1f}',w-110,34,GREEN,0.62,1)
    put(frame,'[ SAFE ]',18,78,GREEN,0.72,2)
    tile(frame,18,90,140,40,'MODE',telemetry['mode'],(0,200,255))
    if not telemetry.get('ai_track',False):
        ai_val, ai_col = 'OFF', (90,90,90)
    elif selected_track_id is None and selected_bbox is None:
        ai_val, ai_col = 'NO TARGET', AMBER
    else:
        ai_val = f'ID #{selected_track_id}' if selected_track_id is not None else 'LOCKED'
        ai_col = (0,140,255)
    tile(frame,164,90,150,40,'AI TRACK', ai_val, ai_col,
         label_col=(170,170,170) if telemetry.get('ai_track',False) else (90,90,90))
    cc=GREEN if telemetry['connected'] else RED
    if w >= 960:
        battery_tile(frame,w-355,52,telemetry.get('battery_r_v'),'BATTERY RIGHT')
        battery_tile(frame,w-175,52,telemetry.get('battery_l_v'),'BATTERY LEFT')
        tile(frame,w-175,96,165,38,'SPINE',
             'ONLINE' if telemetry['connected'] else 'OFFLINE', cc)
    else:
        rv=telemetry.get('battery_r_v'); lv=telemetry.get('battery_l_v')
        power_val=f"R {rv:.1f}V  L {lv:.1f}V" if rv is not None and lv is not None else 'R N/A  L N/A'
        power_col=GREEN if rv is not None and lv is not None else (90,90,90)
        tile(frame,w-220,52,210,40,'BATTERY 3S RIGHT / LEFT',power_val,power_col)
        tile(frame,w-175,96,165,38,'SPINE',
             'ONLINE' if telemetry['connected'] else 'OFFLINE', cc)
    best=None
    best_person=None  # AI TAKİP SADECE bunu kullanır — ARTIK "merkeze en yakın kişi"
                       # DEĞİL, kullanıcının SEÇTİĞİ (tıkladığı) belirli track_id.
                       # Hiç kimse seçilmediyse best_person hep None kalır — robot
                       # rastgele bir insanı takip etmeye BAŞLAMAZ.
    for det in dets:
        if det['confidence']<0.35: continue
        bb=det['bbox']
        x1,y1,x2,y2=int(bb[0]*w),int(bb[1]*h),int(bb[2]*w),int(bb[3]*h)
        bx,by=(x1+x2)//2,(y1+y2)//2
        dc=math.hypot(bx-cx,by-cy); locked=dc<LOCK_RADIUS
        id_match = selected_track_id is not None and det.get('track_id') == selected_track_id
        bbox_match = visual_target['patch'] is None and selected_track_id is None and selected_bbox is not None and bbox_iou(det['bbox'], selected_bbox) >= 0.18
        is_selected = id_match or bbox_match
        col=(0,140,255) if is_selected else (RED if locked else AMBER)
        # Sistem şeridini tespit köşelerinden de koru. Gerçek bbox ve
        # takip merkezi değişmez; yalnızca ekrana çizilen üst kenar kırpılır.
        draw_y1 = max(140, y1)
        if y2 > draw_y1:
            bracket(frame,x1-1,draw_y1-1,x2+1,y2+1,(0,0,0),32,5)
            bracket(frame,x1,draw_y1,x2,y2,col,30,3)
        cv2.circle(frame,(bx,by),5,(0,0,0),6)
        cv2.circle(frame,(bx,by),4,col,-1)
        lbl=f"{det['label'].upper()}  {det['confidence']:.0%}"
        if is_selected: lbl = "* SECILI HEDEF * " + lbl
        (lw,lh),_=cv2.getTextSize(lbl,cv2.FONT_HERSHEY_SIMPLEX,0.55,1)
        # ÖNEMLİ: etiket üst durum kutularının (MODE/AI TRACK/BATTERY/SPINE,
        # y=90-130 bölgesi) İÇİNE girmesin — kişi kutusu üst kenara yakınsa
        # eskiden tam üstlerine biniyordu (test render'ında yakalandı).
        # Görsel stil/padding AYNI, sadece dikey taban konumu sınırlanıyor.
        # Üst telemetri şeridi y=52..134 arasını kullanır. Tespit metni
        # bu bölgenin altına alınır ve sağ kenardan taşması engellenir.
        lbl_baseline = max(154, y1-6)
        label_x = max(6, min(w-lw-10, x1+4))
        cv2.rectangle(frame,(label_x-4,lbl_baseline-lh-10),(label_x+lw+4,lbl_baseline+4),(0,0,0),-1)
        put(frame,lbl,label_x,lbl_baseline,col,0.55,1)
        # Normalized bbox yüksekliği kullanıldığı için HUD render downscale'ı
        # mesafe tahminini değiştirmez. K sabiti sahada kalibre edilebilir.
        bh_ratio=max(0.0, float(bb[3]-bb[1])); dm=(PERSON_DISTANCE_K_M/bh_ratio) if bh_ratio>0.01 else 0
        dm=max(0.0,min(6.0,dm))
        if dm>0: put(frame,f'{dm:.1f}m',x1,y2+24,col,0.55,1)
        # ÖNEMLİ DAVRANIŞ: bir hedef SEÇİLİYSE, dramatik kilitleme görseli
        # (TARGET LOCKED banner'ı, AZ/DIST okuması, nabız atan kırmızı daire)
        # ARTIK SADECE o seçili hedefe ait olsun. Önceden bu görsel "merkeze
        # en yakın HERHANGİ bir nesne" mantığıyla çalışıyordu — seçimden
        # TAMAMEN bağımsızdı. Sen hareket edip merkezden uzaklaşınca, daha
        # merkezi duran biri (Cahit gibi) bu görseli kendine çekiyordu —
        # "her yere kilitleniyor gibi" şikayetinin asıl sebebi tam buydu.
        if selected_track_id is not None or selected_bbox is not None:
            if is_selected:
                best = {'bx':bx,'by':by,'label':det['label'],'conf':det['confidence'],'locked':locked,'dm':dm}
        if is_selected:
            best_person={'bx':bx,'by':by,'dm':dm}
            with selection_lock:
                selected_bbox = tuple(det['bbox'])
                selection_status.update({
                    'state': 'locked', 'track_id': selected_track_id,
                    'visible': True, 'last_seen': time.time(),
                    'bbox': list(selected_bbox),
                })

    if visual_match is not None:
        visual_bbox, visual_score = visual_match
        x1, y1, x2, y2 = (int(visual_bbox[0]*w), int(visual_bbox[1]*h),
                          int(visual_bbox[2]*w), int(visual_bbox[3]*h))
        bracket(frame, x1, y1, x2, y2, (0, 140, 255), 24, 2)
        best_person = {'bx': (x1+x2)//2, 'by': (y1+y2)//2, 'dm': None}
        best = {'bx': best_person['bx'], 'by': best_person['by'],
                'label': 'visual', 'conf': visual_score, 'locked': False, 'dm': 0.0}
        with selection_lock:
            selected_bbox = visual_bbox
            selection_status.update({'state': 'locked', 'visible': True,
                                     'last_seen': time.time(), 'bbox': list(visual_bbox)})

    # Takip durumunu güncelle — tracking_commander() thread'i bunu okuyup
    # hareket komutu üretecek. SADECE seçili kişi, SADECE bx/zaman bilgisi.
    with tracking_lock:
        if best_person is not None:
            tracking_state['visible']   = True
            tracking_state['bx']        = best_person['bx']
            tracking_state['by']        = best_person['by']
            tracking_state['dm']        = best_person['dm']
            tracking_state['frame_w']   = w
            tracking_state['frame_h']   = h
            tracking_state['last_seen'] = time.time()
        else:
            tracking_state['visible'] = False
            if selected_track_id is not None or selected_bbox is not None:
                with selection_lock:
                    selection_status['visible'] = False
                    selection_status['state'] = 'lost'

    if best:
        bx,by=best['bx'],best['by']
        sc=RED if best['locked'] else AMBER
        st='[ TARGET LOCKED ]' if best['locked'] else '[   TRACKING   ]'
        if best['locked']:
            r=int(LOCK_RADIUS*0.85+math.sin(time.time()*8)*4)
            cv2.circle(frame,(cx,cy),r+1,(0,0,0),3)
            cv2.circle(frame,(cx,cy),r,RED,2)
        cv2.line(frame,(cx,cy),(bx,by),(0,0,0),3)
        cv2.line(frame,(cx,cy),(bx,by),sc,1)
        ang=math.degrees(math.atan2(by-cy,bx-cx))
        put(frame,f'AZ: {ang:.1f}deg',cx+55,cy-20,sc,0.55,1)
        info=f"TGT: {best['label'].upper()}   DIST: {best['dm']:.1f}m   CONF: {best['conf']:.0%}"
        (iw,ih),_=cv2.getTextSize(info,cv2.FONT_HERSHEY_SIMPLEX,0.55,1)
        ix=max(6, min(w-iw-6, cx-iw//2))
        cv2.rectangle(frame,(ix-4,h-42),(ix+iw+4,h-14),(0,0,0),-1)
        put(frame,info,ix,h-20,sc,0.55,1)
    else:
        with tracking_lock:
            last_bx, last_seen = tracking_state['bx'], tracking_state['last_seen']
        searching = (telemetry.get('ai_track', False) and last_bx is not None
                     and (time.time() - last_seen) < SEARCH_TIMEOUT_S)
        if searching:
            sc = (0,140,255)
            st = '[ SEARCHING RIGHT -> ]' if last_bx > cx else '[ <- SEARCHING LEFT ]'
        else:
            st = None
    if st:
        (sw,sh),_=cv2.getTextSize(st,cv2.FONT_HERSHEY_SIMPLEX,0.75,2)
        sx=cx-sw//2
        cv2.rectangle(frame,(sx-6,56),(sx+sw+6,86),(0,0,0),-1)
        put(frame,st,sx,82,sc,0.75,2)
    cv2.rectangle(frame,(0,h-20),(w,h),(0,0,0),-1)
    cv2.line(frame,(0,h-20),(w,h-20),DIM,1)
    put(frame,'APEX COMMAND  //  AI VISION SYSTEM  //  HAILO-10H  40 TOPS',
        cx-205,h-5,(0,120,30),0.38,1)
    return frame

class HUDData(app_callback_class):
    def __init__(self): super().__init__()

def _get_track_id(d):
    """Hailo'nun KENDİ tracker'ından kalıcı (kare kare aynı kalan) ID —
    resmi hailo-rpi5-examples referans kodundaki BİREBİR desen. Tracker
    bilgisi yoksa None döner (sentinel — 0 gerçek bir ID olabileceği için
    onu 'yok' anlamında kullanmadık)."""
    try:
        track = d.get_objects_typed(hailo.HAILO_UNIQUE_ID)
        if len(track) == 1:
            return track[0].get_id()
    except Exception:
        pass
    return None

def hud_callback(element,buffer,user_data):
    # ÖNEMLİ MİMARİ DÜZELTME: bu fonksiyon GStreamer'ın KENDİ callback'i —
    # burada YAVAŞ bir şey yapılırsa (eskiden draw_hud()'un TÜMÜ buradaydı),
    # pipeline'ın kendisi gecikip V4L2 kaynağı zaman aşımına uğrayabilir —
    # gerçek bir donanım kopması GİBİ görünen ama aslında YAZILIM kaynaklı
    # bir "poll error" semptomu üretebilir (kamera/güç donanımı artık
    # doğrulanmış şekilde sağlamken bile kesilmeler sürüyorsa, asıl şüpheli
    # buydu). Şimdi burada SADECE hızlı veri çıkarma var — asıl çizim
    # (draw_hud) AYRI bir thread'e (hud_render_worker) bırakıldı.
    try:
        pad=element.get_static_pad('src'); format_,width,height=get_caps_from_pad(pad)
        fps_state['count']+=1; now=time.time()
        if now-fps_state['last']>=1.0:
            fps_state['fps']=fps_state['count']/(now-fps_state['last'])
            fps_state['count']=0; fps_state['last']=now
        roi=hailo.get_roi_from_buffer(buffer)
        dets_raw=roi.get_objects_typed(hailo.HAILO_DETECTION)
        det_list=[{'label':d.get_label(),'confidence':d.get_confidence(),
                   'track_id':_get_track_id(d),
                   'bbox':(d.get_bbox().xmin(),d.get_bbox().ymin(),
                           d.get_bbox().xmax(),d.get_bbox().ymax())} for d in dets_raw]
        if format_ and width and height:
            frame=get_numpy_from_buffer(buffer,format_,width,height)
            if frame is not None:
                frame=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
                # draw_hud() ÇAĞIRMIYORUZ — sadece kuyruğa koyup hemen
                # dönüyoruz, GStreamer'ı asla beklemiyoruz.
                try:
                    _raw_frame_queue.put_nowait((frame, det_list, fps_state['fps']))
                except queue.Full:
                    try: _raw_frame_queue.get_nowait()
                    except queue.Empty: pass
                    try: _raw_frame_queue.put_nowait((frame, det_list, fps_state['fps']))
                    except queue.Full: pass
    except Exception as e:
        print('HUD callback error:',e)

def hud_render_worker():
    """draw_hud()'un AĞIR çizim işini GStreamer callback'inden tamamen ayrı,
    kendi hızında çalıştırır. Video FPS'inden bağımsız — kuyrukta her zaman
    en taze kare olduğu için, render yavaşlasa bile GStreamer'ı bloklamaz.

    KÜÇÜLTME: Hailo'nun KENDİ resmi dokümantasyonu ("Pipeline runs but with
    low FPS or periodic freezing" — TAM bu semptom resmi olarak listeli)
    şunu söylüyor: "video operasyonları RPi5'te hızlandırılmamış, daha
    düşük çözünürlük kullanmayı düşünün." draw_hud() normalize (0-1) bbox
    koordinatı kullanıyor ve w/h'yi dinamik okuyor — bu yüzden küçültme
    BAŞKA HİÇBİR ŞEYİ bozmadan (aynı görsel tasarım, sadece daha az piksel)
    yapılabiliyor."""
    DOWNSCALE = max(0.5, min(1.0, float(os.environ.get("APEX_HUD_RENDER_SCALE", "1.0"))))
    while True:
        frame, dets, fps = _raw_frame_queue.get()
        try:
            # Nesne tarama/kalibrasyon hattı HUD yazıları olmadan gerçek kamera
            # piksellerine ihtiyaç duyar. Aygıtı ikinci kez açmak yerine en taze
            # ham kareyi aynı sahip süreçte tutuyoruz. JPEG yalnız istek gelince
            # kodlanır; normal 30 FPS akışına ek encode yükü bindirilmez.
            raw_frame = frame.copy()
            if DOWNSCALE < 1.0:
                h, w = frame.shape[:2]
                frame = cv2.resize(frame, (int(w*DOWNSCALE), int(h*DOWNSCALE)), interpolation=cv2.INTER_AREA)
            out = draw_hud(frame, dets, fps)
            encode_started = time.perf_counter()
            ok, jpg = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if not ok:
                raise RuntimeError('JPEG kodlama başarısız')
            payload = jpg.tobytes()
            now = time.time()
            with frame_ready:
                latest_frame[0] = out
                latest_frame_ts[0] = now
                latest_jpeg[0] = payload
                latest_jpeg_ts[0] = now
                latest_raw_frame[0] = raw_frame
                latest_raw_frame_ts[0] = now
                stream_stats['encoded_frames'] += 1
                stream_stats['encode_ms'] = (time.perf_counter() - encode_started) * 1000.0
                stream_stats['jpeg_bytes'] = len(payload)
                frame_ready.notify_all()
        except Exception as e:
            print('HUD render error:', e)

# ─── MJPEG HTTP yayını — camera_stream.py ile AYNI mimari/port ─────────────
class HUDStreamHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # konsolu kirletmesin

    def do_GET(self):
        from urllib.parse import urlparse
        path = urlparse(self.path).path
        if path == "/stream":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.send_header("Pragma", "no-cache")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            last_sent_ts = 0.0
            with frame_lock:
                stream_stats['stream_clients'] += 1
                stream_stats['stream_clients_peak'] = max(
                    stream_stats['stream_clients_peak'], stream_stats['stream_clients'])
            try:
                while True:
                    with frame_ready:
                        if latest_jpeg_ts[0] <= last_sent_ts:
                            frame_ready.wait(timeout=0.25)
                        payload = latest_jpeg[0]
                        frame_ts = latest_jpeg_ts[0]
                    age = time.time() - frame_ts if frame_ts else 999.0
                    if payload is None or age > STREAM_STALE_TIMEOUT_S:
                        # Eski kareyi tekrar edip tarayıcıyı kandırma. Akışı
                        # kapat; UI watchdog yeniden bağlansın, process watchdog
                        # gerekirse launcher'a yeniden başlatma fırsatı versin.
                        time.sleep(0.05)
                        break
                    if frame_ts <= last_sent_ts:
                        continue
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(payload)))
                    self.send_header("X-Apex-Frame-Ts", f"{frame_ts:.3f}")
                    self.end_headers()
                    self.wfile.write(payload)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                    last_sent_ts = frame_ts
            except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                pass
            finally:
                with frame_lock:
                    stream_stats['stream_clients'] = max(0, stream_stats['stream_clients'] - 1)
        elif path == "/tracking/status":
            with selection_lock:
                data = dict(selection_status)
            with gimbal_control_lock:
                data['gimbal'] = dict(gimbal_control_state)
            data['selected'] = data.get('state') in {'locked', 'lost'}
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(jsonlib.dumps(data, ensure_ascii=False).encode('utf-8'))
        elif path == "/raw-snapshot":
            with frame_lock:
                raw = None if latest_raw_frame[0] is None else latest_raw_frame[0].copy()
                frame_ts = latest_raw_frame_ts[0]
            age = time.time() - frame_ts if frame_ts else 999.0
            if raw is not None and age <= STREAM_STALE_TIMEOUT_S:
                ok, encoded = cv2.imencode('.jpg', raw, [cv2.IMWRITE_JPEG_QUALITY, 92])
                if ok:
                    payload = encoded.tobytes()
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(payload)))
                    self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                    self.send_header("X-Apex-Frame-Age", f"{age:.3f}")
                    self.send_header("X-Apex-Frame-Ts", f"{frame_ts:.6f}")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(payload)
                    return
            self.send_response(503)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(jsonlib.dumps({"camera": False, "fresh": False, "frame_age": age}).encode('utf-8'))
        elif path == "/snapshot":
            with frame_lock:
                payload = latest_jpeg[0]
                frame_ts = latest_jpeg_ts[0]
            age = time.time() - frame_ts if frame_ts else 999.0
            if payload is not None and age <= STREAM_STALE_TIMEOUT_S:
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                self.send_header("X-Apex-Frame-Age", f"{age:.3f}")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(payload)
            else:
                self.send_response(503)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(jsonlib.dumps({"camera": False, "fresh": False, "frame_age": age}).encode('utf-8'))
        elif path == "/health":
            with frame_lock:
                frame = latest_frame[0]
                frame_ts = latest_jpeg_ts[0]
                stats = dict(stream_stats)
                resolution = None if frame is None else [int(frame.shape[1]), int(frame.shape[0])]
            age = time.time() - frame_ts if frame_ts else None
            data = {
                'camera': age is not None and age <= STREAM_STALE_TIMEOUT_S,
                'fresh': age is not None and age <= STREAM_STALE_TIMEOUT_S,
                'frame_age': age,
                'source_fps': round(float(fps_state.get('fps', 0.0)), 2),
                'resolution': resolution,
                **stats,
            }
            self.send_response(200 if data['camera'] else 503)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(jsonlib.dumps(data).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

def pipeline_watchdog():
    """Hailo/GStreamer bazen process çökmeden kare üretmeyi kesebiliyor.
    Böyle kalırsa launcher bunu fark etmez. Yeni kare belli süre gelmezse
    process bilerek çıkar; apex launcher zaten detection servisini kontrollü
    şekilde yeniden başlatıyor."""
    warned = False
    while True:
        time.sleep(2.0)
        now = time.time()
        with frame_lock:
            ts = latest_frame_ts[0]
        if ts == 0.0:
            if now - process_started_ts > STARTUP_GRACE_S:
                print(f"[HUD] WATCHDOG: {STARTUP_GRACE_S:.0f}s içinde hiç video karesi üretilmedi — detection yeniden başlatılsın diye çıkılıyor", flush=True)
                os._exit(42)
        elif now - ts > PIPELINE_WATCHDOG_S:
            if not warned:
                print(f"[HUD] WATCHDOG: {now-ts:.1f}s yeni kare yok — Hailo/GStreamer pipeline dondu, yeniden başlatılıyor", flush=True)
                warned = True
            os._exit(42)
        else:
            warned = False

class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


def run_stream_server():
    # systemd restart sırasında eski MJPEG socket'i birkaç saniye TIME_WAIT/
    # kapanış durumunda kalabilir. Tek bind hatası thread'i sessizce öldürmesin.
    while True:
        try:
            server = ReusableThreadingHTTPServer(("0.0.0.0", STREAM_PORT), HUDStreamHandler)
            server.daemon_threads = True
            server.request_queue_size = 16
            print(f"[HUD] MJPEG yayını -> http://0.0.0.0:{STREAM_PORT}/stream", flush=True)
            server.serve_forever()
        except OSError as exc:
            print(f"[HUD] 8888 bind bekliyor: {exc}", flush=True)
            time.sleep(1.0)


def _put_latest_render_frame(frame, detections, fps):
    """Never let rendering or a slow browser build a video backlog."""
    item = (frame, detections, fps)
    try:
        _raw_frame_queue.put_nowait(item)
    except queue.Full:
        try:
            _raw_frame_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            _raw_frame_queue.put_nowait(item)
        except queue.Full:
            pass


def run_direct_hailort():
    """Python 3.13 compatible Hailo-10H backend for the new Raspberry Pi OS.

    Camera acquisition, inference and HUD/JPEG rendering are three independent
    stages with one-frame queues.  If any stage is briefly slower, an old frame
    is discarded instead of being displayed seconds later.
    """
    from functools import partial
    from pathlib import Path
    from types import SimpleNamespace
    from libcamera import Transform
    from picamera2 import Picamera2
    from hailo_platform import HEF
    from hailo_apps.python.core.common.hailo_inference import HailoInfer
    from hailo_apps.python.core.common.toolbox import default_preprocess, get_labels
    from hailo_apps.python.core.tracker.byte_tracker import BYTETracker
    from hailo_apps.python.standalone_apps.object_detection.object_detection_post_process import extract_detections

    model_path = Path(os.environ.get(
        "APEX_HUD_HEF",
        "/usr/local/hailo/resources/models/hailo10h/yolov5m_wo_spp.hef",
    ))
    if not model_path.is_file():
        raise FileNotFoundError(f"Hailo HEF bulunamadı: {model_path}")

    capture_width = int(os.environ.get("APEX_HUD_WIDTH", "1280"))
    capture_height = int(os.environ.get("APEX_HUD_HEIGHT", "720"))
    capture_fps = int(os.environ.get("APEX_HUD_FPS", "30"))
    labels = get_labels(None)
    detection_config = {
        "visualization_params": {
            "score_thres": float(os.environ.get("APEX_HUD_SCORE_THRESHOLD", "0.30")),
            "max_boxes_to_draw": int(os.environ.get("APEX_HUD_MAX_BOXES", "100")),
        }
    }
    person_tracker = BYTETracker(SimpleNamespace(
        track_thresh=0.10,
        track_buffer=30,
        match_thresh=0.90,
        aspect_ratio_thresh=2.0,
        min_box_area=500,
        mot20=False,
    ))

    hef = HEF(str(model_path))
    output_count = len(hef.get_output_vstream_infos())
    engine = HailoInfer(
        str(model_path), batch_size=1,
        output_type="FLOAT32" if output_count > 1 else None,
    )
    model_height, model_width, _ = engine.get_input_shape()
    capture_queue = queue.Queue(maxsize=1)
    stop_event = threading.Event()
    counters = {"captured": 0, "inferred": 0, "started": time.monotonic()}

    camera = Picamera2(0)
    camera_config = camera.create_video_configuration(
        main={"size": (capture_width, capture_height), "format": "RGB888"},
        controls={"FrameRate": capture_fps},
        transform=Transform(hflip=CAMERA_HFLIP, vflip=CAMERA_VFLIP),
        buffer_count=4,
    )
    camera.configure(camera_config)

    def capture_loop():
        camera.start()
        print(
            f"[HUD] CSI0 direct capture: {capture_width}x{capture_height} @{capture_fps} FPS, "
            f"hflip={int(CAMERA_HFLIP)} vflip={int(CAMERA_VFLIP)}",
            flush=True,
        )
        try:
            while not stop_event.is_set():
                frame = camera.capture_array("main")
                counters["captured"] += 1
                try:
                    capture_queue.put_nowait(frame)
                except queue.Full:
                    try:
                        capture_queue.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        capture_queue.put_nowait(frame)
                    except queue.Full:
                        pass
        finally:
            camera.stop()

    def completed(completion_info, bindings_list, original_frame, submitted_at):
        if completion_info.exception:
            print("[HUD] Hailo inference hatası:", completion_info.exception, flush=True)
            return
        bindings = bindings_list[0]
        if len(bindings._output_names) == 1:
            result = bindings.output().get_buffer()
        else:
            result = {
                name: np.expand_dims(bindings.output(name).get_buffer(), axis=0)
                for name in bindings._output_names
            }
        parsed = extract_detections(original_frame, result, detection_config)
        h, w = original_frame.shape[:2]
        detections = []
        for box, class_id, confidence in zip(
            parsed["detection_boxes"],
            parsed["detection_classes"],
            parsed["detection_scores"],
        ):
            x1, y1, x2, y2 = map(float, box)
            class_id = int(class_id)
            label = labels[class_id] if 0 <= class_id < len(labels) else f"class-{class_id}"
            detections.append({
                "label": label,
                "confidence": float(confidence),
                "track_id": None,
                "bbox": (
                    max(0.0, min(1.0, x1 / w)),
                    max(0.0, min(1.0, y1 / h)),
                    max(0.0, min(1.0, x2 / w)),
                    max(0.0, min(1.0, y2 / h)),
                ),
            })

        # Preserve stable person IDs across frames.  Target selection in the
        # browser can therefore stay attached to one person when several people
        # cross or briefly overlap, just like the former GStreamer tracker.
        person_indices = [i for i, det in enumerate(detections) if det["label"] == "person"]
        tracker_input = np.asarray([
            [
                detections[i]["bbox"][0] * w,
                detections[i]["bbox"][1] * h,
                detections[i]["bbox"][2] * w,
                detections[i]["bbox"][3] * h,
                detections[i]["confidence"],
            ]
            for i in person_indices
        ], dtype=np.float32)
        if not len(tracker_input):
            tracker_input = np.empty((0, 5), dtype=np.float32)
        for track in person_tracker.update(tracker_input):
            tx1, ty1, tx2, ty2 = map(float, track.tlbr)
            best_index = None
            best_overlap = 0.0
            for index in person_indices:
                det = detections[index]
                dx1, dy1, dx2, dy2 = det["bbox"]
                overlap = bbox_iou(
                    (tx1, ty1, tx2, ty2),
                    (dx1 * w, dy1 * h, dx2 * w, dy2 * h),
                )
                if overlap > best_overlap:
                    best_index, best_overlap = index, overlap
            if best_index is not None and best_overlap >= 0.10:
                detections[best_index]["track_id"] = int(track.track_id)

        with frame_lock:
            stream_stats["detections"] = len(detections)
            stream_stats["detection_labels"] = [d["label"] for d in detections[:12]]
            stream_stats["inference_ms"] = (time.perf_counter() - submitted_at) * 1000.0

        counters["inferred"] += 1
        now = time.time()
        fps_state["count"] += 1
        if now - fps_state["last"] >= 1.0:
            fps_state["fps"] = fps_state["count"] / (now - fps_state["last"])
            fps_state["count"] = 0
            fps_state["last"] = now
        _put_latest_render_frame(original_frame, detections, fps_state["fps"])

    capture_thread = threading.Thread(target=capture_loop, name="csi-capture", daemon=True)
    capture_thread.start()
    print(
        f"[HUD] Direct HailoRT backend: {model_path.name}, model {model_width}x{model_height}",
        flush=True,
    )
    try:
        while True:
            frame = capture_queue.get()
            prepared = default_preprocess(frame, model_width, model_height)
            job = engine.run([
                prepared
            ], partial(
                completed,
                original_frame=frame,
                submitted_at=time.perf_counter(),
            ))
            # A single in-flight frame is intentional: the capture queue keeps
            # only the newest image, so latency stays bounded to one inference.
            job.wait(10000)
    finally:
        stop_event.set()
        engine.close()


def start_hud_threads():
    threading.Thread(target=telemetry_poller, daemon=True).start()
    threading.Thread(target=vision_control_poller, daemon=True).start()
    threading.Thread(target=tracking_commander, daemon=True).start()
    threading.Thread(target=hud_render_worker, daemon=True).start()
    threading.Thread(target=run_stream_server, daemon=True).start()
    threading.Thread(target=pipeline_watchdog, daemon=True).start()

if __name__=='__main__':
    start_hud_threads()
    force_direct = os.environ.get("APEX_HUD_BACKEND", "auto").lower() == "direct"
    if GST_HAILO_AVAILABLE and not force_direct:
        user_data=HUDData()
        app=GStreamerDetectionApp(hud_callback,user_data)
        app.video_sink='fakesink'
        app.run()   # GLib/GStreamer ana döngüsü — bloklar, programın canlı kalmasını sağlar
    else:
        if not GST_HAILO_AVAILABLE:
            print(f"[HUD] TAPPAS Python binding kullanılamıyor ({GST_HAILO_IMPORT_ERROR}); direct HailoRT seçildi.", flush=True)
        run_direct_hailort()
