# Robot olmadan APEX'i çalıştırma

Bu katman gerçek ESP32, servo, kamera ve LiDAR olmadan aşağıdakileri çalıştırır:

- Command Center (`8080`)
- Kinematics Lab / live twin (`8090`)
- stateful dummy ESP32 (`8081`)
- hareket fazı ve 18 eklem command-state telemetrisi
- 300 ms komut deadman/failsafe
- kalıcı leg PWM off/on
- IMU, ToF, GPS ve çift batarya telemetrisi
- dummy LiDAR oda taraması
- dinamik MJPEG kamera ve kamera-freeze senaryosu (`8888`)
- gerçek 32 bayt APEX protokol encode/decode yolu

## 1. PowerShell'i aç

Normal PowerShell yeterlidir; yönetici yetkisi gerekmez.

```powershell
cd C:\Users\ASUSROG\Desktop\Proje\main_v17\APEX_META1_RADIO_SIL
```

Bu makinede `.venv` hazırlandı. Başka bir makinede ilk kez kurulacaksa:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
```

## 2. Tam otomatik smoke test

```powershell
.\.venv\Scripts\python.exe .\tools\dummy_stack.py test
```

Beklenen dört `PASS`:

```text
[PASS] motion -> link loss -> failsafe
[PASS] persistent legOff
[PASS] camera freshness
[PASS] Command Center -> dummy robot -> Kinematics Lab
```

Test servisleri kendisi başlatıp kapatır.

## 3. İnteraktif kullanım

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\run_dummy.ps1
```

Tarayıcıda aç:

- Command Center: <http://127.0.0.1:8080>
- Kinematics Lab: <http://127.0.0.1:8090>
- Dummy Robot API: <http://127.0.0.1:8081/docs>
- Kamera akışı: <http://127.0.0.1:8888/stream>

## 4. Elle güvenlik testleri

### Joystick bırakma

1. Command Center'da `YÜRÜ` seç.
2. Joystick'i sürükle ve sabit tut.
3. Bırak.
4. Telemetride `vx`, `vy`, `vr` sıfıra dönmeli; hareket fazı durmalı.

### Komut/radyo bağlantısı kopması

```powershell
Invoke-RestMethod "http://127.0.0.1:8081/sim/fault?kind=link_drop&state=1"
Start-Sleep -Milliseconds 400
Invoke-RestMethod "http://127.0.0.1:8080/telemetry"
```

Beklenen: `mode=IDLE`, `failsafe=true`, tüm joystick eksenleri `0`.

Geri getir:

```powershell
Invoke-RestMethod "http://127.0.0.1:8081/sim/reset"
```

### Telemetri tamamen kesilsin

```powershell
Invoke-RestMethod "http://127.0.0.1:8081/sim/fault?kind=telemetry_offline&state=1"
```

### IMU veya ToF arızası

```powershell
Invoke-RestMethod "http://127.0.0.1:8081/sim/fault?kind=imu&state=1"
Invoke-RestMethod "http://127.0.0.1:8081/sim/fault?kind=tof&state=1"
```

### Kamera donması

```powershell
Invoke-RestMethod "http://127.0.0.1:8888/sim/freeze?state=1"
Start-Sleep -Seconds 2
Invoke-RestMethod "http://127.0.0.1:8888/status"
```

Beklenen: `fresh=false`. Geri açmak için `state=0` gönder.

### Kalıcı servo PWM off

```powershell
Invoke-RestMethod "http://127.0.0.1:8080/legOff?leg=0&j=1"
Invoke-RestMethod "http://127.0.0.1:8080/legStatus"
```

RF femur `enabled=false` kalmalı. Tekrar aç:

```powershell
Invoke-RestMethod "http://127.0.0.1:8080/legOn?leg=0&j=1"
```

## 5. Servisleri kapat

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\stop_dummy.ps1
```

Loglar `.runtime\dummy` altındadır.

## Önemli sınır

Bu dummy; UI, API, protokol, timeout, hata durumları ve command-state akışını
test eder. Henüz yerçekimi, temas, servo torku, gövde devrilmesi ve gerçek kamera
optiklerini çözmez. Bunlar ROS 2 Humble + Gazebo Fortress SIL aşamasında gelir.
`v56_radio_sil_foundation.ino` deadman entegrasyonu tamamlanmadan flash edilmez.
