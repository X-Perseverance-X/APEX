# APEX META-1 Radio + SIL v17

Bu klasör çalışan `main_v15` runtime'ı, `main_v16` UI/Lab/sensör düzeltmeleri,
yeni güvenlik katmanı, taşıma-bağımsız protokol ve robot olmadan test altyapısını
tek geliştirme hattında birleştirir. Eski sürümlere dokunulmamıştır.

## İlk çalıştırılacak dosyalar

1. Robot olmadan test: [`docs/DUMMY_QUICKSTART.md`](docs/DUMMY_QUICKSTART.md)
2. Mimari: [`docs/ARCHITECTURE_V17.md`](docs/ARCHITECTURE_V17.md)
3. Radyo protokolü: [`docs/RADIO_PROTOCOL_V1.md`](docs/RADIO_PROTOCOL_V1.md)
4. ROS/Gazebo: [`docs/ROS2_GAZEBO_QUICKSTART.md`](docs/ROS2_GAZEBO_QUICKSTART.md)
5. Test kapsamı: [`docs/SIL_PLAN.md`](docs/SIL_PLAN.md)
6. Pi beyin bakım kaydı: [`docs/BRAIN_FIX_2026-08-26.md`](docs/BRAIN_FIX_2026-08-26.md)

## Hızlı doğrulama

```powershell
cd C:\Users\ASUSROG\Desktop\Proje\main_v17\APEX_META1_RADIO_SIL
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe .\tools\dummy_stack.py test
```

## Donanım durumu

- nRF24 protokolü ve codec hazırdır.
- Gerçek RF24 sürücüsü henüz etkin değildir.
- ESP32-S3 kartının tam modeli ve güvenli CE/CSN/IRQ pinleri doğrulanmadan pin
  ataması yapılmayacaktır.
- `v56_radio_sil_foundation.ino` kaynak seviyesinde deadman ve kalıcı PWM maskesi
  içerir fakat gerçek ESP32-S3 Arduino toolchain'iyle board-compile ve bench HIL
  yapılmadan robota flash edilmemelidir.
