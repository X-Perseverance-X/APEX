# APEX META‑1 Complete Integration

Bu paket dört parçayı tek yerde toplar:

1. `v54_meta1_integrated.ino` — ESP32-S3 ana firmware: yürüyüş, body IK, MPU6050, TOF400C, çift batarya ADC telemetrisi, kol ve 18 bacak servo laboratuvarı.
2. `apex_server.py` + `apex_ui.html` — RPi5 Command Center ve gerçek robota komut köprüsü.
3. `apex_kinematics_lab/` — 3D IK/gait/sensör yerleşimi laboratuvarı.
4. `./apex meta-1 start` — iki web sunucusunu başlatır ve Windows/WSL ortamında Command Center ile Kinematics Lab'i iki monitöre açmayı dener.

## Kurulum

```bash
./install.sh
./apex meta-1 start
```

## Portlar

- Command Center: `http://localhost:8080`
- Kinematics Lab: `http://localhost:8090`

## Yeni bacak servo laboratuvarı

Command Center içinde **LEG SERVO LAB** bölümünde:

- RF/RM/RR/LF/LM/LR seçimi,
- Coxa/Femur/Tibia seçimi,
- 5–175° güvenli slider,
- 5° / 90° / 175° hızlı test,
- seçili veya tüm bacaklar için montaj pozu,
- seçili veya tüm bacaklar için HOME/STAND hedefi,
- seçili joint / seçili leg / tüm bacak PWM kesme,
- PCA, kanal, yön, gerçek açı, hedef açı ve home açı teşhisi,
- servo PWM min/max ve offset kalibrasyonu
bulunur.

### Montaj pozları

- Tüm Coxa: `90°`
- Sağ RF/RM/RR Femur + Tibia: `5°`
- Sol LF/LM/LR Femur + Tibia: `175°`

Bunlar yalnızca **montaj/calibration pozu**dur. Normal STAND/HOME açıları eski `homeAngles` değerlerinden devam eder.

## TOF400C

TOF400C/VL53L1X aynı I2C hattına bağlanır:

- SDA → ESP32-S3 GPIO8
- SCL → ESP32-S3 GPIO9
- GND → ortak GND
- Besleme → kullandığın TOF400C breakout kartının giriş gerilimine göre bağla; kart üzerindeki regülatör/level shifting yapısını doğrulamadan doğrudan gerilim varsayma.

Firmware varsayılan I2C adresi `0x29` ile başlatır.

## Batarya voltaj girişleri

- Sağ 3S batarya bölücü çıkışı → GPIO1 (ADC1)
- Sol 3S batarya bölücü çıkışı → GPIO2 (ADC1)
- Her iki bölücünün GND'si → robot ortak GND

Firmware varsayılan olarak `12.6 V → 3.0 V` kabul eder ve oranı `4.20` kullanır. Gerçek direnç oranına göre `BAT_RIGHT_DIVIDER_RATIO` ve `BAT_LEFT_DIVIDER_RATIO` değerlerini düzelt.

ADC girişine 100 nF seramik kondansatör eklemek ve voltajı multimetre ile karşılaştırıp `BAT_*_CAL_FACTOR` ile ince kalibrasyon yapmak önerilir.

## Durdurma / durum

```bash
./apex meta-1 status
./apex meta-1 logs
./apex meta-1 stop
./apex meta-1 restart
```
