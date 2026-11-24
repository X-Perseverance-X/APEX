# APEX Kinematics Lab — Mimari

## Amaç
Bu araç, META‑1 hexapod robotun fizik simülasyonuna geçmeden önce ölçü, frame, IK, servo mapping, gait ve sensör mount parametrelerini doğrulamak için tasarlandı.

## Katmanlar

1. **UI Theme Layer**
   - APEX siyah/gri/kırmızı tema.
   - Orbitron başlık, Share Tech Mono teknik metin.
   - Sol 3D sahne, sağ ayar paneli, alt IK/uyarı/curve paneli.

2. **Robot Parameter State**
   - `config/default_robot.json` ana parametre kaynağıdır.
   - Arayüzde değiştirilen değerler canlı state üzerinde güncellenir.
   - `SAVE CONFIG` ile `config/current_robot.json` oluşur.

3. **Kinematics Layer**
   - 3 DOF bacak modeli.
   - FK: servo/matematik açıları → ayak pozisyonu.
   - IK: ayak hedefi → coxa/femur/tibia açıları.
   - Servo zero/direction/limit mapping.

4. **Body Orientation Layer**
   - Gövde roll/pitch/yaw değişince hedef ayak noktası düzeltilir.
   - Temel formül: `corrected = R_actual^T * R_desired * (hip + target) - hip`.

5. **Gait Layer**
   - Tripod, alternative tripod, ripple, wave, tetrapod, metachronal, crawl, custom.
   - Step length, step height, duty factor, frequency, body height, turn bias.

6. **Servo Motion Layer**
   - Servo hedefe giderken eğri seçimi.
   - Linear, smoothstep, cosine, cubic bezier, min jerk, trapezoid, S-curve.
   - Current/energy proxy ile yöntemlerin sezgisel karşılaştırması.

7. **Sensor Mount Layer**
   - IMU, LiDAR, kamera, TOF, GPS.
   - Her sensör için x/y/z + roll/pitch/yaw.
   - Kamera frustum, LiDAR scan disc, TOF ray gösterimi.

8. **Contact Layer**
   - Force sensor yoksa mikro switch temas simülasyonu.
   - Contact açık ayaklar support polygon hesabına girer.

## Genişletme noktaları

- MuJoCo MJCF export.
- ROS 2 URDF export.
- Gerçek robot log replay.
- Gerçek servo akımı ile energy proxy kalibrasyonu.
- Mikro switchlerden terrain height map üretimi.
