# APEX META-1 sensör ve otonom takip SIL rehberi

Bu ortam Ubuntu 22.04 WSL2, ROS 2 Humble ve Gazebo Fortress içindir. Fiziksel
robot olmadan şu zinciri test eder:

`Gazebo kamera -> hedef algılama -> takip kontrolcüsü + ToF kilidi -> cmd_vel -> firmware tripod gait -> 18 eklem -> ayak teması -> odometri`

LiDAR, IMU, ToF, GPS, eklem durumları ve kamera aynı anda ROS topic'leri olarak
üretilir. Takip kontrolcüsü açılışta daima kapalıdır.

## 1. Derle ve başlat

Ubuntu-22.04 terminalinde:

```bash
cd /mnt/c/Users/ASUSROG/Desktop/Proje/main_v17/APEX_META1_RADIO_SIL/ros2_ws
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install
set +u
source install/setup.bash
set -u
ros2 launch apex_gazebo sil.launch.py profile:=fast
```

İlk kullanımda `fast` ile başla. Kararlıysa `balanced`, kayıt veya son doğrulama
için `fidelity` kullan:

| Profil | Kamera | LiDAR | IMU | ToF | Kullanım |
|---|---:|---:|---:|---:|---|
| `fast` | 320x180 @ 10 Hz | 180 @ 5 Hz | 50 Hz | 15 Hz | WSL GUI, kontrol geliştirme |
| `balanced` | 424x240 @ 15 Hz | 360 @ 10 Hz | 100 Hz | 20 Hz | Varsayılan genel test |
| `fidelity` | 640x480 @ 20 Hz | 720 @ 10 Hz | 100 Hz | 20 Hz | Görüntü/veri doğrulama |

WSL'deki bilinen OGRE2 sürücü çökmesi nedeniyle yazılım render'ı varsayılandır.
Güncel bir WSL GPU sürücüsüyle denemek istersen:

```bash
ros2 launch apex_gazebo sil.launch.py profile:=balanced software_rendering:=false
```

`GL3PlusTextureGpu::copyTo` benzeri hata görülürse tekrar
`software_rendering:=true` kullan.

## 2. İkinci terminali hazırla

Her yeni Ubuntu terminalinde:

```bash
cd /mnt/c/Users/ASUSROG/Desktop/Proje/main_v17/APEX_META1_RADIO_SIL/ros2_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
```

## 3. Kamera ve algılama

Ham kamera ve hedef işaretli debug görüntüsü:

```bash
ros2 run rqt_image_view rqt_image_view /apex/camera
ros2 run rqt_image_view rqt_image_view /apex/tracking/debug_image
```

Mor dikey model, Hailo olmadan deterministik test için kullanılan kişi
vekilidir. Algılama sözleşmesini izle:

```bash
ros2 topic echo /apex/target_observation
```

`vector.x`: yatay hata (`-1` sol, `+1` sağ), `vector.y`: tahmini metre,
`vector.z`: güven. Gerçek Hailo adaptörü daha sonra aynı sözleşmeye bağlanmalı.

## 4. Otonom takip

Önce durum:

```bash
ros2 topic echo /apex/tracking/status
```

Takibi bilinçli olarak aç:

```bash
ros2 service call /apex/tracking/enable std_srvs/srv/SetBool "{data: true}"
```

Robot mor hedefe dönüp yaklaşık `1.2 m` mesafeyi korumaya çalışır:

```bash
ros2 topic echo /apex/cmd_vel
ros2 topic echo /apex/odom
```

Takibi kapatmak anında sıfır hız yollar:

```bash
ros2 service call /apex/tracking/enable std_srvs/srv/SetBool "{data: false}"
```

Algı `0.45 s` bayatlarsa veya hedef güveni düşerse kontrolcü durur. ToF
`0.45 m` altını görürse ileri hareket kesilir.

## 5. Hareketli hedef

Hedef varsayılan olarak sabittir. Hareketi açıp kapat:

```bash
ros2 service call /apex/sim_target/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /apex/sim_target/enable std_srvs/srv/SetBool "{data: false}"
```

Önce yalnız hedefi hareket ettirerek algılamayı, sonra robot takibini de açarak
kapalı çevrimi test et.

## 6. LiDAR, ToF, IMU ve RViz

```bash
ros2 topic echo /apex/lidar_scan --once
ros2 topic echo /apex/tof_scan --once
ros2 topic echo /apex/imu --once
ros2 topic echo /apex/gps --once
rviz2
```

RViz'de `Fixed Frame = base_link` seç. `LaserScan` için `/apex/lidar_scan` ve
`/apex/tof_scan`, `Image` için `/apex/tracking/debug_image` ekle. Frame'ler TF
ile uyumludur: `lidar_link`, `tof_link`, `imu_link`, `camera_optical_frame`.

## 7. Performans

```bash
ros2 topic echo /apex/sim/performance
```

`rtf` simülasyon zamanı / gerçek zaman, `camera_fps` teslim edilen kamera hızı,
`lidar_hz` teslim edilen tarama hızıdır. Bu makinedeki headless `fast` testinde
yaklaşık `RTF 0.96`, kamera `9.6 FPS`, LiDAR `4.8 Hz` ölçüldü. GUI açıkken daha
düşük olabilir. GUI akıcılığı, kamera FPS'i ve fizik RTF'si ayrı ölçülerdir.

## 8. Otomatik testler

```bash
cd /mnt/c/Users/ASUSROG/Desktop/Proje/main_v17/APEX_META1_RADIO_SIL
bash tools/smoke_ros2_sil.sh
bash tools/test_autonomy_sil.sh
```

İlki launch/controller/sensör/TF'yi; ikincisi kamera algılama, başlangıç güvenlik
kilidi, takip komutu, odometrik gövde hareketi ve performansı doğrular.

## 9. Ön sensör geometrisi

Kanonik kaynak
`ros2_ws/src/apex_description/config/model_parameters.yaml` içindeki
`sensor_cluster` bölümüdür. Xacro bunu doğrudan okur.

| Sensör | X (m) | Y (m) | Z (m) | Durum |
|---|---:|---:|---:|---|
| Kamera | 0.210 | 0.000 | 0.060 | geçici, pitch -8 derece |
| ToF | 0.220 | -0.028 | 0.037 | geçici |
| IMU | 0.180 | 0.030 | 0.055 | geçici |
| LiDAR merkezi | 0.170 | 0.000 | 0.115 | geçici |

Yerleşim “hepsi önde ve birbirine yakın” kısıtını, 430 mm gövde boyunu ve
LiDAR muhafazasını korur; CAD ölçümü değildir. Fiziksel robot geldiğinde gövde
merkezinden sensör merkezlerine X/Y/Z ölç ve yalnız YAML değerlerini güncelle.

Robot modelinde gövdeyi doğrudan kaydıran `VelocityControl` yoktur. `/apex/cmd_vel`
v55 ile aynı RF/RM/RR/LF/LM/LR tripod fazlarına, 100 mm adım boyuna, 50 mm
kaldırmaya ve 1000 ms tam-komut çevrimine çevrilir. Gazebo gövdesi yalnız
ayak-zemin temasıyla ilerler. Kütle, atalet, gerçek servo tork/hız eğrileri ve
lastik ayak sürtünmesi fiziksel ölçüm gelene kadar yine yaklaşık değerlerdir.

Aktif fiziksel gait testini çalıştırmak için:

```bash
bash tools/test_tripod_gait_sil.sh
```
