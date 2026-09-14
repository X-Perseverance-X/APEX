# META-1 3D Haritalama, SLAM ve Otonomi Yol Haritası

## 1. Mimari karar

META-1 iki ayrı fakat aynı poz kestirimini kullanan harita üretecek:

1. **Navigasyon haritası:** RPLIDAR A1M8 tabanlı, güvenilir 2D occupancy grid.
   Engel kaçınma, hedef gönderme ve Nav2 planlama bu haritayı kullanacak.
2. **Algısal 3D harita:** LiDAR taramalarının gerçek sensör pozuyla biriktirildiği,
   kamera rengi ve ileride semantik sınıflarla zenginleştirilen point cloud/mesh.

Görsel olarak güzel 3D harita, tek başına güvenlik-kritik navigasyon girdisi
olmayacak. Otonomi önce 2D harita üzerinde doğrulanacak.

## 2. Mevcut durum — doğrulandı

- A1M8, tek tarama düzlemi üreten 360 derece **2D** LiDAR'dır.
- `apex_server.py` ham taramayı yalnızca `(angle_deg, distance_mm)` olarak tutuyor.
  Nokta veya tarama için ölçüm zaman damgası ve sensör pozu taşınmıyor.
- `SLAM_MAPPING_ENABLED = False`; eski özel 2D ICP haritası kararsızlık nedeniyle
  bilinçli olarak kapalı.
- Sunucudaki kapalı ICP kodunda yaklaşık bir yükseklik hesabı bulunmasına rağmen
  arayüz `map_points` içindeki üçüncü değeri kullanmıyor.
- `apex_ui.html/updateLidar3D()` hem ham taramada hem harita modunda Three.js
  yüksekliğini `0` yapıyor. Mevcut “3D BULUT”, perspektif içinde çizilmiş düz bir
  tarama düzlemidir; 3D rekonstrüksiyon değildir.
- MPU6050 firmware'i pitch ve roll üretiyor. Yeni v56 temeli gyro ile yaw da
  üretiyor; ancak MPU6050'de manyetometre olmadığından yaw mutlak yön değildir ve
  zamanla sürüklenir.
- Body IK pitch/roll/yaw komutları **istenen** gövde duruşudur. Servo enkoderi
  bulunmadığı için bunlar tek başına ölçülmüş gerçek poz kabul edilemez.
- Sensörlerin URDF konumları `model_parameters.yaml` içinde `provisional`; fiziksel
  ölçüm ve kalibrasyon yapılmadan projeksiyon/metrik harita için yeterli değildir.
- Raspberry Pi üzerinde şu anda ROS 2 ve `rclpy` kurulu değil. Mevcut çalışan
  APEX servisleri ROS'tan bağımsızdır ve korunacaktır.

## 3. Neden başı/gövdeyi kaldırınca tavan görünmedi?

Bir LiDAR noktası önce LiDAR düzleminde oluşur:

```text
p_lidar = [range*cos(angle), range*sin(angle), 0, 1]
```

Gerçek dünya noktası şu zincirle hesaplanmalıdır:

```text
p_map = T_map_odom(t)
      * T_odom_base(t)
      * T_base_sensor_cluster(t)
      * T_sensor_cluster_lidar
      * p_lidar
```

Mevcut uygulama bu dönüşüm zincirini uygulamıyor ve doğrudan `Z=0` yazıyor.
Ayrıca bir A1 taraması yaklaşık yüzlerce milisaniyeye yayılabildiği için bütün
taramaya tek pose vermek de hareket sırasında duvarları eğer. İdeal çözüm her
lazer örneğini kendi zamanı için enterpole edilmiş pose ile dönüştürmektir
(deskew/motion compensation).

## 4. Hedef veri ve TF mimarisi

```text
map
 └── odom
      └── base_footprint
           └── base_link                 gerçek gövde roll/pitch/z
                └── sensor_cluster_link  fiziksel braket
                     ├── lidar_link
                     ├── imu_link
                     ├── camera_link
                     │    └── camera_optical_frame
                     └── tof_link
```

Gerekli zaman damgalı girdiler:

| Veri | Hedef hız | Zorunlu alanlar | Kullanım |
|---|---:|---|---|
| LiDAR | gerçek cihaz hızı | angle, range, quality, scan start/end time | 2D SLAM ve 3D birikim |
| Kamera | 30 FPS | raw/rectified frame, capture timestamp, intrinsics | VO, renk, semantik |
| IMU | en az 100 Hz | accel XYZ, gyro XYZ, quaternion, covariance, timestamp | roll/pitch ve kısa süreli hareket |
| Body IK | komut değişince | requested pose, command timestamp | yalnızca prior/karşılaştırma |
| Eklem durumu | 30–50 Hz | commanded ve varsa measured angles | gövde/ayak kinematik prior |
| Odometri | 30–50 Hz | pose, twist, covariance | `odom -> base_link` |

Tek saat kaynağı Raspberry Pi monotonic clock olacak. ESP32 paketleri Pi'ye
ulaştığında sadece geliş zamanıyla etiketlenmemeli; mümkünse ESP32 sayaç zamanı ve
paket sıra numarası da taşınmalı, saat ofseti düzenli kestirilmelidir.

## 5. Çalışan sistemi koruma yöntemi

- Mevcut `apex_server.py`, kamera HUD'ı ve LiDAR motor kontrolü production yolu
  olarak kalacak.
- Yeni çalışma önce `mapping_v2/` altında bağımsız bir **sidecar** olacak.
- Sidecar fiziksel komut endpoint'lerine yazmayacak; yalnızca sensör verisi okuyacak.
- Kamera ikinci kez açılmayacak. Kareler mevcut tek kamera sahibinden paylaşımlı
  bellek/lokal socket veya GStreamer `tee` üzerinden alınacak.
- LiDAR seri portu ikinci kez açılmayacak. Command Center, taramaları zaman damgalı
  salt-okunur abone kanalında yayınlayacak.
- Yeni harita hazır olmadan mevcut HUD'daki 2D/3D görünüm değiştirilmemeli.
- Her aşama kayıt üzerinde offline geçmeden canlı sisteme bağlanmamalı.

## 6. Aşamalar ve kabul kapıları

### Aşama 0 — Veri sözleşmesi ve kayıt/tekrar oynatma

Omurilik kapalıyken yapılabilir.

- `mapping_v2` için sensör mesaj şemalarını tanımla.
- LiDAR, kamera, detection, servis durumu ve ileride IMU'yu tek zaman çizgisinde
  kaydeden bir recorder oluştur.
- Disk kotası, dosya döndürme ve temiz kapanma ekle.
- Aynı kayıt her çalıştırmada aynı çıktıyı vermeli.

**Kabul:** Mevcut servis PID'leri değişmeden en az 10 dakikalık kamera+LiDAR kaydı;
kare/tarama sıra boşlukları raporlanmış; replay sırasında fiziksel donanım erişimi yok.

### Aşama 1 — Kamera ve sensör dış kalibrasyonu

Omurilik kapalıyken büyük bölümü yapılabilir.

- 1280x720 için kamera intrinsics/distortion kalibrasyonu yap.
- LiDAR sıfır açısının robot +X yönü olup olmadığını ölç.
- `base_link -> lidar_link`, `base_link -> camera_link` ölçülerini cetvel/CAD ile
  doğrula; `provisional` değerleri ancak ölçümden sonra değiştir.
- Kamera–LiDAR extrinsic dönüşümünü hedef tahta/duvar verisiyle optimize et.
- Kalibrasyon dosyalarına sensör seri numarası, çözünürlük ve tarih yaz.

**Kabul:** Kamera reprojection hatası kayıtlı; düz duvar LiDAR bulutunda düz;
LiDAR noktalarının kamera üzerine projeksiyon hatası ölçülmüş ve tekrarlanabilir.

### Aşama 2 — Omurilik olmadan 3D dönüşüm motoru

- Tam SE(3) dönüşüm ve quaternion matematiğini ayrı, saf fonksiyonlar olarak yaz.
- Yapay pitch/roll/yaw ve kayıtlı taramalarla duvar/tavan/floor testleri oluştur.
- Voxel downsample, zaman aşımı, menzil/kalite filtresi ve point-cloud sınırı ekle.
- PLY/PCD dışa aktarma ve bağımsız doğrulama görüntüleyicisi oluştur.
- Mevcut HUD'a bağlama; önce offline sonuçları doğrula.

**Kabul:** Sentetik odada zemin, dört duvar ve tavan doğru eksen/yükseklikte;
ters eksen, derece/radyan ve dönüşüm sırası testleri otomatik geçiyor.

### Aşama 3 — Omurilik logic-only IMU köprüsü

Pil hazır olduğunda, servo güçleri kapalı/çıktılar disable durumunda yapılacak.

- ESP32'den ham accel+gyro, birleşik orientation, sayaç zamanı ve sequence yayınla.
- MPU eksenlerini REP-103/URDF eksenleriyle eşleştir.
- Sabit durumda bias/noise/covariance ölç; sıcaklık etkisini kaydet.
- Pitch/roll'u yerçekimiyle doğrula. Yaw drift hızını ölç ve mutlak yön gibi kullanma.
- Commanded Body IK ile measured IMU değerini ayrı alanlarda tut.

**Kabul:** 10 dakika sabit kayıt, paket kayıp ve jitter raporu; pitch/roll referans
açı testini geçiyor; hiçbir servo PWM üretilmiyor.

### Aşama 4 — Kontrollü tilt ile gerçek 3D LiDAR birikimi

- Robot mekanik olarak desteklenmişken ve yürüyüş kapalıyken küçük açılarla başla.
- Önce ±5°, sonra ±10° kontrollü pitch/roll taramaları kaydet.
- Her lazer noktasına ölçüm zamanı için enterpole IMU/body pose uygula.
- Gövde pivotundan kaynaklanan translation bileşenini kinematik modelle ekle.
- Düzlem uyumu ile duvar/tavan yüksekliğini metre cinsinden doğrula.

**Kabul:** Aynı duvar farklı tiltlerde çift duvar üretmiyor; tavan yüksekliği fiziksel
ölçüme belirlenen tolerans içinde; sabit sensörde Z gürültüsü raporlanmış.

> Üretim için tüm gövdeyi tarayıcı gibi eğmek yerine enkoderli bağımsız LiDAR tilt
> ekseni daha doğru ve güvenlidir. Gövde tilti önce algoritma doğrulama aracıdır.

### Aşama 5 — Güvenilir poz kestirimi ve 2D SLAM

- Eski özel ICP kodunu yeniden açma; yeni yol yan süreçte kurulacak.
- İlk hedef: `LaserScan + odom -> SLAM Toolbox -> map -> odom`.
- Odometri kaynağı için LiDAR scan matching ve kamera VO ayrı ayrı değerlendirilir.
- IMU + LiDAR/visual odometry, covariance ile EKF/UKF'de birleştirilir.
- Leg odometry yalnızca düşük güvenli prior olur; enkodersiz servo komutu ölçüm
  olarak etiketlenmez.
- Loop closure, yeniden lokalizasyon, harita kaydet/yükle ve kaçırılan tarama
  testleri yapılır.

**Kabul:** Oda çevresinde kapanan turda başlangıç/bitiş hata metriği belirlenmiş;
duvarlar çiftlenmiyor; kayıt tekrarında sonuç deterministik; harita yeniden yükleniyor.

### Aşama 6 — Kamera ile renkli ve semantik 3D harita

İki kalite seviyesi olacak:

1. **Mevcut donanımla gerçekçi hedef:** Tilt edilmiş A1 noktalarını kalibre kamera
   görüntüsüne projekte ederek seyrek fakat metrik renkli point cloud üretmek.
   Hailo detection sınıfları 3D landmark/etiket olarak eklenebilir.
2. **Yoğun 3D hedef:** Stereo veya RGB-D kamera ekleyip RTAB-Map ile renkli yoğun
   point cloud/mesh ve loop closure üretmek.

Tek RGB kamerada ORB-SLAM3 benzeri monocular/visual-inertial yöntemler poz ve
seyrek landmark üretir. Monocular depth AI ile daha dolu görüntü üretilebilir,
ancak derinlik yaklaşık olur; metrik güvenlik haritası yerine kullanılmamalıdır.

**Kabul:** Renk projeksiyonu kalibrasyon hedefinde doğrulanmış; 3D bulut ile 2D
navigasyon haritasının frame'leri uyuşuyor; dinamik insanlar kalıcı duvar olmuyor.

### Aşama 7 — Simülasyonda planlama ve otonomi

- Önce Gazebo'da aynı TF, sensör topic ve command contract kullanılacak.
- Nav2 global planner, local controller ve behavior tree kurulacak.
- Hexapod hareket katmanı `cmd_vel` isteğini mevcut gait komutlarına dönüştürecek.
- Costmap inflation, robot footprint, minimum geçiş genişliği ve durma mesafesi
  fiziksel ölçülerle ayarlanacak.
- Hedef seçimi: harita noktası, kişi takibi ve devriye görevleri aynı command
  arbiter üzerinden geçecek.
- AI karar verici doğrudan PWM/servo sürmeyecek; yalnızca doğrulanmış yüksek seviye
  hedef veya hız isteği üretecek.

**Kabul:** Simülasyonda hedefe gitme, hareketli engel, sensör kaybı, UI kapanması,
gecikme ve bozuk paket senaryoları güvenli biçimde geçiyor.

### Aşama 8 — Kademeli gerçek robot devreye alma

1. Logic-only HIL.
2. Mekanik olarak askıda tek komut zinciri.
3. Düşük hızlı açık alan testi.
4. Statik engel parkuru.
5. Dinamik insan/engel.
6. Haritada hedef ve yeniden lokalizasyon.
7. Kontrollü devriye ve görev yöneticisi.

Her kapıda fiziksel E-stop, command timeout, output-disable ve AI takip kapalı
başlangıç durumu doğrulanmadan sonraki adıma geçilmez.

## 7. Algoritma seçimi

| İhtiyaç | İlk tercih | Neden |
|---|---|---|
| Güvenilir 2D harita | SLAM Toolbox | ROS 2/Nav2 ile doğal, pose-graph ve kayıt/yükleme |
| Sensör füzyonu | robot_localization EKF | IMU ve odometriyi covariance ile birleştirir |
| Yoğun renkli 3D | RTAB-Map + RGB-D/stereo | Görsel loop closure ve metrik derinlik |
| Mevcut mono kamera ile VO | ORB-SLAM3 değerlendirmesi | Mono/visual-inertial poz ve loop closure; bulut seyrek |
| 3D ön işleme | PCL/Open3D veya eşdeğer C++ | Voxel, düzlem, outlier ve point-cloud araçları |
| Navigasyon | Nav2 | Planner/controller/BT ve costmap ekosistemi |

Cartographer 3D güçlüdür fakat A1M8'in tek düzlemi, eksik zaman/pose verisi ve Pi
işlem bütçesi nedeniyle ilk tercih değildir. Önce veri kalitesi ve TF zinciri
kanıtlanmadan ağır SLAM paketine geçilmeyecek.

## 8. Donanım kararları

- Mevcut A1M8, 2D navigasyon ve kontrollü tilt ile seyrek 3D için kullanılabilir.
- Gerçek zamanlı yoğun/renkli 3D isteniyorsa en büyük iyileştirme algoritmadan çok
  RGB-D/stereo sensör olur.
- MPU6050 pitch/roll için kullanılabilir; yaw driftini LiDAR/görsel odometri ve
  loop closure düzeltmelidir. İleride daha iyi timestamp'li bir IMU faydalıdır.
- Gövdeyi sürekli eğerek harita toplamak yürüyüş sırasında uygun değildir. Kalıcı
  çözüm için LiDAR'a enkoderli tilt ekseni veya doğrudan 3D/depth sensör düşünülmeli.
- Pi güç girişinde PD yazısı tek başına yeterli değildir: kaynak, kablo ve Pi giriş
  yolu birlikte yük altında 5 A profilini sürdürebilmelidir. `get_throttled` kayıtları
  kabul testlerinde izlenmeye devam edecektir.

## 9. İlk uygulanacak paket — omurilik kapalı

Sıradaki çalışma yalnızca Aşama 0 ve Aşama 1'in donanım-hareket gerektirmeyen
kısımlarıdır:

1. `mapping_v2` dizini ve salt-okunur veri sözleşmesi.
2. Mevcut kamera/LiDAR'ı ikinci kez açmadan recorder.
3. Replay aracı ve deterministik sentetik oda testi.
4. Tam SE(3) dönüşüm testleri.
5. Kamera kalibrasyon dosyası şablonu ve fiziksel ölçüm kontrol listesi.

Bu paket hiçbir motor/servo komutu göndermeyecek ve mevcut HUD'a bağlanmayacaktır.

## 10. Referanslar

- SLAMTEC A1M8 datasheet: https://bucket.download.slamtec.com/e9e096e9d9f30205d665260abe2cfb0c2dd62efa/LD108_SLAMTEC_rplidar_datasheet_A1M8_v1.0_en.pdf
- Nav2 mapping/localization: https://docs.nav2.org/setup_guides/sensors/mapping_localization.html
- SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox
- RTAB-Map ROS 2: https://github.com/introlab/rtabmap_ros/tree/ros2
- ORB-SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3
- ROS camera calibration: https://docs.ros.org/en/ros2_packages/rolling/api/camera_calibration/doc/components.html
- robot_localization: https://docs.ros.org/en/noetic/api/robot_localization/html/index.html

