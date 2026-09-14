# META-1 mapping_v2 — yerel SLAM ve güvenli rota katmanı

## Çalışan kapsam

- Command Center'ın paylaştığı RPLIDAR taramasından 5 cm çözünürlüklü, 12 x 12 m
  sınırlı aktif alt harita.
- IMU kısa dönem yaw değişimini öncül kabul eden, LiDAR olasılık ızgarasına karşı
  kaba-ince correlative scan matching. IMU işareti ilk güvenilir eşleşmede seçilir.
- Zayıf eşleşen tarama haritaya basılmaz; böylece dönüşte çift duvar/spiral oluşmaz.
- Her taramada hücre başına tek hit/miss güncellemesi, hit önceliği ve 5 cm geniş
  serbest ışın temizliği. Daha uzaktaki yeni dönüş eski yakın engeli tek turda
  rota katmanından kaldırabilir.
- Gürültü eskimesi tarama sayısına değil monotonic geçen zamana bağlıdır; güçlü,
  tekrar görülen duvarlar tek-tur parazitlerinden daha uzun korunur.
- LiDAR seri portu ve kamera aygıtı için tek-sahip ilkesi korunur.
- TOF geçerli olduğunda ön engel hücresi olarak füzyon sözleşmesine girer.
- IMU roll/pitch/yaw zaman damgalı alınır. MPU6050 yaw mutlak pusula sayılmaz;
  yalnız ardışık LiDAR eşleştirmesinin kısa dönem dönüş öncülüdür.
- Kamera ikinci kez açılmadan yalnız mevcut `/health` durumu okunur.
- Haritada tıklanan hedefe robot ayak izi şişirmeli, sekiz komşulu A* rota.
- Manuel onay ve kural tabanlı otonom karar modu.
- Haritalama paketi aktüatör endpoint'i veya donanım sahibi içermez; yürütme
  sunucudaki mevcut ayrı güvenlik kapısından geçer.
- SE(3) dönüşüm ve bounded voxel cloud temeli hazırdır. Ölçülmüş extrinsics ve
  geçerli IMU olmadan canlı 3D buluta veri eklenmez.

## Arayüz

Ana HUD'daki mevcut 2D/3D LiDAR görünümleri değiştirilmedi. LiDAR kontrol
şeridindeki `ROTA ↗` düğmesi `/navigation-console` sayfasını açar.

Navigasyon konsolu:

1. Occupancy grid üzerinde hedef seçer.
2. Rotayı ve uzunluğunu gösterir.
3. Bilinmeyen alan oranını raporlar.
4. Manuel onay veya otonom karar kapısını uygular.
5. Fiziksel hareketin kilitli olduğunu sürekli gösterir.
6. 2D navigasyon, global SLAM, seyrek 3D ve kamera semantik füzyonu için gerçek
   hazır/engelleyici nedenleri listeler.

## Canlı ilk doğrulama

- Pi ve sensör servisleri açık, omurilik çıkışları `DISARMED` ve
  `outputsArmed=false` durumundaydı.
- Kamera 1280 x 720 @ yaklaşık 30 FPS çalıştı.
- LiDAR yaklaşık 5 harita güncellemesi/s üretti.
- Platform/masa çevresindeki yakın engeller nedeniyle deneme hedefleri
  `GOAL_BLOCKED` veya `NO_ROUTE` olarak reddedildi; sahte güvenli rota çizilmedi.
- Servo, gait, gimbal, kol veya joystick komutu gönderilmedi.

## Bilinçli sınırlar ve sonraki kalibrasyon kapıları

Bu, Cartographer'ın temel yerel-SLAM veri akışını Pi için küçük ve bağımlılığı az
biçimde uygular; henüz pose-graph/loop closure içeren tam Cartographer değildir.
Kamera sağlık bilgisi füzyon sözleşmesindedir fakat ölçülmüş intrinsics/extrinsics
olmadan kamera pikselleri metrik haritaya yazılmaz. Yanlış ölçekli "görsel SLAM"
üretmek yerine aşağıdaki kapılar korunur.

1. Kamera intrinsics: 1280 x 720 için checkerboard kalibrasyonu.
2. Fiziksel `base_link -> lidar/camera/tof/imu` X/Y/Z ve R/P/Y ölçümleri.
3. Omurilik logic-only açıkken zaman damgalı IMU bias/covariance kaydı.
4. Kontrollü tilt kayıtlarıyla SE(3) projeksiyon ve düzlem uyumu.
5. Güvenilir LiDAR/visual odometry; ardından `LOCAL_ONLY -> ODOMETRY/SLAM` geçişi.
6. Kamera detection sonuçlarının yapılandırılmış semantik landmark katmanına
   aktarılması.
7. Önce SIL/Nav2; sonra askıda HIL. Gerçek yürütme ancak ayrı operatör izni ve
   güvenlik kabul kapılarıyla eklenecek.
