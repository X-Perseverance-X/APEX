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
- Haritalama oturumu `BAŞLAT -> MAPPING -> BİTİR & KAYDET -> FROZEN` yaşam
  döngüsüne sahiptir. Başlat temiz bir alt harita açıp LiDAR'ı doğrular; bitir
  haritayı değişmez moda alır ve `.runtime/maps` altında atomik NPZ arşivi yazar.
- Bir engel dört ayrı taramada doğrulanınca kesin duvar olur. Geçici tek-tur
  noktaları kırmızı kalır; kesin duvar siyaha döner ve ancak üç ardışık serbest
  ışın kanıtıyla çözülür. Planlayıcı iki sınıfı da engel kabul eder.
- Kayıtlı haritalar konsoldan listelenip yeniden yüklenebilir. Yüklenen haritada
  rota kullanılmadan önce canlı LiDAR ile yerel poz yeniden güvenilirleşir.
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

1. Adlandırılmış haritalama oturumunu başlatır, bitirir, atomik kaydeder ve yükler.
2. Aktif ölçüm ile kesinleşmiş duvarı ayrı renklerde gösterir.
3. Occupancy grid üzerinde hedef seçer.
4. Rotayı ve uzunluğunu gösterir.
5. Bilinmeyen alan oranını raporlar.
6. Manuel onay veya otonom karar kapısını uygular.
7. 2D navigasyon, yerel/global SLAM, seyrek 3D ve kamera semantik füzyonu için gerçek
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
