# APEX mapping_v2

`mapping_v2`, mevcut Command Center'ın sahip olduğu sensör verisini salt okunur
olarak tüketen haritalama ve rota önizleme katmanıdır.

- LiDAR seri portunu açmaz.
- Kamera cihazını ikinci kez açmaz.
- `/joy`, servo, gait veya body-IK uçlarını çağırmaz.
- Rota yürütme derleme zamanında `EXECUTION_LOCKED = True` durumundadır.
- LiDAR/IMU correlative scan matching güvenilirleşince harita `LOCAL_SLAM`
  olarak etiketlenir.
- Seyrek anahtar-kareler bir SE(2) pose graph içinde tutulur. Eski bir konuma
  dönüş yalnızca ICP örtüşme, hata ve düzeltme kapılarının tamamını geçerse loop
  constraint olur. Harita bitirilirken grafik optimize edilir ve occupancy grid
  düzeltilmiş pozlardan yeniden kurulur.
- `GLOBAL_SLAM` etiketi yalnızca en az bir doğrulanmış loop closure optimize
  edildiyse kullanılır; yakınlık tek başına yeterli değildir.
- MPU6050 yaw değeri mutlak yön değil, kısa dönem scan-matching öncülüdür.
- Haritalama oturumları başlatılabilir, kesinleştirilebilir, atomik kaydedilebilir
  ve daha sonra yeniden yüklenebilir.

Koordinat sistemi: `+X ileri`, `+Y sol`, `+Z yukarı`. Zaman kaynağı Raspberry Pi
`monotonic_ns()` değeridir.

Şimdiki kabul kapsamı: occupancy grid, yerel tarama eşleştirme, harita bitiminde
global loop closure/pose-graph düzeltmesi, hedef seçimi ve A* rota önizleme.
Gerçek hareket aktüatör taşıması bu paketin dışında, güvenlik kapılarıyla
`apex_server.py` içinde tutulur.
