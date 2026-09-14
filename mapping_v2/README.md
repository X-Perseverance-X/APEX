# APEX mapping_v2

`mapping_v2`, mevcut Command Center'ın sahip olduğu sensör verisini salt okunur
olarak tüketen haritalama ve rota önizleme katmanıdır.

- LiDAR seri portunu açmaz.
- Kamera cihazını ikinci kez açmaz.
- `/joy`, servo, gait veya body-IK uçlarını çağırmaz.
- Rota yürütme derleme zamanında `EXECUTION_LOCKED = True` durumundadır.
- LiDAR/IMU correlative scan matching güvenilirleşince harita `LOCAL_SLAM`
  olarak etiketlenir; loop closure olmadığı için `GLOBAL SLAM` sayılmaz.
- MPU6050 yaw değeri mutlak yön değil, kısa dönem scan-matching öncülüdür.
- Haritalama oturumları başlatılabilir, kesinleştirilebilir, atomik kaydedilebilir
  ve daha sonra yeniden yüklenebilir.

Koordinat sistemi: `+X ileri`, `+Y sol`, `+Z yukarı`. Zaman kaynağı Raspberry Pi
`monotonic_ns()` değeridir.

Şimdiki kabul kapsamı: hareketsiz platformda occupancy grid, hedef seçimi, A*
rota önizleme, manuel onay veya kural tabanlı otomatik karar. Gerçek hareket
aktüatör taşıması bu paketin dışında, güvenlik kapılarıyla `apex_server.py` içinde tutulur.
