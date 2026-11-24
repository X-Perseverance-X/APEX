# v57.4: hareket zamanlaması ve kamera dönüşü

Kullanıcı hareket sırasında ciddi titreme/ısınma, çok az gerçek hareket
bildirdi. 24 sabit STAND örneğinde 18 komut açısının değişimi 0.0 dereceydi;
gimbal çıktıları sıfırdı. Son denemede failsafeCount=1 ve komut zaman aşımı
kaydı görüldü. Bunlar mekanik/elektriksel nedeni tek başına kanıtlamaz.
Isınma bildirimi üzerine /legOff ile tüm bacak PWM'leri kapatıldı ve
DISARMED / outputsArmed=false doğrulandı. Fiziksel test yapılmadı.

## Değişiklikler

- Kamera kaynağı Hailo'da hem yatay hem dikey çevriliyor: 180 derece.
  İşlem algılama/HUD çiziminden önce; seçim koordinatları ve HUD aynı görüntüde.
  CSI sağlık kontrolü 1280x720, yaklaşık 29.9 kaynak FPS; JPEG incelendi.
- PWM 50 Hz, servo kanal/ofset/min/max ve açı->darbe hesabı aynıdır.
- PCA9685 aynı darbeyi donanımda tekrarlar. Değişmeyen bacak PWM değeri I2C'ye
  tekrar yazılmaz; yalnız başarılı yazım önbelleklenir, kapatmada geçersizlenir.
- Normal hareket filtresi geçen süreye göre ölçeklenir. 20 ms davranışı aynı;
  geciken karelerde tek seferlik ilerleme en fazla 40 ms'dir.
- Yürüyüş fazı sabit 20 ms varsayımı yerine ölçülen kare süresini kullanır.
- Düşük joystick büyüklüğünde kaldırma yüksekliği de azaltılır. 0 girişte
  kaldırma sıfıra döner; >=0.35 girişte eski tam yükseklik korunur.
- Önceki aşırı yavaş kalkış yerine 3 saniyelik S-eğrisi, 30 deg/s sınır.
  Açılış DISARMED kalır; kayıtlı HOME otomatik hareket başlatmaz.

## Doğrulama

78 mevcut test geçti. Filtrenin 20 ms eşdeğerliği ve 20+20/40 ms zaman
tutarlılığı kontrol edildi. ESP derlemesi: 1,097,259 bayt, RAM 49,456 bayt.
Fiziksel titreme/ısınmanın giderildiği henüz doğrulanmadı. Soğumadan ve
operatör onayı olmadan PWM yeniden açılmayacak.

ESP'den yeni yedek OKUNMADI. Önceki v57.3 kaynak/bin dosyaları yerelde
artifacts/repair_20260911_before/v57_3_before_cadence.* olarak saklandı.
Derleme için mevcut artifacts/v57_3_smooth_rise_build önbelleği kullanıldı;
bu klasörün son bin dosyası artık v57.4'tür.

Yükleme tamamlandı; esptool uygulama hash doğrulaması geçti. Canlı sürüm
v57.4-motion-cadence; DISARMED, outputsArmed=false, homeSet/configValid=true,
iki PCA online. Kullanıcının son z=-145, len=50, lift=40, spd=2000 ayarları
PWM kapalıyken geri uygulandı. Kamera 1280x720 / 29.93 kaynak FPS, fresh=true.
Yeni firmware ile motor hareketi başlatılmadı; termal/mekanik doğrulama bekliyor.

## Teknik kaynaklar

- https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
- https://adafruit.github.io/Adafruit-PWM-Servo-Driver-Library/html/class_adafruit___p_w_m_servo_driver.html
- https://github.com/raspberrypi/picamera2/blob/main/examples/rotation.py
- https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all

Pillerin dolu olması servo güç hattının yük altındaki gerilimini/akımını,
mekanik sürtünmeyi veya servo arızasını tek başına doğrulamaz. Sorun sürerse
serbest tek eklemde PWM dalga biçimi ve servo beslemesi ölçülerek ayrıştırılmalı.
