# META-1 Omurilik Güvenliği ve Kontrollü Kalkış Yol Haritası

## Amaç

ESP32-S3 omurilik açıldığında robot kendi kendine hareket etmeyecek. Bacak ve kol
PWM çıkışları ancak operatörün açık komutu, güncel bağlantı ve gerekli güvenlik
kontrolleri birlikte sağlandığında etkinleşecek. Ayağa kalkma tek bir HOME komutu
olmayacak; ölçülebilir, iptal edilebilir ve telemetriden izlenebilir fazlardan
oluşacak.

Bu çalışma, kamera/LiDAR ve çalışan Raspberry Pi servislerinden ayrıdır. İlk
firmware denemeleri servo gücü kapalıyken veya robot güvenli bir sehpa üzerinde
yapılacaktır.

## 1. Doğrulanan mevcut durum

### Kaynak sürümleri

- Pi üzerindeki `v54_meta1_integrated.ino`, yerel `v54` kaynağıyla SHA-256
  seviyesinde aynıdır.
- Pi üzerinde ayrıca `v55_meta1_imu_yaw_battery_cal.ino` bulunur.
- Yerel geliştirme ağacında `v56_radio_sil_foundation.ino` bulunur. Bu sürüm
  komut deadman'i, sınır doğrulamaları ve kalıcı çalışma-süresi PWM maskesi gibi
  iyileştirmeler içerir; fakat gerçek kart toolchain'iyle derlenip HIL testinden
  geçirilmemiştir.
- ESP kapalıyken karta en son hangi dosyanın flashlandığı kaynak dosyalarından
  kesin olarak anlaşılamaz. Firmware kimliği telemetriye eklenmeden bu ayrım
  güvenilir değildir.

### Otomatik kalkışın kesin nedeni

`v54`, `v55` ve yerel `v56` aynı temel davranışı taşır:

1. NVS içinde `homeSet=true` ise `setup()` sonunda mod `HOME_RISE` seçilir.
2. İlk 20 ms kontrol karesinde bacak çıkışlarına PWM yazılır.
3. `firstRun` yolu, kayıtlı HOME açılarını doğrudan 18 kanala uygular.
4. Sonraki karelerde IK stand hedefi çalışır.
5. Ardından A ve B tripodları sırayla yerden kaldıran iki ek faz yürütülür.

Mevcut üç saniyelik `bootSoftStart`, ilk PWM etkinleştirmesindeki bilinmeyen
mekanik açı problemini çözmez. Enkoderi olmayan bir servo, PWM kapalıyken gerçek
açısını bildirmez. Yazılımın tahmin ettiği açı ile gerçek açı farklıysa ilk
etkinleştirmede ani hareket olabilir.

### IK incelemesi

- Kanonik eksenler `+X ileri, +Y sol, +Z yukarı` olarak tutarlıdır.
- Bacak sırası `RF, RM, RR, LF, LM, LR`; tripodlar `RF+RR+LM` ve
  `RM+LF+LR` şeklindedir.
- Sağ/sol servo aynalama, firmware, Kinematics Lab ve URDF sözleşmesinde
  tutarlıdır.
- Mevcut 34 yerel test geçmektedir. Bunların içinde varsayılan duruş ve yürüyüş
  çalışma alanında IK/FK tersleme testleri vardır.
- Kayıtlı varsayılan HOME yaklaşık `Y=165 mm, Z=-100 mm`; çalışma IK standı
  yaklaşık `Y=150 mm, Z=-100 mm` üretir. Dolayısıyla HOME ve STAND aynı kavram
  değildir ve ayrı adlandırılmalıdır.
- Matematik testi fiziksel horn açısını, tibia yönünü, kablo eşlemesini, servo
  yükünü veya ayağın zemine temasını doğrulamaz.

### Mevcut güvenlik açıkları

- Bacak PWM maskesi açılışta tüm kanallar için `true` başlar.
- `DISARMED` ile `IDLE/STAND` birbirinden ayrılmamıştır; `IDLE` hâlâ IK ve PWM
  üretir.
- `/goHome`, açı yumuşatma yolunu atlayarak 18 kanala doğrudan PWM yazabilir.
- `/mode?m=home_rise`, kalkış fazlarını tutarlı biçimde yeniden başlatmadan moda
  geçebilir.
- `/legOn` sonrası gerçek servo açısı bilinmeden bir sonraki hedef uygulanabilir.
- `/bodyIk`, `/params`, manuel servo ve HOME kayıt uçları ayrı bir arm/authority
  kilidine bağlı değildir.
- Deadman yalnız yürüyüş modunu durdurur; genel çıkış yetkilendirmesi değildir.
- Batarya ve IMU verileri telemetride vardır fakat yeni kalkışa izin verme
  koşulu olarak kullanılmaz.
- NVS'den yüklenen kalibrasyon değerleri açılışta bütünsel bir yapılandırma
  doğrulamasından geçirilmez.
- Firmware sürüm/derleme kimliği telemetride bulunmadığı için sahadaki binary
  kesin olarak tanımlanamaz.

## 2. Hedef durum makinesi

```text
BOOT
  |
  | donanım başlat, tüm bacak/kol PWM kapalı
  v
DISARMED -------------------------------> FAULT_LATCHED
  |  geçerli config + PCA + IMU           ^
  |  nötr komut + operatör ARM            | kritik invariant
  v                                        |
ARMED_WAIT_SUPPORT                         |
  | operatör: robot sehpa üzerinde          |
  | PREPARE onayı                           |
  v                                        |
POSE_CAPTURE / PRELOAD --------------------+
  | sıralı, sınırlı eklem etkinleştirme
  v
CROUCH_HOLD
  | operatör STAND + güncel heartbeat
  v
STAND_RAMP
  | ayaklar yerde, minimum-jerk Z rampası
  v
STAND_HOLD <----> WALK_ENABLED
  |
  | SIT
  v
CROUCH_HOLD -> DISARMED

Link kaybı: WALK_ENABLED -> FAILSAFE_HOLD -> STAND_HOLD
Hareket durdur: hız sıfır + sağlam destek duruşu
TORK KES: ayrı, açık ve kilitli komut; yük altındayken robot düşebilir
```

### Değişmez güvenlik kuralları

1. Açılış durumu her zaman `DISARMED`; kayıtlı HOME otomatik hareket başlatmaz.
2. `DISARMED` durumunda bütün bacak ve kol kanalları gerçekten OFF kalır.
3. `ARM` tek başına hareket değildir; yalnız sonraki hazırlık komutuna izin
   veren kısa ömürlü operatör yetkisidir.
4. Her hareket komutu doğru session, artan sequence ve güncel heartbeat ister.
5. Eski, tekrarlı, bozuk veya başka oturuma ait komut süreyi yenilemez.
6. Yürüyüşe geçiş için joystick tam nötr olmalıdır.
7. Kalkış sırasında yeni body IK/gait komutları reddedilir veya kuyruğa alınmadan
   açık hata döndürür.
8. Her faz süre aşımı, IMU eğim sınırı, config hatası veya operatör iptalinde
   belirlenmiş güvenli geri dönüşe gider.
9. PWM OFF ile STOP aynı şey değildir. Ayakta robotta bağlantı kaybı torku
   kesmez; yürüyüşü durdurup destek duruşunu korur.
10. Servo geri bildirimi olmadığı için rastgele güç-kesik pozdan kusursuz ve
    darbesiz kalkış garanti edilemez. İlk tam etkinleştirme sehpa üzerinde yapılır.

## 3. Pozların açık ayrımı

Tek `HOME` kavramı kaldırılacak veya geriye uyumluluk alias'ına indirgenecek:

| Poz | Anlam | Kullanım |
|---|---|---|
| `MOUNT` | Horn takma/elektriksel kalibrasyon uçları | Yüksüz tek servo testi |
| `PARK` | Güç kesilmeden önce mekanik saklama pozu | Oturma/kapatma |
| `CAPTURE` | Güç verildiğinde robotun elle yerleştirildiği bilinen poz | İlk PWM yakalama |
| `CROUCH` | Altı ayağın yerde olduğu alçak ve geniş destek pozu | Kalkış öncesi/iniş sonrası |
| `STAND` | Normal yükseklik ve destek geometrisi | Bekleme/yürüme başlangıcı |

Bu pozların 18 servo açısı yalnız mekanik doğrulamadan sonra kaydedilir.
Başlangıçta sayı tahmin edilmez. Kinematics Lab aday açıları hesaplar; fiziksel
sehpa testi nihai değerleri belirler.

## 4. Kontrollü kalkış fazları

### Faz A — PRECHECK (PWM kapalı)

- Firmware kimliği, config sürümü ve reset nedeni yayınlanır.
- İki PCA9685 adresi görülmelidir.
- MPU hazır ve gyro kalibrasyonu tamamlanmış olmalıdır.
- Kalibrasyon/HOME/PARK verisinin CRC veya config sürümü geçerli olmalıdır.
- Joystick ve AI/autonomy komutları nötr olmalıdır.
- Link yaşı izin verilen sınırda olmalıdır.
- Batarya eşikleri gerçek multimetre kalibrasyonundan sonra gate yapılır.
- Başlangıç roll/pitch sınırı ancak IMU montaj yönü doğrulandıktan sonra açılır.

### Faz B — CAPTURE/PRELOAD (robot sehpa üzerinde)

- Operatör robotun fiziksel olarak `CAPTURE` pozuna yerleştirildiğini onaylar.
- Coxa, femur ve tibia kanalları önceden doğrulanmış sırayla etkinleştirilir.
- Her adım arasında zaman, açı farkı ve iptal kontrolü bulunur.
- Servo akım sensörü mevcut değilse ilk devreye alma tek bacak/tek eklem ile
  yapılır; yazılım akımı varmış gibi davranmaz.
- İlk gerçek testte bütün 18 kanal aynı anda etkinleştirilmez.

### Faz C — CROUCH_HOLD

- Altı ayak zeminde kalacak şekilde `CAPTURE -> CROUCH` minimum-jerk geçişi.
- Body roll/pitch/yaw sıfır hedefte tutulur; gimbal/denge geri beslemesi ilk
  commissioning sırasında kapalıdır.
- Operatör onayı olmadan sonraki faz başlamaz.

### Faz D — STAND_RAMP

- Ayak XY hedefleri sabit tutulur.
- Gövde yüksekliği `Z_crouch -> Z_stand` boyunca sınırlı hız ve ivmeyle artar.
- Mevcut otomatik tripod kaldırma fazları bu kalkıştan çıkarılır. Ayak kaldırma,
  ancak ayrı gait testinde kullanılacaktır.
- Her kontrol karesinde altı bacağın hedefi önce hesaplanır. Herhangi biri
  geçersiz/ulaşılamaz/limit dışıysa o karede hiçbir bacağa yeni komut uygulanmaz.
  Böylece kısmi poz uygulanması engellenir.
- Süre aşımı veya eğim hatası halinde hız sıfırlanır; mümkünse CROUCH'a kontrollü
  dönüş, değilse `FAILSAFE_HOLD` uygulanır.

### Faz E — STAND_HOLD

- İlk ayağa kalkış başarıyla bitse bile yürüyüş otomatik başlamaz.
- Ayrı `ENABLE_WALK` komutu, nötr joystick ve güncel heartbeat gerekir.
- Body IK ve IMU dengeleme ayrı yetki bayraklarıdır; stand başarısı bunları
  kendiliğinden açmaz.

## 5. Firmware uygulama sırası

### S0 — Kaynak dondurma ve kimlik

- Pi'deki `v54` ve `v55` arşivlenir; yerel `v56` değiştirilmeden referans kalır.
- Yeni çalışma ayrı `v57_spine_safe_boot` klasöründe yapılır.
- Telemetriye `firmwareVersion`, kısa git/source hash, build zamanı ve reset
  nedeni eklenir.
- Karta flashlanan `.bin` ve kaynak aynı release klasöründe hashleriyle saklanır.

### S1 — Güvenli boot

- Varsayılan bütün output maskeleri `false` olur.
- PCA başlatılır başlatılmaz tüm kullanılan kanallara OFF uygulanır.
- Başlangıç modu koşulsuz `DISARMED` olur.
- NVS `homeSet` yalnız verinin varlığını belirtir; mod seçmez.
- `/goHome` ve doğrudan `home_rise` hareketi devre dışı bırakılır.

### S2 — Test edilebilir saf durum makinesi

- Durum geçiş mantığı donanım yazma kodundan ayrılır.
- Boot, ARM, yanlış sıra, timeout, iptal, eski paket, config fault ve tekrar boot
  senaryoları bilgisayarda birim testlerinden geçer.
- Çıkış üretme kararı tek merkezden verilir; HTTP endpointleri doğrudan
  `setPWM()` çağıramaz.

### S3 — Poz ve trajectory katmanı

- `CAPTURE`, `CROUCH`, `STAND`, `PARK` yapılandırmaları sürümlü ve doğrulamalı
  saklanır.
- Tüm 18 hedef atomik hesaplanır ve doğrulanır.
- Minimum-jerk trajectory; eklem hız, ivme ve kare başına açı sınırı uygular.
- `/goHome` geçici olarak yeni `requestStand()` akışına yönlendirilir; doğrudan
  PWM yazma kaldırılır.

### S4 — Pi/HUD yetkilendirmesi

- HUD'da belirgin durum: `DISARMED`, `ARMED`, `CROUCH`, `RISING`, `STAND`,
  `WALK`, `FAILSAFE`, `FAULT`.
- `ARM`, `PREPARE`, `STAND`, `SIT`, `STOP MOTION`, `TORQUE OFF` birbirinden
  ayrılır.
- Tehlikeli komutlar onay ister; fakat acil hareket durdurma tek tık olur.
- Pi yeniden başlasa bile ESP kendi başına hareket etmez.

### S5 — Logic-only HIL

- ESP ve PCA mantık gücü açık, servo V+ kapalı.
- Osiloskop/logic analyzer veya PCA register okumasıyla bootta PWM olmadığı
  doğrulanır.
- Pi kapanması, Wi-Fi kesilmesi, endpoint tekrarı ve reset enjeksiyonu denenir.
- Telemetri state/fault/sequence değerleri kaydedilir.

### S6 — Kademeli fiziksel commissioning

1. Servo rayı kapalı; I2C, IMU, reset nedeni, config ve batarya doğrulaması.
2. Sehpa üzerinde tek servo, küçük açı farkı.
3. Her eklem için yön/kanal/horn tablosu imzalanır.
4. Sehpa üzerinde bir bacak.
5. Sehpa üzerinde altı bacak ve `CAPTURE -> CROUCH`.
6. Yerde fakat gövde güvenlik askısında düşük hızlı `CROUCH -> STAND`.
7. Kontrollü `STAND -> CROUCH -> PARK`.
8. En son, ayrı yetkiyle küçük adım gait testi.

## 6. Omurilik ile 3D haritalama bağlantısı

Omurilik güvenliği tamamlanınca haritalama yalnız şu verileri salt-okunur alır:

- zaman damgalı IMU gyro/accel ve filtrelenmiş roll/pitch,
- firmware boot/session kimliği,
- komutlanan body pose,
- ileride varsa gerçek joint/servo geri bildirimi,
- robot state ve fault bayrakları.

Komutlanan body IK açısı ile ölçülen IMU açısı ayrı alanlar olacaktır. 3D harita
gerçek yönelim için IMU/odometri kullanacak; `istenen pitch/roll` değerini gerçek
ölçüm gibi kullanmayacaktır.

## 7. Güç vermeden önce gerekenler

- Robotun gövdesi, bacaklar hareket etse bile yere düşmeyecek sağlam bir sehpa
  veya askıda olmalı.
- Servo V+ hattını ESP/PCA logic gücünden ayrı açıp kapatabilmek gerekir.
- Fiziksel acil durdurma servo güç rayını kesebilmelidir.
- PCA9685 `OE` pinlerini kullanma imkânı kontrol edilmelidir. Uygunsa donanımsal
  pull-up ile açılışta çıkışların kapalı kalması yazılım hatasına karşı ikinci
  güvenlik katmanı sağlar.
- İlk kısa açılış, mevcut unsafe firmware ile servo gücü bağlıyken yapılmaz.
  Yalnız ESP/PCA logic veya fiziksel olarak yüksüz/askıda düzen kullanılır.

## 8. Kabul kriterleri

- 20 ardışık power-cycle boyunca operatör komutu olmadan tek PWM hareketi yok.
- Pi kapalıyken ve Wi-Fi yokken robot DISARMED kalır.
- Kayıtlı HOME mevcutken de otomatik kalkış yok.
- Reset, brownout ve watchdog sonrası otomatik kalkış yok.
- Hatalı/eksik/eski komut hiçbir hareket üretmez.
- Her kalkış fazı telemetride adı, ilerlemesi ve hata nedeni ile görünür.
- Her faz operatör tarafından iptal edilebilir.
- `STOP MOTION` yürüyüşü durdurur ve desteği korur.
- `TORQUE OFF` açıkça ayrı ve kilitli davranır.
- IK toplu doğrulama başarısızsa altı bacağın hiçbiri o karede güncellenmez.
- Kalkış, oturuş ve link-kaybı senaryoları SIL + logic-only HIL + askılı fiziksel
  test sırasıyla geçmeden serbest yürüyüş açılmaz.

## 9. 2026-08-28 uygulama ve canlı doğrulama kaydı

- ESP32-S3 N16R8 donanımı ROM üzerinden doğrulandı: 16 MB flash, 8 MB OPI PSRAM.
- Eski v55'in 16 MB tam flash yedeği alındı:
  `artifacts/esp32s3_n16r8_pre_v57_full_flash_20260828.bin`.
- v57, mevcut 4 MB çift-OTA partition düzeni korunarak derlendi ve karta yazıldı.
- Yazma sonrası NVS'nin 20.480 baytlık `0x9000..0xDFFF` bölgesi eski yedekle
  SHA-256 düzeyinde birebir aynı kaldı.
- Çevre kartları enerjisizken VL53L1X kütüphanesindeki sınırsız init döngüsü
  bulundu. v57 artık önce sınırlı I2C taraması yapıyor; yalnız cevap veren
  MPU/TOF'u başlatıyor.
- Canlı seri açılış doğrulaması:
  `v57.0-spine-safe-boot`, Wi-Fi `192.168.7.50`, `state=DISARMED`,
  `outputsArmed=false`, iki PCA `offline`, config `valid`.
- Canlı HTTP doğrulaması: onaysız kalkış `400`, onaylı fakat HOME kaydı yokken
  `409`; hiçbir çıkış etkinleşmedi.
- Pi HUD butonu `CAPTURE -> STAND` olarak güncellendi ve `confirm=start`
  parametresini geçiriyor. Boşta 10 Hz joystick seli kaldırıldı; telemetri,
  leg-status ve joystick tekrarları kontrol komutunun önüne yığılmıyor.
- Yerel test paketi: 41/41 başarılı. N16R8 derlemesi: program %83, RAM %15.

## 10. Bir sonraki güvenli adım

Canlı telemetride `homeSet=false` görülmüştür. Bu nedenle varsayılan açıları
mekanik gerçek kabul edip kalkış kilidini otomatik açmak yasaktır. Sonraki oturum:

1. Servo V+ kapalıyken omuriliğin I2C logic gücünü aç.
2. Telemetride PCA `0x40/0x41` ve MPU `0x68` çevrimiçi olduğunu doğrula.
3. Gövde destek çıkıntıları üzerinde, tek eklem PWM ve küçük hareketlerle kanal,
   yön, min/max ve gerçek CAPTURE açılarını doğrula.
4. Altı bacağın doğrulanan CAPTURE değerlerini HOME olarak kaydet; `homeSet=true`
   telemetrisini kontrol et.
5. Servo gücü ve askı varken önce CAPTURE, sonra 3 saniyelik STAND rampasını test
   et. İlk testte gimbal kapalı kalır.
6. STAND güvenli olduktan sonra gimbali aç; burun yukarı testinde ön bacak Z'nin
   azalması, arka bacak Z'nin artması gözlenir. Roll yönüne dokunulmaz.

Bu kapılar geçilmeden serbest yürüyüş veya otonom hareket açılmaz.
