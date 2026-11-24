# META-1 omurilik son bağlantı haritası

## Kamera pan/tilt — sağ PCA9685 `0x40`

PCA kanalları sıfırdan, kart üzerindeki servo soketleri birden numaralanır:

| Mekanizma | Kart soketi | PCA kanalı | Elektriksel yön | Yazılım sınırı |
|---|---:|---:|---|---:|
| Alt servo / PAN | 16 | CH15 | 0→180 giderken sola döner | 45°–135° |
| Üst servo / TILT | 15 | CH14 | 0° yukarı, 180° aşağı | 45°–135° |

Her iki servo açılışta 90° merkeze alınır. HUD içinden merkezleme ve yalnız bu iki kanalın PWM'ini kesme mümkündür.

Kamera CH14/15'i aldığı için robot kol eşlemesi çakışmayacak şekilde şöyledir: BASE CH12, JOINT1 CH13, JOINT2 CH3, GRIPPER CH7.

## ESP32-S3 ana pinler

| İşlev | GPIO / hat |
|---|---|
| I2C SDA | GPIO8 |
| I2C SCL | GPIO9 |
| GPS — ESP RX / GPS TX | GPIO16 |
| GPS — ESP TX / GPS RX | GPIO17 |
| DFPlayer — ESP TX / DF RX | GPIO4 |
| DFPlayer — ESP RX / DF TX | GPIO5 |
| Lazer PWM | GPIO14 |
| Kasa fanı PWM | GPIO18 |
| NTC ADC | GPIO15 |
| Sağ pil ADC | GPIO1 |
| Sol pil ADC | GPIO2 |

Beklenen I2C adresleri: sağ PCA `0x40`, sol PCA `0x41`, ToF `0x29`, MPU6050 `0x68`.

SDA/SCL ters bağlanırsa bu dört aygıtın tamamı taramada kaybolur; tek bir aygıtın kaybı genellikle SDA/SCL tersliğinden değil o aygıtın beslemesi/adresi/temasından kaynaklanır.

## Ayak mikro-switch bağlantıları

| Ayak | GPIO |
|---|---:|
| RF — sağ ön | 6 |
| RM — sağ orta | 7 |
| RR — sağ arka | 10 |
| LF — sol ön | 11 |
| LM — sol orta | 12 |
| LR — sol arka | 13 |

Her switch için `C → GND`, `NO → ilgili GPIO`, `NC → boş`. Firmware `INPUT_PULLUP` kullanır; basılınca hat LOW olur. Harici 3.3 V bağlanmaz.

## LiDAR takılınca reset / kesik dönüş

LiDAR motorunun atlayarak dönmesi, LED'lerin yanıp sönmesi ve ESP reseti veri/yazılım belirtisi değil; ortak beslemede ani gerilim çökmesi veya kötü ortak toprak belirtisidir. Yazılım LiDAR motorunu açılışta kapalı tutar, fakat takıldığı anda USB beslemesi çöküyorsa yazılım bunu düzeltemez.

Günü kurtaran güvenli bağlantı:

1. LiDAR'ı kendi adaptörlü USB hub üzerinden besle.
2. Hub–Pi arasında veri bağlantısı kalsın; ayrı 5 V'u Pi'nin 5 V hattına geri besleme.
3. ESP/PCA servo beslemesini Pi USB hattından alma; yalnız ortak GND kullan.
4. LiDAR motorunu HUD'dan, Pi ve ESP tamamen açıldıktan sonra başlat.
5. Pi erişilir olduğunda `vcgencmd get_throttled` sonucu `0x0` değilse önce güç yolunu düzelt; `usb_max_current_enable=1` düşük voltajı çözmez ve zayıf beslemeyi daha fazla zorlayabilir.

