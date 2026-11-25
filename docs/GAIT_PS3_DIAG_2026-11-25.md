# Yürüyüş ve PS3 giriş incelemesi — 2026-11-25

## Canlı bulgular

- Pi'ye `apex.local` üzerinden SSH ve 8080 sağlık kontrolü çalıştı.
- ESP USB telemetrisi canlıydı; inceleme başında `FAILSAFE`, PWM açık, PCA kartları çevrimiçiydi.
- Kablosuz PS3 kolu BlueZ'de bağlı ve `/dev/input/js0` olarak görünüyordu. Hareket servisi durduruldu.
- Ham joystick ölçümünde üç gerçek tuş basışına karşı 411 tuş olayı vardı. Yalnız L1 tutulduğunda başka tuşlar da çok sayıda yanlış olay üretti. Bluetooth oturumu yeniden kurulduktan sonra bile kol dokunulmazken 12 saniyede 135 tuş değişimi sayıldı.
- Bu giriş güvenli bir deadman kaynağı değildir. `apex-ps3-gateway` yeniden açılışta da sahte komut üretmesin diye `disabled` ve `inactive` bırakıldı; USB kablolu karşılaştırma henüz yapılamadı. Doğrulamadan sonra `sudo systemctl enable --now apex-ps3-gateway` ile geri açılabilir.
- Pi'deki PS3 gateway yedeklenip giriş gürültüsü koruması yüklendi. Bir saniyede sekiz tuş değişimi görüldüğünde hareketi durdurur ve üç saniye sessiz giriş bekler; hareket komutu onayı başarısız olursa WALK durumunda kalmaz. Bu koruma kablosuz kolu kullanılabilir ilan etmez; servis hâlâ kapalıdır.
- USB köprüsünde 15 salt-okunur `PARAMS` çağrısı 12–50 ms sürdü; 350 ms üstü örnek görülmedi. Canlı ayarlar `z=-132`, `len=50`, `lift=40`, `spd=2000` idi.

## Hazırlanan firmware

`v57.32-body-frame-gait` kaynak kodu derlendi ve 26 ilgili sözleşme testi geçti. İleri/geri hedefler IK'nin beklediği gövde X eksenine doğrudan veriliyor. Varsayılan 0.18 joystick büyüklüğünde adım uzunluğu yaklaşık 9 mm yerine 36 mm, nominal ayak kaldırma yaklaşık 21 mm yerine 40 mm ve çevrim yaklaşık 4714 ms yerine 2000 ms olacak. Faz geriliği sınırı 0.22'den 0.60'a çıkarıldı. Beğenilen saf dönüş yolunun hız ve kaldırma ölçekleri korundu.

Bu sayılar komut yörüngesinin hesaplarıdır; gerçek servo takibi ve yük altındaki adım yüksekliği fiziksel testle doğrulanmalıdır. Firmware'i yazmak ESP'yi sıfırlayıp PWM'yi keseceğinden yükleme için robot platformda destekli ve servo güç hattı kapalı olmalıdır.
