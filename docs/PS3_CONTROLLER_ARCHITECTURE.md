# META-1 PS3 kumandası: uygulanacak bağlantı ve güvenlik sözleşmesi

Durum (2026-11-25): Orijinal Sony `054c:0268` kol Pi 5'e USB ile tanıtıldı,
ardından Bluetooth üzerinden `/dev/input/js0` olarak doğrulandı. Linux
joystick aygıtı 6 eksen ve 17 düğme bildiriyor; düğme haritası ioctl ile
okundu (0=BTN_SOUTH/X, 4=BTN_TL/L1, 8=SELECT, 9=START, 10=PS,
13..16=D-pad). `apex-ps3-gateway.service` Pi'de etkin. Gerçek servo kartları
bu oturumda bağlı olmadığı için fiziksel gait/tuş davranışı doğrulanmadı.

## Donanım yolu

DualShock 3 / Sixaxis klasik Bluetooth HID kullanır. ESP32-S3 yalnız Bluetooth
LE desteklediğinden kumanda doğrudan omuriliğe eşleştirilemez. Pi 5'in Bluetooth
denetleyicisi kumandayı alır; Pi'deki tek giriş servisi Linux input olaylarını
okur ve omuriliğe mevcut FT232 USB hattından komut yollar. Kumandayı ilk kez
eşleştirmek için veri taşıyan USB kablosuyla Pi'ye takıp model ve olay kodları
okunmalıdır. Klon PS3 kolları farklı eşleştirme davranışı gösterebilir.

Bu Pi'de BlueZ 5.82, Sixaxis'in kablosuz HID bağlantısını varsayılan ayarda
kesiyordu. Kullanıcının geçici onayıyla `/etc/bluetooth/input.conf` içinde
`ClassicBondedOnly=false` etkinleştirildi. Önceki dosya
`/etc/bluetooth/input.conf.apex-pre-ps3-20261125` konumunda saklıdır. Bu ayar
klasik Bluetooth HID güvenliğini düşürür; `Pairable` test sonunda tekrar
`no` yapıldı. Güvenli varsayılana dönmek için yedek dosyayı geri yükleyip
`bluetooth.service` yeniden başlatılmalıdır. Bu durumda PS3'ün tekrar
kablosuz bağlanması beklenmez; daha yeni BlueZ'in hedefli Sixaxis kablo
eşleştirme desteği ayrı bakım işidir.

## Tuşlar (fiziksel doğrulamadan önce yürüyüş etkinleştirilmez)

| Giriş | Amaç |
| --- | --- |
| L1 basılı | Hareket için deadman; bırakınca sıfır hız ve IDLE |
| Sol çubuk | +X ileri/geri, +Y sol/sağ öteleme |
| Sağ çubuk yatay | Yaw; dikey eksen kamera tilt |
| L2 / R2 | Kamerayı sola / sağa çevir |
| Çarpı + L1, 2 sn | Kontrollü CAPTURE → STAND isteği; PCA yoksa reddedilir |
| Daire | Sıfır hız ve IDLE/STAND; HOME/oturuş henüz bağlı değil |
| Üçgen | LiDAR motorunu aç/kapat |
| Kare | AI takip modunu aç/kapat |
| D-pad yukarı/aşağı | Hız sınırını küçük adımlarla değiştir |
| D-pad sol/sağ | Kamera hedefini sola/sağa ince ayarla |
| L3 | Yavaş/normal hız profili; hareket yokken |
| R3 | Kamerayı merkeze al |
| SELECT | Durumu servis günlüğüne yaz; sesli bildirim henüz yok |
| START | Otonom/manual seçim isteği; ancak sıfır hızda |
| SELECT + START, 3 sn | Sıfır hız, hareket kilidi, sonra Pi ağ profilini robot/ev arasında değiştir |
| PS | Yazılımsal STOP/IDLE; PWM'yi kesmez, fiziksel kill switch'in yerine geçmez |

USB hareket protokolü `SPINE MODE=1/0`, `JOY=x,y,r`, `PARAMS` ve `STAND`
komutlarını içerir. Firmware, PWM açık ve STAND/WALK tamam değilse sıfırdan
farklı JOY komutunu reddeder. WALK sırasında 750 ms komut gelmezse son fiziksel
eklem komutlarında HOLD failsafe'e geçer. Kumanda servisi 10 Hz günceller.
Rota yönlendirmesi de USB protokolünü kullanır ve yön hatasını ilerleme ile
aynı anda gönderir. İç tarafın adımı ileri hareket sırasında tersine dönmez.

## Ağ geçişi

Pi'de şu an yalnız `APEX-META1` profili kayıtlıdır. Ev ağı SSID/parolası
Pi'nin NetworkManager profiline kullanıcı tarafından bir kez kaydedilmeden
`apex-network internet` çalışmaz. Ağ geçişi ESP'nin Bluetooth işi değildir;
Pi'de yapılır. Önce tüm hareket komutları sıfırlanır, yeni hareketler kilitlenir,
ardından `nmcli connection up <kayıtlı ev profili>` veya `APEX-META1` açılır.
Tek Wi-Fi radyosu geçiş sırasında bağlantıyı koparır. Geri dönüş için kumanda
aynı Pi'ye Bluetooth üzerinden bağlı kalmalıdır; bu, gerçek kumandayla test
edilmeden güvenilir sayılmaz. Ethernet bakım yolu korunur.

## Doğrulama kapıları

1. USB kimliği, Sixaxis eşleştirmesi, kablosuz bağlantı ve Linux tuş haritası
   doğrulandı. Fiziksel tuş/eksen yönlerini gerçek robot üzerinde sırayla doğrula.
2. 30 dakika bağlantı/deadman testi; kopuşta 750 ms içinde hız sıfır.
3. Ev Wi-Fi profilini kaydet ve Ethernet bağlıyken ağ geçişi/geri dönüşünü dene.
4. PCA ve servo gücü bağlı, robot platformdayken ayrı izinli fiziksel test yap.

Kaynak yedeği GitHub'daki `codex/backup-2026-11-24` dalındadır. Gizli ağ
parolaları depoya eklenmez.
