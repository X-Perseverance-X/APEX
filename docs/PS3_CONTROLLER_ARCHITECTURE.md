# META-1 PS3 kumandası: uygulanacak bağlantı ve güvenlik sözleşmesi

Durum (2026-11-25): Orijinal Sony `054c:0268` kol Pi 5'e USB ile tanıtıldı,
ardından USB çıkarılarak Bluetooth üzerinden `Connected: yes` ve Linux
`/dev/input/js0` ile doğrulandı. X düğmesinin kablosuz basma/bırakma olayları
okundu. Yürüyüş komutları ve diğer tuş kodları henüz doğrulanmadı; bu belge
onlar için önerilen atamaları gösterir.

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
| L1 basılı | Hareket için deadman; bırakınca sıfır hız |
| Sol çubuk | +X ileri/geri, +Y sol/sağ öteleme |
| Sağ çubuk yatay | Yaw; dikey eksen kamera tilt |
| L2 / R2 | Kamerayı sola / sağa çevir |
| Çarpı + L1, 2 sn | Kontrollü CAPTURE → STAND isteği |
| Daire | Sıfır hız; kontrollü HOME/oturuş (ayrı komut hazır olunca) |
| Üçgen | LiDAR motorunu aç/kapat |
| Kare | AI takip modunu aç/kapat |
| D-pad yukarı/aşağı | Hız sınırını küçük adımlarla değiştir |
| D-pad sol/sağ | Kamera hedefini sola/sağa ince ayarla |
| L3 | Yürüyüş modu seçimi; hız sıfırken |
| R3 | Kamerayı merkeze al |
| SELECT | Durumu sesli bildir / HUD'da göster |
| START | Otonom/manual seçim isteği; ancak sıfır hızda |
| SELECT + START, 3 sn | Sıfır hız, hareket kilidi, sonra Pi ağ profilini robot/ev arasında değiştir |
| PS | Yazılımsal STOP ve PWM kesme isteği; fiziksel kill switch'in yerine geçmez |

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

1. USB kimliği, Sixaxis eşleştirmesi ve kablosuz X tuşu doğrulandı. Geri kalan
   tuş/eksen kodlarını olay okuyucusuyla tek tek kaydet.
2. 30 dakika bağlantı/deadman testi; kopuşta 750 ms içinde hız sıfır.
3. Ev Wi-Fi profilini kaydet ve Ethernet bağlıyken ağ geçişi/geri dönüşünü dene.
4. PCA ve servo gücü bağlı, robot platformdayken ayrı izinli fiziksel test yap.

Kaynak yedeği GitHub'daki `codex/backup-2026-11-24` dalındadır. Gizli ağ
parolaları depoya eklenmez.
