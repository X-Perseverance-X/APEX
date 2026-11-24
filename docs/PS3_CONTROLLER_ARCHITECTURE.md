# META-1 PS3 kumandası: uygulanacak bağlantı ve güvenlik sözleşmesi

Durum (2026-11-24): Kumanda henüz Pi'ye USB veya Bluetooth ile bağlanmadı;
eşleştirme, tuş olay kodları ve fiziksel yürüyüş doğrulanmadı. Bu belge tuş
atamalarını ve etkinleştirme koşullarını sabitler, çalışıyor iddiası taşımaz.

## Donanım yolu

DualShock 3 / Sixaxis klasik Bluetooth HID kullanır. ESP32-S3 yalnız Bluetooth
LE desteklediğinden kumanda doğrudan omuriliğe eşleştirilemez. Pi 5'in Bluetooth
denetleyicisi kumandayı alır; Pi'deki tek giriş servisi Linux input olaylarını
okur ve omuriliğe mevcut FT232 USB hattından komut yollar. Kumandayı ilk kez
eşleştirmek için veri taşıyan USB kablosuyla Pi'ye takıp model ve olay kodları
okunmalıdır. Klon PS3 kolları farklı eşleştirme davranışı gösterebilir.

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

1. Kumandayı Pi'ye USB ile tak; `lsusb`, `bluetoothctl` ve `/dev/input` olaylarını
   okuyup gerçek modelin tuş/eksen kodlarını eşleştir.
2. Eşleştirme ve 30 dakika bağlantı/deadman testi; kopuşta 750 ms içinde hız sıfır.
3. Ev Wi-Fi profilini kaydet ve Ethernet bağlıyken ağ geçişi/geri dönüşünü dene.
4. PCA ve servo gücü bağlı, robot platformdayken ayrı izinli fiziksel test yap.

Kaynak yedeği GitHub'daki `codex/backup-2026-11-24` dalındadır. Gizli ağ
parolaları depoya eklenmez.
