# META-1 yürüyüş ve merdiven simülasyonu

Windows çalışma istasyonunda MuJoCo, Gymnasium ve Stable-Baselines3 PPO
kullanılır. İlk hedef, komut verilen ileri hızı koruyarak parkuru tamamlamaktır.
Bacaklar üçlü gruplar halinde basar. Ters kinematik ayak hedeflerini mevcut
servo eklemlerine çevirir. PPO bu referansın adım uzunluğunu, ayak kaldırma
miktarını ve ileri ayak hedefini değiştirir. Gerçek robota aktarım için ayrıca
servo gecikmesi, kütle merkezi, ayak teması ve fiziksel güvenlik doğrulaması gerekir.

## Sırayla çalıştır

Proje kökünde PowerShell aç:

```powershell
$py = '.\.runtime\sim-rl-env\Scripts\python.exe'
& $py -m apex_rl.cli course-view --course flat
```

Bu pencereyi kapatınca aşağıdaki komuta geç. Sol tuşla döndür, kaydırma
tekeriyle yakınlaş. Yeşil çizgi bitiştir.

```powershell
& $py -m apex_rl.cli course-view --course stairs-05 --static
& $py -m apex_rl.cli course-view --course stairs-20 --static
& $py -m apex_rl.cli course-view --course stairs-down-20 --static
```

## Eğitim robotlarını izle

Bir aşama klasöründeki dört ayrı eğitim pistini tek pencerede izle:

```powershell
& $py -m apex_rl.cli course-watch --run robot_learning/runs/course-terrain-02/step-03
```

- `1`–`9` ilgili robotu, `[` ve `]` önceki veya sonraki robotu seçer.
- `F` kamera takibini açar veya kapatır. `O` genel görünüme döner.
- `P` oynatmayı duraklatır. `R` kaydı başa alır.
- Fareyle döndür veya kaydır. Tekerlekle yakınlaş.

Görüntü, PPO'nun gerçek paralel ortamlarından kaydedilen pozları kullanır.
Oynatma yaklaşık 10 Hz hızında son 120 simülasyon saniyesini gösterir.
Tam kayıt `tracks.jsonl` dosyasındadır. Çalışan bir aşamayı izlemek için
aynı komuta `--live` ekle. Canlı izleyici en son örneğe yetişir.

Her koşunun kökünde `report.html` ve `summary.json` bulunur. Raporda
iterasyon başına başarı, düşme, ödül, ilerleme ve geçiş/saniye grafikleri ile
ayrı değerlendirme bölümleri var. Raporu eğitim sırasında yenilemek için:

```powershell
& $py -m apex_rl.cli course-report --run robot_learning/runs/course-terrain-02
```

Canlı grafikler için TensorBoard'u çalıştır. Tarayıcıda
`http://127.0.0.1:6006` adresini aç:

```powershell
& $py -m tensorboard.main --logdir robot_learning/runs/course-terrain-02/tensorboard --host 127.0.0.1 --port 6006
```

## Eğit

Başlangıç eğitimi ve ölçümü:

```powershell
& $py -m apex_rl.cli course-train --through stairs-down-20 --envs 4 --timesteps 200000 --output robot_learning/runs/course-try-01
```

Her seviye ayrı `evaluation.json` ve model dosyası alır. `summary.json`
ilerlemeyi gösterir. `--timesteps` seviye başına toplam ortam geçişi sayısıdır
ve PPO'nun örnek toplama grubuna yukarı yuvarlanabilir. Aynı çıktı klasörünü
ikinci kez kullanmak eski sonuçların üzerine yazılmasını önlemek için hata verir.
CPU yükü yüksekse `--envs 2` kullan. İki bağımsız Windows süreciyle çalıştırma
denendi. Varsayılan 4 süreç de aynı arayüzü kullanır.

Kaydedilen ilk düz zemin politikasını izle veya referansla karşılaştır:

```powershell
& $py -m apex_rl.cli course-view --run robot_learning/runs/course-pilot-final/flat
& $py -m apex_rl.cli course-bench --run robot_learning/runs/course-pilot-final/flat --episodes 10
```

Örneğin bir koşu 5 cm'de durursa, o seviyedeki son politikayı yükleyip
bir deneme daha başlatabilirsin:

```powershell
& $py -m apex_rl.cli course-train --start step-05 --through stairs-05 --resume robot_learning/runs/course-pilot-final/step-05 --envs 4 --timesteps 200000 --output robot_learning/runs/step05-retry-01
```

`--resume` yolundaki aşamanın gerçekten üretilmiş olması gerekir.
Parkuru baştan eğitmek için ilk eğitim komutunu yeni bir çıktı klasörüyle çalıştır.

## Seviye sırası ve ölçüm

### Değişken gövde yüksekliği ve bağımsız ayak hedefleri

`--body-control` seçeneği tek ayağı sırayla kaldırır, ileri taşır ve indirir.
Her ayağın iniş yüksekliği kendi hedef noktasından hesaplanır. Yerdeki ayak
hedefleri sabit tutulurken diğer bacakların uzayıp kısalması gövdeyi taşır.
Gövde, temasların yüksekliğine göre yükselir veya alçalır. Gövde hedefi
ileri-geri ve yana kaydırılırken hesaplanan kütle merkezi hedefi destek
çokgeninin iç bölgesine çekilir. Konum hedefinin hızı ve ivmesi sınırlandırılır.

PPO adım temposu, ayağın yerden açıklığı, ileri-geri gövde hedefi, gövde
yüksekliği, yanal kayma ve pitch için altı sınırlı düzeltme öğrenir.
Ayak temasları MuJoCo'dan, zemin yüksekliği ise parkurun tanımından gelir.

```powershell
& $py -m apex_rl.cli course-view --run robot_learning/runs/course-body-01/step-05
& $py -m apex_rl.cli course-watch --run robot_learning/runs/course-body-01/step-05
& $py -m apex_rl.cli course-train --body-control --start step-05 --through down-05 --envs 4 --timesteps 8192 --output robot_learning/runs/course-body-retry
```

Kayıt oynatılırken `Z` satırı gövdenin ölçülen ve hedeflenen yüksekliğini,
`support` ise denge hesabında kullanılan destek ayağı sayısını gösterir.
Bu modun eğitim kayıtları ve modelleri, üç çıkışlı önceki modellerden
bağımsızdır. `course-view` modu kayıt bildirgesinden seçer.

İlk dört süreçli eğitim `course-body-01` altında kayıtlıdır. 5 cm tek
basamak çıkış ve iniş, ayrı değerlendirme ve tekrar kümelerinde yön
başına 20/20 ve sıfır düşmeyle tamamlandı. İlk değerlendirmede ortalama
gövde yüksekliği değişim aralığı çıkışta 7,2 cm, inişte 5,5 cm oldu.
Öğrenilen düzeltmeler sıfırken kullanılan hareket planlayıcısı da ilk
değerlendirmeyi iki yönde 10/10 tamamladı.

Üç adet 5 cm basamakta iniş politikası ilk ölçümde 9/10, ayrı tekrarda
8/10 tamamladı. Tekrardaki iki düşme %10 düşme sınırını aştı.
Çıkışta iki eğitim 4/10 ve 3/10 ile kaldı. Başlıca hata gövdenin
ilk basamak kenarına değmesiydi. Sonuçlar bu iki parkuru daha yüksek
seviyelere geçmek için yeterince güvenilir göstermedi.

Destek çokgeni yaklaşımı için [hexapod kararlılığı çalışması](https://www.mdpi.com/2076-3417/9/24/5284).

### Parkurlar

#### Hızlı yaklaşma, hassas basış ve RGB deneyi

`hybrid-*` parkurlarında robot önce üçlü yürüyüşle yaklaşır. Yavaş
moda geçiş en az beş ayak teması görüldüğünde, yere basan ayakların
konumunu koruyarak yapılır. Sonra ayaklar tek tek kaldırılır. 8 cm ve
üzeri dar inişlerde arka ayaklar sıraya önce girer. PPO adım ve gövde
hedeflerindeki düzeltmeleri öğrenir. Yürüyüş desenini denetleyici seçer.

| Parkur ailesi | Yükseklik | Ayak için basamak derinliği |
| --- | ---: | ---: |
| `hybrid-step-05`, `hybrid-down-05` | 5 cm tek basamak | Uzun sahanlık |
| `hybrid-narrow-05`, `hybrid-narrow-down-05` | 3 × 5 cm | 23 cm |
| `hybrid-narrow-08`, `hybrid-narrow-down-08` | 3 × 8 cm | 23 cm |
| `hybrid-narrow-10`, `hybrid-narrow-down-10` | 3 × 10 cm | 21 cm |

```powershell
& $py -m apex_rl.cli course-view --run robot_learning/runs/course-hybrid-02/hybrid-step-05 --randomize --seed 1006100
& $py -m apex_rl.cli course-view --run robot_learning/runs/course-hybrid-02/hybrid-down-05 --randomize --seed 1006100
& $py -m apex_rl.cli course-view --run robot_learning/runs/course-hybrid-down08-rear/hybrid-narrow-down-08 --randomize --seed 1006400
& $py -m apex_rl.cli course-view --run robot_learning/runs/course-hybrid-down10-rear/hybrid-narrow-down-10 --randomize --seed 1006500
& $py -m apex_rl.cli course-view --course hybrid-step-05 --camera-gate --randomize --seed 1006120
```

`course-camera` simüle ön kameradan RGB kareleri, yükseklik tahminlerini
ve görüntüde ölçülen çizgileri bir HTML raporuna kaydeder. Görünür bir dik
yüzeyin üst kenarı ve taban birleşimi bulunur. Lens açısı, kamera eğimi
ve düz zemindeki kalibre edilmiş kamera yüksekliğiyle iki piksel ışını
metrik ölçüye çevrilir. Bu yöntemde simülatörün derinlik görüntüsü ve
parkurun yükseklik etiketi kestirim girdisi değildir.

```powershell
& $py -m apex_rl.cli course-camera --course hybrid-narrow-08 --output robot_learning/runs/rgb-08-local
& $py -m apex_rl.cli course-bench --course hybrid-step-05 --camera-gate --episodes 10 --seed 1006120
```

`--camera-gate` tek yukarı basamak deneyidir. RGB tahmini hem desen
değişimini tetikler hem ayak hedeflerinin yerel basamak profilini kurar.
Simüle odometri tahmini yerel koordinattan parkur koordinatına taşır.
Basış denetiminde simüle ayak temasları, sonuç ölçümünde parkur geometrisi
kullanılır. Kameranın taban çizgisini göremediği durumda yürüyüş geçişi
yapılmaz. İniş ve çok basamaklı PPO eğitimi parkur yüksekliği bilgisiyle
yapılır. Tek kameradan güvenilir fiziksel iniş kestirimi ayrıca ölçülmelidir.

Tek 5 cm çıkış ve iniş PPO değerlendirmesinde ayrı ayrı 10/10 tamamlandı.
RGB ile kör başlatılan tek çıkış da ayrı 10/10 tamamlandı. Bu deneyde
yaklaşma ortalama 11,8 cm/s, hassas yürüyüş 3,4 cm/s oldu. Durağan
görüntülerde 8 cm basamak dört karede 8,0–8,4 cm ölçüldü. 10 cm basamakta
dört karenin üçünde 10,1–10,3 cm ölçüm, birinde ölçümsüz sonuç alındı.

Arka ayak öncelikli 3 × 8 cm ve 3 × 10 cm inişler, ayrı değerlendirme
ve tekrar kümelerinde seviye başına 20/20 ve sıfır düşme ile tamamlandı.
Parkur bilgili referans bu inişlerin ilk değerlendirmesinde iki seviyede
de 10/10 tamamlandı. 23 cm derinliğinde çıkış eğitimi 5 cm'de 4/10,
8 cm'de 0/10 kaldı. 8 cm çıkışta gövde teması, ilk basamak öncesinde
zemine kadar alçalmayı da içeriyordu. 10 cm çıkışın üç referans
denemesi tamamlanmadı. Bu çıkışlar ayrı tanılama koşularıdır.

Görüntüdeki paralel basamak çizgileri için [monoküler basamak algılama çalışması](https://www.sciencedirect.com/science/article/pii/S016786551930087X).

#### Arka ayak, 15 cm ve sensör laboratuvarı

RGB ile 5 cm çıkarken arka ayak kenara yakınsa önce yukarı yükselir,
ilerleme daha sonra başlar ve ayak yüzeye indikten sonra destek
ayağı sayılır. Arkada iniş noktası için kenar payı 8 mm artırıldı.
Aynı on fizik tohumu ile kenar temaslı kontrol örneği ortalaması
16,3'ten 2,6'ya indi. İki denemede de 10/10 tamamlandı.
Temas sayacı yalnız arka ayak uçlarının dik basamak yüzüyle temasını
20 ms aralıkla kontrol eder. Bacak kapaklarının yüzey geometrisi
ve fiziksel sürtünmesi için ayrıca ölçüm gerekir.

`course-lab-wide` önce 15 cm tek inişi, sonra 32 cm basış derinliği
olan üç adet 15 cm inişi çalıştırır. `course-lab-descent` 21 cm
derinliğindeki 10 ve 12 cm inişleri içerir. Simülasyon sözleşmesindeki
montaj konumları, eklem açıları, hız ve tork sınırları korunur. Tutunma
ayak–zemin temasından hesaplanır. Alt gövde teması başarısızlıktır.

Altı aşamada 393.216 yeni ortam geçişi toplandı. 15 cm tek iniş iki
bağımsız değerlendirmede 20/20 tamamlandı. 21 cm derinliğinde üç
12 cm iniş 4/10, 32 cm derinliğinde üç 15 cm iniş 3/10 tamamlandı.
7 cm tek çıkış ilk politikada 3/10, daha kısa adımlı ayrı eğitimde
4/10 tamamlandı. Bu üç parkur terfi eşiğini karşılamadı.

```powershell
& $py -m apex_rl.cli course-watch --run robot_learning/runs/course-lab-wide/hybrid-lab-down-15-single
& $py -m apex_rl.cli course-view --run robot_learning/runs/course-lab-wide/hybrid-lab-down-15-single --randomize --seed 1007200 --stabilized-gaze --front-view
& $py -m tensorboard.main --logdir robot_learning/runs/course-lab-wide/tensorboard --host 127.0.0.1 --port 6006
```

Eğitim izleyicisi geçiş toplamı, saniye başına geçiş, iterasyon
ve tamamlanan PPO epoch sayısını gösterir. Aktif aşamayı izlerken
`course-watch` komutuna `--live` eklenebilir. TensorBoard
`http://127.0.0.1:6006` adresinde aynı çalışmanın metriklerini gösterir.

```powershell
& $py -m apex_rl.cli course-sensors --output robot_learning/runs/sensors-local
```

Baş dengelemesi dünya koordinatında bir bakış noktası tutar. Chassis
pozundaki yön hatası 40 ms gecikme ve sentetik yön gürültüsüyle
simüle edilir. İki eksenin komutu nötr çevresinde ±45° ile, komut
hızı 210°/s ile sınırlanır. Bu hız bir simülasyon ayarıdır.
Titreşim uygulanan yürüyüşte dikey RMS hata 2,51°'den 0,54°'ye,
yatay hata 0,92°'den 0,40°'ye düştü. Gimbalın iki ekseni var.

2B LiDAR deneyinde altı ayak yerinde tutulurken gövdenin yüksekliği
ve pitch'i değiştirilir. Ters kinematik sınırları, ayak konumu, destek
alanı ve gövde teması kontrol edilen pozlardan menzil kesitleri alınır.
Riser yüksekliği, yüzeyden dönüşün üst sınırı ve ışının kenarı aştığı
ilk kesitlerle kestirilir. 5, 7 ve 20 cm örnekleri 5,2, 7,2 ve
20,1 cm olarak ölçüldü. Bu senaryoda çıkış sınırı 5,5 cm.
Son iki basamakta karar durdur. Karar, PPO hareketinden ayrı bir
kinematik tarama deneyinde sınandı. Gimbalın tek başına 57° yukarı
isteyeceği hedef, gövde burnunun 10° kaldırıldığı erişilebilir pozda
servo aralığına girdi. Tarama pozlarının kesintisiz dinamik geçişi
ve sensör birleştirmesi ayrıca doğrulanmalıdır.

Düzlemsel lazer ışınlarının fizik motorunda uygulanışı [MuJoCo ray API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html).

#### Bacak gövdeleriyle iniş testi

`hybrid-contact-down-10` ve `hybrid-contact-down-15` sırasıyla
21 cm basış derinliğindeki üç adet 10 cm inişi ve tek 15 cm inişi
CAD bacak gövdelerine temas çiftleri ekleyerek çalıştırır. MuJoCo
coxa, femur ve tibia parçalarının dışbükey çarpışma zarflarını
kullanır. Aynı taraftaki farklı bacakların parçaları birbirini,
bağlantı parçaları da zemini ve basamakları fiziksel olarak iter.
Destek ayağı için tabanın zemine veya basamağa basması gerekir.

```powershell
& $py -m apex_rl.cli course-view --course hybrid-contact-down-10 --randomize --seed 1007470
& $py -m apex_rl.cli course-view --course hybrid-contact-down-15 --randomize --seed 1007470
& $py -m apex_rl.cli course-bench --course hybrid-contact-down-15 --episodes 5 --seed 1007540
```

`course-view` bacaklar arası en yüksek örtüşmeyi ve arka bağlantı
parçasının basamağa en büyük girmesini milimetre olarak gösterir.
Ölçüm her 2 ms fizik adımında yapılır. Temas modelindeki esneklik
küçük örtüşmelere izin verir. Doğrulama sınırı 2 mm'dir. Eski
politikaları kendi orijinal parkur adlarıyla oynatmak, ilgili
deneyin fizik modelini yeniden üretir.

İki ayrı tohum grubunda her parkur 10/10 tamamlandı ve düşme olmadı.
En büyük bacaklar arası örtüşme 10 cm parkurunda 0,96 mm, 15 cm
parkurunda 0,75 mm oldu. Arka bacak bağlantı parçalarında basamağa
girme ölçülmedi.

Fizik temas çiftleri ve CAD zarfları için [MuJoCo çarpışma referansı](https://mujoco.readthedocs.io/en/stable/computation/).

#### Temaslı çıkışlar ve taşlı zemin

Çıkış parkurları `hybrid-contact-up-05`, `hybrid-contact-up-07`,
`hybrid-contact-up-10` ve `hybrid-contact-up-15` adlarıyla tek
basamak olarak çalışır. `hybrid-contact-stairs-up-10`, 21 cm basış
derinliğinde üç adet 10 cm basamaktır. Bacak gövdesi çarpışması
ve 2 mm örtüşme sınırı iniş parkurlarıyla aynıdır. Çıkışta
yaklaşma sabit üçlü yürüyüşle başlar. PPO'nun adım ve gövde
düzeltmeleri hassas yürüyüşte sınırlandırılır. 7 cm ve üstünde
denetleyici üst yüzeyi daha erken dikkate alıp ön tarafı yükseltir.

```powershell
& $py -m apex_rl.cli course-view --run robot_learning/runs/course-solid-ascent-02/hybrid-contact-up-05 --randomize --seed 1007810
& $py -m apex_rl.cli course-watch --run robot_learning/runs/course-solid-ascent-02/hybrid-contact-up-05
```

`hybrid-rubble-01` düşük bloklarla başlar. `hybrid-rubble-02`
farklı dizilim ve daha yüksek engel içerir. Robot önceden
tanımlanmış ara noktalara göre yana kayar. Her adımda tabanın
tamamına yer olan, kenardan pay bırakan bir yüzey arar. Ayağın
izlediği hat ve basacağı yer parkurun 2B yükseklik haritasından
kontrol edilir. Harita ile çarpışma geometrisi aynı taş diziliminden
üretilir. Yeşil işaret bitiş alanıdır, başarı ölçümü ileri ve
yanal konumu birlikte kontrol eder. Bu aşama sabit taş ve bilinen
haritayla ayak yerleştirmeyi ölçer. Sensörden harita çıkarma
ve serbest rota bulma ayrı aşamalardır.

```powershell
& $py -m apex_rl.cli course-view --course hybrid-rubble-01 --static
& $py -m apex_rl.cli course-view --course hybrid-rubble-02 --static
& $py -m apex_rl.cli course-view --run robot_learning/runs/course-rubble-01/hybrid-rubble-01 --randomize --seed 1007920
& $py -m apex_rl.cli course-report --run robot_learning/runs/course-rubble-01
```

5 cm çıkış için ikinci politika iki test grubunda toplam 15/15
tamamlandı, düşme veya bacak örtüşmesi ölçülmedi. 7 cm çıkış
4/10 tamamlandı ve üç denemede düşme oldu. 21 cm basış
derinliğindeki üçlü 10 cm parkuru ve tek 15 cm çıkış bağımsız
eğitimlere rağmen 0/10 kaldı. İlk taşlı parkur 8/10 tamamlandı.
İki denemede gövde temasıyla düşüş görüldü, bu nedenle daha zorlu
taş dizilimine otomatik geçiş yapılmadı.

#### Femur–tibia mafsal açıklığı ve üç eksenli çıkış

MARK_2 montajının parça dönüşümleriyle femura bağlı
`Tibia_Holder_3` ve hareketli `Tibia_Head` ayrı incelendi.
1 mm örnek aralığındaki katı kesit taramasında örtüşme ilk
kez −153°'de görüldü. −152°'de örnek örtüşmesi yoktu.
Yeni parkurlarda simüle dizin fizik sınırı −150°, komut sınırı
−149° oldu. Bunlar modelin diz açısıdır ve gerçek servo kalibrasyonu
olarak kullanılmaz. Değerlendirme her 2 ms fizik adımında diz
açısını örnekler ve −153°'ye ulaşan koşuyu geçirmez.

PPO sekiz çıkış kullanır. Altı eski çıkışa roll ve yaw eklendi.
Gövde dönüşümü aynı anda roll, pitch ve yaw uygular. Referans
roll değeri destek ayaklarının sağ ve sol yükseklik farkından
gelir. Salınım ayağı kaldırılırken, yüksek basamak denetimi
ayak ucunu aynı yükseklikte tutup dizin katlanma sınırından
doğan iç erişim yarıçapının dışına taşır. Gövdenin ön tarafı
üst yüzeyi daha erken algıladığında pitch hedefi artırılır.
Ayak destekleri ve CAD bacak çarpışmaları fizik motorunda kalır.

```powershell
& $py -m apex_rl.cli course-view --run robot_learning/runs/course-pose-ascent/hybrid-pose-up-07 --randomize --seed 1008150
& $py -m apex_rl.cli course-watch --run robot_learning/runs/course-pose-ascent/hybrid-pose-up-07
& $py -m apex_rl.cli course-view --run robot_learning/runs/course-steep-10-direct/hybrid-steep-up-10 --randomize --seed 1008301
& $py -m apex_rl.cli course-watch --run robot_learning/runs/course-steep-10-direct/hybrid-steep-up-10
```

Tek robot ekranında hedeflenen ve ölçülen R/P/Y ile bölümdeki
en kapalı diz açısı görünür. Dört robot izleyicisi iterasyon
ve PPO epoch sayısını taşır. Kaynak montaj dosyaları elindeyse
aynı kesit ölçümünü yeniden üretebilirsin.

```powershell
& $py robot_learning/tools/import_mark2_leg.py 'C:/Users/ASUSROG/Desktop/META-2/MARK_2' --components-output .runtime/knee-audit
& $py robot_learning/tools/check_knee_clearance.py .runtime/knee-audit
```

5 cm 10/10, 7 cm iki bağımsız testte 20/20, 8 ve 9 cm ayrı ayrı
10/10 tamamlandı ve düşme olmadı. 7 cm ayrıca üç eksende bozucu
torkla 5/5 tamamlandı. 10 cm kademeli politikanın ilk eğitimi
7/10'da kaldı. 7 cm politikasından doğrudan eğitilen 10 cm modeli
iki testte 19/20 ve bir düşme ile daha iyi sonuç verdi. Bu modelin
en kapalı ölçülen diz açısı −151,8°, en büyük R/P/Y hedefleri
6,6°, 12,2° ve 3,1° oldu.

Toplam 622.592 yeni simülasyon geçişi toplandı. 12 cm doğrudan
eğitim 2/10, 15 cm daha yüksek engel deneyi 0/10 sonuç verdi.
Çapraz yük aktaran 12 cm profili, ön ayak basamağa çıkarken aynı
taraftaki gövdeyi yükseltti ve yaw hedefiyle o tarafı öne aldı.
Ayrı değerlendirmede bu profil 0/10 kaldı.

#### Öne yük aktarımı ve daha yüksek basamaklar

`hybrid-transfer-up-12` ve daha yüksek aktarım parkurlarında
arka ayak geçişi için ön ve orta dört ayak üst yüzeye basmış
olmalıdır. Kontrolcü bu dört ayağı yerinde tutarken arka
ayakları sırayla kaldırır. Kütle merkezi hedefini öne doğru
en fazla 26 mm kaydırır ve yeni hedefi destek çokgeninin
içine geri taşır. Hedef kayması gövdenin simülasyon kütlesini
veya ataletini değiştirmez.

`hybrid-probe-up-*` parkurlarında basamağa yaklaşınca ayrıca
iki ön ayağın üst yüzeye yerleşmesine öncelik verilir. Eğitim
üst yüzeye gerçekten basan her farklı ayak için bölüm başına
bir kez ödül verir. Çıkış denemeleri 11, 12, 13, 14, 16, 18 ve
20 cm olarak düzenlendi. Sonraki aşamaya geçiş için başarı,
düşme, diz açıklığı ve örtüşme ölçütleri birlikte uygulanır.

```powershell
& $py -m apex_rl.cli course-view --course hybrid-transfer-up-12 --randomize --seed 1008380
& $py -m apex_rl.cli course-view --course hybrid-probe-up-14 --randomize --seed 1008620
```

Bu iki tohum, başarılı örneklerdir. 12 cm ilk aktarım modeli
7/10, referans 8/10 tamamlandı. Düşme sınırı sağlanmadı.
14 cm referans denetimi ilk grupta 5/5, bağımsız tekrarda
4/10 tamamlandı. 14 cm PPO politikası 1/10 tamamlandı.
16 cm 1/5, 18 ve 20 cm 0/5 kaldı.
20 cm'de üst yüzeyde birden fazla ayağı aynı anda tutamadı.
Simülasyonda gövde içi kütle merkezini öne kaydıran üçer
denemelik karşılaştırmada başarı artmadı.
Toplam 229.376 yeni simülasyon geçişi toplandı. Bu koşu sonunda
güvenilir çıkış eşiğini koruyan yükseklik 10 cm'dir.

### Ev merdiveni parkuru

Fotoğraflardaki düz kol 20 cm rıht, ilk sırada 30 cm ve sonraki sıralarda 27 cm
basamak derinliğiyle modellenir. İnişte derinlikler ters sırayla gelir. 90 cm
genişlik, beş düz basamak ve sola dönen üç adet 30° kama fotoğrafa dayalı
tahminlerdir. Dönüş `home-corner-preview` sahnesinde incelenebilir. Fiziksel
yürüyüş denemeleri düz kolda, bir ve üç basamaklı parkurlarda yapılır.

```powershell
Set-Location 'C:\Users\ASUSROG\Desktop\Proje\main_v17\APEX_META1_RADIO_SIL'
$py = '.\.runtime\sim-rl-env\Scripts\python.exe'
& $py -m apex_rl.cli course-view --course home-corner-preview --static
& $py -m apex_rl.cli course-view --course hybrid-home-down-20-single --randomize --seed 1008953
& $py -m apex_rl.cli course-view --course hybrid-home-up-20-single --randomize --seed 1009051
```

Çıkış görünümü henüz bitişi geçemeyen geliştirme testini tekrarlar. Rıht önünde ayak önce geri çekilir
ve yükselir. Simülasyonda ölçülen yükseklik kenarı geçmeye yetince yatay hareket
başlar. İki ön ayak basınca orta ayaklara sıra gelir. Gövde yüksekliği,
öne eğimi ve yanal eğimi destek ayaklarına göre değişir. Dik yüz teması olan
fizik adımlarının toplam süresi ile sert bacak parçalarının örtüşmesi ayrı ölçülür.
Tabanın basamak üstündeki esnemesi ayrıca raporlanır.

MARK_2 diz ara parçası ile `Tibia_Head` ve `Tibia_1` 0,4 mm kesitlerle ve
0,25° aralıkla tarandı. İki parçada da kapanışta ilk örtüşme −152,25°'de
görüldü. Ev parkuru değerlendirmesi ölçülen diz açısının −152° üzerinde
kalmasını ve sert örtüşmenin en fazla 2 mm olmasını ister. Bir bölümde dik yüz
teması toplam 0,25 saniyeyi aşarsa denetim o seviyeyi geçmez.

```powershell
& $py robot_learning/tools/check_knee_clearance.py .runtime/knee-audit --spacing-mm 0.4 --min-deg -154 --max-deg -149 --angle-step 0.25 --moving-part Tibia_1
```

İnişte tek 20 cm basamakta referans denetleyici ilk 5 denemenin tamamını
dik yüze temas etmeden geçti. Ek 10 denemenin 8'i tamamlandı, ikisi düşüşle
sonuçlandı. Farklı bir onlu grupta PPO 2/10, aynı koşullardaki referans
10/10 tamamlandı. Üç adet 20 cm basamak ilk iniş ölçümünde 0/5 tamamlandı.
Tek basamak çıkışının son referans turu 0/7 tamamlandı. En iyi denemede
dört ayak üstte kaldı. Beş denemede gövde teması görüldü. Sert uzuv/basamak
örtüşmesi en fazla 0,04 mm, bacaklar arası örtüşme 1,43 mm, en kapalı diz
−150,75° oldu. Son PPO politikası ayrı on tohumda 0/10 ve 1 düşme ile bitti.
O gruptaki referans da 0/10 tamamladı ve beş defa düştü. Bu koşular tüm
merdiveni çıkıp inmek için yeterlilik sağlamıyor.

Altı PPO koşusunda toplam 294.912 yeni geçiş toplandı. Koşular ve HTML raporları
`robot_learning/runs/course-home-*` klasörlerinde tutulur. Eğitim ve fizik bu
oturumda dört paralel CPU ortamında çalıştı. Windows
GPU sorgusu erişim hatası verdi ve yüklü PyTorch CUDA algılamadı. Toplu GPU
fiziği için MuJoCo MJX ayrı bir çalışma altyapısı sunuyor.

- [MuJoCo MJX](https://mujoco.readthedocs.io/en/stable/mjx.html)

### Gövde destekli tek basamak müfredatı

`hybrid-brace-up-13` ile `hybrid-brace-up-20` arasında yükseklik birer santimetre
artar. Her seviye tek basamaklıdır. Yükseklik eşiği, 20 ayrı değerlendirme
tohumunda en az %90 tamamlanma ve en fazla %5 düşme ister. Sert gövde darbesi,
diz sınırı veya uzuv örtüşmesi ihlali varsa ve herhangi bir denemede dik yüzle
toplam temas 0,25 saniyeyi aşarsa sıradaki seviyeye geçilmez.

Basamağın üstündeki, yukarı yönlü yük taşıyan gövde teması bir destek noktası
olarak kullanılır. Alt gövdenin çarpışma geometrisi, CAD modeli ve mafsal
sınırları kullanılır. Temas basıncı ve donanım dayanımı için fiziksel ölçüm
bulunmadığından kuvvet eşikleri simülasyon amaçlıdır. Tork ve hedef açı hızı,
batarya ve servo belirsizliğini temsil etmek için bölümden bölüme kısılır.
Yan duvara yapılan uzun baskı, üst yüzeye sert iniş ve devrilme denemeyi bitirir.

PPO aktörü ve değer ağı ikişer 128 nöronlu katman kullanır. Politika gövdenin
yüksekliğini ve eğimini, adımın ne kadar yana veya öne atıldığını, uygun bacak
çiftinin hangi tarafının kaldırıldığını ve gövdeyle destek alma miktarını
ayarlayabilir. Fizik ortamına temas yükü ve servo tork sınırının ne kadarının
kullanıldığı da eklenmiştir. Eğitimde bazı bölümler basamağa daha yakın başlar.
Değerlendirmelerde robot bütün yaklaşmayı yapar.

```powershell
& $py -m apex_rl.cli course-train --start hybrid-brace-up-13 --through hybrid-brace-up-20 --timesteps 524288 --envs 8 --eval-episodes 20 --eval-seed 1012200 --seed 6201 --output robot_learning/runs/course-brace-524k
& $py -m apex_rl.cli course-watch --run robot_learning/runs/course-brace-524k/hybrid-brace-up-13 --live
```

13 cm için 524.288 geçişli standart PPO ve `--gamma 0.9975 --gae-lambda 0.98`
ile 262.144 geçişli uzun ufuk PPO koşuları tamamlandı. İkisi de 1012200–1012219
tohumlarında 0/20 çıktı. Sırasıyla dokuz ve 15 düşme vardı. Aynı tohumlarda
referans denetleyici 0/20, üç düşme ile kaldı. Referans dört ayağı üst yüzeye
yerleştirdi ve üst yüzeye dayandığı örneklerde gövdesinden taşıttığı yük anlık
olarak robot ağırlığının %96'sına ulaştı. İki PPO politikası bu doğrulamada
gövdeyle yük taşıyan bir duruşa geçemedi. Müfredat 13 cm'de kaldı. 14 ve 15 cm
için ayrı beşer referans denemesinde tamamlanan çıkış yoktu.

Politikalar, manifestler, eğitim eğrileri ve tohum bazlı ölçümler
`robot_learning/experiments/body-support-13` altında arşivlendi. Bunlar
simülasyon kontrol noktalarıdır ve merdiven yeterlilik eşiğini geçmemiştir.

`hybrid-brace-transfer-up-13` arka ayak aktarımını ayrı bir parkurda sınar.
Arka ayakların basamağa yakın bir noktaya inmesi için daha uzun süre tanınır.
Gövde hedefi bu aktarımda düşürülür. Denetleyici sonraki hareketi seçerken
basamak üzerinde ölçülen ayak temaslarını hatırlar. Görüntüleyicide gövdeden
taşınan yük, temas süresi ve servo tork sınırına ulaşan örneklerin oranı gösterilir.

```powershell
& $py -m apex_rl.cli course-view --course hybrid-brace-transfer-up-13 --randomize --seed 1013208
```

Arka ayak aktarımıyla referans 1013200–1013219 grubunda 2/20 tamamladı.
1013208 koşusunda altı ayak da çıktı. Dik yüz teması ve sert uzuv örtüşmesi
ölçülmedi. Aynı parkurda 65.536 geçişle eğitilen PPO 0/20 ve üç düşme ile
kaldı. Üç PPO koşusunda toplam 851.968 yeni geçiş toplandı.

Arka ayak aktarımı parkurları 13–20 cm arasında birer santimetre artar. Daha
yüksek parkurlar tekil referans denemelerinde de açılabilir. PPO müfredatı
başarısız olduğu aşamada durur. `--action-scale 0.25` ağın yürüyüşe yaptığı
düzeltmeleri tam aralığın dörtte biriyle sınırlar. Ölçek manifestte saklanır ve
aynı politika yeniden oynatılırken veya eğitimi sürdürülürken kullanılır.

```powershell
& $py -m apex_rl.cli course-view --course hybrid-brace-transfer-up-15 --randomize --seed 1013600
& $py -m apex_rl.cli course-view --run robot_learning/experiments/body-support-13/residual-quarter/hybrid-brace-transfer-up-13 --randomize --seed 1013213
& $py -m apex_rl.cli course-policy-view --run robot_learning/experiments/body-support-13/residual-quarter/hybrid-brace-transfer-up-13 --seed 1013213 --output robot_learning/runs/meta1-policy-success.html
```

Politika izleyicisi aynı başlangıç koşulunda PPO ile referans yürüyüşünü
karşılaştırır. Zamanı sürükleyince 127 girişin, iki gizli katmanın ve 12
çıkışın gerçek değerleri değişir. Seçili nöron için bir önceki katmandan
gelen en büyük üç işaretli `ağırlık × değer` katkısı gösterilir. Bu değerler,
seçili anda tek katmandaki hesabı açıklar. Çıkış satırındaki filtrelenmiş
düzeltme kırpılmış ağ çıkışı, komut ölçeği ve zaman filtresinden üretilir.
Yaklaşmada yürüyüş denetleyicisi sabit tripod komutunu kullanır.

%25 düzeltme ölçeğiyle 131.072 yeni geçişte eğitilen politika 13 cm'de
1013200–1013219 grubunu 1/20 tamamlama ve üç düşmeyle bitirdi. Önceki PPO aynı
grupta 0/20 kalmıştı. 1013213 koşusunda altı ayak da üst yüzeye geçti. Dik
yüzle 0,40 saniye temas, bacaklar arasında 0,44 mm azami örtüşme ölçüldü.
0,25 saniye temas sınırı ve tamamlanma oranı sağlanmadığı için müfredat 13 cm'de
kaldı. Bağımsız 1013800–1013811 tekrarı 0/12 tamamlanma ve sıfır düşmeyle
sonuçlandı. 14–20 cm'nin her birinde beş bağımsız referans denemesi yapıldı.
13 cm politikası ayrıca 14 ve 15 cm'ye doğrudan aktarıldı. İki yükseklikte de
0/8 tamamlandı ve ulaşılan en yüksek temas sayıları sırasıyla beş ve dört ayaktı.

- [ANYmal Parkour, ETH Zürich](https://arxiv.org/abs/2306.14874)

### Üst basamakta ayak yerleşimi

`hybrid-spread-up-13` ile `hybrid-spread-up-20` parkurları basış adaylarını
üst yüzeyde daha öne ve yana arar. Her aday için taban çevresinin aynı
basamakta kalması, hedefin mevcut eklem sınırları içinde erişilebilir olması
ve komşu ayak merkezlerine en az 7,5 cm mesafe bırakması kontrol edilir. Yerinde
ölçülen mesafe süpürme, gövde hareketi ve temas esnemesine bağlıdır. Her bölümde
üst yüzeyde aynı anda basan ayakların en küçük mesafesi ayrıca kaydedilir.

Basıştan önce kaldırılacak ayak destek hesabından çıkarılır. Kalan temasların
oluşturduğu destek alanı ve ağırlık merkezinin yatay konumu kontrol edilir.
Gövde gerektiğinde önce o yöne kayar. Üç ana destek ayağı kalabilir. Kütle,
servo torku, eklem hızları ve rıht sürtmesi eşikleri korunur. Bu yöntem, ayak
teması seçimini gövde duruşuyla birlikte ele alan planlama araştırmalarını
izler. [Bütün gövde planlama](https://onlinelibrary.wiley.com/doi/10.1002/rob.21974),
[Altı ayaklı destek çokgeni](https://comrob.fel.cvut.cz/papers/ecmr21gaitfree.pdf),
[Basamak çıkışında gövde duruşu](https://doi.org/10.1002/rob.22037)

```powershell
& $py -m apex_rl.cli course-train --start hybrid-spread-up-13 --through hybrid-spread-up-20 --timesteps 262144 --envs 8 --eval-workers 6 --eval-episodes 20 --eval-seed 1014200 --seed 6281 --resume robot_learning/experiments/body-support-13/residual-quarter/hybrid-brace-transfer-up-13 --output robot_learning/runs/course-spread-curriculum
& $py -m apex_rl.cli course-view --course hybrid-spread-up-15 --randomize --seed 1013208
```

1013208–1013213 tohumlarıyla yapılan eşleştirilmiş 13 cm referans denemelerinde
minimum ayak aralığı 5,4 cm'den 11,6 cm'ye, dik yüze ortalama temas 0,77 saniyeden
0,16 saniyeye değişti. Önceki denetleyici 1/6, yeni denetleyici 0/6 tamamladı.
Yeni denetleyici en fazla dört ayağı üst yüzeye yerleştirdi. Aralık ve temas
ölçümleri, tamamlanma sayısı ve üstteki ayak sayısıyla birlikte okunmalıdır.

Mevcut PPO kontrol noktasından 262.144 yeni geçişle devam edildi. Ayrı
1014200–1014219 grubunda önceki politika 1/20 tamamlama ve dört düşme,
geniş basış planlayıcısıyla eğitilen politika 0/20 tamamlama ve sıfır düşme
verdi. Yeni politikada ortalama dik yüz teması 2,08 saniyeydi. 13–20 cm'nin
her birinde yapılan altı referans denemesinde hiçbir çıkış tamamlanmadı.
20 cm'de üstte ölçülen en fazla ayak sayısı birdi. Ön basış evresi ayrıca üç
tohumda ölçüldü. Ortalama gövde yüksekliği hedefin 4,2–5,9 cm altında kaldı.
Aralığı açan denetleyici araştırma parkurlarında tutulur. Önceki 13 cm
politikasının kayıtlı koşusu ayrı olarak açılabilir.

### Planlı basamak çıkışı (ayrı deney)

`hybrid-staged-up-13` … `hybrid-staged-up-20` parkurları, `hybrid-brace-transfer-up-*`
ile aynı basamak, kütle, servo tork ve hız sınırları, diz durdurucusu ve CAD
çarpışma modelini kullanır. Rıhta 36 cm kala her ayak hareketi önceden planlanır.
Sıra ön ayaklar, gövde aktarımı, orta ayaklar ve arka ayaklardır. Her adımda
kalan ayakların erişimi, destek çokgeni, statik eklem torku, tabanın basamağa
sığması, komşu ayak mesafesi ve salınım yolunun tamamında rıht açıklığı
denetlenir. Kaldırmadan önce ölçülen temaslarla destek çokgeni yeniden hesaplanır.
Ayrıntılar, tohum grupları ve bütün ölçümler
[deney klasöründedir](../experiments/staged-ascent/README.md).

Ayar ve eğitimde kullanılmayan 1017300–1017319 grubunda sonuçlar:
13 cm 18/20, 14 cm 18/20 tamamlandı. Düşme ve gövde darbesi olmadı. En uzun rıht
teması 0,17 ve 0,19 s oldu. Aynı tohumlarda referans 1/20 ve 0/20 tamamladı.
Denetleyici etrafında eğitilen artık PPO 13 cm'de 20/20 tamamladı. 14 cm PPO adayı
17/20 verdi; bu aşama yalnız denetleyiciyle geçti. 15 cm'de denetleyici 15/20 ve
0,27 s rıht teması, PPO 16/20 verdi; müfredat 15 cm'de durdu. 18–20 cm'de ön
ve orta ayaklar basamağa çıktı. Arka aktarım gövde desteği olmadan uygun duruş bulamadı.

20 cm için ayrı deney parkuru `hybrid-wide-rear-up-20` son arka ayağı önce mevcut
pozda kaldırır, ayak havadayken gövdeyi burun aşağı yeni poza getirir ve sonra
basamağa taşır. 1017300–1017319 grubunda 13/20 tamamladı (düşme ve gövde darbesi
yok, başarılı koşuların hiçbirinde rıht teması yok, 12'si 60 s içinde). Eşik 18/20
olduğu için 20 cm geçmiş sayılmaz. `tools/check_servo_mounts.py` femur ve tibia
servo kasalarının STL vida delikleriyle ve eklem eksenleriyle uyumunu denetler.

```powershell
& $py -m apex_rl.cli course-view --course hybrid-staged-up-13 --randomize --seed 1017317
& $py -m apex_rl.cli course-view --course hybrid-staged-up-13 --randomize --seed 1017309
& $py -m apex_rl.cli course-bench --course hybrid-staged-up-14 --episodes 20 --seed 1017300 --eval-workers 2
& $py -m apex_rl.cli course-view --course hybrid-wide-rear-up-20 --randomize --seed 1017317 --replay-seconds 90
& $py -m apex_rl.cli course-policy-view --run robot_learning/experiments/staged-ascent/ppo-13/hybrid-staged-up-13 --seed 1017317 --output robot_learning/runs/staged-13-best.html
```

#### Tam ev merdiveni

`home-flight-preview` evdeki merdivenin tamamını kurar: 20 cm rıht ve 30 cm derin ilk
basamak, 27 cm derin düz basamaklar, sola dönen üç kama, üst kol, duvar ve korkuluk.
Ölçülmemiş değerler (diğer rıhtlar, genişlik, basamak sayıları) parametredir. Düz kolda
yapılan statik yürüyüş araması, 27 cm basamakta rıht 17 cm ise tekrarlanan bir ayak
sırası buldu. 18 ve 20 cm'de arka bacak çifti hiçbir durumda bir üst basamağa
çıkamıyor; tork sınırını 1,5 katına çıkarmak, tibiayı 20 cm'ye uzatmak ya da arka ayakları
yana açmak bunu değiştirmedi. Ayrıntılar [deney klasöründedir](../experiments/home-flight/README.md).

```powershell
& $py -m apex_rl.cli course-view --course home-flight-preview --static
& $py robot_learning\tools\stair_gait_search.py --rise 0.17 --tread 0.27 --output robot_learning\runs\gait-17.json
```

#### Önceki sabit desenli parkurlar

| Seviye | Parkur | Başarı |
| --- | --- | --- |
| `flat` | 1,2 m düz zemin | Dengeli biçimde bitişi geç |
| `step-02`, `down-02` | 2 cm tek basamak | Altı ayağın tamamıyla çık veya in |
| `step-03`, `down-03` | 3 cm tek basamak | Altı ayağın tamamıyla çık veya in |
| `step-04`, `down-04` | 4 cm tek basamak | Altı ayağın tamamıyla çık veya in |
| `step-05`, `down-05` | 5 cm tek basamak | Altı ayağın tamamıyla çık veya in |
| `stairs-05`, `stairs-down-05` | 3 × 5 cm | Üç basamağı tamamla |
| `stairs-10`, `stairs-down-10` | 3 × 10 cm | Üç basamağı tamamla |
| `stairs-15`, `stairs-down-15` | 3 × 15 cm | Üç basamağı tamamla |
| `stairs-20`, `stairs-down-20` | 3 × 20 cm | Üç basamağı tamamla |

Bir seviyeden sonraki seviyeye geçiş için ayrı tohumlarla en az %80 başarı
ve en fazla %10 düşme kabul edilir. Bu ilk proje eşikleri. Fiziksel aktarım
öncesinde daha büyük örneklem ve eğitim sırasında kullanılmamış ayrı bir
nihai değerlendirme kümesi gerekir. Kısa eğitimde 20 cm'ye ulaşma garantisi
yok. Adım yüksekliği, eklem torku, gövde teması ve destek alanı sınır olabilir.

### İlk çıkış ve iniş kayıtları

İlk dört süreçli koşu 2 cm çıkış politikasından başladı. 2 cm iniş 10/10
tamamlandı. İlk 3 cm çıkış denemesi 5/10'da kaldı. Basamağa yaklaşırken daha
erken ön bakış ve daha uzun kaldırma süresiyle yapılan ikinci koşuda 3 cm
çıkış 8/10 ve 3 cm iniş 10/10 tamamlandı. Farklı tohumlarla tekrarda çıkış
7/10, iniş 10/10 oldu. İkinci koşu 4 cm çıkışta 2/10 başarı ve dört düşme
ile durdu. Bu nedenle 3 cm çıkışın güvenilirlik payı yetersiz.

5, 10 ve 20 cm merdivenler ayrıca referans yürüyüşle denendi. Üçer
bölümlük taramada bu çok basamaklı parkurlar tamamlanmadı. Bu tarama
müfredat geçişinden bağımsızdır. Kayıtlar çalışma istasyonunda
`course-terrain-01` ve `course-terrain-02` klasörlerinde tutulur.

Raporlar ilerlemeyi, süreyi, ortalama ileri hızı, ayak temas sayısını,
son ayak konumlarını, ayak kaymasını, maksimum kaldırma mesafesini,
düşüşü, gövde temasını ve mekanik işin yaklaşık toplamını içerir.
Simülasyonda basit bir pil gerilimi profili 6,8–8,0 V arasında örneklenir ve
temsili tork sınırını %85–100 arasında değiştirir. Bu profil servo besleme
önerisi veya akım, sıcaklık ve batarya tüketimi ölçümü değildir.
Sürtünme, gövde kütlesi, servo kazancı ve hedef hız da bölümler arasında
değişebilir.

Parkurda gövde temasını belirlemek için alt plaka, hoparlör ve pil
gruplarının ayrı yaklaşık çarpışma hacimleri vardır. Mevcut kütle ve
atalet hesabı korunur. İlk kontrol noktasındaki yürüme ve düşme sonuçları
yalnızca bu fizik modeli için geçerlidir.

Parkur yükseklikleri simülatör zemininden doğrudan okunuyor. Gerçek robotta
aynı veri arayüzü için sensörlerden üretilecek yerel bir yükseklik haritası,
komut ve telemetri gecikmesi modeli ve arıza anında düşüşü sınırlayan
denetim katmanı gerekir. Yürüyüş denetimi ile ev içi rota planlaması ayrı
aşamalardır. İlki verilen hızı uygular, ikincisi gidilecek yön ve hızı seçer.

Algoritma ilk burada, bağımsız birkaç CPU süreciyle ölçülüyor. Binlerce ortam
gerekirse aynı gözlem, eylem, ödül ve seviye eşikleri Isaac Lab gibi GPU
üzerinde toplu çalıştıran bir arka uca taşınabilir. Taşıma öncesi ayak teması,
kütle ve servo sınırlarının iki simülatörde karşılaştırılması gerekir.

- [ETH, Learning to Walk in Minutes](https://proceedings.mlr.press/v164/rudin22a.html)
- [NVIDIA Isaac Lab arazi üretimi](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.terrains.html)
- [Stable-Baselines3 paralel ortamlar](https://stable-baselines3.readthedocs.io/en/master/guide/vec_envs.html)
