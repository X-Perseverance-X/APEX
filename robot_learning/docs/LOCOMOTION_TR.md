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
