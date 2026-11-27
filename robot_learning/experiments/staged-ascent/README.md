# Planlı basamak çıkışı (staged ascent) deneyi

Bu klasör, `hybrid-brace-transfer-up-*` denetleyicisinin yanında ayrı bir deney
olarak geliştirilen planlı çıkış denetleyicisinin ölçümlerini tutar. Her şey
MuJoCo simülasyonundadır; hiçbir dosya robota komut göndermez ve kayıtlı
politikalar `policy_export_allowed: false` işaretlidir.

## Korunanlar

- Geometri, montaj açıları, eklem sınırları, servo tork ve hedef hız sınırları,
  diz durdurucusu (−150° fizik, −149° komut), CAD bacak çarpışma modeli ve
  parkurlar değişmedi. `hybrid-staged-up-13` … `-20` parkurları,
  `hybrid-brace-transfer-up-*` ile **aynı parkur nesnesini** kullanır.
- `locomotion.py`, `body_motion.py`, `env.py`, `model.py` ve diğer parmak izi
  dosyaları değişmedi. Eski koşuların model özetleri aynı kaldı; bunu
  `tests/test_staged_ascent.py` denetler.
- Regresyon: `tests/test_regression_replays.py` iki kayıtlı koşuyu tekrar oynatır.
  - Referans `hybrid-brace-transfer-up-13`, tohum 1013208: çıktı, 6/6 ayak üstte,
    rıht teması 0 s, sert uzuv/basamak örtüşmesi 0 mm, bacaklar arası 0 mm.
  - `residual-quarter` PPO, tohum 1013213: çıktı, 6/6 ayak üstte, rıht teması
    0,40 s, bacaklar arası örtüşme 0,44 mm.

## Yeni dosyalar

| Dosya | İşlev |
| --- | --- |
| `src/apex_rl/ascent_planner.py` | Tek bacak adımı için statik denetim: eklem erişimi ve payı, destek çokgeni ve kütle merkezi payı, yay modeliyle ayak yükü paylaşımı, eklem torku, femur/tibia–basamak açıklığı, salınım yolu boyunca erişim, taban–rıht ve komşu ayak açıklığı |
| `src/apex_rl/staged_ascent.py` | `hybrid-staged-up-*` parkurları, `StagedAscent` denetleyicisi, `StagedEnv`, ara durum kaydı/geri yükleme |
| `src/apex_rl/gait_lab.py` | Düz zeminde her yöne tripod: eski profil (`reference`) ve yeni hız tümlevli yürüyüş (`phase`), taban kayması, sert basış, salınım ve tork ölçümleri |
| `src/apex_rl/climb_trace.py` | Çıkışı evrelere ayıran iz: hedef/gerçek gövde yüksekliği, ayak konumları ve yükleri, eklem yükleri, rıht teması |
| `tools/staged_experiments.py` | Bu klasördeki ölçümleri yeniden üretir |
| `tools/staged_train.py` | Planlı denetleyici etrafında artık (residual) PPO, ara durumlardan başlatma, kontrol noktası seçimi |

## Denetleyici

1. Tripod yaklaşma rıhta 50 cm kalana kadar sürer (eski: 72 cm). Sonra mevcut
   dalga yürüyüşü robotu rıhtın 36 cm gerisine getirir.
2. Buradan sonra her ayak hareketi başlamadan planlanır. Evre sırası
   ön ayaklar → gövdenin ileri/yukarı taşınması → orta ayaklar → arka ayaklardır.
   Her evrede hedef ayak dizilimi ve gövde duruşu tanımlıdır. Önce geride kalan
   ayaklar öne alınır, sonra evrenin ayakları basamağa kaldırılır.
3. Bir adım için ayak, iniş noktası ve gövde duruşu `(x, y, z, eğim)` birlikte
   seçilir. Koşullar:
   - Kalan beş ayak, mevcut eklem sınırlarının en az 4° içinde erişilebilir.
   - Kütle merkezi, kalan ayakların destek çokgeninin en az 2 cm içindedir.
   - Ayak yükleri, konum servolu bacakların düşey esnekliğiyle paylaştırılır.
     Bu yöntem MuJoCo'nun ölçtüğü taban yüklerini ağırlığın %7'si içinde verdi.
     Statik tork tahmini bu bölümün servo sınırının 1,15 katını aşamaz. Tahmin
     ölçülen servo yükünün 1,2–1,45 katı çıktığından bu, gerçek yükte sınırın
     yaklaşık %80–95'ine karşılık gelir.
   - Tabanın tamamı basamakta kalır ve diğer üst ayaklara en az 7,5 cm mesafe bırakır.
   - Salınım yolu boyunca erişim, diz payı, taban–rıht açıklığı, femur ve tibia ile
     rıht kenarı arasındaki açıklık ve komşu tabanlar denetlenir.
   - Gövde kutusu basamak yüzeyine değmez. Alt basamaktaki hiçbir taban, nötr
     konumunun 15 cm'den fazla gerisinde kalmaz.
4. Gövde, bütün ayaklar yerdeyken seçilen duruşa geçer. Kaldırmadan hemen önce
   ölçülen temaslardan destek çokgeni yeniden hesaplanır. Üç ayakla taşınacaksa
   bu duruşta eklem torku da denetlenir. Uygun değilse sıradaki ayak veya gövde
   duruşu seçilir. Hiçbiri uygun değilse yalnız gövde, gerilen bacakları
   rahatlatacak yöne kaydırılır.
5. Salınım ölçümle kapılıdır. Taban, tepe yüksekliğine ölçülerek ulaşmadan kenarı
   geçmeye başlamaz. Kenar üstünde taban hâlâ aşağıdaysa geçiş bekler. Bu,
   servo gecikmesinin tabanı rıht yüzünde sürüklemesini önler.
6. Altı taban üst basamağa çıkınca robot, düz zemin tripoduyla (`phase`) bitişe yürür.

PPO, 12 çıkışlı aynı politika arayüzünü kullanır. Planlı denetleyicide yalnız
gövde ileri/geri, yükseklik, yana kayma, eğim ve iniş noktası X/Y düzeltmeleri
etkindir. Diğer çıkışlar sıfırlanır.

## Servo yuvaları (femur ve tibia)

Femur ve tibia servo kasaları artık STL'deki dört vida deliğine göre yerleştirildi
(`src/apex_rl/model.py`). Femur yuvasında delikler yerel X/Z düzleminde 10 × 48,1 mm
dikdörtgendir ve yuva dik durur. Tibia yuvasında delikler 41,36 × 24,65 mm uç uca
ölçüdür; bu da yuvanın tibia parçası içinde yaklaşık 31° eğik olduğunu gösterir.
Kasalar yalnız görseldir (`contype=0`); kütle, atalet ve çarpışma modeli değişmedi,
bu yüzden hareket sonuçları kasa düzeltmesinden etkilenmez.

`tools/check_servo_mounts.py` on iki kasanın hepsini (`servo-mounts.txt`) denetler:
mil ekseni eklem ekseniyle 0,00° farklı, mil kasa merkezinden 10,05 mm (femur) ve
10,08 mm (tibia) kaymış (gerçek 40 mm servoda milin merkezden uzaklığı ≈ 10 mm),
holder ağındaki hiçbir köşe noktası kasanın içinde değil, dört delik merkezi kasanın
±24,05 mm × ±5 mm desenine 0,000 mm hatayla oturuyor.

Not: bu doğrulama modelin STL'e ve kendi bildirdiği vida delik ölçülerine göre
tutarlılığını gösterir. Fiziksel parçada servo mili iki vida çiftinden birine
yakındır: modelde mil, delik çiftlerinden birine 14 mm, diğerine 34 mm uzaklıktadır.
Servoyu robota takmadan önce çıkış milinin yuvadaki yakın delik çiftinin tarafında
olduğunu bir kez elle kontrol et.

Fotoğraflarla karşılaştırma (yerel `robot_learning/runs/montaj-karsilastirma.png`, fotoğraf içerdiği için Git'e girmedi): femur servosu DS3240 (etiket
40 kg, 4,8–6,8 V) coxa yuvasında dik durur ve modelde de dik, mil alt uçtadır. Diz
servosunun etiketi **BLS3235SG**'dir; modelde ve sözleşmede adı BLS3225 geçer. İki servonun
kasası aynı ölçüdedir (40 × 20 × 38,5 mm), bu yüzden yerleşim değişmez. Simülasyondaki
tibia sınırı (1,2 N·m) BLS3235 sınıfının katalog torkunun altındadır, yani temkinlidir.
Sınırlar değiştirilmedi; böylece bütün ölçümler karşılaştırılabilir kalır.

## Tohum grupları

| Grup | Kullanım |
| --- | --- |
| 1013200–1013219 | Önceki raporlardaki 13 cm grubu. Planlı denetleyicinin ayarında da kullanıldı, bu nedenle geliştirme grubu sayılır |
| 1014400–1014409, 1020400–1020405 | İlk 14 ve 20 cm denemeleri (ayar) |
| 500000–500011 | PPO ara durum kayıtları |
| 1016100–1016109 | PPO kontrol noktası seçimi (doğrulama) |
| **1017300–1017319** | **Son değerlendirme.** Ayar ve eğitimde hiç kullanılmadı. Bütün yükseklikler ve denetleyiciler aynı tohumlarla |

Geçiş ölçütü: en az 18/20 tamamlama, en fazla 1 düşme, sert gövde darbesi yok,
diz −152°'nin üstünde, sert uzuv örtüşmesi ≤ 2 mm, her denemede rıht teması ≤ 0,25 s.

## Sonuçlar

Aşağıdaki tablolar `climb/` klasöründeki dosyalardan derlenmiştir.
`stage-table.json` aynı verinin özetidir.

### 13 cm eşleştirilmiş karşılaştırma

| Denetleyici | Grup | Tamamlama | Düşme | Gövde darbesi | Rıht teması en çok / ort. (s) | Üstteki ayaklar arası en küçük mesafe | Servo ≥%95 yük oranı | Geçiş |
| --- | --- | ---: | ---: | ---: | --- | ---: | ---: | --- |
| `hybrid-brace-transfer-up-13` referans | 1013200 | 2/20 | 5 | 4 | 7,06 / 2,01 | 5,4 cm | 0,198 | hayır |
| `residual-quarter` PPO | 1013200 | 1/20 | 2 | 2 | 13,62 / 1,64 | 5,6 cm | 0,193 | hayır |
| `hybrid-staged-up-13` | 1013200 | 16/20 | 0 | 0 | 0,17 / 0,06 | 8,5 cm | 0,027 | hayır (16/20) |
| `hybrid-brace-transfer-up-13` referans | **1017300** | 1/20 | 3 | 3 | 7,90 / 2,04 | 5,0 cm | 0,213 | hayır |
| `residual-quarter` PPO | **1017300** | 2/20 | 1 | 1 | 4,98 / 1,26 | 7,5 cm | 0,177 | hayır |
| `hybrid-staged-up-13` | **1017300** | **18/20** | 0 | 0 | 0,17 / 0,06 | 8,8 cm | 0,028 | **evet** |
| `hybrid-staged-up-13` + PPO (`ppo-13`) | **1017300** | **20/20** | 0 | 0 | 0,16 / 0,05 | 8,5 cm | 0,027 | **evet** |

`residual-quarter` politikası 1013200 grubunda yine 1/20 tamamladı ve tamamlanan
tohum yine 1013213 oldu. Önceki raporda bu grupta 3 düşme vardı; bu ölçümde 2
düşme var. Ölçümler bu oturumda Linux üzerinde aynı kodla alındı. İki regresyon
koşusu birebir tekrarlanıyor.

### 14–20 cm (son değerlendirme grubu 1017300–1017319)

| Yükseklik | Planlı: tamamlama / düşme / rıht en çok | Planlı: üstteki ayak sayısı (bölüm sayısı) | Referans: tamamlama / düşme | Referans: üstteki ayak | Aşama |
| --- | --- | --- | --- | --- | --- |
| 14 cm | **18/20** / 0 / 0,19 s | 6 ayak: 18, 4 ayak: 2 | 0/20 / 2 | 0 ayak: 8, en çok 6 | **geçti** |
| 15 cm | 15/20 / 0 / 0,27 s | 6: 15, 4: 3, 3: 2 | 0/20 / 4 | en çok 4 | geçmedi |
| 16 cm | 15/20 / 0 / 0,43 s | 6: 15, 4: 4, 3: 1 | 0/20 / 5 | en çok 4 | teşhis |
| 17 cm | 13/20 / 2 / 0,24 s | 6: 14, 5: 2, 4: 4 | 0/20 / 7 | en çok 4 | teşhis |
| 18 cm | 0/20 / 0 / 0 s | 5: 16, 4: 4 | 0/20 / 6 | en çok 4 | teşhis |
| 19 cm | 0/20 / 0 / 0 s | 5: 14, 4: 6 | 0/20 / 6 | en çok 4 | teşhis |
| 20 cm | 0/20 / 0 / 0 s | 5: 4, 4: 15, 3: 1 | 0/20 / 6 | en çok 3 | teşhis |

Müfredat 15 cm'de durur. 15 cm'de 15/20 tamamlama ve 0,27 s rıht teması geçiş
ölçütünü karşılamaz. 16–20 cm yalnız teşhis denemesidir. Planlı denetleyicide
gövde kutusu 18 cm dışında hiçbir yükseklikte yük taşımadı. 18 cm'de anlık
olarak ağırlığın 1,36 katına ulaşan bir temas oldu ve sert darbe sayılmadı.

### 20 cm: ön ayak aktarımı nerede tıkanıyordu

`phases/20-reference.json` ve `phases/20-spread.json` (tohum 1013208, 1013210,
1013213) eski denetleyicinin evre izidir. İki parkurda iz aynıdır, çünkü geniş
basış planlayıcısı en az iki ayak üstte olmadan devreye girmez. 20 cm'de bu hiç olmadı.

- Ön ayak evresinde hedef gövde yüksekliği 0,191 m, hedef öne eğim 28,6°tir.
  Arka tabanlar kalçalarının 27–30 cm gerisinde kalır. Bu duruşta bazı bacaklar
  tabanlarına erişemez ve yükten kurtulur. Örneğin 1013213'te RR, LF, LM ve LR'nin
  ortalama yükü ağırlığın %1'inin altındadır. Tabanların toplam yükü ağırlığın
  yalnız %40'ıdır; gövde kutusu yük taşımaz.
- Gövde 2–4 tabana kalır. Tohuma göre RF tibia, RM coxa/tibia, LF coxa/femur ve
  LM tibia, açığın 2 cm'yi aştığı örneklerin %50–100'ünde tork sınırındadır.
- Bu örneklerde gövde hedefin ortalama 4,5–6,2 cm, en çok 10,8 cm altındadır.
  Salınımdaki ön ayak (1013208 ve 1013213'te LF, 1013210'da RF) 6–43 s boyunca
  basamağa çıkamaz. Üstte en fazla 1 taban görüldü; 1013208 ve 1013210 düşmeyle bitti.

Planlı denetleyicide (`phases/20-staged.json`) aynı tohumlarda ön ayak evresinde
gövde hedefin ortalama 1,7–2,1 mm, en çok 6,6 mm altında kalır. Gerideki
ayakların toplanmasıyla birlikte iki ön ayak 17–18 s içinde, gövde aktarımı ve
orta ayaklar 13–19 s içinde basamağa çıkar. Tıkanma arka
aktarıma kayar. Planlayıcının kayıtlı gerekçeleri `six-foot RR reach` ve
`five-foot torque LR_femur` olur. Gövde kutusunun 20 cm basamağın üstünden
geçmesi için gövde ≈0,27–0,29 m yükseğe çıkmalıdır. Bu durumda yerdeki arka
tabanlar, 27,1 cm'lik femur+tibia boyunun erişebileceği pozda kalmaz. Statik
taramada da gövde desteği olmadan 20 cm arka aktarım için uygun duruş bulunmadı.
Bu yüzden 20 cm için ayrı bir parkur ve kaldır-sonra-eğ hareketi geliştirildi
(aşağıda).

### Artık PPO (`ppo-13/`, `ppo-14/`, `ppo-15/`)

`tools/staged_train.py` ile eğitildi: 2 paralel ortam, `--action-scale 0.5`,
128×128 aktör ve değer ağı, `log_std_init −1.7`. Bölümlerin yarısı 500000–500011
tohumlarındaki başarılı denetleyici koşularından alınan ön, orta ve arka evre
ara durumlarından başlar (13 cm'de 121, 14 cm'de 130, 15 cm'de 130 durum). Geri
yükleme fizik durumunu, bölümün rastgele dinamiklerini ve denetleyici durumunu
birebir kurar; testte sapma 0'dır. Her 32.768 geçişte kontrol noktası kaydedilir
ve 1016100–1016109 doğrulama grubunda parkurun başından değerlendirilir. Seçim
sırası: temas sınırları (gövde darbesi yok, rıht ≤ 0,25 s, örtüşme ≤ 2 mm,
diz > −152°), tamamlama, düşme, rıht teması. Seçilen model bir kez 1017300 grubunda,
aynı tohumlarda yalnız denetleyiciyle karşılaştırılır.

| Aşama | Eğitim | Doğrulama (10): denetleyici → seçilen | Son grup (20): denetleyici | Son grup (20): PPO | Aşama sonucu |
| --- | --- | --- | --- | --- | --- |
| 13 cm | 196.608 yeni geçiş, sıfırdan | 7/10 → 10/10 (196.608. adım) | 18/20, rıht en çok 0,17 s | **20/20**, 0 düşme, rıht en çok 0,16 s | PPO ile geçti |
| 14 cm | 13 cm modelinden 131.072 yeni geçiş | 8/10 → 9/10 (229.376) | **18/20**, rıht en çok 0,19 s | 17/20, 0 düşme | denetleyiciyle geçti; PPO adayı geçmedi |
| 15 cm | 14 cm modelinden 196.608 yeni geçiş | 9/10 (rıht 0,37 s, sınır dışı) → 8/10 (327.680) | 15/20, rıht en çok 0,27 s | 16/20, 0 düşme, rıht en çok 0,21 s | **geçmedi**, müfredat burada durur |

13 cm'de denetleyicinin takıldığı iki tohumu (1017309, 1017316) PPO tamamladı.
14 cm'de doğrulamada daha iyi görünen model son grupta 1017301'de ön ayak
evresinde takıldı ve 1017313'te arka aktarımı bitiremedi. Bu yüzden 14 cm
aşaması yalnız denetleyiciyle geçmiş sayılır.

15 cm'de PPO 20 bölümün 18'inde altı ayağı da basamağa çıkardı. Bunların ikisi
(1017302, 1017316) 60 s sınırında bitişe ulaşamadı. 1017313 arka aktarımda,
1017319 ön ayak evresinde takıldı. Aynı tohumlarda yalnız denetleyici 1017301
ve 1017304'te orta ayak evresinde, 1017309, 1017313 ve 1017316'da arka aktarımda
takıldı.

### Düz zemin (`flat-gaits.json`, 10 tohum, her biri 8 s)

| Ölçüm | Referans ileri | Yeni ileri | Referans sol | Yeni sol | Referans dönüş | Yeni dönüş |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Hız (m/s veya rad/s) | 0,116 | 0,114 | 0,079 | 0,079 | 0,301 | 0,303 |
| Yükte taban kayması, 8 s toplamı (mm) | 396 | 337 | 381 | 332 | 380 | 332 |
| Sekme (0,1 s içinde yeniden temas) | 42,0 | 9,7 | 53,5 | 14,4 | 48,4 | 6,7 |
| Yuvarlanma RMS (°) | 0,41 | 0,25 | 0,50 | 0,25 | 0,51 | 0,23 |
| Gövde yükseklik sapması (mm) | 0,50 | 0,10 | 0,70 | 0,26 | 0,69 | 0,12 |
| Sert basış (bölüm başına ortalama) | 4,2 | 0 | 0 | 2,0 | 0 | 0 |
| Herhangi bir eklem tork sınırında (s / 8 s) | 5,55 | 1,20 | 5,42 | 2,88 | 5,64 | 0,98 |

Geri ve sağ yönler `flat-gaits.json` dosyasındadır. Eski profilde zamanın büyük
kısmında orta femurlar tork sınırındadır (ileri yürüyüşte 5,0 s). Üçlü destekte
orta bacak kendi tarafını tek başına taşır. Yeni yürüyüş orta tabanı 2 cm içeri
alır. Bu, femur kaldıracını kısaltır ve orta femur sınır süresini 0,24 s'ye indirir.
Buna karşılık tibia sınır süreleri artar (ileri yürüyüşte ön 0,20→0,53 s, orta
0,07→0,68 s, arka 0,29→0,45 s); toplam eklem·s 6,0'dan 2,3'e iner.
Yürüyüşten duruşa geçişte yeni yürüyüş 10/10 bölümde duruş pozunda bitti; eskisi
0/10. Durma sırasındaki ayak merkezi kayması 0,184 m'den 0,054 m'ye indi. Yeni
yürüyüş komut sonrası 3,6 cm daha ilerler. Sola yürüyüşteki 2 sert basış henüz
çözülmedi.

### 20 cm: kaldır-sonra-eğ (`hybrid-wide-rear-up-20`, ayrı deney parkuru)

Bu parkur `hybrid-staged-up-20` ile aynı basamağı, kütleyi, servo sınırlarını ve
çarpışma modelini kullanır. Yalnız son iki arka ayağın aktarımı farklıdır; 13–19 cm
parkurlarının kodu bu parkurun kurallarından etkilenmez.

**Neden tıkanıyordu.** Dört ayak üstteyken son arka ayak kaldırılamıyordu. Planlayıcı
bir poz arıyordu ki altı ayak yerdeyken tüm bacaklar erişsin ve o pozda arka bacak
tepeye katlanabilsin. Böyle bir poz yoktur. Gövde yüksekte (z ≈ 0,29 m) olduğu için
yerdeki arka taban neredeyse tam gergindir, ama basamağa taşınırken bacağın katlanacak
yeri kalmaz. Ağırlık merkezini dört+bir ayağın çokgenine sokmak için gövdeyi öne almak
ise yerdeki arka ayağın erişimini bozar. Planlayıcı bu yüzden iki gövde kaydırma
önerisi arasında gidip geliyordu (`shift` döngüsü, 21 s ve daha uzun).
Poz arama listesi ayrıca en düşük öne eğimi +5° ile sınırlıydı; burun aşağı poz hiç
denenmiyordu.

**Yeni hareket.** Ayak mevcut pozda birkaç santim kaldırılır; ayak havadayken gövde
hedef pozuna (burun aşağı, 5–10° daha az öne eğim ve gerekirse 8 cm'ye kadar sağa/sola)
getirilir; sonra ayak basamağa taşınır. Her aşama ayrı denetlenir: mevcut pozda beş
ayaklı denge, hedef pozda beş ayaklı denge, aradaki poz, kaldırma yolu ve geri kalan yol.
Gövde hedef poza ölçülen konumla yerleşmeden ayak basamağın kenarını geçmez.

**Sonuçlar** (`climb/20-*.json`, süre sınırı 90 s; eski parkurlardaki 60 s sınırı
başarılı koşuların yarısını kesiyordu):

| Parkur ve grup | Tamamlama | Düşme | 60 s içinde | Ortanca süre | Rıht teması (başarılılarda) |
| --- | --- | --- | --- | --- | --- |
| Önceki geniş arka ayak, 1013200–19 (120 s) | 1/20 | 4 | 0 | 64,1 s | 0 |
| **Kaldır-sonra-eğ, 1013200–19** (geliştirme) | 11/20 | 1 | 9 | 59,4 s | 0 |
| **Kaldır-sonra-eğ, 1017300–19** (son değerlendirme) | **13/20** | **0** | **12** | 59,3 s | **0** |

Son değerlendirmede gövde darbesi yok, diz −148,9°'den fazla kapanmadı, başarılı
koşuların hiçbirinde rıht teması yok. Geçiş ölçütü 18/20 olduğu için **20 cm geçmedi**.
Takılan 7 koşunun 6'sı beş ayak üstteyken son arka ayakta (rear), biri orta ayakta
(middle) kaldı. Takılan koşularda tabanın rıhta değdiği süre 0,5–4,1 s arasındadır.

**Denenip bırakılanlar** (6 tohumluk kısa denemeler ve 1013200–19 grubu):
gövdeyi 0,105–0,12 m yükseltmek 0–1/6 verdi (temel ayar 2/6); ön ayakları 7 cm geri
çekmek 2/6 verdi ama başka tohumları bozdu; başlangıç pozunda tork ve ağırlık
merkezi payını gevşetmek 9 takılı koşunun 2'sini çözdü ama daha önce geçen 3 koşuyu
bozdu (toplam 11/20 → 9/20); tork payını sıkılaştırmak hiç plan bırakmadı (0/6).
Beş ayaklı duruşta RR tibia/femur ve LM femur/tibia sürekli tork sınırındadır; ağırlığın
%30–40'ını bu iki bacak taşır, ön ayaklar 0,2 m önde kaldığı için neredeyse yük almaz.
Bu yüzden payı simülasyonda bu servo sınırlarıyla artırmak mümkün görünmüyor. Güçlü
femur servoları (DS5160) veya gövde kutusunu basamakta destek yapmak gerekir.

## Tekrar oynatma

Proje kökünde PowerShell:

```powershell
$py = '.\.runtime\sim-rl-env\Scripts\python.exe'
$ppo13 = 'robot_learning\experiments\staged-ascent\ppo-13\hybrid-staged-up-13'
$ppo14 = 'robot_learning\experiments\staged-ascent\ppo-14\hybrid-staged-up-14'
# En iyi çıkış: 13 cm PPO, tohum 1017317, 50,1 s, rıht teması 0 s
& $py -m apex_rl.cli course-policy-view --run $ppo13 --seed 1017317 --output robot_learning\runs\staged-13-best.html
& $py -m apex_rl.cli course-view --run $ppo13 --randomize --seed 1017317
# Tipik hata 1: yalnız denetleyici 1017309'da arka aktarımda takılır; aynı sayfada PPO tamamlar
& $py -m apex_rl.cli course-policy-view --run $ppo13 --seed 1017309 --output robot_learning\runs\staged-13-rear-block.html
# Tipik hata 2: 14 cm PPO adayı 1017301'de ön ayak evresinde takılır; denetleyici tamamlar
& $py -m apex_rl.cli course-policy-view --run $ppo14 --seed 1017301 --output robot_learning\runs\staged-14-front-block.html
# 15 cm: PPO altı ayağı çıkarır ama 60 s içinde bitişe varamaz (1017302)
$ppo15 = 'robot_learning\experiments\staged-ascent\ppo-15\hybrid-staged-up-15'
& $py -m apex_rl.cli course-policy-view --run $ppo15 --seed 1017302 --output robot_learning\runs\staged-15-timeout.html
# Yalnız denetleyici: aynı iki tohum
& $py -m apex_rl.cli course-view --course hybrid-staged-up-13 --randomize --seed 1017317
& $py -m apex_rl.cli course-view --course hybrid-staged-up-13 --randomize --seed 1017309
# 20 cm: ön ve orta ayaklar üstte, arka aktarım tıkanıyor
& $py -m apex_rl.cli course-view --course hybrid-staged-up-20 --randomize --seed 1017300
# 20 cm kaldır-sonra-eğ: en iyi çıkış (tohum 1017317, 58,3 s) ve takılan örnek (1017301, arka ayak)
& $py -m apex_rl.cli course-view --course hybrid-wide-rear-up-20 --randomize --seed 1017317 --replay-seconds 90
& $py -m apex_rl.cli course-view --course hybrid-wide-rear-up-20 --randomize --seed 1017301 --replay-seconds 90
# 20 cm ölçümünü yeniden üret (yaklaşık 6 dk, iki çekirdek)
& $py robot_learning\tools\staged_experiments.py climb --course hybrid-wide-rear-up-20 --seed 1017300 --episodes 20 --output robot_learning\runs\wide-rear-20.json
# Servo kasası / yuva doğrulaması
& $py robot_learning\tools\check_servo_mounts.py
# Ölçümleri yeniden üret
& $py robot_learning\tools\staged_experiments.py climb --course hybrid-staged-up-13 --seed 1017300 --episodes 20 --output robot_learning\runs\staged-13.json
& $py robot_learning\tools\staged_experiments.py climb --course hybrid-staged-up-13 --run $ppo13 --seed 1017300 --episodes 20 --output robot_learning\runs\staged-ppo-13.json
# PPO eğitimi (yaklaşık 25 dk, iki CPU çekirdeğinde)
& $py robot_learning\tools\staged_train.py --course hybrid-staged-up-13 --timesteps 196608 --output robot_learning\runs\staged-ppo-13-retry
```

`course-policy-view` sayfası aynı başlangıç koşulunda PPO ile yalnız denetleyiciyi
(sıfır düzeltme) yan yana oynatır. Planlı parkurlarda 12 çıkışın 6'sı etkindir;
diğer 6 çıkışın değeri ağda görünür ama denetleyiciye ulaşmaz.

Eğitim sırasındaki bütün kontrol noktaları, kayıtlı ara durumlar ve izleme kayıtları
yerel `robot_learning/runs/staged-ppo-13`, `-14`, `-15` klasörlerindedir. Git'e yalnız
seçilen modeller ve özetler girdi.
