# Ev merdiveni (home flight) deneyi

Her şey MuJoCo simülasyonundadır; hiçbir dosya robota komut göndermez.

## Merdiven modeli (`src/apex_rl/home_flight.py`)

| Ölçü | Değer | Kaynak |
| --- | --- | --- |
| İlk rıht | 20 cm | ölçüldü |
| İlk basamak derinliği | 30 cm | ölçüldü |
| Sonraki düz basamaklar | 27 cm | ölçüldü (yaklaşık) |
| Sonraki rıhtlar | 20 cm | **varsayım**, ölçülmedi |
| Genişlik (duvar – korkuluk) | 90 cm | fotoğraftan tahmin |
| Alt kol | 5 düz basamak | fotoğraftan sayıldı |
| Dönüş | 3 adet 30° kama, sola, korkuluk köşesi etrafında | fotoğraftan |
| Üst kol | 6 düz basamak ve sahanlık (toplam 14 rıht, 2,80 m) | fotoğraftan sayıldı |
| Basamak burnu | 2 cm taşma, 3 cm tahta | tahmin |

Duvar çıkarken sağda, korkuluk solda. Kamalar köşe karesine kırpılmış katı prizmalardır,
dış köşede boşluk yoktur. Bütün parçalar (basamaklar, burunlar, kamalar, üst kol,
duvarlar) temas ölçümüne dahildir.

Mevcut `home_stairs.py` ve `locomotion.py` değiştirilmedi. Kayıtlı politikaların parmak
izi bu dosyalardan hesaplanıyor; yeni parkur adları ayrı modülde kaydedilir ve eğitim
seviye listesine (`STAGES`) girmez.

| Parkur | İçerik |
| --- | --- |
| `home-flight-preview` | Tam merdiven, robot ilk rıhtın önünde |
| `home-flight-up-1`, `-2`, `-5` | Aynı sahne; bitiş 1., 2. basamağın ortası veya alt kolun sonu |

```powershell
& $py -m apex_rl.cli course-view --course home-flight-preview --static
# Mevcut dalga yürüyüşü bu merdivende: 1017300 60 s'de takıldı (üstte en çok 2 ayak), 1017301 20,7 s'de düştü
& $py -m apex_rl.cli course-view --course home-flight-up-2 --body-control --randomize --seed 1017300
```

`ev-merdiveni-model.png`: modelin dört görünüşü. Fotoğraflarla yan yana karşılaştırma yerel
`robot_learning/runs/merdiven-karsilastirma.png` dosyasındadır (ev fotoğrafı içerdiği için Git'e girmedi).

## Düz kolda statik yürüyüş araması (`tools/stair_gait_search.py`)

Basamaklar 27 cm derin, robotun ön ve arka tabanları ise ≈50 cm aralıklı; altı ayak
tek basamağa sığmaz, robot 2–3 basamağa yayılmak zorundadır. Denetleyici yazmadan
önce bunun bu robotla mümkün olup olmadığı tarandı.

Durum: her bacak çifti (ön, orta, arka) için basamak seviyesi ve basamak üstündeki
yeri (kullanılabilir derinliğin %10, 30, 50, 70, 90'ı). Bir hamle bir çifti bir basamak
yukarı taşır veya basamak üstünde öne kaydırır; sağ ve sol bacak sırayla hareket eder.
Her tek ayak kaldırışı için planlayıcının denetimlerinin hepsi aranır: kalan beş ayakta
erişim ve eklem payı, ağırlık merkezi payı, tork, bacak/basamak açıklığı, gövde altı ile
basamak burnu arasında 1 cm, salınım yolunun tamamında erişim ve rıht açıklığı. Gövde
pozu tek poz veya **kaldır-sonra-eğ** (ayak bir pozda kalkar, havadayken gövde ikinci poza
geçer, ara poz da denetlenir) olabilir; gövde kaldırılan ayağın karşı tarafına 6 cm'ye
kadar kayabilir. Bir yürüyüş, aynı göreli duruma bir basamak yukarıda dönen hamle
dizisidir.

Bu arama **gerekli** bir koşulu sınar: bulunan dizi dinamik olarak da yürünebilir mi,
ayrıca simülasyonda koşulması gerekir. Bulunamaması ise bu varsayımlarla (gövde
desteği yok, sabit yanal ayak yerleri, yarı-statik hareket) imkânsızlık demektir.

| Rıht | Tibia | Tork sınırı | Arka yanal | Ayakta duran durum | Seviye atlayan hamle (ön / orta / arka, kabul/deneme) | Tekrarlanan yürüyüş |
| --- | --- | --- | --- | --- | --- | --- |
| 17 cm | 16,84 cm | model | 0,205 m | 275/685 | 121/243, 89/250, **3/310** | **var** |
| 18 cm | 16,84 cm | model | 0,205 m | 261/685 | 103/353, 70/305, **0/395** | yok |
| 20 cm | 16,84 cm | model | 0,205 m | 208/685 | 56/226, 60/125, **0/271** | yok |
| 20 cm | 16,84 cm | ×1,5 | 0,205 m | 254/685 | 168/353, 142/260, **0/355** | yok |
| 20 cm | 18,5 cm | model | 0,205 m | 238/685 | 51/300, 59/205, **0/334** | yok |
| 20 cm | 20 cm | model | 0,205 m | 246/685 | 51/296, 58/215, **0/371** | yok |
| 20 cm | 16,84 cm | model | 0,265 m | 140/685 | 20/130, 44/100, **0/90** | yok |
| 20 cm | 16,84 cm | model | 0,290 m | 126/685 | 17/99, 19/100, **0/65** | yok |

17 cm'de bulunan dizi (çift başına seviye ve basamak üstü yeri):
ön %90, orta %50, arka %10 aynı basamakta → ön bir üst basamağa (%90) → orta bir üst
basamağa (%50) → arka kendi basamağında %50'ye kayar → arka bir üst basamağa (%10).

Arka çiftin 20 cm'de çıkamamasının gerekçeleri (temsilî durumda, ret sayısına göre):
gövde altının basamak burnuna 1 cm'den fazla yaklaşması, karşı arka bacağın erişimi,
ön bacağın erişimi, karşı arka tibia torku ve kaldırılan bacağın salınım erişimi. Tork
sınırını 1,5 katına çıkarmak veya tibiayı 20 cm'ye uzatmak arka çiftin hiçbir hamlesini
açmadı; arka ayakları 0,265 veya 0,29 m yana açmak da açmadı. Engel geometridir.

## Sonuç

- Gerçek basamaklar 17 cm ise düz kol için statik olarak geçerli bir ayak sırası vardır;
  sıradaki iş bu sırayı planlı denetleyiciye dönüştürüp simülasyonda koşmaktır.
- Basamaklar 18–20 cm ise bu robot, gövde desteği olmadan düz kolu yarı-statik
  çıkamaz. Güçlü femur servosu veya uzun tibia tek başına çözmez. Gövdenin basamak
  burnuna dayanması (yedinci destek noktası) ayrıca denenmelidir.
- Dönüş kamaları ve iniş henüz denenmedi.

## Yeniden üretme

```powershell
& $py robot_learning\tools\stair_gait_search.py --rise 0.17 --tread 0.27 --output robot_learning\runs\gait-17.json
& $py robot_learning\tools\stair_gait_search.py --rise 0.20 --tread 0.27 --tibia 0.20 --output robot_learning\runs\gait-20-t20.json
```

Her koşu iki çekirdekte 5–15 dakika sürer. `gait-search/` klasöründeki dosyalar
yukarıdaki tablonun kaynağıdır.
