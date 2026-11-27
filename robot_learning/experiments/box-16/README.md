# 16 cm kutu: çık, üstünde yürü, in (`hybrid-box-16`)

Her şey MuJoCo simülasyonundadır; hiçbir dosya robota komut göndermez.

## Parkur

16 cm yüksek, 60 cm geniş, 60 cm uzun katı kutu; ön kenarı başlangıçtan 1,05 m ileride.
Bitiş, arka kenarın 45 cm ötesinde ve altı tabanın da kenarı geçmiş olmasını ister. Kütle,
servo tork/hız sınırları, diz durdurucusu ve çarpışma modeli diğer parkurlarla aynıdır.
Her bölümde zemin sürtünmesi ±%15, gövde kütlesi ±%5, servo kazancı ±%10 ve pil
gerilimine bağlı tork sınırı rastgeledir. Süre sınırı 150 s.

## Denetleyici (`src/apex_rl/box_obstacle.py`)

1. **Yaklaşma ve çıkış:** `staged_ascent.py` değişmeden (planlı, tek tek ayak; erişim,
   destek çokgeni, tork, salınım yolu ve rıht açıklığı denetimli).
2. **Üstte yürüyüş:** altı ayak üstteyken düz zemin tripodu yürür; ön tabanlar arka
   kenara 10 cm kalınca durur ve altı ayağın yere basmasını bekler.
3. **İniş:** tek tek ayak, çıkışla aynı denetimlerle. Kenardan ileri inmek, geometri
   olarak kenara geri geri çıkmakla aynıdır; denetimler x ekseni aynalanıp ön ve arka
   bacaklar yer değiştirerek çağrılır (doğruluğu testte 200 rastgele duruşta birebir).
   Aşamalar, 16 cm çıkışın aşama düzenlerinin aynası ve ters sırasıdır: ön çift, orta
   çift, arka çift. Her çift inmeden önce kenara yürür. Ön ayak düz ve yüksek bir pozda
   kalkıp kenarı aşar, kenarın altına inmeye başlayınca gövde burun-aşağı ikinci poza
   geçer; ayak bu sırada bekler ve ancak ölçülen gövde yerleşince yere iner. İki poz ve
   aradaki poz da denetlenir.
4. **Yerde yürüyüş:** düz zemin tripodu bitişe yürür.

Kilitlenme onarımı (yalnız bu parkurda): çıkışın arka aşamasında gövde iki kaydırma
pozu arasında gidip gelirse arka ayak için kaldır-sonra-eğ denenir; art arda başarısız
planlarda "geride ayak" sınırı 4 cm gevşer. Üstte duran tripodun bir tabanı yük almazsa
1 s sonra beş yüklü tabanla iniş başlar.

## Sonuçlar

| Grup | Denetleyici | Tamamlama | Düşme | Gövde darbesi | Rıht teması > 0,25 s | Ortanca süre |
| --- | --- | --- | --- | --- | --- | --- |
| 1017300–1017319 (son değerlendirme, ayarda kullanılmadı) | mevcut hibrit dalga yürüyüşü | 0/20 | 7 | 7 | 20 koşu | — |
| 1017300–1017319 | **planlı çık-yürü-in** | **16/20** | **0** | **0** | 4 koşu (3'ü başarılı, en çok 0,45 s) | 109,6 s |
| 1013200–1013219 (geliştirme, ayarda kullanıldı) | planlı çık-yürü-in | 15/20 | 0 | 0 | 2 koşu | 112,5 s |

Geçiş ölçütü 18/20, en çok 1 düşme, gövde darbesi yok, her koşuda rıht teması ≤ 0,25 s:
**geçmedi.** En kapalı diz −151,6° (sınır −152°).

Rıht teması sınırı da sayılınca temiz tamamlama 13/20'dir. Son değerlendirmede
başarısızların yeri: çıkışın arka aşaması 2, inişin ön aşaması 2. (Üstte duran tripodun
bir tabanı yük almadan kaldığında iniş hiç başlamıyordu; bu düzeltildi ve yalnız o koşuyu,
1017313'ü, değiştirdi.) Başarılı koşularda aşama süreleri (ortanca): çıkış 44 s, üstte yürüyüş
ve durma 2,8 s, iniş ve bitişe yürüyüş 52 s.

Çıkıştaki takılma, aynı 16 cm tek basamaklı parkurdaki planlı çıkışın bilinen
kilitlenmesidir: ön tabanlar hedefin birkaç santim gerisinde kalır, gövde iki kaydırma
pozu arasında gidip gelir. Ön tabanları "hazır" sayıp arka ayağı zorlamak denendi;
arka ayak hamleleri o duruşta da denetimlerden geçmedi ve değişiklik geri alındı.

## Gerçek dünya koşullarına dayanıklılık (10 tohum, normal koşulda hepsi tamamlanıyor)

| Koşul | Tamamlama | Düşme | Not |
| --- | --- | --- | --- |
| Normal | 10/10 | 0 | |
| Kutu, denetleyicinin bildiğinden 2 cm uzak (iki kenar) ve 1 cm yüksek | 9/10 | 0 | 1 koşu çıkışta takıldı, 5,95 s rıht teması |
| Kutu 2 cm yakın ve 1 cm alçak | 7/10 | 1 | 2'si çıkışın arka aşamasında (biri düştü, biri 64 s rıhta dayandı), 1'i inişte |
| Zemin sürtünmesi 0,6–1,2, gövde kütlesi ±%15, her servo %15'e kadar zayıf | 5/10 | 0 | 4'ü çıkışta, 1'i inişte takıldı; bazı koşularda rıht teması 0,3–31 s |

Sonuç: kutuyu olduğundan **yakın ve alçak** sanmak, uzak ve yüksek sanmaktan çok daha
zararlıdır; ölçüm belirsizse kenar biraz uzak, yükseklik biraz fazla girilmelidir.
Gerçek zemin ve servo dağılımı simülasyondakinden genişse çıkış en zayıf halkadır.


## Gerçek robota aktarmadan önce

Simülasyondaki denetleyici üç şeyi doğrudan okur; robotta bunların karşılığı gerekir:

- **Kutunun yeri ve yüksekliği.** Ölçülüp girilebilir; yukarıdaki ölçü hatası testi
  ±2 cm / ±1 cm hatayı gösterir. Kamera/ToF ile ölçmek ayrı iştir.
- **Ayak teması.** Her taban inişinde ve kaldırmadan önceki destek çokgeni denetiminde
  kullanılır. Servo akımı veya taban anahtarıyla algılanmalı.
- **Gövde konumu ve eğimi.** Eğim IMU'dan; konum, bacak odometrisinden.

Planlama süresi de önemlidir: bir ayak hamlesi için yüzlerce poz denenir. Simülasyonda
kilitlenme anlarında tek bir yeniden planlama saniyeler sürebiliyor; robottaki işlemcide
ölçülmeli. İlk gerçek deneme askıda ve düşük hızda yapılmalı.

## Tekrar oynatma ve ölçüm

```powershell
$py = '.\.runtime\sim-rl-env\Scripts\python.exe'
& $py -m apex_rl.cli course-view --course hybrid-box-16 --randomize --seed 1017317
& $py -m apex_rl.cli course-view --course hybrid-box-16 --randomize --seed 1017309   # çıkışta takılır
& $py -m apex_rl.cli course-bench --course hybrid-box-16 --episodes 20 --seed 1017300 --eval-workers 2
```

`results/` klasöründeki dosyalar tablonun kaynağıdır.

## Windows'ta iniş denetimi

`course-bench` kutu parkurlarında rıht temas süresini, 0,25 s sınırı içinde
bitiren koşu sayısını ve geçiş sonucunu yazdırır. `hybrid-box-16-guarded`
aynı kutuda iniş denetiminin deneysel sürümüdür. Ayak kenarı ölçülen konumda
aşınca alçalır. İkinci gövde pozu yerleşmezse veya iniş teması doğrulanamazsa
simülasyon bölümü güvenli durdurma gerekçesiyle sonlanır.

| Tohum | Mevcut sürüm | Ölçümle korunan iniş |
| --- | --- | --- |
| 1013200 | 117,80 s, 0,162 s rıht teması | 75,58 s, ön ayağın alt zemin teması doğrulanamadı |
| 1017300 | 109,58 s, 0 s rıht teması | 110,12 s, 0 s rıht teması |
| 1017317 | 103,30 s, 0 s rıht teması | 105,46 s, 0 s rıht teması |

Bu üç karşılaştırmada düşme ve gövde darbesi görülmedi. Örnekler iniş
denetimlerinin çalışmasını gösteriyor. Dağılım testi ve donanım doğrulaması için
önceki bölümdeki açıklar sürüyor. ESP yazılımında ayak kontağı telemetrisi
mevcut. Gerçek robotta altı anahtarın ve gövde konum tahmininin ölçümle
doğrulanması gerekiyor. HOME ve STAND firmware kalibrasyonu bağımsızdır.

```powershell
& $py -m apex_rl.cli course-view --course hybrid-box-16-guarded --randomize --seed 1017317
```
