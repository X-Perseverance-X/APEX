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
