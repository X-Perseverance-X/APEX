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
```

Başlangıç eğitimi ve ölçümü:

```powershell
& $py -m apex_rl.cli course-train --through stairs-20 --envs 4 --timesteps 200000 --output robot_learning/runs/course-try-01
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

| Seviye | Parkur | Başarı |
| --- | --- | --- |
| `flat` | 1,2 m düz zemin | Dengeli biçimde bitişi geç |
| `step-02` | 2 cm tek basamak | Altı ayağın tamamını üst platforma taşı |
| `step-05` | 5 cm tek basamak | Altı ayağın tamamını üst platforma taşı |
| `stairs-05` | 3 × 5 cm | Üç rıhtı geç ve üst platforma ulaş |
| `stairs-10` | 3 × 10 cm | Aynı test |
| `stairs-15` | 3 × 15 cm | Aynı test |
| `stairs-20` | 3 × 20 cm | Aynı test |

Bir seviyeden sonraki seviyeye geçiş için ayrı tohumlarla en az %80 başarı
ve en fazla %10 düşme kabul edilir. Bu ilk proje eşikleri. Fiziksel aktarım
öncesinde daha büyük örneklem ve eğitim sırasında kullanılmamış ayrı bir
nihai değerlendirme kümesi gerekir. Kısa eğitimde 20 cm'ye ulaşma garantisi
yok. Adım yüksekliği, eklem torku, gövde teması ve destek alanı sınır olabilir.

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
