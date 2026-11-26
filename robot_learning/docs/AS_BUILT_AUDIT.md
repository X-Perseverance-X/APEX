# META-1 MuJoCo modeli ve fiziksel robot

Üst LiDAR'ın referans konumu ve kasa modeli önceki sürümden geliyor. Bacakların braketleri,
delikleri ve kavisli tibia plakaları gönderilen `MARK_2_LEG.SLDASM` ve STL
parçalarından alındı. Assembly'deki görünür parça dönüşümleriyle yerleştirilen
geometri sağ ve sol bacaklar için aynalandı. Meshler gerçek ölçekte, yalnızca
görselleştirme için kullanılıyor. Ayak temasları ve eklem merkezleri firmware
v57.33 ile eşleşen fizik modeli üzerinden hesaplanıyor. CAD'den çıkarılan
eklem aralıkları ile fizik modeli arasındaki fark üç bağlantıda da 1 mm altında.
Diskteki montaj dosyalarını değiştirmeden dönüşümü yeniden üretmek için
`tools/import_mark2_leg.py` betiğine kaynak CAD klasörünü verin.

## Gövde ve kamera montajı

Gövde, 300 × 110 × 8 mm alt plaka dahil 24 ayrı STL'den toplandı. Kapağın
kesiti, yan panel yuvaları ve pil kafesinin delikleri kaynak çizimlerle aynı
ölçekte. Yerleşim için alt fotoğraftaki sıra kullanıldı. Arkada üç yuvalı
ana pil kapağı, ortada iki yuvalı pil tablası ve üçgen yan destekleri, önde
90 × 50 mm hoparlör ızgarası ve arkasında amfi var. Fan ön siyah ara parçanın
arkasında, gövde içinde. Fan, hoparlör sürücüleri, kart ve piller temsili
geometriler. Arka uç kapağı dik konumda, üst açıklığında güç düğmesi var.
LiDAR kapağının dairesel kesiti kayıtlı merkeze hizalandı. Ön LED çubuğu
LiDAR'ın hemen altında ve gövdeye sabit.

Kamera üç montaj parçası kullanıyor. `Pan_and_tilt_holder` sabit yuvadır.
`Camera_Holder_2` pan miliyle dönen ara koldur. `Camera_Holder` tilt
miline takılan lens ve IR kartı taşıyıcısıdır. Dizilim, STL'deki mil ve yuva
geometrisinden çıkarıldı. Kamera optik merkezi korundu ve yeni ön fotoğrafa
göre ToF kartı tilt taşıyıcısına bağlandı.

Gövde STL'leri ayrı parça ihraçları. Yerleşim ölçüleri, fotoğraflar ve
mevcut sensör eksenleriyle hizalanarak [body_layout.json](../src/apex_rl/assets/body_layout.json)
dosyasında ayarlanabilir hale getirildi. Alternatif pil bağlantısı
`Battery_Con` bu montaja takılmadı.
`tools/import_mark2_body.py` ile kullanılan STL'ler kaynak klasörden yeniden
üretilebilir.

## Gövde merkezinden sensör ve eklem konumları

+X ileri, +Y sol, +Z yukarı. Tablodaki ölçüler milimetre, [model
ayarlarındaki](../src/apex_rl/assets/meta1.json) konum değerleri metredir.
Kamera ve pan/tilt konumları nötr poz için verilmiştir.

| Parça | X, Y, Z (mm) | Kaynak ve simülasyondaki karşılığı |
| --- | --- | --- |
| Ön / orta / arka coxa | +125 / 0 / −125, ±98, 0 | Firmware v57.33. Y işareti sol için pozitif, sağ için negatif. |
| MPU6050 | +115, +15, +48 | X ve Y kullanıcı ölçüsü. Z önceki montaj kaydından. İvmeölçer ve jiroskop ideal sinyal üretir. |
| LiDAR | +135, 0, +108 | Kullanıcının ölçtüğü tarama merkezi. Gövdeye bağlı referans çerçevesi ve görünür kasa. 2B tarama paketi üretimi sonraki aşama. |
| Pan ekseni | +130, 0, 0 | Kayıtlı eksen noktası. MuJoCo'da +Z etrafında, artı açı sola döner. |
| Tilt ekseni | +178, 0, +40 | Kayıtlı eksen noktası. Pan ile taşınır, artı açı kamerayı aşağı çevirir. |
| Kamera optik merkezi | +213, 0, +50 | Kullanıcı ölçüsü. Tilt ile taşınır. Nötrde ileri bakar. Yatay görüş açısı 16:9 görüntüde yaklaşık 78°. PCB, lens ve iki IR aydınlatıcı Waveshare Camera (F) benzeri. Kamera varyantı doğrulanmadı. |
| ToF | +214,5, 0, +64 | Nötr lensin yaklaşık 14 mm üstünde. Konum kamera taşıyıcısındaki delik sırasına göre çıkarıldı. Pan ve tilt ile taşınır. |

Pan ve tilt için 90° elektriksel merkez simülasyonda 0° olarak gösterilir.
45°–135° komut aralığı ±45° eklem aralığına karşılık gelir. Gerçek sistemdeki
kamera nötr yönü ve lens kalibrasyonu ölçülmeyi bekliyor. Yürüyüş ortamı, kamera
eklemlerini nötrde tutarak 18 eylem ve 72 gözlemli arayüzünü kullanır. MuJoCo
görüntüleyicisinde iki kamera eklemi ayrı hedeflerle sürülebilir.
Bazı yukarı bakış ve pan bileşimlerinde lamba ya da üst kapak, simülasyonda
kamera ve ToF görüşünü kısmen kapatır. Açısal sınırlar gerçek montajda
ölçülmüş optik açıklığı temsil etmiyor.

## Servo ve kütle

| Parça | Eldeki bilgi | Model durumu |
| --- | --- | --- |
| 6 femur servosu | DS3240 referansı. Kasa 40 × 20 × 40,5 mm. Belge 5 V'ta 3,53 N·m kilitli mil torku ve 300°/s yüksüz hız veriyor. | Temsili kasa, 2,15 N·m tork sınırı ve RL ortamında 100°/s hedef değişim sınırı. |
| 12 coxa/tibia servosu | BLS3225 referansı. Kasa 40 × 20 × 38,5 mm. Belge 4,8 V'ta 1,47 N·m kilitli mil torku ve 750°/s yüksüz hız veriyor. Takılı varyant fotoğraftan doğrulanmadı. | Temsili kasa, 1,20 N·m tork sınırı ve RL ortamında 145°/s hedef değişim sınırı. |
| 2 kamera servosu | TowerPro SG90 Analog referansı. Kasa 23 × 12,2 × 29 mm. Kilitli mil torku 4,8 V'ta yaklaşık 0,177 N·m. | Pan ve tilt gövdelerinde temsili kasa, 0,095 N·m tork sınırı. |
| Gövde ve bacaklar | Kullanıcı tahmini toplam 5 kg. Parça tartımları ve ağırlık merkezi henüz ölçülmedi. | %20 yük payıyla 6,00 kg toplam. Bacak başı 0,40 kg, gövde 3,564 kg, hareketli kamera parçaları 0,036 kg. |

Tam dolu standart 2S paket 8.4 V verir. Eldeki femur servoları ETR373
belgesindeki 6.8 V azami gerilimli DS3240 varyantıysa doğrudan besleme bu sınırı
aşar. Takılı varyantın gerilim sınırı ve V+ ölçümü, servo kalibrasyonundan
önce kayda alınmalı. Stall torku kilitli mil koşuludur. Yürüyüş simülasyonu
için yüklü hız, gecikme, akım, boşluk ve tork sınırı ayrıca ölçülür.

- [ETR373 DS3240 ürün belgesi](https://etechrobot.com/wp-content/uploads/2026/04/DS-3240-40KG-High-Torque-Digital-Servo-Motor-Datasheet.pdf)
- [DSSERVO BLS3225 ölçü çizimi ve ürün özellikleri](https://www.zennixtek.com/products/dsservo-bls3225-servo-motor)
- [TowerPro SG90 üretici verisi](https://towerpro.com.tw/product/sg90-analog/)
- [Waveshare RPi Camera (F) ve IR aydınlatıcılar](https://www.waveshare.com/wiki/RPi_Camera_%28F%29)
- [MuJoCo sensör ve eklem tanımları](https://mujoco.readthedocs.io/en/stable/XMLreference.html)

Bu aşamanın çıktısı MuJoCo MJCF modeli ve yanında taşınan `mark2_leg/` ile
`mark2_body/` mesh klasörleridir. Bacak çarpışma geometrisi, sürtünme, kütle dağılımı ve servo tepki
modeli fiziksel ölçümlerle doğrulandığında gerçek yürüyüşle karşılaştırılabilir.
