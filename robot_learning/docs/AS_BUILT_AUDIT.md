# META-1 MuJoCo modeli ve fiziksel robot

Model, üç robot fotoğrafındaki gri kafes gövdeyi, siyah çift plakalı bacakları,
beyaz ayak uçlarını, üst LiDAR'ı ve öndeki hareketli kamera başını yaklaşık
olarak gösterir. Bacak uzunlukları ve coxa montaj noktaları firmware v57.33 ile
eşleşir. Kabuk, plaka ve sensör kasası ölçüleri fotoğraflardan yorumlandı.

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
| Kamera optik merkezi | +213, 0, +50 | Kullanıcı ölçüsü. Tilt ile taşınır. Nötrde ileri bakar. Yatay görüş açısı 16:9 görüntüde yaklaşık 78°. |
| ToF | +230, 0, +28 | Kinematik laboratuvarındaki tahmini konum. ROS yapılandırmasında başka bir tahmin var. Model +X yönünde ideal mesafe ölçer. |

Pan ve tilt için 90° elektriksel merkez simülasyonda 0° olarak gösterilir.
45°–135° komut aralığı ±45° eklem aralığına karşılık gelir. Gerçek sistemdeki
kamera nötr yönü ve lens kalibrasyonu ölçülmeyi bekliyor. Yürüyüş ortamı, kamera
eklemlerini nötrde tutarak 18 eylem ve 72 gözlemli arayüzünü kullanır. MuJoCo
görüntüleyicisinde iki kamera eklemi ayrı hedeflerle sürülebilir.

## Servo ve kütle

| Parça | Eldeki bilgi | Model durumu |
| --- | --- | --- |
| 6 femur servosu | Kullanıcı kaydı DS3240. ETR373 ürün belgesi 4.8–6.8 V, yüksüz 0.20–0.17 s/60°, stall torku 36–45 kgf·cm ve 60 g bildiriyor. | Belge referans. Temsili konum denetleyici ve femur komut hızı gerçek yük altında ölçülmüş değerler içermiyor. |
| 12 coxa/tibia servosu | Kullanıcı kaydı BLS3225. Fotoğrafta bazı kasalarda 35 kg ibaresi görülüyor ve tam model kodu okunmuyor. Üreticinin DS3225 belgesi ayrı bir ürün için. | Takılı varyant ve yük altında tepki henüz doğrulanmadı. |
| Gövde ve bacaklar | Fiziksel tartım ve ağırlık merkezi kaydı yok. | 8 kg gövde ve 12.802 kg toplam, simülasyon varsayımı. Kamera pan/tilt için iki küçük atalet gövdesi dahil. |

Tam dolu standart 2S paket 8.4 V verir. Eldeki femur servoları ETR373
belgesindeki 6.8 V azami gerilimli DS3240 varyantıysa doğrudan besleme bu sınırı
aşar. Takılı varyantın gerilim sınırı ve V+ ölçümü, servo kalibrasyonundan
önce kayda alınmalı. Stall torku kilitli mil koşuludur. Yürüyüş simülasyonu
için yüklü hız, gecikme, akım, boşluk ve tork sınırı ayrıca ölçülür.

- [ETR373 DS3240 ürün belgesi](https://etechrobot.com/wp-content/uploads/2026/04/DS-3240-40KG-High-Torque-Digital-Servo-Motor-Datasheet.pdf)
- [DSSERVO DS3225 teknik belgesi](https://www.dsservo.com/d_file/DS3225%20datasheet.pdf)
- [MuJoCo sensör ve eklem tanımları](https://mujoco.readthedocs.io/en/stable/XMLreference.html)

Bu aşamanın çıktısı çalışan MuJoCo MJCF modelidir. Ölçüye birebir yüzey ve
çarpışma geometrisi için parça CAD'leri veya ölçekli ortografik ölçüler gerekir.
Simülasyon ile gerçek yürüyüşü karşılaştırmak için de çalışmaya hazır robotun
tartımı, ağırlık merkezi, bacak kütleleri ve yüklü servo tepki kaydı gerekir.
