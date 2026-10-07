# SLAM Lab V0

ROS kullanmadan RPLIDAR A1 ve ESP32 uzerinden gelen IMU verisini dogrulayan,
kaydeden ve sinirli 2D haritalama yapan V0 prototipi.

Koordinat sistemi sag ellidir: `x ileri`, `y sol`, `z yukari`.
`base_link` baslangicta `lidar_link` ile aynidir.

## Dogrulanan donanim (2026-10-07)

- Raspberry Pi 4 Model B Rev 1.5, 4 GB
- Debian 13 (trixie), aarch64
- Yerel ekran: 480x320, `fb_ili9486`
- RPLIDAR A1 family; local test unit reported firmware 1.29, hardware rev 7,
  health OK. Unique device serial is kept out of this public repository.
- Canli scan: resmi SLAMTEC `simple_grabber` ile dogrulandi
- Fiziksel on/sol hedef testi: on hedef ham 0 derece, sol hedef ham ~270 derece;
  `lidar_link` icin aci isareti -1 olarak dogrulandi

Iki USB aygit da `10c4:ea60`, urun `CP2102` ve seri `0001` bildirdi.
Bu nedenle `/dev/serial/by-id` tek basina guvenli degildir; link son takilan
aygita kayabilir. Mevcut fiziksel USB yerlesiminde:

- LiDAR ve ESP32 USB portlarini her Pi kurulumunda `tools/detect_hardware.py`
  ile bulun ve `config/hardware.yaml` icindeki portlari guncelleyin. Bu public
  kopyada cihaza ozel path ve LiDAR seri numarasi yer tutucu olarak birakildi.

Fiziksel USB soketleri degistirilirse `tools/detect_hardware.py` ve protokol
probe'u yeniden calistirilmalidir.

## LiDAR motor kontrolu

```bash
cd ~/slam_lab/drivers/lidar
make
./lidar_control --port "$LIDAR_PORT" status
./lidar_control --port "$LIDAR_PORT" off
./lidar_control --port "$LIDAR_PORT" on
```

Program once cihaz bilgisi ve health sorgulayarak portun gercek bir SLAMTEC
LiDAR oldugunu dogrular. Dogrulama basarisizsa motor komutu gondermez.
`tools/lidar_motor off` komutu portu acik tutan bir arka plan sureci baslatir.
A1 adaptoru port kapaninca DTR durumunu geri aldigindan sadece tek seferlik
`setMotorSpeed(0)` kalici kapanma saglamaz. `on` bu sureci durdurur.
Bu nedenle gunluk kullanimda asagidaki wrapper komutlarini kullanin; dogrudan
`lidar_control off` portu kapatinca motor tekrar donebilir. Mapper calisirken
wrapper once localhost-only kontrol API'sinden taramayi durdurur ve ardindan
SDK portunu acik tutan OFF surecini baslatir. Web panosu ve IMU acik kalir.

Kisa komutlar:

```bash
~/slam_lab/tools/lidar_motor off
~/slam_lab/tools/lidar_motor on
~/slam_lab/tools/lidar_motor status
```

## Canli sistem ve replay

Ev icinde baslatma, yavas duzlem hareketi, durdurma ve gercek 3D sinirlari icin
[`HOUSE_MAPPING.md`](HOUSE_MAPPING.md) adimlarini izleyin.

```bash
cd ~/slam_lab
tools/run_mapper
# laptop: http://PI_IP:8765/
tools/local_display  # 480x320 framebuffer; X11 ayarlarina dokunmaz
.venv/bin/python tools/replay_session.py sessions/YYYYMMDD_HHMMSS
.venv/bin/python tools/replay_session.py sessions/YYYYMMDD_HHMMSS --serve --port 8766 --speed 4
.venv/bin/python tools/imu_axis_test.py  # once mapper'i durdur; seri port tek sahipli
.venv/bin/python tools/calibrate_gyro.py --seconds 10 --session sessions/YYYYMMDD_HHMMSS
```

Web sunucusu icin `requirements.txt` paketleri proje `.venv` icine kurulabilir.
Her calisma `sessions/` altinda IMU ham, fusion, LiDAR scan, pose, event ve
config snapshot dosyalari olusturur. `metadata.yaml` gercek LiDAR seri/firmware
bilgisini, ESP32 protokol/IMU kimligini ve temiz kapatmada gozlenen sensor
hizlarini kaydeder. `monotonic_ns` integration icin,
wall-clock sadece klasor adi ve metadata icin kullanilir.
Replay web arayuzu, canli sensorlere hic dokunmadan kaydi ayni islemci
katmanindan gecirir; laptopta `http://PI_IP:8766/` adresinden gorulebilir.
Yerel SPI ekran Xorg shadow-buffer yenileme sorunu nedeniyle Pillow/NumPy ile
dogrudan `/dev/fb1` RGB565'e cizilir; ekran surucusu/config degistirilmez.

2026-10-07 sabit IMU testinde ~943 ornekte jiroskop bias kalibre edildi.
Yuz yukari/yuz asagi Z olcumleri yaklasik +11.28 ve -8.59 m/s2 idi; bu
yaklasik +1.35 m/s2 sifir kaymasina isaret ediyor. Tam 3 eksen accelerometer
kalibrasyonu henuz yapilmadi; bu iki poz yalnizca kismi Z duzeltmesi saglar.
Montajli sistemin onunu yukari kaldirma, sol kenarini yukari kaldirma ve
saat yonunde yaw testleri `+x/+y/+z` ham IMU eksenlerinin `base_link` ile
uyumlu oldugunu dogruladi. Yalniz Z ivme ofseti/olcegi kismi olarak
duzeltildi; web arayuzu ham ve duzeltilmis normu ayri gosterir.
NodeMCU-32S seri portu acilirken DTR/RTS aktif tutulursa ESP32 reset/boot
durumunda sessiz kalabilir. `drivers/imu_serial/port.py` hatlari port acilmadan
pasif ayarlar; tum IMU araclari ayni yolu kullanir.

**Haritalama guvenlik kapisi:** `imu.axis_frame_verified` ve
`lidar.angle_frame_verified` fiziksel testlerden sonra true yapilana kadar
occupancy map yazilmaz. Roll/pitch esikleri asilirsa da map integration durur;
ham scan gorunmeye ve kaydedilmeye devam eder.

Ilk scan matcher, onceki scan noktalarini 10 cm hucrelere indirger, IMU yaw
degisimini baslangic tahmini alir ve config ile sinirlanan x/y/yaw aramasinda
hucre komsuluk eslesme oranini maksimize eder. Dusuk skorda pose guncellenmez.
Sabit scan'lerde skor avantaji en az 0.04 degilse hareket sifir kabul edilir;
bu, sabit dururken poz suruklenmesini azaltir. Ham LiDAR yaklasik 7.4 Hz,
IMU yaklasik 94 Hz, harita yaklasik 2.5 Hz guncellenir (Pi 4 uzerinde olculdu).
Kucuk hareketler ve yatay, sabit yukseklikte tasima icin V0 algoritmasidir;
loop closure veya 3D translation saglamaz. 5 dereceyi asan egimde harita
PAUSED durumuna gecer, yataya donunce otomatik ACTIVE olur; fiziksel test edildi.

## Laptopta deneysel 3D egme taramasi

Web panelindeki `3D PIVOT SWEEP` sadece kisa, sabit IMU merkezi varsayimli
manuel egme denemeleri icindir. IMU, LiDAR merkezinden x yonunde yaklasik
+4 cm ileride ve z yonunde 4-5 cm asagidadir; `frames.yaml` nominal
`[+0.04, 0, -0.045]` m ve z araligini kaydeder. y=0 olculmus degil,
merkez hatti varsayimidir. Bu tam 3D SLAM veya XYZ konum takibi degildir.

Laptopta `3D yakalamayi baslat` ile en cok 30 saniyelik, 15.000 noktalik
tarama alin; cihaz konumunu sabit tutup yavasca egin. Baslangic/bitis IMU
zaman eslesmesi 40 ms'yi veya tek scan donusu 3 dereceyi asarsa o scan atlanir.
Varsayilan on gorus sag/sol 100'er derece (toplam 200 derece); arkanizdaki
operator 3D buluta ve PLY dosyasina alinmaz. Ham kayit ve 2D harita 360
derece kalir. Fiziksel Pi ekrani degistirilmemistir.

Pi oturum kaydi 512 MB oturum boyutunda veya 512 MB bos disk alaninda durur;
sensorler ve web paneli calismayi surdurur. `RECORD` satirini kontrol edin.

## IMU kablolama

NodeMCU-32S (klasik ESP32) ile I2C baglantisi:

| IMU modulu | NodeMCU-32S | Not |
|---|---|---|
| VCC | 3V3 | Modulun gerilim bilgisini teyit etmeden 5V/VIN kullanma |
| GND | GND | Ortak toprak |
| SDA | GPIO21 | I2C data |
| SCL | GPIO22 | I2C clock |
| AD0 | GND | Adres 0x68; 3V3 olursa 0x69 |
| NCS | 3V3 | I2C modunu secmek icin high |
| INT | baglama | Ilk surum polling kullaniyor |
| FSYNC | GND | Kullanilmiyor; floating birakma |
| EDA | baglama | Yardimci I2C, host baglantisi degil |
| ECL | baglama | Yardimci I2C, host baglantisi degil |

IMU `WHO_AM_I=0x70` ile cevap verdi. Bu kimlik TDK'nin MPU-6500 register
belgesine gore MPU-6500 (6 eksen) ile uyumludur; kartin disinda yazan 9250
etiketini dogrulamaz. Firmware 0x70, 0x71 ve 0x73 kimliklerini ayri raporlar.
AK8963 bulunmazsa magnetometre alanlari `nan` olarak gonderilir. Fusion
varsayilan olarak kapali kalir.

## Guvenlik sinirlari

- Pi ekraninin GPIO/SPI veya boot/display konfigurasyonuna dokunulmaz.
- SSH ve ag konfigurasyonu bu proje tarafindan degistirilmez.
- Sabit 2D LiDAR + IMU ile ivmeyi iki kez entegre edip sahte 3D konum uretilmez.
- IMU extrinsic translation yalniz yaklasik olculdu; y ekseni varsayimdir.
