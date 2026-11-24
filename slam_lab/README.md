# SLAM Lab V0

ROS kullanmadan RPLIDAR A1 ve ESP32 uzerinden gelen IMU verisini dogrulayan,
kaydeden ve sinirli 2D haritalama yapan V0 prototipi.

Koordinat sistemi sag ellidir: `x ileri`, `y sol`, `z yukari`.
`base_link` baslangicta `lidar_link` ile aynidir.

## Dogrulanan donanim (2026-10-07)

- Raspberry Pi 4 Model B Rev 1.5, 4 GB
- Debian 13 (trixie), aarch64
- Yerel ekran: 480x320, `fb_ili9486`
- RPLIDAR seri: `EAE9ECF0C3E09ED5A0EA98F379314117`
- RPLIDAR firmware: 1.29, hardware rev: 7, health: OK
- Canli scan: resmi SLAMTEC `simple_grabber` ile dogrulandi
- Fiziksel on/sol hedef testi: on hedef ham 0 derece, sol hedef ham ~270 derece;
  `lidar_link` icin aci isareti -1 olarak dogrulandi

Iki USB aygit da `10c4:ea60`, urun `CP2102` ve seri `0001` bildiriyor.
Bu nedenle `/dev/serial/by-id` tek basina guvenli degildir; link son takilan
aygita kayabilir. Mevcut fiziksel USB yerlesiminde:

- LiDAR: `...usb-0:1.2:1.0-port0`
- ESP32: `...usb-0:1.1:1.0-port0`

Fiziksel USB soketleri degistirilirse `tools/detect_hardware.py` ve protokol
probe'u yeniden calistirilmalidir.

## LiDAR motor kontrolu

```bash
cd ~/slam_lab/drivers/lidar
make
./lidar_control --port /dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0 status
./lidar_control --port /dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0 off
./lidar_control --port /dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0 on
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
# laptop: http://192.168.1.76:8765/
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

Pi oturum kaydi varsayilan olarak 512 MB oturum buyuklugunde veya diskte 512 MB
bos alan kalinca durur; canli sensor ve tarayici akisi calismaya devam eder.
Web panelindeki `RECORD` satiri kaydin durumunu gosterir.

Web panelindeki buyuk 3D alan, kullanici `3D yakalamayi baslat` dediginde
ilk duragin sabit IMU merkezli egme taramasini biriktirir. Tarama elle
durdurulur; 30 saniyelik sinir kaldirildi. Tarayici belleği icin 60.000
nokta siniri ve 2 cm voxel azaltma vardir. LiDAR taramalarinin baslangic
ve bitis saatindeki IMU quaternion'lari
host monotonic zamanla eslestirilir; ortadaki yonelim kullanilir. Eslesme 40 ms
disindaysa veya 134 ms civarindaki tek taramada donus 3 dereceyi asarsa o scan
3D buluta alinmaz. Pi sadece kucuk scan/yönelim metadata'si gonderir; nokta
donusumu, 5 cm voxel azaltma, cizim ve PLY indirme laptop tarayicisinda olur.

IMU, LiDAR merkezinden x yonunde yaklasik +4 cm ileride ve z yonunde 4–5 cm
asagidadir. `frames.yaml` icinde x=+0.04 m, z=-0.045 m nominali ve z araligi
acikca kayitlidir. y=0 yalniz merkez hatti varsayimidir; olculmemistir.
Sabit IMU merkezi kabulunde LiDAR noktasinin ilk scan frame'indeki konumu:
`p_0 = R(q_0)^T R(q_t) (p_lidar - t_base_imu) + t_base_imu`.
Bu yaklasim el kaymasini, Z otelemesini ve LiDAR'in tek donus icindeki hareket
bozulmasini tamamen duzeltemez. Laptop etrafinda baska bir konuma gecmek icin
`Konum degistir` ile yakalamayi duraklatin, LiDAR'i duz tutarak tasiyin,
2D harita `ACTIVE` ve score en az 0.5 olunca `Yeni konumda surdur`u secin.
Duraklar arasindaki XY kayma mevcut 2D scan matcher pozundan gelir; sabit
yukseklik ve iyi scan ortusmesi gerektirir. Sabit konumdaki birkac cm 2D poz
kaymasini yeni durak saymamak icin 15 cm alti fark ayni durak kabul edilir.
Egiterek tasirken XYZ takip yoktur.

Yeri taramak icin durakta IMU merkezini sabit tutup LiDAR'i yavasca asagi
egin. Gorunen noktalarin Z degeri ilk LiDAR merkezine goredir. LiDAR merkezinin
yerden yuksekligi cm olarak girilirse turuncu yaklasik zemin duzlemi ve bu
duzleme yakin nokta sayisi gosterilir. Masa tablasinin veya esyanin engelledigi
zemin gorulmez; alt tarafi acik bir konum secin.

Varsayilan gorus x ileri eksenine gore sol/sag 100'er derece, toplam 200
derecedir. Laptopun on tarama cizimi, 3D bulutu ve PLY buna uyar; ham LiDAR
kaydi ve 2D occupancy grid 360 derece kalir. Yakalama 60.000
noktada durur. Pi'nin fiziksel ekran UI'si degistirilmemistir. Bu bir yaklasik
3D nokta bulutudur; tam 6-DoF 3D SLAM veya tum evin 3D haritasi degildir.

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
- IMU/LiDAR otelemesi yaklasik ve y ekseni henuz olculmedi; kesin 3D kayit icin
  montaj olcumu ile hareket referansi gerekir.
