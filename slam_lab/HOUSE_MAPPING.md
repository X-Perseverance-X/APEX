# Ev Icindeki Haritalama — V0

## Bu donanim ne uretebilir?

RPLIDAR A1 yatay bir tarama duzlemi olcer; mevcut yazilim bu taramalardan 2D
occupancy grid ve kisa menzilli 2D poz tahmini olusturur. ESP32'deki IMU roll,
pitch ve goreli yaw saglar, ancak guvenilir XYZ konumu saglamaz. Bu nedenle bu
kurulumla evin gercek 3D geometrisini veya katlar arasi modeli cikarmayin.
Haritalama kodu 5 dereceyi asan egimde durur. Bugunku hareket denemesinde
yaklasik 20–30 cm'lik elle hareket ~14 cm ve yana sapmayla tahmin edildi; bu
sonuc metrik hassasiyet iddiasi icin yeterli degildir.

## Baslatma

1. LiDAR ve IMU'yu sabit, yatay bir tasiyiciya baglayin. LiDAR'in donen ust
   parcasina el, kablo veya gevsek esya yaklastirmayin. Kablolari cekilmeyecek
   bicimde sabitleyin.
2. Raspberry Pi'yi acin; Pi ile laptop ayni ev aginda olsun. Pi'nin LAN
   adresini Pi'de `hostname -I` komutuyla ogrenin ve asagidaki `PI_IP` yerine
   yazin.
3. Windows PowerShell'den Pi'ye baglanin:

   ```powershell
   ssh PI_USER@PI_IP
   ```

4. SSH oturumunda mapper'i baslatin:

   ```bash
   cd ~/slam_lab
   nohup tools/run_mapper > .runtime/mapper.log 2>&1 < /dev/null &
   sleep 3
   tail -n 5 .runtime/mapper.log
   tools/lidar_motor status
   ```

   Logda `SESSION=...` gorunmeli; motor durumu `STREAMING` olmali.
5. Laptopta `http://PI_IP:8765/` adresini acin. Pi ekraninda
   `SLAM LAB V0` paneli gorunmelidir. `MAPPING ACTIVE`, IMU ve LiDAR hizi
   gorunmeden hareket etmeyin.

## Ev icinde toplama

Ilk deneysel 3D denemede evde dolasmayin: cihaz masa ustunde, ayni noktada
kalirken laptop web panelinde `3D yakalamayi baslat` deyin ve yavasca egin.
Varsayilan on gorus 200 derecedir (sag/sol 100 derece); arka taraftaki kisi
3D buluta girmez. 30 saniye dolmadan `Durdur` ve `PLY indir` kullanilabilir.
Bu nokta bulutu tam 3D harita degildir; Pi'nin fiziksel ekrani henuz ayni
gorunumu korur. `RECORD` satiri disk siniri nedeniyle durabilir.

1. Sistemi baslangic noktasinda duz ve sabit tutun; haritanin ilk scan'lerini
   olusturmasi icin birkac saniye bekleyin.
2. Once tek, engelsiz bir odada deneyin. Aygiti zemine yakin ama darbe
   almayacak sabit bir yukseklikte tasiyin. Yavas ilerleyin; ani donuslerden,
   egimden, yukari kaldirmaktan ve kabloyu germekten kacinin.
3. Ekran/laptopta canli scan, occupancy grid ve `ACTIVE` durumunu gozleyin.
   Eğim 5 dereceyi asarsa harita entegrasyonu durur; sistemi yataylastirinca
   otomatik devam eder. Bu esnada LiDAR donmeye devam eder.
4. Bir oda icin kisa bir tur yapip baslangic bolgesine geri donun. Bu surumde
   loop closure yoktur; eski duvarlar yeni konumla hizalanmazsa harita kayabilir.
   Ilk denemede tum evi veya merdivenleri taramayin.
5. Hareketi bitirince sistemi yatay bir yere koyup scan'in sabitlenmesini
   bekleyin. Tum kaydi kapatmak icin Pi'de:

   ```bash
   kill -TERM "$(cat ~/slam_lab/.runtime/mapper.pid)"
   ```

   Temiz kapanis oturumu `~/slam_lab/sessions/` altina yazar ve LiDAR'i
   `OFF_HELD` durumunda durdurur. Sadece motoru durdurup panel/IMU'yu acik
   birakmak icin `~/slam_lab/tools/lidar_motor off`; yeniden baslatmak icin
   `~/slam_lab/tools/lidar_motor on` kullanin.

## Kaydi inceleme

Oturum adini Pi'de bulun:

```bash
cd ~/slam_lab
ls -dt sessions/* | head -5
```

`metadata.yaml`, `imu_raw.csv`, `imu_fused.csv`, `lidar_scans.jsonl`, `pose.csv`
ve `events.log` dosyalari her oturumda bulunur. Eski bir oturumu sensorlere
baglanmadan web uzerinden tekrar oynatmak icin:

```bash
.venv/bin/python tools/replay_session.py sessions/OTURUM_ADI --serve --port 8766 --speed 4
```

Laptop adresi `http://PI_IP:8766/` olur. Bitirirken replay terminalinde
Ctrl+C kullanin.

## Gercek 3D icin gerekenler

Bu V0 akisi ile yalnizca 2D oda izi ve yonelim onizlemesi alinabilir. Guvenilir
3D harita icin 3D derinlik sensoru veya 3D LiDAR ve 6-DoF konum takibi/odometri
gerekir. Elle egme yalniz sabit IMU merkezi varsayimiyla yaklasik bir 3D
nokta bulutu verir; egim sirasinda 2D harita guvenlik nedeniyle durur.
