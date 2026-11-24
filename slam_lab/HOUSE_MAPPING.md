# Ev Icindeki Haritalama — V0

## Bu donanim ne uretebilir?

RPLIDAR A1 tek bir tarama duzlemi olcer; yazilim 2D occupancy grid ve kisa
menzilli 2D poz tahmini olusturur. Laptop panelinde, IMU merkezi yaklasik sabit
kalirken kisa bir manuel egme taramasindan deneysel 3D nokta bulutu alinabilir.
ESP32'deki IMU roll, pitch ve goreli yaw saglar, ancak guvenilir XYZ konumu
saglamaz. Bu nedenle tum evin gercek 3D geometrisi veya katlar arasi model
bu duzenekle cikarilamaz.
Haritalama kodu 5 dereceyi asan egimde durur. Bugunku hareket denemesinde
yaklasik 20–30 cm'lik elle hareket ~14 cm ve yana sapmayla tahmin edildi; bu
sonuc metrik hassasiyet iddiasi icin yeterli degildir.

## Baslatma

Laptopun ana panelinde 3D tarama solda buyuk; on gorus ve 2D harita sagda
ust-alt duzenindedir. Ilk konumda LiDAR'i duz tutup `3D yakalamayi baslat`in.
Durakta yavasca egerek laptopu, masayi ve asagi yone acik zemini tarayin.
Laptop etrafinda yeni duraga gececekseniz `Konum degistir`e basin, cihaz
duzken yavasca yer degistirin, 2D harita ACTIVE olunca `Yeni konumda surdur`u
secin. Sonunda `Durdur` ve `PLY indir` kullanin. 30 saniye siniri yoktur;
60.000 nokta ust siniri vardir. Duraklarin yaklasik XY konumu 2D haritadan
gelir; egikken tasimak modeli bozar. Zemin icin LiDAR merkezinin yerden
yuksekligini cm olarak girin; zemin gosterimi bu olcume baglidir.
15 cm altindaki 2D poz farki sabit durak kaymasi sayilir.

1. LiDAR ve IMU'yu sabit, yatay bir tasiyiciya baglayin. LiDAR'in donen ust
   parcasina el, kablo veya gevsek esya yaklastirmayin. Kablolari cekilmeyecek
   bicimde sabitleyin.
2. Raspberry Pi'yi acin; Pi ile laptop ayni ev aginda olsun. Pi'nin bilinen
   adresi `192.168.1.76`.
3. Windows PowerShell'den Pi'ye baglanin:

   ```powershell
   ssh betul_enis@192.168.1.76
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
5. Laptopta tarayiciyi acin: `http://192.168.1.76:8765/`. Pi ekraninda
   `SLAM LAB V0` paneli gorunmelidir. `MAPPING ACTIVE`, IMU ve LiDAR hizi
   gorunmeden hareket etmeyin.

## Ev icinde toplama

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

Laptop adresi `http://192.168.1.76:8766/` olur. Bitirirken replay terminalinde
Ctrl+C kullanin.

## Masa ustunde kisa 3D egme denemesi

1. Pi'ye adaptorden guc verirken kablolari cekilmeyecek sekilde duzenleyin.
   LiDAR + IMU montajini masa uzerinde neredeyse ayni noktada tutun; kaydirma
   3D nokta bulutunu bozabilir.
2. Laptop panelinde `3D TARAMA · DENEYSEL` alaninda varsayilan `ON 200° (±100°)`
   gorusunu kontrol edin. Cihazin arkasi 3D buluta/PLY dosyasina girmez.
   Ham LiDAR kaydi ve 2D harita ise tam 360° kalir.
3. `3D yakalamayi baslat` dugmesine basin. Cihazi yavasca once az onde,
   sonra yanlara egip her konumda kisa sure sabit tutun. 2D harita 5° ustunde
   PAUSED gosterebilir; 3D egme taramasi devam eder. Tek LiDAR devri icinde
   3°'ten fazla donus varsa o tarama 3D buluta alinmaz.
4. Istediginiz zaman `Durdur` ve `PLY indir` dugmelerini kullanin. Otomatik
   30 saniye siniri yoktur; tarayici bellegi icin 60.000 nokta ust siniri
   vardir. Dosya laptopa iner. Fareyle suruklemek 3D gorunumu dondurur,
   tekerlek yakinlastirir. `Temizle` yeni bir deneme baslatir.

IMU ofseti yaklasik x=+4 cm, z=-4–5 cm ve y=0 merkez hatti varsayimi olarak
uygulanir. Model, IMU merkezi sabitken LiDAR'in onun etrafinda dondugunu
varsayar. Durak icinde masada kaydirma veya elde tasima sonucunda nokta bulutu
hatali olur. Duraklar arasinda ise yalnizca cihaz duzken `Konum degistir` /
`Yeni konumda surdur` adimlariyla 2D poz farki yaklasik uygulanir; bu gercek
6-DoF konum takibi veya ICP ile 3D bulut hizalamasi degildir.

## Gercek 3D icin gerekenler

Bu deneysel PLY, sabit merkezli taramalar ve varsa 2D poza dayali yaklasik
durak birlestirmesiyle olusan buluttur. Odalar
arasi guvenilir 3D harita icin 3D derinlik sensoru veya 3D LiDAR ile 6-DoF
konum takibi/odometri gerekir. Mevcut 2D harita egimde durur ve 3D gorunum
cihazin elde tasinirkenki surekli otelemesini cozumlemez.
