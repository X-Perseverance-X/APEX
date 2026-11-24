# Ubuntu 22.04 WSL2 + ROS 2 Humble + Gazebo Fortress SIL

Bu makinede doğrulanan dağıtım `Ubuntu-22.04` / WSL2'dir. Ubuntu 24.04 kurma;
22.04 için resmi önerilen eşleşme ROS 2 Humble + Gazebo Fortress'tır.

## 1. Windows tarafında dağıtımı doğrula

PowerShell:

```powershell
wsl --list --verbose
wsl -d Ubuntu-22.04
```

Ubuntu terminalinde şu ikisi `22.04` ve `jammy` göstermeli:

```bash
. /etc/os-release
echo "$VERSION_ID $VERSION_CODENAME"
```

## 2. Projeye WSL içinden git

```bash
cd /mnt/c/Users/ASUSROG/Desktop/Proje/main_v17/APEX_META1_RADIO_SIL
```

## 3. ROS/Gazebo bağımlılıklarını kur

Bu adım internet ve `sudo` parolası ister. Bir kez çalıştırılır:

```bash
chmod +x tools/setup_ros2_wsl.sh
./tools/setup_ros2_wsl.sh
```

Script Ubuntu sürümünü kontrol eder; ROS 2 Humble, Gazebo Fortress köprüsü,
`gz_ros2_control`, controller paketleri ve geliştirme araçlarını kurup workspace'i
derler. Eski `/etc/apt/sources.list.d/ros2.list` ile yeni `ros2-apt-source`
Deb822 kaydı birlikte bulunursa oluşan `Conflicting values ... Signed-By`
hatasını, eski kaydı silmeden `.disabled` olarak yedekleyerek otomatik giderir.

## 4. SIL'i başlat

Yeni Ubuntu terminali:

```bash
cd /mnt/c/Users/ASUSROG/Desktop/Proje/main_v17/APEX_META1_RADIO_SIL/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch apex_gazebo sil.launch.py
```

WSL'de varsayılan `software_rendering:=true` kullanılır. Bu, Fortress OGRE2'nin
WSL D3D12 sürücüsündeki `GL3PlusTextureGpu::copyTo` çökmesini önler. İleride
güncel WSL GPU sürücüsüyle donanım hızlandırmayı denemek için:

```bash
ros2 launch apex_gazebo sil.launch.py software_rendering:=false
```

Windows 11 + WSLg varsa Gazebo penceresi doğrudan açılır. GUI açılmazsa önce:

```bash
echo "$WAYLAND_DISPLAY"
ign gazebo --versions
```

## 5. Doğrulama

Önce otomatik, GUI'siz tam-stack smoke testi:

```bash
cd /mnt/c/Users/ASUSROG/Desktop/Proje/main_v17/APEX_META1_RADIO_SIL
chmod +x tools/smoke_ros2_sil.sh
./tools/smoke_ros2_sil.sh
```

Manuel doğrulama için ikinci Ubuntu terminali:

```bash
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/ASUSROG/Desktop/Proje/main_v17/APEX_META1_RADIO_SIL/ros2_ws
source install/setup.bash
ros2 topic list
ros2 topic echo /joint_states --once
ros2 topic echo /apex/imu --once
ros2 topic echo /apex/tof_scan --once
ros2 topic echo /apex/lidar_scan --once
ros2 topic echo /apex/gps --once
ros2 topic hz /apex/camera
ros2 control list_controllers
```

Beklenen model sözleşmesi:

- Gövde çerçevesi `+X ileri, +Y sol, +Z yukarı`.
- Hip eksenleri firmware ile aynıdır: X `+120/0/-120 mm`; sağ Y
  `-80/-90/-80 mm`, sol Y `+80/+90/+80 mm`.
- Nötr coxa iki tarafta dışarı bakar.
- Firmware stand hedefi `Y=150 mm, Z=-100 mm`; fiziksel seri-eklem karşılığı
  yaklaşık femur `+41.48°`, tibia `-130.46°`'dir.
- Simülasyonda gövde merkezi `Z=100 mm` spawn edilir ve ayaklar zemine gelir.

## 6. Doğruluk sınırı

Link uzunlukları, hip noktaları, servo yönleri ve stand geometrisi v52/v56
firmware'den doğrulanmıştır. Gövde dış kabuğunun `430×260×80 mm` ölçüsü, link
kütleleri/ataletleri, ayak sürtünmesi, servo tork-hız eğrisi ve kol montajı CAD
veya fiziksel ölçüm gelene kadar provizyondur. Bunlar nihai dinamik performans
tahmini olarak kullanılmamalıdır.
