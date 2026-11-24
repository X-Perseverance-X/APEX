#!/usr/bin/env bash
set -euo pipefail

if ! grep -q 'VERSION_ID="22.04"' /etc/os-release; then
  echo "Bu script Ubuntu 22.04 Jammy bekliyor." >&2
  exit 1
fi

# ros2-apt-source, ROS deposunu Deb822 .sources biçiminde ve gömülü anahtarla
# yönetir. Daha eski kurulumlardan kalan ros2.list aynı depoyu ayrı Signed-By
# ile tanımlarsa apt tamamen durur. Yeni kaynak mevcutsa eski kaydı silmeden
# devre dışı bırak.
disable_legacy_ros2_source() {
  local legacy="/etc/apt/sources.list.d/ros2.list"
  local managed="/etc/apt/sources.list.d/ros2.sources"
  if [[ -e "$managed" && -f "$legacy" ]] &&
     grep -q 'packages\.ros\.org/ros2/ubuntu' "$legacy"; then
    echo "Eski ROS apt kaydı devre dışı bırakılıyor: $legacy"
    sudo mv -f "$legacy" "${legacy}.disabled"
  fi
}

disable_legacy_ros2_source

sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe

if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
  ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F 'tag_name' | awk -F'"' '{print $4}')"
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi
disable_legacy_ros2_source
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-dev-tools

grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
# ROS Humble setup files reference optional variables before defining them.
# This is valid in normal interactive shells but conflicts with this script's
# `set -u`; disable nounset only for the environment import.
set +u
source /opt/ros/humble/setup.bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT/ros2_ws"
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

echo "ROS 2/Gazebo SIL hazır. Yeni terminalde:"
echo "  cd $ROOT/ros2_ws"
echo "  source /opt/ros/humble/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 launch apex_gazebo sil.launch.py"
