#!/usr/bin/env bash
# Sistem bağımlılıklarını kurar: colcon/rosdep araçları + rosdep ile src/ altındaki tüm
# package.xml bağımlılıkları (Nav2, SLAM Toolbox, robot_localization, ros_gz, gz_ros2_control,
# Gazebo Harmonic, python3-* paketleri).
#
# Kullanım (workspace kökünden):
#   scripts/install_deps.sh            # kur (sudo ister)
#   scripts/install_deps.sh --check    # yalnızca denetle, hiçbir şey kurmaz (sudo gerekmez)
#
# Ön koşul: Ubuntu 24.04 + ROS 2 Jazzy (/opt/ros/jazzy). ROS 2'nin kendisini kurmaz:
#   https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
#
# Yan etkiler (kurulum modunda):
#   - sudo apt-get update ve apt-get install (sistem geneli paketler)
#   - rosdep ilk kez kullanılıyorsa: sudo rosdep init → /etc/ros/rosdep/sources.list.d/20-default.list
#   - rosdep update → ~/.ros/rosdep/ önbelleği
set -euo pipefail

ROS_DISTRO_EXPECTED=jazzy
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE=install
[[ "${1:-}" == "--check" ]] && MODE=check

log() { printf '[install_deps] %s\n' "$*"; }

if [[ ! -f /opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash ]]; then
  log "HATA: /opt/ros/${ROS_DISTRO_EXPECTED} bulunamadı. Önce ROS 2 Jazzy kurulmalı."
  exit 1
fi
. /etc/os-release
if [[ "${VERSION_ID:-}" != "24.04" ]]; then
  log "UYARI: Ubuntu ${VERSION_ID:-?} algılandı; proje 24.04 (Noble) ile denendi."
fi
# shellcheck disable=SC1090
set +u; source /opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash; set -u

if [[ "$MODE" == install ]]; then
  log "apt araçları kuruluyor (sudo)"
  sudo apt-get update
  sudo apt-get install -y python3-rosdep python3-colcon-common-extensions python3-venv python3-pip
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "rosdep init (sudo)"
    sudo rosdep init
  fi
  rosdep update --rosdistro "${ROS_DISTRO_EXPECTED}"
  log "rosdep install: ${WS_ROOT}/src"
  rosdep install --from-paths "${WS_ROOT}/src" --ignore-src --rosdistro "${ROS_DISTRO_EXPECTED}" -y -r
else
  for t in rosdep colcon; do
    command -v "$t" >/dev/null || { log "EKSİK: $t"; exit 1; }
  done
  log "rosdep check: ${WS_ROOT}/src"
  rosdep check --from-paths "${WS_ROOT}/src" --ignore-src --rosdistro "${ROS_DISTRO_EXPECTED}"
fi
log "tamam. Sonraki adım: cd ${WS_ROOT} && colcon build && source install/setup.bash"
