#!/usr/bin/env bash
# =============================================================
# VTOL 워크스페이스 원클릭 셋업 스크립트
# 대상 OS: Ubuntu 22.04 (ROS2 Humble)
# 사용법:  bash setup.sh
# =============================================================
set -euo pipefail

VTOL_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="humble"

info()  { echo -e "\e[32m[INFO]\e[0m  $*"; }
warn()  { echo -e "\e[33m[WARN]\e[0m  $*"; }
error() { echo -e "\e[31m[ERROR]\e[0m $*" >&2; exit 1; }

# ── 0. root 여부 확인 ─────────────────────────────────────────
if [[ $EUID -eq 0 ]]; then
  SUDO=""
else
  SUDO="sudo"
fi

# ── 1. 시스템 패키지 업데이트 ─────────────────────────────────
info "시스템 패키지 업데이트 중..."
$SUDO apt-get update -qq

# ── 2. ROS2 Humble 설치 ───────────────────────────────────────
if ! command -v ros2 &>/dev/null; then
  info "ROS2 ${ROS_DISTRO} 설치 중..."

  $SUDO apt-get install -y -qq locales curl gnupg2 lsb-release software-properties-common
  $SUDO locale-gen en_US en_US.UTF-8
  $SUDO update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  $SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

  echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
$(lsb_release -cs) main" \
    | $SUDO tee /etc/apt/sources.list.d/ros2.list > /dev/null

  $SUDO apt-get update -qq
  $SUDO apt-get install -y -qq ros-${ROS_DISTRO}-ros-base
  info "ROS2 ${ROS_DISTRO} 설치 완료"
else
  info "ROS2 이미 설치됨 → 건너뜀"
fi

# ── 3. ROS2 추가 패키지 ───────────────────────────────────────
info "ROS2 추가 패키지 설치 중..."
$SUDO apt-get install -y -qq \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-serial \
  ros-${ROS_DISTRO}-foxglove-bridge \
  ros-${ROS_DISTRO}-vision-msgs \
  ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-cv-bridge

# ── 4. Python 패키지 ──────────────────────────────────────────
info "Python 패키지 설치 중..."
pip3 install -q \
  opencv-python \
  ultralytics \
  numpy \
  pyserial

# ── 5. rosdep 초기화 ──────────────────────────────────────────
info "rosdep 초기화 중..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  $SUDO rosdep init
fi
rosdep update --rosdistro ${ROS_DISTRO}

# ── 6. 워크스페이스 의존성 해소 ───────────────────────────────
info "rosdep install 실행 중..."
cd "${VTOL_WS}"
# shellcheck disable=SC1091
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

# ── 7. colcon 빌드 ────────────────────────────────────────────
info "colcon build 실행 중..."
colcon build --symlink-install

# ── 8. bashrc 소싱 등록 ───────────────────────────────────────
BASHRC="${HOME}/.bashrc"
ROS_SOURCE="source /opt/ros/${ROS_DISTRO}/setup.bash"
WS_SOURCE="source ${VTOL_WS}/install/setup.bash 2>/dev/null || true"

grep -qxF "${ROS_SOURCE}" "${BASHRC}" || echo "${ROS_SOURCE}" >> "${BASHRC}"
grep -qxF "${WS_SOURCE}"  "${BASHRC}" || echo "${WS_SOURCE}"  >> "${BASHRC}"

# ── 완료 ──────────────────────────────────────────────────────
echo ""
echo "══════════════════════════════════════════"
echo " 셋업 완료!"
echo "══════════════════════════════════════════"
echo " 새 터미널을 열거나 아래 명령어를 실행하세요:"
echo "   source ~/.bashrc"
echo ""
echo " 빌드:   colcon build --symlink-install"
echo " 테스트: colcon test"
echo "══════════════════════════════════════════"
