#!/usr/bin/env bash
# HH_260428: One-time workspace setup — clones external dependencies and installs ROS deps.
#
# Run this ONCE on a new machine or after a fresh git clone.
# It will NOT overwrite existing external directories unless --update is given.
#
# Usage:
#   ./setup_camrod.sh             # clone missing externals + rosdep (safe, idempotent)
#   ./setup_camrod.sh --update    # also pull latest for each external (REWRITES working copies)
#   ./setup_camrod.sh --no-rosdep # skip rosdep (externals only)
#
# WARN: --update overwrites any local changes inside external/ directories.
#
# Fork override:
#   Set CAMROD_AGILEX_BASE before running to use your own forks of agilexrobotics repos:
#     CAMROD_AGILEX_BASE="https://github.com/your-org" ./setup_camrod.sh

set -euo pipefail

# WARN: Do NOT run this script with 'sudo bash setup_camrod.sh'.
# Internal sudo calls handle privileged operations individually.
# Running as root causes git-cloned external/ files to be owned by root,
# which breaks subsequent 'git restore' and normal user file operations.
if [[ "${EUID}" -eq 0 ]]; then
  echo "[setup_camrod] ERROR: Do not run as root (sudo bash ...). Run as normal user: ./setup_camrod.sh" >&2
  exit 1
fi

# HH_260428: Upstream URLs — override CAMROD_AGILEX_BASE for custom forks of agilexrobotics.
AGILEX_BASE="${CAMROD_AGILEX_BASE:-https://github.com/agilexrobotics}"

log() { echo "[setup_camrod] $*"; }

resolve_ws_root() {
  local probe="$1"
  while [[ "${probe}" != "/" ]]; do
    if [[ -d "${probe}/src/camrod_bringup" ]]; then echo "${probe}"; return 0; fi
    probe="$(dirname "${probe}")"
  done
  return 1
}

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS_ROOT="$(resolve_ws_root "${SCRIPT_DIR}" || resolve_ws_root "$(pwd)" || true)"
if [[ -z "${WS_ROOT}" ]]; then
  echo "[setup_camrod] ERROR: cannot find workspace root (expected <ws>/src/camrod_bringup)" >&2
  exit 1
fi
SRC_ROOT="${WS_ROOT}/src"

UPDATE=0
DO_ROSDEP=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --update)      UPDATE=1; shift ;;
    --no-rosdep)   DO_ROSDEP=0; shift ;;
    -h|--help)
      sed -n '2,20p' "${BASH_SOURCE[0]}" | grep '^#' | sed 's/^# \?//'
      exit 0 ;;
    *) echo "[setup_camrod] unknown arg: $1" >&2; exit 1 ;;
  esac
done

if [[ "${UPDATE}" -eq 1 ]]; then
  echo ""
  echo "  ╔══════════════════════════════════════════════════════════╗"
  echo "  ║  WARNING: --update will REWRITE all external/ working    ║"
  echo "  ║  copies. Any local changes in external/ will be lost.    ║"
  echo "  ╚══════════════════════════════════════════════════════════╝"
  echo ""
  read -r -p "  Continue? [y/N] " confirm
  [[ "${confirm}" =~ ^[Yy]$ ]] || { echo "Aborted."; exit 0; }
fi

# shellcheck disable=SC1091
set +u; source /opt/ros/humble/setup.bash; set -u

# HH_260428: Clone external repo if missing; update only when --update is given.
clone_ext() {
  local url="$1" ref="$2" rel="$3"
  local abs="${SRC_ROOT}/${rel}"
  mkdir -p "$(dirname "${abs}")"
  if [[ ! -d "${abs}" ]]; then
    log "clone ${url} (${ref}) -> ${rel}"
    git clone --branch "${ref}" --depth 1 "${url}" "${abs}"
    return
  fi
  if [[ "${UPDATE}" -eq 1 ]] && [[ -d "${abs}/.git" || -f "${abs}/.git" ]]; then
    log "update ${rel} (${ref})"
    git -C "${abs}" fetch --depth 1 origin "${ref}"
    git -C "${abs}" checkout -f FETCH_HEAD
  else
    log "keep existing ${rel}"
  fi
}

# HH_260522: camrod_sensing camera component needs nvjpeg.h + libnvjpeg.so.
# Some Jetson images include runtime only; install matching dev packages when needed.
has_nvjpeg_header() {
  [[ -f /usr/include/nvjpeg.h ]] || \
  [[ -f /usr/local/cuda/include/nvjpeg.h ]] || \
  [[ -f /usr/local/cuda/targets/aarch64-linux/include/nvjpeg.h ]]
}

has_nvjpeg_library() {
  ldconfig -p 2>/dev/null | grep -q "libnvjpeg\\.so"
}

apt_has_candidate() {
  local pkg="$1" cand
  cand="$(apt-cache policy "${pkg}" 2>/dev/null | awk '/Candidate:/ {print $2; exit}')"
  [[ -n "${cand}" && "${cand}" != "(none)" ]]
}

select_nvjpeg_packages() {
  local pair dev rt
  local pairs=(
    "libnvjpeg-dev-12-6 libnvjpeg-12-6"
    "libnvjpeg-dev-12-5 libnvjpeg-12-5"
    "libnvjpeg-dev-12-4 libnvjpeg-12-4"
    "libnvjpeg-dev-12-3 libnvjpeg-12-3"
    "libnvjpeg-dev libnvjpeg12"
  )
  for pair in "${pairs[@]}"; do
    dev="${pair%% *}"
    rt="${pair#* }"
    if apt_has_candidate "${dev}"; then
      if apt_has_candidate "${rt}"; then
        echo "${dev} ${rt}"
      else
        echo "${dev}"
      fi
      return 0
    fi
  done
  return 1
}

# ── System packages required before rosdep ──────────────────────────────────
# Keep critical runtime deps here so a fresh machine can launch core stacks
# even when rosdep is skipped or partially unresolved.
REQUIRED_SYS_PKGS=(
  ros-humble-nav2-graceful-controller
  ros-humble-magic-enum
  python3-fastapi
  python3-uvicorn
  libsdl2-dev        # HH_260615: camrod_voice — SDL2 오디오 백엔드
  libsdl2-mixer-dev  # HH_260615: camrod_voice — WAV 재생 (Mix_* API)
  # rosdep 으로 해결되어야 하지만 interactive sudo 없이 실패하는 패키지들을 명시 추가
  ros-humble-nav2-map-server        # nav2 맵 서버 (camrod_planning)
  ros-humble-behaviortree-cpp-v3    # Nav2 BT 네비게이터 (camrod_planning)
  ros-humble-controller-manager     # ugv_sdk 의존
  ros-humble-rviz2                  # RViz2 (camrod_bringup sim 모드)
  ros-humble-rviz-common            # RViz2 공통 라이브러리
  ros-humble-rviz-default-plugins   # RViz2 기본 플러그인
  ros-humble-rtcm-msgs              # ublox GNSS RTK (camrod_sensing)
  ros-humble-test-msgs              # 테스트 의존
  python3-serial                    # ublox, radar 시리얼 통신
  libpugixml-dev                    # Lanelet2 XML 파서 (camrod_map)
  libnanoflann-dev                  # PCL/포인트클라우드 KNN (camrod_perception)
)

# HH_260615: camrod_docking — Isaac ROS apt 저장소 자동 등록 및 런타임 패키지 설치.
# aarch64(Jetson/JetPack)에서만 동작. x86_64는 건너뛰고 skip 안내를 출력합니다.
setup_isaac_ros_apt() {
  local _arch
  _arch="$(uname -m)"
  if [[ "${_arch}" != "aarch64" && "${_arch}" != "arm64" ]]; then
    log "skip Isaac ROS apt setup (${_arch} is not aarch64)"
    log "  x86_64 빌드 시 camrod_docking 제외: colcon build --packages-skip camrod_docking"
    return 0
  fi

  if [[ ! -f /etc/apt/sources.list.d/isaac-ros.list ]]; then
    log "NVIDIA Isaac ROS apt 저장소 등록 중 (arm64)"
    sudo apt-get install -y curl gnupg lsb-release
    curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
      | gpg --dearmor \
      | sudo tee /usr/share/keyrings/isaac-ros-archive-keyring.gpg > /dev/null
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/isaac-ros-archive-keyring.gpg] \
https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" \
      | sudo tee /etc/apt/sources.list.d/isaac-ros.list > /dev/null
    sudo apt-get update
  else
    log "Isaac ROS apt 저장소 이미 등록됨"
  fi

  local _isaac_pkgs=(
    ros-humble-isaac-ros-common
    ros-humble-isaac-ros-nitros
    ros-humble-isaac-ros-apriltag
    ros-humble-isaac-ros-image-proc
    ros-humble-isaac-ros-image-pipeline
  )
  local _missing_isaac=()
  for _ipkg in "${_isaac_pkgs[@]}"; do
    dpkg -l "${_ipkg}" 2>/dev/null | grep -q "^ii" || _missing_isaac+=("${_ipkg}")
  done
  if [[ ${#_missing_isaac[@]} -gt 0 ]]; then
    log "Isaac ROS 패키지 설치 중: ${_missing_isaac[*]}"
    sudo apt-get install -y "${_missing_isaac[@]}"
  else
    log "Isaac ROS 패키지 이미 설치됨"
  fi
  unset _arch _isaac_pkgs _ipkg _missing_isaac

  # HH_260616: isaac_ros cmake export 파일이 numpy include 경로를 빌드 시점 절대경로로
  # 하드코딩한다 (/usr/local/lib/python3.10/dist-packages/numpy/core/include).
  # user-install numpy (/home/.../.local/) 가 있으면 cmake Generate 단계에서 실패하므로
  # 시스템 경로(sudo pip3)에 numpy를 보장한다.
  if [[ ! -d /usr/local/lib/python3.10/dist-packages/numpy ]]; then
    log "numpy 시스템 경로 설치 중 (isaac_ros cmake 경로 호환)"
    # 기존 user-install numpy 버전과 동일하게 고정 설치.
    # 버전 불일치 시 cv_bridge 등 C 확장 패키지에서 numpy ABI 오류가 발생할 수 있다.
    _numpy_ver="$(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null || echo '1.26.4')"
    sudo pip3 install "numpy==${_numpy_ver}"
    unset _numpy_ver
  else
    log "numpy 시스템 경로 확인됨"
  fi
}

# HH_260611: Keep nvjpeg dependency handling Jetson-only so x86_64 sensing/GNSS
# builds do not warn about unavailable CUDA/JetPack runtime libraries.
if [[ -d "${SRC_ROOT}/camrod_sensing" ]]; then
  _arch="$(uname -m)"
  if [[ "${_arch}" == "aarch64" || "${_arch}" == "arm64" ]]; then
    if ! has_nvjpeg_header || ! has_nvjpeg_library; then
      # HH_260616: apt-cache는 stale 상태일 수 있으므로 nvjpeg 탐색 전 갱신.
      # 갱신 없이 apt_has_candidate가 실패하면 libnvjpeg-dev가 설치되지 않아
      # nvjpeg.h 누락으로 camrod_sensing 빌드가 실패한다.
      log "apt-get update (nvjpeg 탐색 전 패키지 목록 갱신)"
      sudo apt-get update -q
      if _nvjpeg_pkgs="$(select_nvjpeg_packages)"; then
        # shellcheck disable=SC2206
        _nvjpeg_arr=(${_nvjpeg_pkgs})
        REQUIRED_SYS_PKGS+=("${_nvjpeg_arr[@]}")
        log "nvjpeg missing; queued packages: ${_nvjpeg_pkgs}"
      else
        log "WARN: nvjpeg missing but no apt candidate package found (header/lib may stay unresolved)"
      fi
    fi
  fi
  unset _arch _nvjpeg_pkgs _nvjpeg_arr
fi

_missing=()
for _pkg in "${REQUIRED_SYS_PKGS[@]}"; do
  dpkg -l "${_pkg}" 2>/dev/null | grep -q "^ii" || _missing+=("${_pkg}")
done
if [[ ${#_missing[@]} -gt 0 ]]; then
  log "install system packages: ${_missing[*]}"
  sudo apt-get install -y "${_missing[@]}" || \
    log "WARN: apt-get install failed — build may fail if packages are absent"
fi
unset _pkg _missing

# HH_260615: Isaac ROS apt 등록 — camrod_docking 모듈 존재 시에만 실행
if [[ -d "${SRC_ROOT}/camrod_docking" ]]; then
  setup_isaac_ros_apt
  # Install non-Isaac docking dependencies via apt (available on all architectures)
  _docking_apt_pkgs=(
    ros-humble-image-pipeline
    ros-humble-negotiated
    ros-humble-opennav-docking
  )
  _docking_missing=()
  for _dpkg in "${_docking_apt_pkgs[@]}"; do
    dpkg -l "${_dpkg}" 2>/dev/null | grep -q "^ii" || _docking_missing+=("${_dpkg}")
  done
  if [[ ${#_docking_missing[@]} -gt 0 ]]; then
    log "install docking packages: ${_docking_missing[*]}"
    sudo apt-get install -y "${_docking_missing[@]}"
  else
    log "docking packages already installed"
  fi
  unset _docking_apt_pkgs _docking_missing _dpkg
fi

if [[ -d "${SRC_ROOT}/camrod_sensing" ]]; then
  # HH_260611: Report missing nvjpeg only on Jetson targets where the camera
  # pipeline actually depends on NVIDIA's nvjpeg runtime.
  _arch="$(uname -m)"
  if [[ "${_arch}" == "aarch64" || "${_arch}" == "arm64" ]]; then
    if ! has_nvjpeg_header; then
      log "WARN: nvjpeg header (nvjpeg.h) is still missing after setup"
    fi
    if ! has_nvjpeg_library; then
      log "WARN: nvjpeg library (libnvjpeg.so) is still missing after setup"
    fi
  fi
  unset _arch
fi

# ── External repositories ────────────────────────────────────────────────────
log "bootstrapping external repositories (workspace: ${WS_ROOT})"

clone_ext "https://github.com/KumarRobotics/ublox.git"                      "master"       "camrod_sensing/external/ublox"
clone_ext "https://github.com/cra-ros-pkg/robot_localization.git"            "humble-devel"  "camrod_localization/external/robot_localization"
clone_ext "https://github.com/fzi-forschungszentrum-informatik/Lanelet2.git" "master"       "camrod_map/external/lanelet2"
clone_ext "https://github.com/ros-perception/vision_msgs.git"                "ros2"         "camrod_common/external/vision_msgs"
clone_ext "https://github.com/ros-perception/vision_opencv.git"              "3.2.1"        "camrod_common/external/vision_opencv"
clone_ext "https://github.com/ros-perception/perception_pcl.git"             "humble"       "camrod_sensing/external/perception_pcl"
clone_ext "https://github.com/ros-perception/laser_geometry.git"             "ros2"         "camrod_planning/external/laser_geometry"
# HH_260428: Agilex platform drivers — set CAMROD_AGILEX_BASE to use custom forks.
clone_ext "${AGILEX_BASE}/ugv_sdk.git"                                        "main"         "camrod_platform/external/ugv_sdk"
clone_ext "${AGILEX_BASE}/ranger_ros2.git"                                    "humble"       "camrod_platform/external/ranger_ros2"
# HH_260528: Replaced camrod_parking external repos with camrod_docking equivalents.
# camrod_docking uses Isaac ROS AprilTag (GPU-accelerated) instead of cpu-based apriltag_ros.
# opencv4_vendor/yaml_cpp_vendor are custom forks embedded in the CAMROD repo itself
# (camrod_docking/external/) and do not need separate clone_ext calls.
# opennav_docking is cloned below (open-navigation fork, humble branch).
# camrod_docking externals installed via apt (see setup_isaac_ros_apt above):
#   isaac_ros_common, isaac_ros_nitros, isaac_ros_apriltag, isaac_ros_image_pipeline (apt)
#   image_pipeline, negotiated, opennav_docking (apt)

# ── VIO bridge SDK installers (disable/vio_bridge — not built by default) ────
# HH_260428: These large SDK binaries are NOT stored in git. Download manually
# only if you intend to enable the vio_bridge package.
#
#   ZED SDK (Ubuntu 22, CUDA 12.x):
#     https://www.stereolabs.com/developers/release
#     -> place .run file in disable/vio_bridge/sdk_installers/ and run it
#
#   Orbbec SDK (amd64 / arm64):
#     https://github.com/orbbec/OrbbecSDK/releases
#     -> place .deb file in disable/vio_bridge/sdk_installers/ and run:
#        sudo dpkg -i OrbbecSDK_*.deb

# ── rosdep ───────────────────────────────────────────────────────────────────
if [[ "${DO_ROSDEP}" -eq 1 ]]; then
  # init: 이미 초기화된 경우 에러를 무시
  if ! rosdep init 2>/dev/null; then
    log "rosdep already initialized — skipping init"
  fi
  log "rosdep update"
  rosdep update || log "WARN: rosdep update failed — continuing"

  log "rosdep install"
  mapfile -t _ROSDEP_PATHS < <(
    find "${SRC_ROOT}" -name package.xml -type f \
      | grep -Ev '/vendor/.*/extract/' \
      | xargs -r -n1 dirname \
      | while read -r p; do readlink -f "${p}"; done \
      | sort -u
  )
  if ! rosdep install --from-paths "${_ROSDEP_PATHS[@]}" --ignore-src -r -y; then
    log "WARN: rosdep failed (no sudo or unresolved keys) — continuing"
  fi
fi

log "setup complete. now run: ./colcon_build.sh"
