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
)

# HH_260528: camrod_docking depends on Isaac ROS packages (GPU-accelerated AprilTag).
# These are NOT available via apt on standard Ubuntu; they require the NVIDIA Isaac ROS
# apt repository to be configured first.  On a fresh Jetson / NVIDIA machine:
#
#   Step 1 — Add Isaac ROS apt repo (one-time):
#     sudo apt-get install -y curl
#     curl -sSL https://isaac.download.nvidia.com/isaac-ros/repos.key | \
#       sudo apt-key add -
#     echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.2" | \
#       sudo tee /etc/apt/sources.list.d/isaac-ros.list
#     sudo apt-get update
#
#   Step 2 — Install Isaac ROS runtime packages:
#     sudo apt-get install -y \
#       ros-humble-isaac-ros-common \
#       ros-humble-isaac-ros-nitros \
#       ros-humble-isaac-ros-apriltag \
#       ros-humble-isaac-ros-image-pipeline
#
#   If the apt repo is not available (e.g. x86_64 dev machine without JetPack),
#   colcon will still build all other packages; only camrod_docking will fail to
#   find isaac_ros_apriltag_interfaces.  In that case build with:
#     ./colcon_build.sh --packages-skip camrod_docking

# HH_260522: Add nvjpeg packages only when camrod_sensing exists and nvjpeg is missing.
if [[ -d "${SRC_ROOT}/camrod_sensing" ]]; then
  _arch="$(uname -m)"
  if [[ "${_arch}" == "aarch64" || "${_arch}" == "arm64" ]]; then
    if ! has_nvjpeg_header || ! has_nvjpeg_library; then
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

if [[ -d "${SRC_ROOT}/camrod_sensing" ]]; then
  if ! has_nvjpeg_header; then
    log "WARN: nvjpeg header (nvjpeg.h) is still missing after setup"
  fi
  if ! has_nvjpeg_library; then
    log "WARN: nvjpeg library (libnvjpeg.so) is still missing after setup"
  fi
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
# opennav_docking/opennav_docking_core/opennav_docking_msgs/opencv4_vendor/yaml_cpp_vendor
# are custom forks embedded in the CAMROD repo itself (camrod_docking/external/) and do not
# need separate clone_ext calls — they are already present after git clone.
clone_ext "https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git"         "release-3.2"  "camrod_docking/external/isaac_ros_common"
clone_ext "https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git"         "release-3.2"  "camrod_docking/external/isaac_ros_nitros"
clone_ext "https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git"       "release-3.2"  "camrod_docking/external/isaac_ros_apriltag"
clone_ext "https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git" "release-3.2"  "camrod_docking/external/isaac_ros_image_pipeline"
clone_ext "https://github.com/ros-perception/image_pipeline.git"             "humble"       "camrod_docking/external/image_pipeline"
clone_ext "https://github.com/osrf/negotiated.git"                           "master"       "camrod_docking/external/negotiated"

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
