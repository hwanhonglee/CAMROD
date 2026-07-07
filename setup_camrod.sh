#!/usr/bin/env bash
# HH_260428: One-time workspace setup — clones external dependencies and installs ROS deps.
#
# Run this ONCE on a new machine or after a fresh git clone.
# It will NOT overwrite existing external directories unless --update is given.
#
# Usage:
#   ./setup_camrod.sh             # install deps + clone missing externals + rosdep (safe, idempotent)
#   ./setup_camrod.sh --update    # also pull latest for each external (REWRITES working copies)
#   ./setup_camrod.sh --no-rosdep # skip rosdep after explicit system-package + external setup
#
# WARN: --update overwrites any local changes inside external/ directories.
# HH_260630: setup covers the current split runtime:
#   - camrod_parking is a local source package, not a legacy external.
#   - camrod_voice requires SDL2_mixer; setup installs libsdl2-mixer-dev.
#   - camrod_docking/Isaac ROS remains Jetson-only and is skipped on x86_64.
#   - Ranger/SocketCAN tools are installed here; runtime CAN activation is handled
#     by camrod_platform/scripts/setup_can0.sh or the matching systemd service.
#   - camrod_ui frontend build is handled by colcon_build.sh before packaging.
#   - sim validation is installed by camrod_bringup and run after bringup, not here.
#   - colcon tests/lint are separate from setup because ament lint policy is package-specific.
# HH_260703: current field setup assumptions:
#   - ZED-F9P GNSS rover config uses /dev/ttyACM1; CV7 IMU currently
#     enumerates as ttyACM0; CH9344 USB ports are radar.
#   - LiDAR runtime is tuned in package configs, not installed here.
#   - planning soft-estop wiring is built/installed by colcon_build.sh.
# HH_260702: setup is still dependency-only. Do not add runtime launch/test
#   side effects here; use colcon_build.sh for install sync and
#   camrod_bringup/sim_validation_runner.py for deterministic manual, obstacle,
#   campsite, and drop-zone validation after bringup is running.
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

# HH_260617: Keep setup usable from CI/Codex/non-interactive shells.  When sudo
# cannot prompt for a password, report the exact apt command instead of hanging
# or turning optional dependency setup into a hard failure.
apt_install_pkgs() {
  local label="$1"
  shift
  if [[ "$#" -eq 0 ]]; then
    return 0
  fi
  if sudo -n true 2>/dev/null || [[ -t 0 ]]; then
    sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y "$@"
    return $?
  fi
  log "WARN: sudo password is required but no TTY is available; cannot install ${label}: $*"
  log "      Run manually: sudo apt-get install -y $*"
  return 1
}

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
  libsdl2-dev        # HH_260615 - camrod_voice SDL2 audio backend.
  libsdl2-mixer-dev  # HH_260615 - camrod_voice WAV playback via Mix_* API.
  # HH_260630: Explicitly install deps that rosdep may not resolve in
  # non-interactive field setup sessions.
  ros-humble-nav2-map-server        # Nav2 map server for planning bringup.
  ros-humble-behaviortree-cpp-v3    # Nav2 BT navigator runtime.
  # YH_260706: Global planners listed in planner_server planner_plugins (nav2_base.yaml).
  #   Missing → planner_server on_configure throws (pluginlib class not found) and rolls back
  #   → planning lifecycle never activates → /planning/navigate_to_pose has no server
  #   → docking Phase 1 aborts instantly with error 903 (FAILED_TO_STAGE).
  ros-humble-nav2-navfn-planner       # planner_plugins: "NavFn"
  ros-humble-nav2-theta-star-planner  # planner_plugins: "ThetaStar"
  # YH_260706: Controllers listed in controller_server controller_plugins (nav2_base.yaml).
  #   Same failure mode as the planners above — a missing controller plugin makes
  #   controller_server on_configure throw and roll back, so planning never activates.
  #   (RPP is vendored/built in camrod_planning/external; Graceful is nav2-graceful-controller above.)
  ros-humble-nav2-mppi-controller          # controller_plugins: "MPPI" (CAMROD default)
  ros-humble-nav2-dwb-controller           # controller_plugins: "DWB"
  ros-humble-nav2-rotation-shim-controller # controller_plugins: "RotationShim"
  # YH_260706: Recovery behaviors (spin/backup/wait/drive_on_heading). nav2_lanelet.launch.py
  #   only launches behavior_server when this package is present; the nav-to-pose BT (default
  #   and CAMROD's) references <Spin>/<BackUp>, so without it bt_navigator on_activate throws
  #   "Action server spin not available" and stays inactive → navigate_to_pose unusable.
  ros-humble-nav2-behaviors               # behavior_server recovery actions
  # YH_260706: smoother_server(smooth_path). nav2_lanelet.launch.py launches it only when
  #   present; CAMROD nav-to-pose BT calls <SmoothPath>, so without it bt_navigator on_activate
  #   throws "Action server smooth_path not available" and stays inactive.
  ros-humble-nav2-smoother                # smoother_server (smooth_path action)
  ros-humble-controller-manager     # ugv_sdk controller dependency.
  ros-humble-rviz2                  # RViz2 for operator and sim bringup.
  ros-humble-rviz-common            # RViz2 common libraries.
  ros-humble-rviz-default-plugins   # RViz2 default plugins.
  ros-humble-rtcm-msgs              # ublox GNSS RTK (camrod_sensing)
  ros-humble-test-msgs              # ROS 2 test message dependency.
  python3-serial                    # ublox/radar serial communication.
  python3-yaml                      # HH_260617: camrod_parking/camrod_bringup YAML config parsing
  python3-setuptools                # HH_260617: ament_python entry point install for camrod_parking/ui
  can-utils                         # HH_260629: SocketCAN diagnostics for Ranger bringup.
  iproute2                          # HH_260629: Provides `ip link` for setup_can0.sh.
  libpugixml-dev                    # Lanelet2 XML parser for camrod_map.
  libnanoflann-dev                  # PCL/point-cloud KNN support for perception.
)

# HH_260615 - camrod_docking registers the Isaac ROS apt repository and runtime packages.
# Runs only on aarch64 Jetson/JetPack targets; x86_64 prints a skip notice.
setup_isaac_ros_apt() {
  local _arch
  _arch="$(uname -m)"
  if [[ "${_arch}" != "aarch64" && "${_arch}" != "arm64" ]]; then
    log "skip Isaac ROS apt setup (${_arch} is not aarch64)"
    log "  x86_64 builds skip camrod_docking: colcon build --packages-skip camrod_docking"
    return 0
  fi

  if [[ ! -f /etc/apt/sources.list.d/isaac-ros.list ]]; then
    log "register NVIDIA Isaac ROS apt repository (arm64)"
    apt_install_pkgs "Isaac ROS apt repository tools" curl gnupg lsb-release
    curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
      | gpg --dearmor \
      | sudo tee /usr/share/keyrings/isaac-ros-archive-keyring.gpg > /dev/null
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/isaac-ros-archive-keyring.gpg] \
https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" \
      | sudo tee /etc/apt/sources.list.d/isaac-ros.list > /dev/null
    sudo apt-get update
  else
    log "Isaac ROS apt repository already registered"
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
    log "install Isaac ROS packages: ${_missing_isaac[*]}"
    apt_install_pkgs "Isaac ROS runtime packages" "${_missing_isaac[@]}"
  else
    log "Isaac ROS packages already installed"
  fi
  unset _arch _isaac_pkgs _ipkg _missing_isaac

  # HH_260616 - isaac_ros CMake exports hard-code the build-time numpy include path
  # (/usr/local/lib/python3.10/dist-packages/numpy/core/include). User-installed
  # numpy under /home/.../.local can break CMake generation, so ensure numpy also
  # exists in the system sudo-pip path.
  if [[ ! -d /usr/local/lib/python3.10/dist-packages/numpy ]]; then
    log "install numpy into system path for Isaac ROS CMake compatibility"
    # HH_260630: Pin to the active user numpy version to avoid ABI drift in
    # cv_bridge and other C extension packages.
    _numpy_ver="$(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null || echo '1.26.4')"
    sudo pip3 install "numpy==${_numpy_ver}"
    unset _numpy_ver
  else
    log "numpy system path exists"
  fi
}

# HH_260611: Keep nvjpeg dependency handling Jetson-only so x86_64 sensing/GNSS
# builds do not warn about unavailable CUDA/JetPack runtime libraries.
if [[ -d "${SRC_ROOT}/camrod_sensing" ]]; then
  _arch="$(uname -m)"
  if [[ "${_arch}" == "aarch64" || "${_arch}" == "arm64" ]]; then
    if ! has_nvjpeg_header || ! has_nvjpeg_library; then
      # HH_260616 - apt-cache may be stale, so refresh it before nvjpeg discovery.
      # Without this, apt_has_candidate can miss libnvjpeg-dev and camrod_sensing
      # can fail later because nvjpeg.h is absent.
      log "apt-get update before nvjpeg package discovery"
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
  apt_install_pkgs "system packages" "${_missing[@]}" || \
    log "WARN: apt-get install failed — build may fail if packages are absent"
fi
unset _pkg _missing

# HH_260615 - Register Isaac ROS apt only when the camrod_docking module exists.
if [[ -d "${SRC_ROOT}/camrod_docking" ]]; then
  _arch="$(uname -m)"
  setup_isaac_ros_apt

  # HH_260616: Match colcon_build.sh behavior. On x86_64 camrod_docking is
  # skipped because Isaac ROS/VPI is Jetson-only in this workspace, so setup
  # must not fail while installing optional docking runtime packages.
  if [[ "${_arch}" == "aarch64" || "${_arch}" == "arm64" ]]; then
    # Install non-Isaac docking dependencies via apt on Jetson targets.
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
      apt_install_pkgs "docking packages" "${_docking_missing[@]}" || \
        log "WARN: docking apt install failed — docking may be unavailable"
    else
      log "docking packages already installed"
    fi
    unset _docking_apt_pkgs _docking_missing _dpkg
  else
    log "skip docking apt packages (${_arch}; camrod_docking is skipped by colcon_build.sh)"
  fi
  unset _arch
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
  # HH_260630: rosdep init is idempotent; ignore the already-initialized case.
  if ! rosdep init 2>/dev/null; then
    log "rosdep already initialized — skipping init"
  fi
  log "rosdep update"
  rosdep update || log "WARN: rosdep update failed — continuing"

  # HH_260617: These keys are either source-built in this workspace, handled by
  # explicit apt setup above, Jetson-only, or legacy upstream package.xml names
  # that break idempotent x86_64 setup.  Keep rosdep focused on actionable deps.
  _ROSDEP_SKIP_KEYS=(
    ament_python
    catkin
    OpenCV
    cuda_cudart
    isaac_ros_apriltag
    isaac_ros_apriltag_interfaces
    isaac_ros_image_proc
    libsdl2-dev
    libsdl2-mixer-dev
    opencv
    opennav_docking
    opennav_docking_core
    opennav_docking_bt
    opennav_docking_msgs
  )

  log "rosdep install (skip keys: ${_ROSDEP_SKIP_KEYS[*]})"
  mapfile -t _ROSDEP_PATHS < <(
    find "${SRC_ROOT}" -name package.xml -type f \
      | grep -Ev '/vendor/.*/extract/' \
      | xargs -r -n1 dirname \
      | while read -r p; do readlink -f "${p}"; done \
      | sort -u
  )
  if ! rosdep install --from-paths "${_ROSDEP_PATHS[@]}" --ignore-src -r -y \
      --skip-keys="${_ROSDEP_SKIP_KEYS[*]}"; then
    log "WARN: rosdep failed (no sudo or unresolved keys) — continuing"
  fi
  unset _ROSDEP_SKIP_KEYS
fi

log "setup complete. now run: ./colcon_build.sh"
