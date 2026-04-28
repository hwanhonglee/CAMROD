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

# ── System packages required before rosdep ──────────────────────────────────
REQUIRED_SYS_PKGS=(ros-humble-nav2-graceful-controller)
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

# ── External repositories ────────────────────────────────────────────────────
log "bootstrapping external repositories (workspace: ${WS_ROOT})"

clone_ext "https://github.com/KumarRobotics/ublox.git"                      "master"       "camrod_sensing/external/ublox"
clone_ext "https://github.com/cra-ros-pkg/robot_localization.git"            "humble-devel"  "camrod_localization/external/robot_localization"
clone_ext "https://github.com/fzi-forschungszentrum-informatik/Lanelet2.git" "master"       "camrod_map/external/lanelet2"
clone_ext "https://github.com/ros-perception/vision_opencv.git"              "humble"       "camrod_perception/external/vision_opencv"
clone_ext "https://github.com/ros-perception/vision_msgs.git"                "ros2"         "camrod_common/external/vision_msgs"
clone_ext "https://github.com/ros-perception/perception_pcl.git"             "humble"       "camrod_sensing/external/perception_pcl"
clone_ext "https://github.com/ros-perception/laser_geometry.git"             "ros2"         "camrod_planning/external/laser_geometry"
# HH_260428: Agilex platform drivers — set CAMROD_AGILEX_BASE to use custom forks.
clone_ext "${AGILEX_BASE}/ugv_sdk.git"                                        "main"         "camrod_platform/external/ugv_sdk"
clone_ext "${AGILEX_BASE}/ranger_ros2.git"                                    "humble"       "camrod_platform/external/ranger_ros2"
clone_ext "https://github.com/open-navigation/opennav_docking.git"           "0.0.2"        "camrod_parking/external/opennav_docking"
clone_ext "https://github.com/christianrauch/apriltag_ros.git"               "3.3.0"        "camrod_parking/external/apriltag_ros"
clone_ext "https://github.com/christianrauch/apriltag_msgs.git"              "2.0.1"        "camrod_parking/external/apriltag_msgs"

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
