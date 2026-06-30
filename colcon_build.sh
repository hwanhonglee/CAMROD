#!/usr/bin/env bash
# HH_260428: Pure colcon build wrapper — no downloads, no external repo changes.
# Safe to run repeatedly for incremental development.
#
# Usage:
#   ./colcon_build.sh                                    # full workspace build
#   ./colcon_build.sh --packages-select camrod_platform  # single package
#   ./colcon_build.sh --packages-up-to camrod_bringup    # package + deps
#   ./colcon_build.sh --packages-select camrod_platform --cmake-args -DCMAKE_BUILD_TYPE=Debug
#
# All unknown arguments are forwarded directly to `colcon build`.
# Run setup_camrod.sh first if external/ directories are missing.
# HH_260630: camrod_ui robot frontend assets are built here before colcon installs
# the package, so a normal full build also refreshes the UI bundle. This wrapper
# runs `colcon build` only; runtime sim validation and package lint/test commands
# are intentionally run separately so field builds are not blocked by lint-scope
# issues in vendored external packages.

set -euo pipefail

log() { echo "[colcon_build] $*"; }

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
  echo "[colcon_build] ERROR: cannot find workspace root (expected <ws>/src/camrod_bringup)" >&2
  exit 1
fi
SRC_ROOT="${WS_ROOT}/src"

mkdir -p "${WS_ROOT}/build" "${WS_ROOT}/install" "${WS_ROOT}/log"
cd "${WS_ROOT}"

# HH_260616: Scope-aware cleanup for generated artifacts. Package-selected builds
# must not delete artifacts owned by packages that are not being rebuilt.
_build_scope_includes_pkg() {
  local pkg="$1" prev="" token
  local saw_selector=0
  for token in "$@"; do
    case "${prev}" in
      --packages-select|--packages-up-to|--packages-above|--packages-above-and-dependencies|--packages-select-by-dep|--packages-start|--packages-end)
        saw_selector=1
        [[ "${token}" == "${pkg}" ]] && return 0
        ;;
    esac
    case "${token}" in
      --packages-select|--packages-up-to|--packages-above|--packages-above-and-dependencies|--packages-select-by-dep|--packages-start|--packages-end)
        saw_selector=1
        ;;
    esac
    prev="${token}"
  done
  [[ "${saw_selector}" -eq 0 ]]
}

# HH_260611: Remove stale per-package CMake build directories whose cached source
# path no longer exists. This handles external repo moves without requiring a full
# workspace clean.
_clean_stale_cmake_build_dirs() {
  local cache src build_dir
  shopt -s nullglob
  for cache in "${WS_ROOT}"/build/*/CMakeCache.txt; do
    src="$(awk -F= '/^CMAKE_HOME_DIRECTORY:INTERNAL=/ {print $2; exit}' "${cache}" || true)"
    [[ -n "${src}" ]] || continue
    [[ -d "${src}" ]] && continue
    build_dir="$(dirname "${cache}")"
    rm -rf "${build_dir}"
    log "removed stale build dir: ${build_dir} (missing source: ${src})"
  done
  shopt -u nullglob
}
_clean_stale_cmake_build_dirs

# HH_260611: ament_python symlink-install can fail when an installed data-file
# symlink already exists from an older build. Remove known generated launch
# artifacts before rebuilding so setup.py can recreate them.
_clean_stale_install_artifacts() {
  local path
  if _build_scope_includes_pkg camrod_ui "$@"; then
    for path in \
      "${WS_ROOT}/install/camrod_ui/share/camrod_ui/launch/ui.launch.py"
    do
      if [[ -e "${path}" || -L "${path}" ]]; then
        rm -f "${path}"
        log "removed stale install artifact: ${path}"
      fi
    done
  fi

  # HH_260611: Remove stale isolated install prefixes for packages that are no
  # longer part of this x86_64 build graph. If left in place, setup.bash tries
  # to source missing local_setup.bash files and launch reports unrelated
  # packages as "not built".
  # HH_260617: Keep camrod_parking out of this legacy cleanup list because it is
  # now a first-class source package built from src/camrod_parking.
  local prefix
  for prefix in \
    "${WS_ROOT}/install/apriltag_msgs" \
    "${WS_ROOT}/install/apriltag_ros" \
    "${WS_ROOT}/install/opennav_docking_msgs" \
    "${WS_ROOT}/install/opennav_docking_bt" \
    "${WS_ROOT}/install/opennav_docking_core" \
    "${WS_ROOT}/install/opennav_docking" \
    "${WS_ROOT}/install/ntrip_client_node" \
    "${WS_ROOT}/install/ublox_dgnss" \
    "${WS_ROOT}/install/ublox_dgnss_node" \
    "${WS_ROOT}/install/ublox_nav_sat_fix_hp_node" \
    "${WS_ROOT}/install/ublox_ubx_interfaces" \
    "${WS_ROOT}/install/ublox_ubx_msgs"
  do
    if [[ -e "${prefix}" || -L "${prefix}" ]]; then
      rm -rf "${prefix}"
      log "removed stale install prefix: ${prefix}"
    fi
  done
}
_clean_stale_install_artifacts "$@"

# HH_260629: Build the camrod_ui robot React frontend before colcon so setup.py can
# package the generated build/ tree into install (see camrod_ui/setup.py). The guest
# frontend is static (no build step). Only runs when camrod_ui is in the build scope.
_build_camrod_ui_frontend() {
  _build_scope_includes_pkg camrod_ui "$@" || return 0

  local frontend_dir="${SRC_ROOT}/camrod_ui/camrod_ui_robot/assets/frontend"
  [[ -f "${frontend_dir}/package.json" ]] || { log "camrod_ui frontend not found, skip npm build"; return 0; }

  if ! command -v npm >/dev/null 2>&1; then
    log "ERROR: npm not found but camrod_ui requires a frontend build (install Node.js/npm)" >&2
    return 1
  fi

  log "camrod_ui frontend: npm install (${frontend_dir})"
  ( cd "${frontend_dir}" && npm install )

  log "camrod_ui frontend: npm run build"
  ( cd "${frontend_dir}" && npm run build )
}
_build_camrod_ui_frontend "$@"

# shellcheck disable=SC1091
set +u; source /opt/ros/humble/setup.bash; set -u

# HH_260428: Sanitize stale install paths from prefix vars to avoid picking up
# packages from a previous build that no longer exists on disk.
sanitize_path_var() {
  local var_name="$1"
  local raw="${!var_name:-}" out="" item pkg_name manifest
  IFS=':' read -r -a parts <<< "${raw}"
  for item in "${parts[@]}"; do
    [[ -z "${item}" ]] && continue
    [[ -d "${item}" ]] || continue
    if [[ "${item}" != /opt/ros/* && "${item}" != "${WS_ROOT}/install/"* ]]; then continue; fi
    if [[ "${item}" == "${WS_ROOT}/install/"* ]]; then
      pkg_name="$(basename "${item}")"
      manifest="${item}/share/${pkg_name}/package.xml"
      [[ -f "${manifest}" ]] || continue
    fi
    out="${out:+${out}:}${item}"
  done
  export "${var_name}=${out}"
}
sanitize_path_var AMENT_PREFIX_PATH
sanitize_path_var CMAKE_PREFIX_PATH
sanitize_path_var COLCON_PREFIX_PATH

# HH_260428: Collect all external/ base directories for colcon --base-paths.
# Excludes .git internals, build/install/log artifacts, and disabled packages.
ARCH="$(uname -m)"
mapfile -t EXTERNAL_BASES < <(
  cd "${WS_ROOT}" && find src -type d -name external \
    -not -path 'src/todo/*' \
    -not -path '*/.git/*' \
    -not -path '*/build/*' \
    -not -path '*/install/*' \
    -not -path '*/log/*' \
    -not -path '*/disable/*' | sort
)
BUILD_SKIP_PACKAGES=()
if [[ "${ARCH}" != "aarch64" && "${ARCH}" != "arm64" ]]; then
  # HH_260611: Isaac ROS / VPI docking dependencies are Jetson-only in this workspace.
  # Keep x86_64 development builds focused on buildable CAMROD modules such as sensing/GNSS.
  _filtered_external_bases=()
  for _base in "${EXTERNAL_BASES[@]}"; do
    [[ "${_base}" == "src/camrod_docking/external" ]] && continue
    _filtered_external_bases+=("${_base}")
  done
  EXTERNAL_BASES=("${_filtered_external_bases[@]}")
  if [[ -d "src/camrod_docking/external/opencv4_vendor" ]]; then
    EXTERNAL_BASES+=("src/camrod_docking/external/opencv4_vendor")
  fi
  BUILD_SKIP_PACKAGES+=(camrod_docking)
  log "skip camrod_docking and docking external packages on ${ARCH} (Isaac ROS/VPI not available)"
  unset _base _filtered_external_bases
fi

# HH_260616: Keep planning/sensing development builds usable on machines where the
# optional voice stack has not been provisioned yet. setup_camrod.sh installs
# libsdl2-mixer-dev; until then camrod_voice cannot configure because SDL2_mixer
# is a required pkg-config dependency.
if [[ -d "src/camrod_voice" ]] && ! pkg-config --exists SDL2_mixer 2>/dev/null; then
  BUILD_SKIP_PACKAGES+=(camrod_voice)
  log "skip camrod_voice (missing SDL2_mixer; install libsdl2-mixer-dev via setup_camrod.sh)"
fi

BASE_PATHS=("src" "${EXTERNAL_BASES[@]}")
BUILD_SKIP_ARGS=()
if [[ ${#BUILD_SKIP_PACKAGES[@]} -gt 0 ]]; then
  BUILD_SKIP_ARGS=(--packages-skip "${BUILD_SKIP_PACKAGES[@]}")
fi

# HH_260428: Mark extracted vendor binary trees (e.g. tier4_adapi) so colcon
# does not try to build them as source packages.
_mark_vendor_trees() {
  local root
  mapfile -t roots < <(find "${SRC_ROOT}" -type d -path '*/vendor/*/extract' | sort -u)
  for root in "${roots[@]}"; do
    if [[ ! -e "${root}/COLCON_IGNORE" ]]; then
      printf '# auto: extracted vendor binary tree\n' > "${root}/COLCON_IGNORE"
      log "marked vendor extract: ${root}"
    fi
  done
}
_mark_vendor_trees

# HH_260428: tier4_adapi ships aarch64-only vendor libs.
# On x86_64 mark the three arch-specific packages so colcon skips them.
_TIER4="${SRC_ROOT}/camrod_map/external/tier4_adapi"
_ARCH="$(uname -m)"
if [[ -d "${_TIER4}" ]]; then
  if [[ "${_ARCH}" == "aarch64" ]]; then
    set +u
    source "${_TIER4}/setup_vendor_env.bash" 2>/dev/null || true
    set -u
    log "sourced tier4_adapi vendor env (aarch64)"
  else
    log "skip tier4_adapi vendor env on ${_ARCH}; marking arch-only packages COLCON_IGNORE"
    for _pkg in tier4_system_msgs autoware_component_interface_utils tier4_adapi_rviz_plugin; do
      _ci="${_TIER4}/${_pkg}/COLCON_IGNORE"
      [[ -e "${_ci}" ]] || echo "# auto: aarch64-only vendor dep" > "${_ci}"
    done
    unset _pkg _ci
  fi
fi
unset _TIER4 _ARCH

# HH_260428: nova_carter_docking is an NVIDIA Isaac ROS example; not buildable
# without isaac_ros_apriltag_interfaces (Jetson-only). No CAMROD package depends on it.
# HH_260528: Path updated from camrod_parking to camrod_docking.
_NCD="${SRC_ROOT}/camrod_docking/external/opennav_docking/nova_carter_docking"
if [[ -d "${_NCD}" && ! -e "${_NCD}/COLCON_IGNORE" ]]; then
  echo "# auto: NVIDIA Isaac ROS example (requires isaac_ros_apriltag_interfaces)" \
    > "${_NCD}/COLCON_IGNORE"
  log "marked nova_carter_docking COLCON_IGNORE"
fi
unset _NCD

log "colcon build  base-paths: ${BASE_PATHS[*]}"
colcon --log-base "${WS_ROOT}/log" build \
  --symlink-install \
  --base-paths "${BASE_PATHS[@]}" \
  --build-base  "${WS_ROOT}/build" \
  --install-base "${WS_ROOT}/install" \
  "${BUILD_SKIP_ARGS[@]}" \
  "$@"

log "done.  source ${WS_ROOT}/install/setup.bash"
