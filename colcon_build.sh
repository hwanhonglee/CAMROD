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
# HH_260701: this wrapper is the canonical way to install the current field
# baseline after config-only updates too: planning soft-estop wiring, synchronized
# GNSS/LiDAR diagnostics configs, radar no-target handling, and README updates all
# land in the install tree through this script.
# HH_260702: this wrapper also preserves the current validation contract:
# - validate/reuse matching npm dependencies and run npm build before packaging
#   camrod_ui when it is in scope;
# - install synchronized bringup/package configs and README updates together;
# - leave sim/manual/obstacle/camping/drop-zone validation to
#   camrod_bringup/scripts/sim_validation_runner.py after launch.
# HH_260703: use this wrapper after config-only field safety changes too; the
# install tree must receive cmd_vel gate, diagnostics, GNSS, and README updates
# before outdoor validation.
# HH_260706: v2.0.1 field updates also depend on this wrapper: it rebuilds the
# UI bundle before packaging and syncs adaptive cmd_vel gate/config docs into
# install before tagging.
# HH_260707: runtime-load update also depends on this wrapper: perception queue
# tuning, LiDAR/cost-grid cache parameters, map/path marker throttles, filtered
# system diagnostics, and README updates are installed through normal colcon
# package data installation.
# HH_260708: field_test_tool.sh and field_test_runbook.md are installed through
# this wrapper so outdoor tests can collect config-sync, diagnostics, Hz, CPU,
# and gate-state evidence from the same installed package graph.
# HH_260807: single-config CMake packages now default to Release so compute-heavy
# sensor components use compiler optimization in normal robot builds. An explicit
# -DCMAKE_BUILD_TYPE override remains authoritative for debugging/profiling.
# HH_260825 - Always resolve and enter the workspace root and pass explicit
# build/install/log bases. Direct `colcon build` uses the caller's current
# directory, which is why running it under src previously created src/build,
# src/install, and src/log.

set -euo pipefail

log() { echo "[colcon_build] $*"; }

_has_explicit_cmake_build_type() {
  local arg
  local definition_follows=0
  for arg in "$@"; do
    if [[ "${definition_follows}" -eq 1 ]]; then
      case "${arg}" in
        CMAKE_BUILD_TYPE=*|CMAKE_BUILD_TYPE:*=*) return 0 ;;
      esac
      definition_follows=0
    fi

    case "${arg}" in
      -DCMAKE_BUILD_TYPE=*|-DCMAKE_BUILD_TYPE:*=*|\
      --cmake-args=-DCMAKE_BUILD_TYPE=*|--cmake-args=-DCMAKE_BUILD_TYPE:*=*)
        return 0
        ;;
      -D|--cmake-args=-D)
        # CMake also accepts the split spelling: -D CMAKE_BUILD_TYPE=Debug.
        definition_follows=1
        ;;
    esac
  done
  return 1
}

_prepare_colcon_build_args() {
  COLCON_BUILD_ARGS=("$@")
  COLCON_BUILD_TYPE_SOURCE="user"
  _has_explicit_cmake_build_type "$@" && return 0

  local release_arg="-DCMAKE_BUILD_TYPE=Release"
  local last_cmake_args_index=-1
  local index inline_value
  for ((index = 0; index < ${#COLCON_BUILD_ARGS[@]}; index++)); do
    case "${COLCON_BUILD_ARGS[index]}" in
      --cmake-args|--cmake-args=*) last_cmake_args_index="${index}" ;;
    esac
  done

  if [[ "${last_cmake_args_index}" -lt 0 ]]; then
    COLCON_BUILD_ARGS+=(--cmake-args "${release_arg}")
  elif [[ "${COLCON_BUILD_ARGS[last_cmake_args_index]}" == "--cmake-args" ]]; then
    # Keep the default in the user's effective (last) --cmake-args group. A
    # second group would replace, rather than extend, the first in argparse.
    COLCON_BUILD_ARGS=(
      "${COLCON_BUILD_ARGS[@]:0:last_cmake_args_index + 1}"
      "${release_arg}"
      "${COLCON_BUILD_ARGS[@]:last_cmake_args_index + 1}"
    )
  else
    # Normalize --cmake-args=-DFOO so Release and the inline value remain in
    # the same group instead of allowing one argparse occurrence to win.
    inline_value="${COLCON_BUILD_ARGS[last_cmake_args_index]#--cmake-args=}"
    COLCON_BUILD_ARGS=(
      "${COLCON_BUILD_ARGS[@]:0:last_cmake_args_index}"
      --cmake-args
      "${inline_value}"
      "${release_arg}"
      "${COLCON_BUILD_ARGS[@]:last_cmake_args_index + 1}"
    )
  fi
  COLCON_BUILD_TYPE_SOURCE="default-release"
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
  echo "[colcon_build] ERROR: cannot find workspace root (expected <ws>/src/camrod_bringup)" >&2
  exit 1
fi
SRC_ROOT="${WS_ROOT}/src"

mkdir -p "${WS_ROOT}/build" "${WS_ROOT}/install" "${WS_ROOT}/log"
cd "${WS_ROOT}"

# HH_260616: Scope-aware cleanup for generated artifacts. Package-selected builds
# must not delete artifacts owned by packages that are not being rebuilt.
_build_scope_includes_pkg() {
  local pkg="$1" token
  shift
  # HH_260702: Colcon selectors accept multiple package names after one option.
  # Keep scanning selector arguments until the next option so camrod_ui is
  # recognized even when it is not the first package in --packages-select.
  local in_selector=0
  local in_skip_selector=0
  local saw_selector=0
  local matched_selector=0
  for token in "$@"; do
    case "${token}" in
      --packages-select|--packages-up-to|--packages-above|--packages-above-and-dependencies|--packages-select-by-dep|--packages-start|--packages-end)
        saw_selector=1
        in_selector=1
        in_skip_selector=0
        continue
        ;;
      --packages-skip)
        in_selector=0
        in_skip_selector=1
        continue
        ;;
      --*)
        in_selector=0
        in_skip_selector=0
        ;;
    esac
    if [[ "${in_skip_selector}" -eq 1 && "${token}" == "${pkg}" ]]; then
      return 1
    fi
    if [[ "${in_selector}" -eq 1 && "${token}" == "${pkg}" ]]; then
      matched_selector=1
    fi
  done
  [[ "${saw_selector}" -eq 0 || "${matched_selector}" -eq 1 ]]
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

# HH_260706: Recreate camrod_ui build/install prefixes as a unit. Removing only
# one generated launch symlink can make ament_python symlink_data fail on the
# next rebuild, leaving the robot UI launch missing from install.
_clean_stale_install_artifacts() {
  if _build_scope_includes_pkg camrod_ui "$@"; then
    rm -rf "${WS_ROOT}/build/camrod_ui" "${WS_ROOT}/install/camrod_ui"
    log "removed camrod_ui build/install prefixes for a clean UI reinstall"
  fi

  # HH_260611: Remove stale isolated install prefixes for packages that are no
  # longer part of this x86_64 build graph. If left in place, setup.bash tries
  # to source missing local_setup.bash files and launch reports unrelated
  # packages as "not built".
  # HH_260720 - Parking controllers are built inside camrod_control; no legacy package prefix remains.
  # HH_260721 - The ground-segmentation core is integrated into its ROS 2 node package.
  local prefix
  for prefix in \
    "${WS_ROOT}/install/apriltag_msgs" \
    "${WS_ROOT}/install/apriltag_ros" \
    "${WS_ROOT}/install/ground_segmentation" \
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

  local dependency_stamp="${frontend_dir}/node_modules/.camrod-deps.sha256"
  local hidden_lock="${frontend_dir}/node_modules/.package-lock.json"
  local dependency_fingerprint=""
  local dependencies_ready=0

  if [[ -f "${frontend_dir}/package-lock.json" ]]; then
    dependency_fingerprint="$({
      sha256sum "${frontend_dir}/package.json" "${frontend_dir}/package-lock.json"
      node --version
      npm --version
    } | sha256sum | awk '{print $1}')"

    if [[ -f "${dependency_stamp}" ]] \
      && [[ "$(<"${dependency_stamp}")" == "${dependency_fingerprint}" ]]; then
      dependencies_ready=1
    elif [[ -f "${hidden_lock}" ]] \
      && [[ "${hidden_lock}" -nt "${frontend_dir}/package.json" ]] \
      && [[ "${hidden_lock}" -nt "${frontend_dir}/package-lock.json" ]]; then
      # Bootstrap the stamp for an existing npm install only after npm confirms
      # the complete dependency tree is usable.
      if ( cd "${frontend_dir}" && npm ls --ignore-scripts --all >/dev/null 2>&1 ); then
        printf '%s\n' "${dependency_fingerprint}" > "${dependency_stamp}"
        dependencies_ready=1
      fi
    fi
  fi

  if [[ "${dependencies_ready}" -eq 1 ]] \
    && ( cd "${frontend_dir}" && npm ls --ignore-scripts --all >/dev/null 2>&1 ); then
    log "camrod_ui frontend: dependencies unchanged; skip npm install"
  else
    log "camrod_ui frontend: dependency manifest changed; npm ci (${frontend_dir})"
    ( cd "${frontend_dir}" && npm ci --prefer-offline --no-audit --no-fund )
    dependency_fingerprint="$({
      sha256sum "${frontend_dir}/package.json" "${frontend_dir}/package-lock.json"
      node --version
      npm --version
    } | sha256sum | awk '{print $1}')"
    printf '%s\n' "${dependency_fingerprint}" > "${dependency_stamp}"
  fi

  log "camrod_ui frontend: npm run build"
  ( cd "${frontend_dir}" && npm run build )
}
_build_camrod_ui_frontend "$@"

_verify_camrod_ui_frontend_install() {
  _build_scope_includes_pkg camrod_ui "$@" || return 0

  local source_index="${SRC_ROOT}/camrod_ui/camrod_ui_robot/assets/frontend/build/index.html"
  local installed_index="${WS_ROOT}/install/camrod_ui/share/camrod_ui/camrod_ui_robot/assets/frontend/build/index.html"
  local source_bundle installed_bundle
  [[ -f "${source_index}" ]] || { log "ERROR: frontend source build index is missing" >&2; return 1; }
  [[ -f "${installed_index}" ]] || { log "ERROR: installed frontend index is missing" >&2; return 1; }
  source_bundle="$(grep -o 'main\.[a-z0-9]*\.js' "${source_index}" | head -1)"
  installed_bundle="$(grep -o 'main\.[a-z0-9]*\.js' "${installed_index}" | head -1)"
  if [[ -z "${source_bundle}" || "${source_bundle}" != "${installed_bundle}" ]]; then
    log "ERROR: frontend install mismatch (source=${source_bundle:-missing}, install=${installed_bundle:-missing})" >&2
    return 1
  fi
  log "camrod_ui frontend verified: ${installed_bundle}"
}

# shellcheck disable=SC1091
set +u; source /opt/ros/humble/setup.bash; set -u

# HH_260428: Sanitize stale install paths from prefix vars to avoid picking up
# packages from a previous build that no longer exists on disk.
#
# The virtual/CARLA profile has two deliberate underlays outside this workspace:
# the standard CARLA ROS bridge and the Ranger 4WS controller workspace.  Its
# wrapper passes their install roots through CAMROD_EXTRA_PREFIX_ROOTS.  Keep
# only descendants of those explicit roots; arbitrary ambient overlays (for
# example a previously sourced Autoware workspace) remain rejected.
_is_allowed_extra_prefix() {
  local candidate="$1" root candidate_real root_real
  # HH_260825 - An explicit empty array is required under `set -u`; a machine
  # without CARLA underlays must still build the ordinary CAMROD graph.
  local allowed_roots=()
  candidate_real="$(readlink -m "${candidate}")"
  IFS=':' read -r -a allowed_roots <<< "${CAMROD_EXTRA_PREFIX_ROOTS:-}"
  for root in "${allowed_roots[@]}"; do
    [[ -n "${root}" ]] || continue
    root_real="$(readlink -m "${root}")"
    case "${candidate_real}" in
      "${root_real}"|"${root_real}"/*) return 0 ;;
    esac
  done
  return 1
}

sanitize_path_var() {
  local var_name="$1"
  local raw="${!var_name:-}" out="" item pkg_name manifest
  IFS=':' read -r -a parts <<< "${raw}"
  for item in "${parts[@]}"; do
    [[ -z "${item}" ]] && continue
    [[ -d "${item}" ]] || continue
    if [[ "${item}" != /opt/ros/* && \
          "${item}" != "${WS_ROOT}/install/"* ]] && \
       ! _is_allowed_extra_prefix "${item}"; then
      continue
    fi
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

if [[ -n "${CAMROD_EXTRA_PREFIX_ROOTS:-}" ]]; then
  log "preserved approved extra prefix roots: ${CAMROD_EXTRA_PREFIX_ROOTS}"
fi

# HH_260721 - Reject the retired standalone ground-segmentation package instead
# of silently building two identical copies on machines with stale source trees.
_reject_legacy_ground_segmentation_package() {
  local legacy_package="${SRC_ROOT}/camrod_sensing/external/ground_segmentation/package.xml"
  local runtime_package="${SRC_ROOT}/camrod_sensing/external/ground_segmentation_ros2/package.xml"
  if [[ -f "${legacy_package}" && -f "${runtime_package}" ]]; then
    log "ERROR: remove legacy camrod_sensing/external/ground_segmentation; its headers are integrated into ground_segmentation_ros2" >&2
    return 1
  fi
}
_reject_legacy_ground_segmentation_package

# HH_260428: Collect all external/ base directories for colcon --base-paths.
# Excludes .git internals, build/install/log artifacts, and disabled packages.
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
# HH_260720 - The removed docking package no longer needs architecture-specific build skips.

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

_prepare_colcon_build_args "$@"
if [[ "${COLCON_BUILD_TYPE_SOURCE}" == "default-release" ]]; then
  log "CMake build type: Release (default; override with --cmake-args -DCMAKE_BUILD_TYPE=<type>)"
else
  log "CMake build type: explicit user override"
fi

log "colcon build  base-paths: ${BASE_PATHS[*]}"
colcon --log-base "${WS_ROOT}/log" build \
  --symlink-install \
  --base-paths "${BASE_PATHS[@]}" \
  --build-base  "${WS_ROOT}/build" \
  --install-base "${WS_ROOT}/install" \
  "${BUILD_SKIP_ARGS[@]}" \
  "${COLCON_BUILD_ARGS[@]}"

_verify_camrod_ui_frontend_install "$@"

log "done.  source ${WS_ROOT}/install/setup.bash"
