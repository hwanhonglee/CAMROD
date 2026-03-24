#!/usr/bin/env bash
set -euo pipefail

# Build CAMROD including nested external trees.
# Works for:
# - source-tree root layout:   CAMROD/<packages...>
# - workspace root layout:     camrod_ws/src/<packages...>

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# workspace layout: /path/to/ws/src/<packages...>
if [[ "$(basename "${SCRIPT_DIR}")" == "src" && -d "${SCRIPT_DIR}/camrod_bringup" && -d "${PARENT_DIR}/.colcon" ]]; then
  WS_ROOT="${PARENT_DIR}"
  SOURCE_ROOT="${SCRIPT_DIR}"
  BASE_ROOT="src"
# source-tree root layout: /path/to/CAMROD/<packages...>
elif [[ -d "${SCRIPT_DIR}/camrod_bringup" ]]; then
  WS_ROOT="${SCRIPT_DIR}"
  SOURCE_ROOT="${SCRIPT_DIR}"
  BASE_ROOT="."
elif [[ -d "${SCRIPT_DIR}/src/camrod_bringup" ]]; then
  WS_ROOT="${SCRIPT_DIR}"
  SOURCE_ROOT="${SCRIPT_DIR}/src"
  BASE_ROOT="src"
else
  echo "[build_camrod_all] unsupported tree layout at: ${SCRIPT_DIR}" >&2
  exit 1
fi

cd "${WS_ROOT}"

# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
set -u

BASE_PATHS=("${BASE_ROOT}")
while IFS= read -r d; do
  BASE_PATHS+=("${d}")
done < <(find "${BASE_ROOT}" -mindepth 3 -maxdepth 3 -type d -path '*/external/*' | sort -u)

sanitize_colon_path_var() {
  local var_name="$1"
  local raw="${!var_name:-}"
  local out=""
  local item=""
  local pkg_name=""
  local expected_manifest=""
  IFS=':' read -r -a parts <<< "${raw}"
  for item in "${parts[@]}"; do
    [[ -z "${item}" ]] && continue
    [[ -d "${item}" ]] || continue
    if [[ "${item}" != /opt/ros/* && "${item}" != "${WS_ROOT}/install/"* ]]; then
      continue
    fi
    if [[ "${item}" == "${WS_ROOT}/install/"* ]]; then
      pkg_name="$(basename "${item}")"
      expected_manifest="${item}/share/${pkg_name}/package.xml"
      [[ -f "${expected_manifest}" ]] || continue
    fi
    if [[ -z "${out}" ]]; then
      out="${item}"
    else
      out="${out}:${item}"
    fi
  done
  export "${var_name}=${out}"
}

sanitize_colon_path_var AMENT_PREFIX_PATH
sanitize_colon_path_var CMAKE_PREFIX_PATH
# Keep COLCON_PREFIX_PATH aligned with current workspace + /opt/ros only.
# This prevents stale paths from other workspaces (e.g. cart_test_ws/ros2_ws)
# from leaking into package discovery and causing confusing build warnings.
sanitize_colon_path_var COLCON_PREFIX_PATH

echo "[build_camrod_all] ws_root=${WS_ROOT}"
echo "[build_camrod_all] source_root=${SOURCE_ROOT}"
echo "[build_camrod_all] base-paths:"
for p in "${BASE_PATHS[@]}"; do
  echo "  - ${p}"
done

colcon build --symlink-install --base-paths "${BASE_PATHS[@]}" "$@"
