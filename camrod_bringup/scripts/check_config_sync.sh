#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

find_ws_root() {
  local d="${SCRIPT_DIR}"
  while [[ "${d}" != "/" ]]; do
    if [[ -d "${d}/src/camrod_bringup" ]]; then
      echo "${d}"
      return 0
    fi
    d="$(dirname "${d}")"
  done
  return 1
}

WS_ROOT="$(find_ws_root || true)"
if [[ -z "${WS_ROOT}" ]]; then
  echo "Unable to locate workspace root (expected src/camrod_bringup)" >&2
  exit 2
fi
WS_SRC="${WS_ROOT}/src"

# Source-of-truth policy (2026-03-18):
# - Runtime defaults are owned by camrod_bringup/config.
# - Package-level config mirrors below are kept identical for standalone runs.
# - nav2_lanelet.yaml is intentionally excluded because layered runtime now uses:
#     nav2_base + nav2_vehicle + nav2_lanelet_overlay + nav2_behavior.
pairs=(
  "camrod_map/config/lanelet_cost_grid.yaml camrod_bringup/config/map/lanelet_cost_grid.yaml"
  "camrod_planning/config/path_cost_grids.yaml camrod_bringup/config/planning/path_cost_grids.yaml"
  "camrod_perception/config/perception_params.yaml camrod_bringup/config/perception/perception_params.yaml"
)

status=0
for pair in "${pairs[@]}"; do
  left="${pair%% *}"
  right="${pair##* }"
  echo "[CHECK] ${left}  <=>  ${right}"
  if diff -u "${WS_SRC}/${left}" "${WS_SRC}/${right}" > /tmp/camrod_cfg_diff.$$; then
    echo "  OK: identical"
  else
    echo "  MISMATCH"
    cat /tmp/camrod_cfg_diff.$$
    status=1
  fi
  echo
  rm -f /tmp/camrod_cfg_diff.$$ || true
done

echo "[INFO] intentionally excluded from strict sync check:"
echo "  - camrod_planning/config/nav2_lanelet.yaml (legacy reference)"
echo "  - camrod_localization/config/kimera_bridge.yaml (site/environment-specific paths)"
echo

if [[ ${status} -ne 0 ]]; then
  echo "Config sync check FAILED"
  exit 1
fi

echo "Config sync check PASSED"
