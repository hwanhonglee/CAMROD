#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

find_ws_root() {
  local d="${SCRIPT_DIR}"
  while [[ "${d}" != "/" ]]; do
    if [[ -d "${d}/src/camping_cart_bringup" ]]; then
      echo "${d}"
      return 0
    fi
    d="$(dirname "${d}")"
  done
  return 1
}

WS_ROOT="$(find_ws_root || true)"
if [[ -z "${WS_ROOT}" ]]; then
  echo "Unable to locate workspace root (expected src/camping_cart_bringup)" >&2
  exit 2
fi
WS_SRC="${WS_ROOT}/src"

pairs=(
  "camping_cart_map/config/lanelet_cost_grid.yaml camping_cart_bringup/config/map/lanelet_cost_grid.yaml"
  "camping_cart_planning/config/path_cost_grids.yaml camping_cart_bringup/config/planning/path_cost_grids.yaml"
  "camping_cart_planning/config/nav2_lanelet.yaml camping_cart_bringup/config/planning/nav2_lanelet.yaml"
)

status=0
for pair in "${pairs[@]}"; do
  left="${pair%% *}"
  right="${pair##* }"
  echo "[CHECK] ${left}  <=>  ${right}"
  if diff -u "${WS_SRC}/${left}" "${WS_SRC}/${right}" > /tmp/camping_cart_cfg_diff.$$; then
    echo "  OK: identical"
  else
    echo "  MISMATCH"
    cat /tmp/camping_cart_cfg_diff.$$
    status=1
  fi
  echo
  rm -f /tmp/camping_cart_cfg_diff.$$ || true
done

if [[ ${status} -ne 0 ]]; then
  echo "Config sync check FAILED"
  exit 1
fi

echo "Config sync check PASSED"
