#!/usr/bin/env bash
set -euo pipefail

# HH_260326: Sync package config YAML files into camrod_bringup/config mirrors.
# - Copies recursively by module.
# - Keeps bringup-specific files that don't exist in source packages.
# - Excludes backup files such as "*copy_org*".

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_SRC="${WS_ROOT}/src"
BRINGUP_CFG="${WS_SRC}/camrod_bringup/config"

declare -a MAP=(
  "camrod_map:map"
  "camrod_planning:planning"
  "camrod_localization:localization"
  "camrod_perception:perception"
  "camrod_platform:platform"
  "camrod_sensor_kit:sensor_kit"
  "camrod_sensing:sensing"
  "camrod_system:system"
)

for item in "${MAP[@]}"; do
  src_pkg="${item%%:*}"
  dst_dir="${item##*:}"
  src_cfg="${WS_SRC}/${src_pkg}/config/"
  dst_cfg="${BRINGUP_CFG}/${dst_dir}/"

  if [[ ! -d "${src_cfg}" ]]; then
    echo "[SKIP] missing source config: ${src_cfg}"
    continue
  fi

  mkdir -p "${dst_cfg}"
  echo "[SYNC] ${src_pkg}/config -> camrod_bringup/config/${dst_dir}"

  rsync -a --prune-empty-dirs \
    --include='*/' \
    --exclude='*copy_org*' \
    --exclude='* (copy_org).yaml' \
    --include='*.yaml' \
    --exclude='*' \
    "${src_cfg}" "${dst_cfg}"
done

echo "[DONE] bringup config mirror sync completed."
