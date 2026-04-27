#!/usr/bin/env bash
set -euo pipefail

# Unified config sync/check script for package <-> bringup mirrors.
# - Sync mode: copies package config YAML into camrod_bringup/config/<module>.
# - Check mode: verifies source YAML and mirrored YAML are identical.
# - Keeps bringup-only extra files (does not delete mirror-only files).

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

usage() {
  cat <<'EOF'
Usage:
  sync_package_configs_to_bringup.sh [sync|check|all]

Modes:
  sync   Copy YAML configs from each package to camrod_bringup/config mirrors (default)
  check  Verify mirrored files are identical to package configs
  all    Run sync, then check
EOF
}

MODE="${1:-sync}"
if [[ "${MODE}" == "-h" || "${MODE}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ "${MODE}" != "sync" && "${MODE}" != "check" && "${MODE}" != "all" ]]; then
  echo "[ERROR] invalid mode: ${MODE}" >&2
  usage
  exit 2
fi

WS_ROOT="$(find_ws_root || true)"
if [[ -z "${WS_ROOT}" ]]; then
  echo "[ERROR] Unable to locate workspace root (expected src/camrod_bringup)" >&2
  exit 2
fi

WS_SRC="${WS_ROOT}/src"
BRINGUP_CFG="${WS_SRC}/camrod_bringup/config"
TMP_DIFF="$(mktemp -t camrod_cfg_diff.XXXXXX)"
trap 'rm -f "${TMP_DIFF}"' EXIT

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

list_yaml_relpaths() {
  local base_dir="$1"
  (cd "${base_dir}" && find . -type f -name '*.yaml' \
    ! -name '*copy_org*' \
    ! -name '* (copy_org).yaml' \
    | sed 's#^\./##' | sort)
}

run_sync() {
  local item src_pkg dst_dir src_cfg dst_cfg
  for item in "${MAP[@]}"; do
    src_pkg="${item%%:*}"
    dst_dir="${item##*:}"
    src_cfg="${WS_SRC}/${src_pkg}/config/"
    dst_cfg="${BRINGUP_CFG}/${dst_dir}/"

    if [[ ! -d "${src_cfg}" ]]; then
      echo "[SYNC][SKIP] missing source config: ${src_cfg}"
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
  echo "[SYNC][DONE] bringup config mirror sync completed."
}

run_check() {
  local item src_pkg dst_dir src_cfg dst_cfg rel src_file dst_file
  local status=0
  local missing_count=0
  local mismatch_count=0
  local matched_count=0

  for item in "${MAP[@]}"; do
    src_pkg="${item%%:*}"
    dst_dir="${item##*:}"
    src_cfg="${WS_SRC}/${src_pkg}/config"
    dst_cfg="${BRINGUP_CFG}/${dst_dir}"

    if [[ ! -d "${src_cfg}" ]]; then
      echo "[CHECK][SKIP] missing source config: ${src_cfg}"
      continue
    fi
    if [[ ! -d "${dst_cfg}" ]]; then
      echo "[CHECK][MISS] missing bringup mirror directory: ${dst_cfg}"
      status=1
      continue
    fi

    echo "[CHECK] ${src_pkg}/config <=> camrod_bringup/config/${dst_dir}"
    while IFS= read -r rel; do
      src_file="${src_cfg}/${rel}"
      dst_file="${dst_cfg}/${rel}"
      if [[ ! -f "${dst_file}" ]]; then
        echo "  [MISS] ${dst_file}"
        status=1
        missing_count=$((missing_count + 1))
        continue
      fi
      if diff -u "${src_file}" "${dst_file}" > "${TMP_DIFF}"; then
        matched_count=$((matched_count + 1))
      else
        echo "  [DIFF] ${rel}"
        sed -n '1,120p' "${TMP_DIFF}"
        echo "  [DIFF] (truncated to first 120 lines)"
        status=1
        mismatch_count=$((mismatch_count + 1))
      fi
    done < <(list_yaml_relpaths "${src_cfg}")
  done

  echo
  echo "[CHECK][SUMMARY] matched=${matched_count} missing=${missing_count} mismatched=${mismatch_count}"
  if [[ "${status}" -ne 0 ]]; then
    echo "[CHECK][FAIL] Config mirror check failed."
    return 1
  fi
  echo "[CHECK][PASS] Config mirror check passed."
  return 0
}

case "${MODE}" in
  sync)
    run_sync
    ;;
  check)
    run_check
    ;;
  all)
    run_sync
    run_check
    ;;
esac
