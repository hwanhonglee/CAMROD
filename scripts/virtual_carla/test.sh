#!/usr/bin/env bash
# Run source contracts and the built ROS package tests without starting CARLA.

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: test.sh [--source-only | --adapter-only | --all-camrod]

Default: source contracts, Ranger 4WS package tests, then the CAMROD packages
that participate directly in CARLA integration.

  --source-only   syntax/static virtual-CARLA contracts only
  --adapter-only  source contracts plus camrod_carla_adapter tests
  --all-camrod    source contracts, Ranger tests, then every built CAMROD test

No mode starts a server, spawns an actor, publishes a command, or sends a goal.
Run build.sh successfully before package-level test modes.
EOF
}

mode=integration
while [[ $# -gt 0 ]]; do
  case "$1" in
    --source-only) mode=source; shift ;;
    --adapter-only) mode=adapter; shift ;;
    --all-camrod) mode=all; shift ;;
    -h|--help) usage; exit 0 ;;
    *)
      printf '[virtual_carla] ERROR: unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${script_dir}/env.sh"

shell_tools=(env.sh setup.sh build.sh test.sh run.sh)
for tool in "${shell_tools[@]}"; do
  bash -n "${script_dir}/${tool}"
done

if find "${CAMROD_SRC_ROOT}" -name .gitmodules -print -quit | grep -q .; then
  virtual_carla_die \
    "submodule-like metadata is forbidden: remove every .gitmodules file"
  exit 1
fi
if git -C "${CAMROD_SRC_ROOT}" ls-files --stage | \
    awk '$1 == "160000" { found = 1 } END { exit(found ? 0 : 1) }'; then
  virtual_carla_die "mode-160000 Git links are forbidden in CAMROD"
  exit 1
fi

PYTHONDONTWRITEBYTECODE=1 python3 -m pytest \
  -q -p no:cacheprovider \
  "${CAMROD_SRC_ROOT}/camrod_carla_adapter/test/test_full_launch_contracts.py" \
  "${CAMROD_SRC_ROOT}/camrod_carla_adapter/test/test_virtual_carla_scripts.py" \
  "${CAMROD_SRC_ROOT}/camrod_bringup/test/test_colcon_build_policy.py" \
  "${CAMROD_SRC_ROOT}/camrod_bringup/test/test_workspace_shell_tools.py"

if [[ "${mode}" == "source" ]]; then
  virtual_carla_log "source contracts passed"
  exit 0
fi

virtual_carla_verify_external_prefixes
virtual_carla_source_ros true true
virtual_carla_verify_package_prefix \
  camrod_carla_adapter "${CAMROD_WS_ROOT}/install"

run_workspace_tests() {
  local workspace="$1"
  shift
  local packages=("$@") package
  (
    cd "${workspace}"
    colcon --log-base "${workspace}/log" test \
      --build-base "${workspace}/build" \
      --install-base "${workspace}/install" \
      --packages-select "${packages[@]}"
    for package in "${packages[@]}"; do
      if ! colcon test-result \
          --test-result-base "${workspace}/build/${package}" --verbose; then
        return 1
      fi
    done
  )
}

run_all_workspace_tests() {
  local workspace="$1"
  (
    cd "${workspace}"
    colcon --log-base "${workspace}/log" test \
      --build-base "${workspace}/build" \
      --install-base "${workspace}/install"
    if ! colcon test-result \
        --test-result-base "${workspace}/build" --verbose; then
      return 1
    fi
  )
}

if [[ "${mode}" == "adapter" ]]; then
  run_workspace_tests "${CAMROD_WS_ROOT}" camrod_carla_adapter
  exit 0
fi

run_workspace_tests "${RANGER_ROS_WS}" \
  carla_extended_ackermann_msgs \
  carla_extended_ackermann_control \
  rqt_extended_ackermann

if [[ "${mode}" == "all" ]]; then
  run_all_workspace_tests "${CAMROD_WS_ROOT}"
else
  run_workspace_tests "${CAMROD_WS_ROOT}" \
    camrod_carla_adapter \
    camrod_bringup \
    camrod_control \
    camrod_localization \
    camrod_map \
    camrod_planning \
    camrod_ui
fi

virtual_carla_log "offline test suite passed"
