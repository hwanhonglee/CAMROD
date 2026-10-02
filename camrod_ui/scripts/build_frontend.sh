#!/usr/bin/env bash
# HH_261002 - Build away from the served/symlinked tree so a running Robot UI
# never loses all public images while react-scripts empties its output folder.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
frontend_dir="$(cd "${script_dir}/../camrod_ui_robot/assets/frontend" && pwd)"
build_stage="$(mktemp -d "${frontend_dir}/.ui-build.XXXXXX")"
cleanup() {
  # HH_261002 - Remove only this invocation's generated temporary output.
  if [[ "${build_stage}" == "${frontend_dir}/.ui-build."* && -d "${build_stage}" ]]; then
    rm -rf -- "${build_stage}"
  fi
}
trap cleanup EXIT
cd "${frontend_dir}"
export REACT_APP_RANGER_ASSET_REV
REACT_APP_RANGER_ASSET_REV="$(sha256sum public/models/ranger-navigation.glb \
  public/models/woraksan-side-wrap.png public/models/woraksan-front-wrap.png \
  public/models/woraksan-rear-wrap.png | sha256sum | cut -c1-16)"
BUILD_PATH="${build_stage}" node node_modules/react-scripts/scripts/build.js
CAMROD_FRONTEND_BUILD_SOURCE="${build_stage}" bash "${script_dir}/sync_frontend_build.sh"
