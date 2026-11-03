#!/usr/bin/env bash
# HH_261103 - Archive immutable refs into a new test workspace; never checkout, reset or rewrite an existing source tree.
set -euo pipefail
[[ $# == 1 ]] || { printf 'Usage: bash setup_overlay.sh /absolute/root.env\n' >&2; exit 2; }
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
set -a
source "$1"
source "${package_dir}/config/overlay_sources.env"
set +a
: "${CAMROD_NATIVE_REPO:?}" "${CAMROD_VIRTUAL_WS_ROOT:?}" "${CAMROD_SERVICE_WS_ROOT:?}"
[[ "${UNIT2_SHA}" =~ ^[0-9a-f]{40}$ ]] || { printf 'Native SHA pin must be filled before setup\n' >&2; exit 2; }
target="$(realpath -m -- "${CAMROD_SERVICE_WS_ROOT}")"
native_repo="$(realpath -- "${CAMROD_NATIVE_REPO}")"
virtual_ws="$(realpath -- "${CAMROD_VIRTUAL_WS_ROOT}")"
[[ "${CAMROD_SERVICE_WS_ROOT}" == /* && ! -e "${target}" && "${target}" != / ]] || exit 2
case "${target}/" in "${native_repo}/"*|"${virtual_ws}/"*) printf 'Test workspace must be outside source/underlay roots\n' >&2; exit 2;; esac
actual="$(git -C "${native_repo}" rev-parse "${UNIT2_REF}^{commit}")"
[[ "${actual}" == "${UNIT2_SHA}" ]] || { printf 'Native tag/SHA mismatch\n' >&2; exit 2; }
git -C "${virtual_ws}/src" merge-base --is-ancestor "${VIRTUAL_UNDERLAY_SHA}" HEAD
source_repo="$(git -C "${package_dir}" rev-parse --show-toplevel)"
[[ -z "$(git -C "${source_repo}" status --porcelain --untracked-files=all -- camrod_carla_service_test)" ]] || {
  printf 'Commit the reviewed test package before exporting it\n' >&2; exit 2;
}
mkdir -p -- "${target}/src"
git -C "${native_repo}" archive "${UNIT2_SHA}" | tar -x -C "${target}/src"
git -C "${source_repo}" archive HEAD camrod_carla_service_test | tar -x -C "${target}/src"
cd -- "${target}/src"
for patch in native_parking_compat.patch native_underlay_compat.patch; do
  git apply --no-index --check "${package_dir}/tools/${patch}"
done
for patch in native_parking_compat.patch native_underlay_compat.patch; do
  git apply --no-index "${package_dir}/tools/${patch}"
done
python3 - "${target}/overlay_sources.json" "${UNIT2_SHA}" "$(git -C "${source_repo}" rev-parse HEAD)" <<'PY'
import json, sys
with open(sys.argv[1], 'x') as stream:
    json.dump({'schema': 'camrod.service_overlay.sources.v1', 'native_sha': sys.argv[2],
               'virtual_package_sha': sys.argv[3], 'status': 'PREPARED_NOT_BUILT_NOT_VALIDATED'}, stream, indent=2)
PY
printf 'Prepared isolated source only: %s\n' "${target}"
