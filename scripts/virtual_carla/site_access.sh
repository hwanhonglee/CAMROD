#!/usr/bin/env bash
# Select the isolated B1-B13 access-map cohort, then delegate to run.sh.

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Help is intentionally independent of the caller's runtime environment.  In
# particular, environment.env may still name the direct/default map; requiring
# it to match the site-access map just to print usage makes offline validation
# fail before any lifecycle command is selected.
case "${1:-}" in
  -h|--help|help)
    exec "${script_dir}/run.sh" --help
    ;;
esac

# shellcheck disable=SC1091
source "${script_dir}/map_profiles.sh"

reject_conflicting_value() {
  local name="$1" expected="$2" actual="${!1:-}"
  if [[ -n "${actual}" && "${actual}" != "${expected}" ]]; then
    printf '[virtual_carla] ERROR: site-access profile refuses conflicting %s: %s (expected %s)\n' \
      "${name}" "${actual}" "${expected}" >&2
    exit 2
  fi
}

selected_profile="${CAMROD_CARLA_MAP_PROFILE:-${CAMROD_CARLA_SITE_ACCESS_PROFILE_ID}}"
case "${selected_profile}" in
  "${CAMROD_CARLA_SITE_ACCESS_PROFILE_ID}")
    selected_ue_map="${CAMROD_CARLA_SITE_ACCESS_UE_MAP}"
    selected_town="${CAMROD_CARLA_SITE_ACCESS_TOWN}"
    ;;
  "${CAMROD_CARLA_SITE_ACCESS_LEGACY_V13_PROFILE_ID}")
    selected_ue_map="${CAMROD_CARLA_SITE_ACCESS_LEGACY_V13_UE_MAP}"
    selected_town="${CAMROD_CARLA_SITE_ACCESS_LEGACY_V13_TOWN}"
    ;;
  "${CAMROD_CARLA_SITE_ACCESS_LEGACY_V12_PROFILE_ID}")
    selected_ue_map="${CAMROD_CARLA_SITE_ACCESS_LEGACY_V12_UE_MAP}"
    selected_town="${CAMROD_CARLA_SITE_ACCESS_LEGACY_V12_TOWN}"
    ;;
  "${CAMROD_CARLA_SITE_ACCESS_LEGACY_V11_PROFILE_ID}")
    selected_ue_map="${CAMROD_CARLA_SITE_ACCESS_LEGACY_V11_UE_MAP}"
    selected_town="${CAMROD_CARLA_SITE_ACCESS_LEGACY_V11_TOWN}"
    ;;
  *)
    printf '[virtual_carla] ERROR: unsupported site-access map profile: %s\n' \
      "${selected_profile}" >&2
    exit 2
    ;;
esac

reject_conflicting_value CARLA_UE_MAP "${selected_ue_map}"
reject_conflicting_value CARLA_TOWN "${selected_town}"

export CAMROD_CARLA_MAP_PROFILE="${selected_profile}"
export CARLA_UE_MAP="${selected_ue_map}"
export CARLA_TOWN="${selected_town}"
export CAMROD_VIRTUAL_CARLA_ENTRYPOINT="${script_dir}/site_access.sh"

exec "${script_dir}/run.sh" "$@"
