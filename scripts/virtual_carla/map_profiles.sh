#!/usr/bin/env bash
# Repository-controlled map identities for opt-in virtual-CARLA profiles.
# This file declares constants only; sourcing it never launches or mutates CARLA.

CAMROD_CARLA_SITE_ACCESS_PROFILE_ID="woraksan-camrod-site-geometry-v13"
CAMROD_CARLA_SITE_ACCESS_UE_MAP="/Game/map_package/Maps/Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v13/Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v13"
CAMROD_CARLA_SITE_ACCESS_TOWN="${CAMROD_CARLA_SITE_ACCESS_UE_MAP#/Game/}"

# Previous builder outputs remain explicitly selectable for replaying
# historical evidence.  New site-geometry runs default to v13.
CAMROD_CARLA_SITE_ACCESS_LEGACY_V12_PROFILE_ID="woraksan-camrod-site-geometry-v12"
CAMROD_CARLA_SITE_ACCESS_LEGACY_V12_UE_MAP="/Game/map_package/Maps/Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v12/Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v12"
CAMROD_CARLA_SITE_ACCESS_LEGACY_V12_TOWN="${CAMROD_CARLA_SITE_ACCESS_LEGACY_V12_UE_MAP#/Game/}"

CAMROD_CARLA_SITE_ACCESS_LEGACY_V11_PROFILE_ID="woraksan-camrod-site-geometry-v11"
CAMROD_CARLA_SITE_ACCESS_LEGACY_V11_UE_MAP="/Game/map_package/Maps/Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v11/Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v11"
CAMROD_CARLA_SITE_ACCESS_LEGACY_V11_TOWN="${CAMROD_CARLA_SITE_ACCESS_LEGACY_V11_UE_MAP#/Game/}"
