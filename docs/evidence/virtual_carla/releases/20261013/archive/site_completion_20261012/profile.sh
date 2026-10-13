#!/usr/bin/env bash
# HH_261012 - One frozen CARLA-only profile for every site; no per-site switches.
export CAMROD_CARLA_CAMPING_SITES_YAML=/home/hong/camrod_ws/src/camrod_carla_adapter/config/camping_sites_b1_lower_entry_experimental.yaml
export CAMROD_CARLA_EXPERIMENTAL_ZERO_TURN_YAW_INTEGRAL=true
export RANGER_ENV_FILE=/home/hong/Downloads/ranger-carla-4ws-pipeline/.work/runtime/site_completion_20261012/candidate.env
export RANGER_RUNTIME_ROOT=/home/hong/Downloads/ranger-carla-4ws-pipeline/.work/runtime/site_completion_20261012
export RANGER_EVIDENCE_ROOT=/home/hong/Downloads/ranger-carla-4ws-pipeline/.work/evidence/site_completion_20261012
export RANGER_BASELINE_MANIFEST=/home/hong/Downloads/ranger-carla-4ws-pipeline/.work/evidence/ranger_ros_backend_gate.json
export RANGER_PHYSICAL_MANIFEST=${RANGER_EVIDENCE_ROOT}/ranger_physical_4ws_acceptance_gate.json
export CAMROD_CARLA_MISSION_RECORDER_QUOTA_BYTES=1073741824
export XDG_STATE_HOME=/home/hong/camrod_ws/state/virtual_carla/site_completion_20261012
export CAMROD_CARLA_MISSION_RECORDS_ROOT=${XDG_STATE_HOME}/mission_records
export ROS_LOG_DIR=/home/hong/camrod_ws/log/virtual_carla/site_completion_20261012
