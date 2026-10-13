#!/usr/bin/env bash
# HH_261007 - Keep observer segment variables inside this script: transient
# systemd ExecStart otherwise expands ${part} before the child shell receives it.
# This wrapper only starts passive recorders; it never sends robot commands.
set -euo pipefail
if [[ $# -ne 1 || ! -d "$1" ]]; then
  printf '%s\n' 'usage: record_sim_forward_segments.sh EXISTING_OUTPUT_DIRECTORY' >&2
  exit 64
fi
direction_output_dir=$(realpath -- "$1")
trap 'exit 130' INT TERM
for direction_part in {01..24}; do
  python3 tools/record_sim_forward_direction.py \
    --output "${direction_output_dir}/direction_segment_${direction_part}.jsonl" \
    --duration-s 900
done
