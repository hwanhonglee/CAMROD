#!/bin/bash
OUT=/tmp/claude-1000/-home-nvidia-camrod-ws-src/f46cd451-3321-48bf-b4aa-3c2cbd2057db/scratchpad/hz_result4.txt
: > "$OUT"
source /opt/ros/humble/setup.bash
source /home/nvidia/camrod_ws/install/setup.bash

run_case () {
  local label="$1"; shift
  echo "############ $label ############" >> "$OUT"
  setsid ros2 launch camrod_sensing gnss.launch.py "$@" gnss_log_level:=warn \
    > /tmp/claude-1000/-home-nvidia-camrod-ws-src/f46cd451-3321-48bf-b4aa-3c2cbd2057db/scratchpad/launch_$label.log 2>&1 &
  local LP=$!
  sleep 20
  echo "-- nodes --" >> "$OUT"
  ros2 node list 2>/dev/null | tr '\n' ' ' >> "$OUT"; echo >> "$OUT"
  echo "-- fix hz --" >> "$OUT"
  timeout 14 ros2 topic hz /sensing/gnss/ublox_gps_node/fix 2>&1 | tail -4 >> "$OUT"
  echo "-- rxmrtcm hz --" >> "$OUT"
  timeout 8 ros2 topic hz /sensing/gnss/rxmrtcm 2>&1 | tail -2 >> "$OUT"
  kill -INT -$LP 2>/dev/null; sleep 6; kill -9 -$LP 2>/dev/null; sleep 3
  pkill -f ntrip_ros.py 2>/dev/null; pkill -f rtcm_serial_writer 2>/dev/null; sleep 2
}

run_case "ntripON_writerOFF" enable_ntrip:=true ublox_dual_base_rtcm_device:=
run_case "ntripOFF_baseline" enable_ntrip:=false

echo "DONE" >> "$OUT"
