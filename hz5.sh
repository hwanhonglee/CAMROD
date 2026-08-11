#!/bin/bash
OUT=/tmp/claude-1000/-home-nvidia-camrod-ws-src/f46cd451-3321-48bf-b4aa-3c2cbd2057db/scratchpad/hz_result5.txt
: > "$OUT"
source /opt/ros/humble/setup.bash
source /home/nvidia/camrod_ws/install/setup.bash
SP=$(ros2 pkg prefix camrod_sensing)/share/camrod_sensing/config/gnss
SCRATCH=/tmp/claude-1000/-home-nvidia-camrod-ws-src/f46cd451-3321-48bf-b4aa-3c2cbd2057db/scratchpad

# ublox driver alone, NTRIP off
setsid ros2 launch camrod_sensing gnss.launch.py enable_ntrip:=false gnss_log_level:=warn \
  > $SCRATCH/launch_hz5_ublox.log 2>&1 &
UP=$!
sleep 20

echo "=== A: ublox alone ===" >> "$OUT"
timeout 12 ros2 topic hz /sensing/gnss/ublox_gps_node/fix 2>&1 | tail -3 >> "$OUT"

# B: add ntrip_client WITHOUT fix remap (no GGA feedback from fix)
setsid ros2 run ntrip_client ntrip_ros.py --ros-args -r __ns:=/sensing/gnss -r __node:=ntrip_client \
  --params-file $SP/ntrip_client.yaml -p rtcm_topic:=ntrip_client/rtcm \
  -r rtcm:=ntrip_client/rtcm -r /rtcm:=ntrip_client/rtcm \
  > $SCRATCH/launch_hz5_ntrip_nofix.log 2>&1 &
NP=$!
sleep 15
echo "=== B: + ntrip_client, fix NOT subscribed ===" >> "$OUT"
ros2 topic info /sensing/gnss/ublox_gps_node/fix 2>/dev/null >> "$OUT"
timeout 12 ros2 topic hz /sensing/gnss/ublox_gps_node/fix 2>&1 | tail -3 >> "$OUT"
kill -INT -$NP 2>/dev/null; sleep 4; kill -9 -$NP 2>/dev/null; pkill -f ntrip_ros.py; sleep 3

# C: ntrip_client WITH fix remap (GGA sent per fix)
setsid ros2 run ntrip_client ntrip_ros.py --ros-args -r __ns:=/sensing/gnss -r __node:=ntrip_client \
  --params-file $SP/ntrip_client.yaml -p rtcm_topic:=ntrip_client/rtcm \
  -r rtcm:=ntrip_client/rtcm -r /rtcm:=ntrip_client/rtcm \
  -r fix:=ublox_gps_node/fix -r /fix:=ublox_gps_node/fix \
  > $SCRATCH/launch_hz5_ntrip_fix.log 2>&1 &
NP2=$!
sleep 15
echo "=== C: + ntrip_client, fix SUBSCRIBED (GGA per fix) ===" >> "$OUT"
ros2 topic info /sensing/gnss/ublox_gps_node/fix 2>/dev/null >> "$OUT"
timeout 14 ros2 topic hz /sensing/gnss/ublox_gps_node/fix 2>&1 | tail -4 >> "$OUT"

kill -INT -$NP2 2>/dev/null; sleep 4; kill -9 -$NP2 2>/dev/null
kill -INT -$UP 2>/dev/null; sleep 6; kill -9 -$UP 2>/dev/null; sleep 2
pkill -f ntrip_ros.py; pkill -x ublox_gps_node; sleep 2
echo "DONE" >> "$OUT"
