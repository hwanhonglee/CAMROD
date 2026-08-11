#!/bin/bash
OUT=/tmp/claude-1000/-home-nvidia-camrod-ws-src/f46cd451-3321-48bf-b4aa-3c2cbd2057db/scratchpad/hz_result6.txt
: > "$OUT"
source /opt/ros/humble/setup.bash
source /home/nvidia/camrod_ws/install/setup.bash
SCRATCH=/tmp/claude-1000/-home-nvidia-camrod-ws-src/f46cd451-3321-48bf-b4aa-3c2cbd2057db/scratchpad

# Full production topology: ublox + ntrip_client + moving_base_rtcm_writer
setsid ros2 launch camrod_sensing gnss.launch.py enable_ntrip:=true gnss_log_level:=warn \
  > $SCRATCH/launch_hz6.log 2>&1 &
LP=$!
sleep 22
echo "=== nodes ===" >> "$OUT"
ros2 node list 2>/dev/null | tr '\n' ' ' >> "$OUT"; echo >> "$OUT"
echo "=== fix hz (writer ON) ===" >> "$OUT"
timeout 14 ros2 topic hz /sensing/gnss/ublox_gps_node/fix 2>&1 | tail -4 >> "$OUT"
echo "=== rxmrtcm hz (writer ON) ===" >> "$OUT"
timeout 10 ros2 topic hz /sensing/gnss/rxmrtcm 2>&1 | tail -3 >> "$OUT"
echo "=== ntrip rtcm hz ===" >> "$OUT"
timeout 8 ros2 topic hz /sensing/gnss/ntrip_client/rtcm 2>&1 | tail -2 >> "$OUT"
echo "=== ublox node cpu ===" >> "$OUT"
top -b -n 1 | grep -E "ublox|PID" | head -4 >> "$OUT"
kill -INT -$LP 2>/dev/null; sleep 6; kill -9 -$LP 2>/dev/null; sleep 3
pkill -f ntrip_ros.py; pkill -f rtcm_serial_writer; pkill -x ublox_gps_node; sleep 2
echo "DONE" >> "$OUT"
