#!/bin/bash
OUT=/tmp/claude-1000/-home-nvidia-camrod-ws-src/f46cd451-3321-48bf-b4aa-3c2cbd2057db/scratchpad/hz_result3.txt
: > "$OUT"
source /opt/ros/humble/setup.bash
source /home/nvidia/camrod_ws/install/setup.bash

setsid ros2 launch camrod_sensing gnss.launch.py enable_ntrip:=true gnss_log_level:=warn \
  > /tmp/claude-1000/-home-nvidia-camrod-ws-src/f46cd451-3321-48bf-b4aa-3c2cbd2057db/scratchpad/gnss_launch3.log 2>&1 &
LP=$!
sleep 22
echo "=== topics ===" >> "$OUT"
ros2 topic list 2>/dev/null | grep -i gnss >> "$OUT"
echo "=== fix subscribers ===" >> "$OUT"
ros2 topic info -v /sensing/gnss/ublox_gps_node/fix 2>/dev/null | head -30 >> "$OUT"
echo "--- fix (ntrip ON) ---" >> "$OUT"
timeout 14 ros2 topic hz /sensing/gnss/ublox_gps_node/fix >> "$OUT" 2>&1
echo "=== node cpu ===" >> "$OUT"
ps -o pid,pcpu,comm -C ublox_gps_node >> "$OUT" 2>&1
top -b -n 1 | head -14 >> "$OUT"
kill -INT -$LP 2>/dev/null
sleep 6
kill -9 -$LP 2>/dev/null
sleep 2
pgrep -c ublox_gps_node >> "$OUT" 2>&1
echo "DONE" >> "$OUT"
