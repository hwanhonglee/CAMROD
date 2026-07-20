# HH_260720 - Define one generated command contract for control-owned motion operations.
uint8 START=1
uint8 CANCEL=2
uint8 RETURN=3
uint8 ALIGN_FOR_PARKING=4
uint8 EXIT=5

avg_msgs/AvgHeader header
uint8 operation
string source
