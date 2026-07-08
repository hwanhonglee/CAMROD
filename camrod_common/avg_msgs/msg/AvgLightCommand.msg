# 260708: CAMROD exterior light command (headlight relay + WS2815 turn indicators).
# Published by camrod_platform/light_controller_node, consumed by mcu_serial_bridge_node.
avg_msgs/AvgHeader header

uint8 INDICATOR_OFF=0
uint8 INDICATOR_LEFT=1
uint8 INDICATOR_RIGHT=2
uint8 INDICATOR_HAZARD=3

bool headlight
uint8 indicator
# Decision source for diagnostics, e.g. "hazard:estop", "crab:left", "turn:right", "off".
string source
