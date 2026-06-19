# HH_260617: CAMROD planning scenario command/status payload.
std_msgs/Header header

uint8 WAIT_DROP_ZONE=0
uint8 DELIVERY_TO_SITE=1
uint8 RETURN_TO_DROP_ZONE=2
uint8 RECALL_TO_SITE=3

uint8 scenario_id
string label
string source
