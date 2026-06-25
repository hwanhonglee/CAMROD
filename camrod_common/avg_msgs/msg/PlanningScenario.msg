# HH_260617: CAMROD planning scenario command/status payload.
std_msgs/Header header

uint8 WAIT_DROP_ZONE=0
uint8 DELIVERY_TO_SITE=1
uint8 RETURN_TO_DROP_ZONE=2
uint8 RECALL_TO_SITE=3
# HH_260621 - Extended mission phases distinguish site entry from road-only recall.
uint8 SITE_ENTRY=4
uint8 UNLOAD_WAIT=5
uint8 RECALL_TO_SITE_ROAD=6
uint8 GUEST_LOADING_WAIT=7
uint8 RETURN_WITH_CARGO=8
uint8 DROP_ZONE_PARKING=9

uint8 scenario_id
string label
string source
