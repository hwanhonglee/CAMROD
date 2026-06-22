# AMR service state with human-readable description.
# Published by camrod_ui (state=1,4) and external AMR nodes (state=0,3).

# State constants
int32 DROP_ZONE_WAIT=0
int32 MOVING_TO_SITE=1
int32 SITE_ARRIVED=2
int32 RETURNING_TO_DROP_ZONE=3
int32 GUEST_RECALL_SERVICE=4  # HJ_260601: Guest UI recall request state.
# HHL_260621 - Extended service lifecycle states for reservation-aware campsite operations.
int32 SITE_ENTRY=5
int32 UNLOAD_WAIT=6
int32 RECALL_TO_SITE_ROAD=7
int32 GUEST_LOADING_WAIT=8
int32 RETURN_WITH_CARGO=9
int32 DROP_ZONE_PARKING=10

int32 state
string description
