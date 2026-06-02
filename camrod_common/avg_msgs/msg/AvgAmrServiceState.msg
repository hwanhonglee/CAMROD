# AMR service state with human-readable description.
# Published by camrod_ui (state=1,4) and external AMR nodes (state=0,3).

# State constants
int32 DROP_ZONE_WAIT=0
int32 MOVING_TO_SITE=1
int32 SITE_ARRIVED=2
int32 RETURNING_TO_DROP_ZONE=3
int32 GUEST_RECALL_SERVICE=4  # HJ_260601: Guest UI recall request state.

int32 state
string description
