# HH_260721 - Describe CAMROD service operation independently from a specific chassis model.

# Mission/service lifecycle states. These are operational states, not health severities.
int32 DROP_ZONE_WAIT=0
int32 MOVING_TO_SITE=1
int32 SITE_ARRIVED=2
int32 RETURNING_TO_DROP_ZONE=3
int32 GUEST_RECALL_SERVICE=4
int32 SITE_ENTRY=5
int32 UNLOAD_WAIT=6
int32 RECALL_TO_SITE_ROAD=7
int32 GUEST_LOADING_WAIT=8
int32 RETURN_WITH_CARGO=9
int32 DROP_ZONE_PARKING=10
# HH_260721 - Expose normal wait, charging, and departure phases without using health severity.
int32 WAITING_FOR_RETURN_REQUEST=11
int32 WAITING_FOR_CHARGING=12
int32 CHARGING=13
int32 DEPARTING_CHARGER=14
int32 DEPARTING_DROP_ZONE=15
# HH_260724 - Operator stop/cancel is neither parked nor actively servicing a site.
int32 OPERATOR_STOPPED=16

int32 state
string state_name
string description
