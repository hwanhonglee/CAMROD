# HH_260617: CAMROD mission/state-machine status payload.
std_msgs/Header header

uint8 INIT=0
uint8 READY=1
uint8 WAIT_DZ=2
uint8 RUNNING=3
uint8 GOAL_REACHED=4
uint8 RECALLED=5
uint8 RETURNING=6
uint8 WARN_RECOVERY=7
uint8 ERROR_STOP=8

uint8 state
string label

uint8 scenario_id
string scenario_label

string active_mission_key
string active_goal_source

bool estop
bool return_requested
bool recall_requested
