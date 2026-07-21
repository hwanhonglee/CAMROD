builtin_interfaces/Time stamp
avg_msgs/ModuleState state

bool sim
bool rviz
# HH_260721 - Preserve the ESKF selection field for interface compatibility; EKF bringup leaves it false.
bool use_eskf
bool enable_goal_replanner
bool enable_state_machine
bool enable_module_validators

string map_path
float64 origin_lat
float64 origin_lon
float64 origin_alt

bool map_ready
bool sensing_ready
bool localization_ready
bool planning_ready
bool platform_ready
bool perception_ready
bool sensor_kit_ready
bool system_ready
