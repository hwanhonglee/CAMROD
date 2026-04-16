builtin_interfaces/Time stamp
std_msgs/Header header

# HH_260409: Unified platform runtime status payload for CAN/state bridge users.
avg_msgs/ModuleState state

nav_msgs/Odometry odometry
geometry_msgs/TwistStamped velocity
geometry_msgs/TwistStamped wheel

bool estop
uint8 vehicle_state
uint8 control_mode
uint16 error_code
float64 battery_voltage
uint8 motion_mode

float32[] motor_rpm
float32[] motor_speed
float32[] motor_angle
