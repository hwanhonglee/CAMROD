builtin_interfaces/Time stamp
avg_msgs/AvgHeader header

# HH_260409: Unified platform runtime status payload for CAN/state bridge users.
avg_msgs/ModuleState state

avg_msgs/AvgOdometry odometry
avg_msgs/AvgTwistStamped velocity
avg_msgs/AvgTwistStamped wheel

bool estop
uint8 vehicle_state
uint8 control_mode
uint16 error_code
float64 battery_voltage
uint8 motion_mode

float32[] motor_rpm
float32[] motor_speed
float32[] motor_angle
