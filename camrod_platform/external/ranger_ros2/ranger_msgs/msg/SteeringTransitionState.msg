uint8 MOTION_MODE_DUAL_ACKERMAN = 0
uint8 MOTION_MODE_PARALLEL = 1
uint8 MOTION_MODE_SPINNING = 2
uint8 MOTION_MODE_SIDE_SLIP = 3

########################################

std_msgs/Header header

# Command mode selected from the incoming Twist.
uint8 command_mode

# True when target/limited steering and translation_scale apply to this command.
bool steering_angle_valid

# Original Twist received by ranger_base.
float64 requested_linear_x_mps
float64 requested_linear_y_mps
float64 requested_angular_z_radps

# Steering transition state in the Ranger SDK's inner-wheel command convention.
# Convert separately when comparing with a central bicycle-model steering angle.
float64 target_steering_rad
float64 limited_steering_rad
float64 steering_error_rad
float64 translation_scale

# Signed scalar speed passed to the Ranger SDK after transition scaling.
float64 commanded_speed_mps
