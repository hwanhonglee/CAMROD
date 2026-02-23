# HH_260123 Supervisor-facing status (mode + confidence + sensor health).
std_msgs/Header header
camping_cart_localization/LocalizationMode mode
float32 confidence
bool gnss_ok
bool imu_ok
bool wheel_ok
float32 gnss_innovation_norm
float32 wheel_innovation_norm
