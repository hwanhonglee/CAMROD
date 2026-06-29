# HH_260326: GNSS pose custom message scaffold (custom node/link integration will be added later)
avg_msgs/AvgHeader header

# HH_260326: pose in map frame (same convention as /sensing/gnss/pose)
avg_msgs/AvgPose pose

# HH_260326: covariance matrix in row-major order (6x6)
float64[36] covariance

# HH_260326: optional GNSS quality metadata
uint8 fix_type
uint8 num_satellites
float32 hdop
float32 vdop
