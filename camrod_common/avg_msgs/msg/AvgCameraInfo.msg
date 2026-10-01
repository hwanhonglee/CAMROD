# HH_260629 - Add a CAMROD-owned camera calibration and image geometry contract for module-to-module transport.
avg_msgs/AvgHeader header
uint32 height
uint32 width
string distortion_model
float64[] d
float64[9] k
float64[9] r
float64[12] p
uint32 binning_x
uint32 binning_y
avg_msgs/AvgRegionOfInterest roi
