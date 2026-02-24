cmake_minimum_required(VERSION 3.8)
project(camping_cart_sensing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# HH_260109 Build sensing nodes for LiDAR/camera preprocessing and velocity conversion.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(lidar_preprocessor_node src/lidar_preprocessor_node.cpp)
ament_target_dependencies(lidar_preprocessor_node
  rclcpp sensor_msgs geometry_msgs)

# HH_260109 Add camera preprocessor for raw-to-processed image streams.
add_executable(camera_preprocessor_node src/camera_preprocessor_node.cpp)
ament_target_dependencies(camera_preprocessor_node
  rclcpp sensor_msgs)

# HH_260109 Convert platform velocity + IMU into twist_with_covariance for downstream checks.
add_executable(platform_velocity_converter_node src/platform_velocity_converter_node.cpp)
ament_target_dependencies(platform_velocity_converter_node
  rclcpp geometry_msgs sensor_msgs)

# HH_260220
add_executable(sen0592_radar_node src/sen0592_radar_node.cpp)
ament_target_dependencies(sen0592_radar_node rclcpp sensor_msgs)

add_executable(radar_cost_grid_node src/radar_cost_grid_node.cpp)
ament_target_dependencies(radar_cost_grid_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs)

install(TARGETS
  lidar_preprocessor_node
  camera_preprocessor_node
  platform_velocity_converter_node
  sen0592_radar_node # HH_260220
  radar_cost_grid_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
