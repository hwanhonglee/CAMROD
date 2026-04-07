cmake_minimum_required(VERSION 3.8)
project(camrod_localization)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(avg_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(camrod_map REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(yaml-cpp REQUIRED)

# ------------------------------------------------------------------------------
# Executables
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# 1. Input adapters
#   - NavSatFix -> Pose / PoseWithCovariance
#   - Pose -> PoseWithCovariance
#   - Odometry -> Pose / PoseWithCovariance
#   - Wheel input -> unified wheel odometry
# ------------------------------------------------------------------------------
add_executable(localization_input_adapter_node
  src/localization_input_adapter_node.cpp
)
ament_target_dependencies(localization_input_adapter_node
  rclcpp
  avg_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

# ------------------------------------------------------------------------------
# 2. Monitor / supervisor
#   - Sensor freshness monitoring
#   - Filter diagnostics evaluation
#   - Mode / confidence / degraded-state publishing
# ------------------------------------------------------------------------------
add_executable(localization_monitor_node
  src/localization_monitor_node.cpp
)
ament_target_dependencies(localization_monitor_node
  rclcpp
  avg_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

# ------------------------------------------------------------------------------
# 3. Map helper
#   - Lanelet centerline snapping
#   - Drop-zone based initial pose matching
# ------------------------------------------------------------------------------
add_executable(localization_map_helper_node
  src/localization_map_helper_node.cpp
)
ament_target_dependencies(localization_map_helper_node
  rclcpp
  avg_msgs
  geometry_msgs
  std_msgs
  camrod_map
  lanelet2_core
  lanelet2_io
  lanelet2_projection
)
target_link_libraries(localization_map_helper_node
  yaml-cpp
)

# ------------------------------------------------------------------------------
# 4. ESKF core (kept as-is)
# ------------------------------------------------------------------------------
add_executable(localization_eskf_node
  src/localization_eskf_node.cpp
)
ament_target_dependencies(localization_eskf_node
  rclcpp
  avg_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
target_include_directories(localization_eskf_node PRIVATE
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(localization_eskf_node Eigen3::Eigen)

# ------------------------------------------------------------------------------
# 5. Pose selector (kept as-is)
# ------------------------------------------------------------------------------
add_executable(localization_pose_selector_node
  src/localization_pose_selector_node.cpp
)
ament_target_dependencies(localization_pose_selector_node
  rclcpp
  avg_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
)

# ------------------------------------------------------------------------------
# Optional external bridge kept as separate executable
# ------------------------------------------------------------------------------
add_executable(kimera_csv_bridge_node
  external/kimera_vio_bridge/kimera_csv_bridge_node.cpp
)
ament_target_dependencies(kimera_csv_bridge_node
  rclcpp
  avg_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(TARGETS
  localization_input_adapter_node
  localization_monitor_node
  localization_map_helper_node
  localization_eskf_node
  localization_pose_selector_node
  kimera_csv_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Testing
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
ament_package()
