cmake_minimum_required(VERSION 3.8)
project(camping_cart_localization)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
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
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(yaml-cpp REQUIRED)

# HH_260123 Force legacy PYTHON_EXECUTABLE for rosidl_find_python interp checks.
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE} CACHE FILEPATH "Python executable for rosidl" FORCE)

# ------------------------------------------------------------------------------
# Messages
# ------------------------------------------------------------------------------
set(msg_files
  "msg/LocalizationMode.msg"
  "msg/LocalizationDiagnostics.msg"
  "msg/LocalizationStatus.msg"
)

# HH_260123 Explicitly mark rosidl interface group to satisfy rosidl_generate_interfaces check.
set(${PROJECT_NAME}_MEMBER_OF_GROUPS "rosidl_interface_packages")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# HH_260123 Typesupport handle for linking custom message users.
rosidl_get_typesupport_target(${PROJECT_NAME}__rosidl_typesupport_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ------------------------------------------------------------------------------
# Executables
# ------------------------------------------------------------------------------
# HH_260109 Removed gnss_imu_localizer_node (replaced by EKF fusion).

add_executable(navsat_to_pose_node
  src/navsat_to_pose_node.cpp
)

ament_target_dependencies(navsat_to_pose_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

# HH_260109 Bridge filtered odometry into pose topics for downstream consumers.
add_executable(odometry_to_pose_node
  src/odometry_to_pose_node.cpp
)

ament_target_dependencies(odometry_to_pose_node
  rclcpp
  geometry_msgs
  nav_msgs
)

# HH_260109 cost_field_node moved to camping_cart_visualizer.

add_executable(localization_health_monitor_node
  src/localization_health_monitor_node.cpp
)
ament_target_dependencies(localization_health_monitor_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

# HH_260123 Extended localization stack: ESKF + supervisor + wheel bridge.
add_executable(localization_eskf_node
  src/localization_eskf_node.cpp
)
ament_target_dependencies(localization_eskf_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
target_include_directories(localization_eskf_node PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
target_link_libraries(localization_eskf_node
  Eigen3::Eigen
  ${${PROJECT_NAME}__rosidl_typesupport_cpp}
)
add_dependencies(localization_eskf_node ${PROJECT_NAME}__rosidl_generator_cpp)

add_executable(localization_supervisor_node
  src/localization_supervisor_node.cpp
)
ament_target_dependencies(localization_supervisor_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)
target_include_directories(localization_supervisor_node PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
target_link_libraries(localization_supervisor_node
  ${${PROJECT_NAME}__rosidl_typesupport_cpp}
)
add_dependencies(localization_supervisor_node ${PROJECT_NAME}__rosidl_generator_cpp)

add_executable(localization_pose_selector_node
  src/localization_pose_selector_node.cpp
)
ament_target_dependencies(localization_pose_selector_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)
target_include_directories(localization_pose_selector_node PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
target_link_libraries(localization_pose_selector_node
  ${${PROJECT_NAME}__rosidl_typesupport_cpp}
)
add_dependencies(localization_pose_selector_node ${PROJECT_NAME}__rosidl_generator_cpp)

add_executable(wheel_odometry_bridge_node
  src/wheel_odometry_bridge_node.cpp
)
ament_target_dependencies(wheel_odometry_bridge_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)

# 2026-02-02: Drop zone matcher for initial localization status.
add_executable(drop_zone_matcher_node
  src/drop_zone_matcher_node.cpp
)
ament_target_dependencies(drop_zone_matcher_node
  rclcpp
  geometry_msgs
  std_msgs
)
target_link_libraries(drop_zone_matcher_node
  yaml-cpp
)

# 2026-02-09: Bridge Kimera CSV pose outputs into ROS fallback localization topics.
add_executable(kimera_csv_bridge_node
  external/kimera_vio_bridge/kimera_csv_bridge_node.cpp
)
ament_target_dependencies(kimera_csv_bridge_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

# HH_260109 Removed pose_mux_node (A/B switching deprecated).

# HH_260109 Removed centerline_fallback_node (A/B switching deprecated).

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(TARGETS
  navsat_to_pose_node
  odometry_to_pose_node
  localization_health_monitor_node
  localization_eskf_node
  localization_supervisor_node
  localization_pose_selector_node
  wheel_odometry_bridge_node
  drop_zone_matcher_node
  kimera_csv_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  msg
  DESTINATION share/${PROJECT_NAME}
)

# HH_260113 Rely on default ament_cmake environment hooks (manual package.dsv override removed).

# ------------------------------------------------------------------------------
# Testing
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
ament_package()
