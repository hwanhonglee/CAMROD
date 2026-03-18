cmake_minimum_required(VERSION 3.8)
project(camrod_sensor_kit)
# HH_260109 Sensor kit package (URDF/TF/specs) renamed from legacy package name.

###############################################
# 1. Find packages
###############################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(avg_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
ament_export_include_directories(include)  # HH_260109 export headers for downstream packages

###############################################
# 2. Include directories
###############################################
include_directories(
  include
)

###############################################
# 3. Build shared robot params library
###############################################
# HH_260109 Provide reusable robot params helper without requiring robot_info_node.
add_library(camrod_sensor_kit_lib
  src/robot_params.cpp
)
ament_target_dependencies(camrod_sensor_kit_lib
  rclcpp
)
target_include_directories(camrod_sensor_kit_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# HH_260224 Robot visualization node moved to camrod_platform package.

###############################################
# 4. Install
###############################################
install(
  TARGETS camrod_sensor_kit_lib
  EXPORT camrod_sensor_kit_lib_targets  # HH_260109 export for downstream find_package
  DESTINATION lib  # HH_260109 install to standard lib/ so export can resolve it.
)

# HH_260109 Install export set so ament can resolve the library target.
install(
  EXPORT camrod_sensor_kit_lib_targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/sensor_kit_diagnostic_node.py
  DESTINATION lib/${PROJECT_NAME}
)
###############################################
ament_export_targets(camrod_sensor_kit_lib_targets)  # HH_260109 expose export set.
ament_package()
