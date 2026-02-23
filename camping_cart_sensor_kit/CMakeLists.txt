cmake_minimum_required(VERSION 3.8)
project(camping_cart_sensor_kit)
# HH_260109 Sensor kit package (URDF/TF/specs) renamed from camping_cart_common.

###############################################
# 1. Find packages
###############################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

###############################################
# 2. Message generation
###############################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotSpecifications.msg"
  "msg/SensorPose.msg"
  "msg/RobotInfo.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)  # HH_260109 export headers for downstream packages

###############################################
# 3. Include directories
###############################################
include_directories(
  include
)

###############################################
# 4. Build shared robot params library AFTER msg is generated
###############################################
# HH_260109 Provide reusable robot params helper without requiring robot_info_node.
add_library(camping_cart_sensor_kit_lib
  src/robot_params.cpp
)
ament_target_dependencies(camping_cart_sensor_kit_lib
  rclcpp
)
target_include_directories(camping_cart_sensor_kit_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

# HH_260109 Robot visualization node moved to camping_cart_visualizer package.

###############################################
# 5. Install
###############################################
install(
  TARGETS camping_cart_sensor_kit_lib
  EXPORT camping_cart_sensor_kit_lib_targets  # HH_260109 export for downstream find_package
  DESTINATION lib  # HH_260109 install to standard lib/ so export can resolve it.
)

# HH_260109 Install export set so ament can resolve the library target.
install(
  EXPORT camping_cart_sensor_kit_lib_targets
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
###############################################
ament_export_targets(camping_cart_sensor_kit_lib_targets)  # HH_260109 expose export set.
ament_package()
