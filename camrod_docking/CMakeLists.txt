cmake_minimum_required(VERSION 3.8)
project(camrod_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)
find_package(avg_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(opennav_docking_core REQUIRED)
find_package(opennav_docking_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# HH_260528: Source renamed from parking_apriltag_bridge.cpp to docking_apriltag_bridge.cpp.
add_executable(docking_apriltag_bridge src/docking_apriltag_bridge.cpp)
ament_target_dependencies(docking_apriltag_bridge
  rclcpp
  geometry_msgs
  isaac_ros_apriltag_interfaces
  avg_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# CamrodDockingPlugin: pluginlib-exported ChargingDock implementation.
add_library(camrod_docking_plugin SHARED src/camrod_docking_plugin.cpp)
target_include_directories(camrod_docking_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(camrod_docking_plugin
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  opennav_docking_core
)
pluginlib_export_plugin_description_file(opennav_docking_core camrod_docking_plugin.xml)

add_executable(manual_dock_server_node src/manual_dock_server_node.cpp)
target_include_directories(manual_dock_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(manual_dock_server_node
  rclcpp
  rclcpp_action
  avg_msgs
  opennav_docking_msgs
)

add_executable(tag_distance_node src/tag_distance_node.cpp)
ament_target_dependencies(tag_distance_node
  rclcpp
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
)

add_executable(odom_yaw_corrector src/odom_yaw_corrector.cpp)
ament_target_dependencies(odom_yaw_corrector
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS
  docking_apriltag_bridge
  camrod_docking_plugin
  manual_dock_server_node
  tag_distance_node
  odom_yaw_corrector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES camrod_docking_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
