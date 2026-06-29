cmake_minimum_required(VERSION 3.8)
project(camrod_platform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# HH_260109 Platform package for CAN, wheel odometry, and vehicle state integration.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# HH_260617: Ranger BMS bridge publishes sensor_msgs/BatteryState from CAN 0x361.
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(camrod_sensor_kit REQUIRED)
find_package(avg_msgs REQUIRED)
find_package(ranger_msgs QUIET)

add_executable(robot_visualization_node
  src/robot_visualization_node.cpp
)
ament_target_dependencies(robot_visualization_node
  avg_msgs
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(robot_visualization_node
  camrod_sensor_kit::camrod_sensor_kit_lib
)

add_executable(cmd_vel_gate_node
  src/cmd_vel_gate_node.cpp
)
ament_target_dependencies(cmd_vel_gate_node
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
)

install(TARGETS
  robot_visualization_node
  cmd_vel_gate_node
  DESTINATION lib/${PROJECT_NAME}
)

# HH_260428: ranger_platform_bridge_node — built only when ranger_msgs is available
# (ranger_ros2 external cloned by build_camrod.sh). Bridges CAN-derived ranger topics
# to /platform/status/* with primary/fallback odom, id-based actuator state extraction,
# and comprehensive AvgPlatformStatus aggregation (avg_msgs, all DBC fields).
if(ranger_msgs_FOUND)
  add_executable(ranger_platform_bridge_node
    src/ranger_platform_bridge_node.cpp
  )
  ament_target_dependencies(ranger_platform_bridge_node
    rclcpp
    nav_msgs
    # HH_260617: Required by /platform/status/battery_state publisher.
    sensor_msgs
    geometry_msgs
    std_msgs
    ranger_msgs
    avg_msgs
  )
  install(TARGETS ranger_platform_bridge_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "ranger_msgs not found — ranger_platform_bridge_node will not be built")
endif()

install(DIRECTORY config launch scripts systemd
  DESTINATION share/${PROJECT_NAME})

ament_package()
