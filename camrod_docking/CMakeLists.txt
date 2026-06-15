cmake_minimum_required(VERSION 3.8)
project(camrod_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)
find_package(avg_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(opennav_docking_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# AprilTag bridge: camera frame PoseStamped 직접 발행 (Nova Carter 패턴)
# TF 변환은 SimpleChargingDock 이 내부 처리 → tf2 의존성 불필요
add_executable(docking_apriltag_bridge src/docking_apriltag_bridge.cpp)
ament_target_dependencies(docking_apriltag_bridge
  rclcpp
  geometry_msgs
  isaac_ros_apriltag_interfaces
  avg_msgs
)

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

add_executable(odom_yaw_corrector src/odom_yaw_corrector.cpp)
ament_target_dependencies(odom_yaw_corrector
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS
  docking_apriltag_bridge
  manual_dock_server_node
  odom_yaw_corrector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
