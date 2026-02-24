cmake_minimum_required(VERSION 3.8)
project(camping_cart_platform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# HH_260109 Platform package placeholder (CAN, wheel odometry, vehicle state).
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(camping_cart_sensor_kit REQUIRED)

add_executable(robot_visualization_node
  src/robot_visualization_node.cpp
)
ament_target_dependencies(robot_visualization_node
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(robot_visualization_node
  camping_cart_sensor_kit::camping_cart_sensor_kit_lib
)

install(TARGETS
  robot_visualization_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
