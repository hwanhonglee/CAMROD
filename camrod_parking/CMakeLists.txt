cmake_minimum_required(VERSION 3.8)
project(camrod_parking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(avg_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(parking_apriltag_bridge src/parking_apriltag_bridge.cpp)
ament_target_dependencies(parking_apriltag_bridge
  rclcpp
  geometry_msgs
  apriltag_msgs
  avg_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  parking_apriltag_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
