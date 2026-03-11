cmake_minimum_required(VERSION 3.8)
project(vanjee_dbscan_bbox)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(dbscan_bbox_node src/dbscan_bbox_node.cpp)
ament_target_dependencies(dbscan_bbox_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
)

install(TARGETS dbscan_bbox_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()