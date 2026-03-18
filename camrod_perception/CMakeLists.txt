cmake_minimum_required(VERSION 3.8)
project(camrod_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(avg_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(obstacle_fusion_node src/obstacle_fusion_node.cpp)
ament_target_dependencies(obstacle_fusion_node
  avg_msgs
  rclcpp
  sensor_msgs
  geometry_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_geometry
)

add_executable(obstacle_lidar_node src/obstacle_lidar_node.cpp)
ament_target_dependencies(obstacle_lidar_node
  avg_msgs
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
)

install(TARGETS
  obstacle_fusion_node
  obstacle_lidar_node
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/perception_diagnostic_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
