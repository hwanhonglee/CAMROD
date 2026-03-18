cmake_minimum_required(VERSION 3.8)
project(camping_cart_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
find_package(visualization_msgs REQUIRED)  # ✅ MarkerArray

# -------- obstacle_fusion_node --------
add_executable(obstacle_fusion_node src/obstacle_fusion_node.cpp)
ament_target_dependencies(obstacle_fusion_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_geometry
)

# -------- obstacle_lidar_node (DBSCAN+BBox) --------
add_executable(obstacle_lidar_node src/obstacle_lidar_node.cpp)
ament_target_dependencies(obstacle_lidar_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
)

install(TARGETS
  obstacle_fusion_node
  obstacle_lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()