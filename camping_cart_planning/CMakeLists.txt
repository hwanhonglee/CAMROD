cmake_minimum_required(VERSION 3.8)
project(camping_cart_planning)

# HH_260105: Planning package now wraps Nav2 nodes/BT configs only.
# Legacy in-house planner/controller sources have been removed.

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(camping_cart_map REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)

# HH_260112 Add goal snapper node for lanelet-constrained goals.
add_executable(goal_snapper_node src/goal_snapper_node.cpp)
ament_target_dependencies(goal_snapper_node
  rclcpp
  geometry_msgs
  camping_cart_map
  lanelet2_core
  lanelet2_io
  lanelet2_projection
)
target_include_directories(goal_snapper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../camping_cart_map/include>
)

add_executable(centerline_snapper_node src/centerline_snapper_node.cpp)
ament_target_dependencies(centerline_snapper_node
  rclcpp
  geometry_msgs
  camping_cart_map
  lanelet2_core
  lanelet2_io
  lanelet2_projection
)
target_include_directories(centerline_snapper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../camping_cart_map/include>
)

# 2026-02-02: Bridge ComputePathToPose results to /planning/global_path for RViz.
add_executable(compute_path_bridge_node src/compute_path_bridge_node.cpp)
ament_target_dependencies(compute_path_bridge_node
  rclcpp
  nav_msgs
  nav2_msgs
  action_msgs
)

install(
  TARGETS goal_snapper_node centerline_snapper_node compute_path_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
