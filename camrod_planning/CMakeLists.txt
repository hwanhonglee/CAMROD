cmake_minimum_required(VERSION 3.8)
project(camrod_planning)

# HH_260105: Planning package now wraps Nav2 nodes/BT configs only.
# Legacy in-house planner/controller sources have been removed.

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(avg_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(camrod_map REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)

# HH_260112 Add goal snapper node for lanelet-constrained goals.
add_executable(goal_snapper_node src/goal_snapper_node.cpp)
ament_target_dependencies(goal_snapper_node
  avg_msgs
  rclcpp
  geometry_msgs
  camrod_map
  lanelet2_core
  lanelet2_io
  lanelet2_projection
)
target_include_directories(goal_snapper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../camrod_map/include>
)

add_executable(centerline_snapper_node src/centerline_snapper_node.cpp)
ament_target_dependencies(centerline_snapper_node
  avg_msgs
  rclcpp
  geometry_msgs
  camrod_map
  lanelet2_core
  lanelet2_io
  lanelet2_projection
)
target_include_directories(centerline_snapper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../camrod_map/include>
)

add_executable(goal_replanner_node src/goal_replanner_node.cpp)
ament_target_dependencies(goal_replanner_node
  avg_msgs
  rclcpp
  rclcpp_action
  action_msgs
  geometry_msgs
  nav2_msgs
)

add_executable(local_path_extractor_node src/local_path_extractor_node.cpp)
ament_target_dependencies(local_path_extractor_node
  avg_msgs
  rclcpp
  geometry_msgs
  nav_msgs
)

install(
  TARGETS goal_snapper_node centerline_snapper_node goal_replanner_node local_path_extractor_node
  DESTINATION lib/${PROJECT_NAME}
)

# HH_260312-00:00 Install planning diagnostics + state machine scripts.
install(
  PROGRAMS
    scripts/planning_state_machine_node.py
    scripts/planning_diagnostic_node.py
    scripts/nav2_lifecycle_startup_retry_node.py
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
