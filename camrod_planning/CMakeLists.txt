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
find_package(rclcpp_lifecycle REQUIRED)
find_package(action_msgs REQUIRED)
find_package(avg_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(camrod_map REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)

# HH_260112 Add goal snapper node for lanelet-constrained goals.
add_executable(goal_snapper_node src/goal_snapper_node.cpp)
ament_target_dependencies(goal_snapper_node
  avg_msgs
  action_msgs
  rclcpp
  geometry_msgs
  nav_msgs
  camrod_map
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
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

# HH_260619: Nav2 global-planner plugin that produces lanelet centerline
# routes. This is the primary planner for campground lane-following; Smac/NavFn
# remain selectable fallbacks for free-space diagnostics and recovery tests.
add_library(lanelet_route_planner SHARED
  src/lanelet_route_planner.cpp
)
ament_target_dependencies(lanelet_route_planner
  camrod_map
  geometry_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  nav2_core
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)
target_include_directories(lanelet_route_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../camrod_map/include>
)
pluginlib_export_plugin_description_file(nav2_core global_planner_plugins.xml)

# HH_260330: Publish lateral/heading tracking error against local/global path.
add_executable(path_tracking_error_node src/path_tracking_error_node.cpp)
ament_target_dependencies(path_tracking_error_node
  avg_msgs
  rclcpp
)

add_executable(nav2_lifecycle_startup_retry_node src/nav2_lifecycle_startup_retry_node.cpp)
ament_target_dependencies(nav2_lifecycle_startup_retry_node
  rclcpp
  geometry_msgs
  nav2_msgs
  std_msgs
  std_srvs
)

# HH_260618: Keep Nav2 FollowPath active while operators inspect a route before engage.
add_library(engage_aware_progress_checker SHARED
  src/engage_aware_progress_checker.cpp
)
ament_target_dependencies(engage_aware_progress_checker
  geometry_msgs
  nav2_core
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)
pluginlib_export_plugin_description_file(nav2_core progress_checker_plugins.xml)

install(
  TARGETS goal_snapper_node centerline_snapper_node goal_replanner_node local_path_extractor_node path_tracking_error_node nav2_lifecycle_startup_retry_node
  DESTINATION lib/${PROJECT_NAME}
)

# HH_260618: Install engage-aware Nav2 progress checker plugin library.
install(
  TARGETS engage_aware_progress_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# HH_260619: Install LaneletRoutePlanner plugin library for Nav2 planner_server.
install(
  TARGETS lanelet_route_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# HH_260312-00:00 Install planning status_stream + state machine scripts.
install(
  PROGRAMS
    scripts/planning_cmd_vel_gate_node.py
    scripts/planning_state_machine_node.py
    scripts/planning_progress_node.py
    scripts/nav2_selector_latch_node.py
    scripts/path_visualizer_node.py
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
