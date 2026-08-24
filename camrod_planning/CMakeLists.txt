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
  std_msgs
  nav_msgs
  camrod_map
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
)
target_include_directories(goal_snapper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
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
  # HH_260720 - Publish generated route lanelet and turn-segment metadata.
  avg_msgs
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
  # HH_260720 - Internal localization readiness uses generated CAMROD messages.
  avg_msgs
  nav2_msgs
  std_srvs
)

# HH_260618: Keep Nav2 FollowPath active while operators inspect a route before engage.
add_library(engage_aware_progress_checker SHARED
  src/engage_aware_progress_checker.cpp
)
ament_target_dependencies(engage_aware_progress_checker
  # HH_260720 - Read the generated control command state inside the Nav2 plugin.
  avg_msgs
  geometry_msgs
  nav2_core
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(nav2_core progress_checker_plugins.xml)

install(
  # HH_260721 - Keep maintained target names readable and within the CMake lint limit.
  TARGETS
    goal_snapper_node
    centerline_snapper_node
    goal_replanner_node
    local_path_extractor_node
    path_tracking_error_node
    nav2_lifecycle_startup_retry_node
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
    scripts/planning_state_machine_node.py
    scripts/planning_progress_node.py
    scripts/nav2_selector_latch_node.py
    scripts/path_visualizer_node.py
    scripts/obstacle_replan_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# HH_260720 - Exclude generated Python caches from planning runtime resources.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# HH_260720 - Install the bundled SmacLattice primitives so launch files never depend on a user home path.
install(
  DIRECTORY external/nav2_smac_planner/lattice_primitives/sample_primitives
  DESTINATION share/${PROJECT_NAME}/lattice_primitives
  # HH_260720 - Do not package caches generated while validating lattice tools.
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  # HH_260824 - Active campsite YAML is consumed only by the native map
  # regression; runtime planning binaries retain their existing dependency set.
  find_package(yaml-cpp REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # HH_260721 - Lint CAMROD-owned planning files without re-linting vendored Nav2 sources.
  set(AMENT_LINT_AUTO_FILE_EXCLUDE external)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cppcheck
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_lint_cmake
    ament_cmake_pep257
    ament_cmake_uncrustify
    ament_cmake_xmllint
  )
  ament_lint_auto_find_test_dependencies()

  # HH_260721 - Apply the repository's maintained Python and C++ style to owned paths only.
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(src LANGUAGE c++)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake(CMakeLists.txt)
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
    launch scripts
    CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/test/ament_flake8.ini"
  )
  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257(launch scripts --add-ignore D213)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(src)
  find_package(ament_cmake_xmllint REQUIRED)
  file(GLOB_RECURSE CAMROD_PLANNING_XML CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/config/*.xml"
  )
  ament_xmllint(
    package.xml
    global_planner_plugins.xml
    progress_checker_plugins.xml
    ${CAMROD_PLANNING_XML}
  )

  # HH_260721 - Exercise the installed planner algorithm against the active campground map.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lanelet_route_planner test/test_lanelet_route_planner.cpp)
  ament_target_dependencies(test_lanelet_route_planner
    action_msgs
    avg_msgs
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
  target_compile_definitions(
    test_lanelet_route_planner PRIVATE CAMROD_PLANNING_GOAL_SNAPPER_NO_MAIN
  )
  target_include_directories(test_lanelet_route_planner PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../camrod_map/include>
  )

  # HH_260824 - Keep B1-B13 map geometry in a small native target. The main
  # route-planner test compiles two production translation units and is too
  # expensive to duplicate merely to validate static map/YAML contracts.
  ament_add_gtest(
    test_active_campsite_geometry
    test/test_active_campsite_geometry.cpp
  )
  ament_target_dependencies(test_active_campsite_geometry
    action_msgs
    avg_msgs
    camrod_map
    geometry_msgs
    lanelet2_core
    lanelet2_io
    lanelet2_projection
    lanelet2_routing
    lanelet2_traffic_rules
    nav_msgs
    rclcpp
    std_msgs
  )
  target_compile_definitions(
    test_active_campsite_geometry PRIVATE CAMROD_PLANNING_GOAL_SNAPPER_NO_MAIN
  )
  target_include_directories(test_active_campsite_geometry PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../camrod_map/include>
  )
  target_link_libraries(test_active_campsite_geometry yaml-cpp)

  # HH_260729 - Exercise bounded Nav2 goal recovery without a running map,
  # action server, or control gate.
  ament_add_gtest(test_route_goal_recovery test/test_route_goal_recovery.cpp)
  target_include_directories(test_route_goal_recovery PRIVATE include)

  find_package(ament_cmake_test REQUIRED)
  ament_add_test(
    test_nav2_selector_policy
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_nav2_selector_policy.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  ament_add_test(
    test_planning_runtime_coalescing
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_planning_runtime_coalescing.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_package()
