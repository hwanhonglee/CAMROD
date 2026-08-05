cmake_minimum_required(VERSION 3.8)
project(camrod_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# HH_260326: system package unifies robot_diagnostics checkers into camrod_system.
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(avg_msgs REQUIRED)
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
  find_package(yaml REQUIRED)
  set(CAMROD_SYSTEM_YAML_LIB yaml)
else()
  set(CAMROD_SYSTEM_YAML_LIB yaml-cpp)
endif()

# HH_260326: ranger_msgs is optional; build platform checker only when available.
find_package(ranger_msgs QUIET)

set(CAMROD_SYSTEM_COMMON_DEPS
  rclcpp
  diagnostic_msgs
  diagnostic_updater
  map_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  action_msgs
  lifecycle_msgs
  unique_identifier_msgs
  vision_msgs
  avg_msgs
)

include_directories(include)

function(camrod_system_add_checker target source_file)
  add_executable(${target} ${source_file})
  target_compile_features(${target} PUBLIC cxx_std_17)
  ament_target_dependencies(${target} ${CAMROD_SYSTEM_COMMON_DEPS})
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

# HH_260326: migrated checker executables (former robot_diagnostics_* packages).
camrod_system_add_checker(hw_checker_node src/diagnostics/hw_checker_node.cpp)
target_link_libraries(hw_checker_node stdc++fs)
camrod_system_add_checker(gpu_checker_node src/diagnostics/gpu_checker_node.cpp)
camrod_system_add_checker(network_checker_node src/diagnostics/network_checker_node.cpp)

camrod_system_add_checker(gnss_checker_node src/diagnostics/gnss_checker_node.cpp)
camrod_system_add_checker(imu_checker_node src/diagnostics/imu_checker_node.cpp)
camrod_system_add_checker(lidar_checker_node src/diagnostics/lidar_checker_node.cpp)
camrod_system_add_checker(radar_checker_node src/diagnostics/radar_checker_node.cpp)
camrod_system_add_checker(camera_checker_node src/diagnostics/camera_checker_node.cpp)
camrod_system_add_checker(wheel_odometry_checker_node src/diagnostics/wheel_odometry_checker_node.cpp)
camrod_system_add_checker(cost_grid_checker_node src/diagnostics/cost_grid_checker_node.cpp)
camrod_system_add_checker(velocity_converter_checker_node src/diagnostics/velocity_converter_checker_node.cpp)

camrod_system_add_checker(localization_gnss_checker_node src/diagnostics/localization_gnss_checker_node.cpp)
camrod_system_add_checker(localization_mode_checker_node src/diagnostics/localization_mode_checker_node.cpp)
camrod_system_add_checker(localization_pose_checker_node src/diagnostics/localization_pose_checker_node.cpp)
camrod_system_add_checker(localization_init_checker_node src/diagnostics/localization_init_checker_node.cpp)
camrod_system_add_checker(localization_source_checker_node src/diagnostics/localization_source_checker_node.cpp)
camrod_system_add_checker(localization_lanelet_checker_node src/diagnostics/localization_lanelet_checker_node.cpp)

camrod_system_add_checker(map_cost_grid_checker_node src/diagnostics/map_cost_grid_checker_node.cpp)
camrod_system_add_checker(perception_obstacle_checker_node src/diagnostics/perception_obstacle_checker_node.cpp)

camrod_system_add_checker(planning_lifecycle_checker_node src/diagnostics/planning_lifecycle_checker_node.cpp)
camrod_system_add_checker(planning_costmap_checker_node src/diagnostics/planning_costmap_checker_node.cpp)
camrod_system_add_checker(planning_nav_status_checker_node src/diagnostics/planning_nav_status_checker_node.cpp)
camrod_system_add_checker(planning_path_checker_node src/diagnostics/planning_path_checker_node.cpp)
# HH_260617: Validate semantic planning state before parking/control progression.
camrod_system_add_checker(planning_state_checker_node src/diagnostics/planning_state_checker_node.cpp)

# HH_260805 - Keep component libraries and standalone executables for every
# checker. Humble full-stack shutdown tests make standalone executables the
# production default; these libraries retain an explicit bench/future option.
function(camrod_system_add_checker_component_library target)
  add_library(${target} SHARED ${ARGN})
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_definitions(${target} PRIVATE CAMROD_SYSTEM_COMPONENT_BUILD)
  ament_target_dependencies(
    ${target}
    ${CAMROD_SYSTEM_COMMON_DEPS}
    rclcpp_components
  )
  install(
    TARGETS ${target}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endfunction()

camrod_system_add_checker_component_library(
  camrod_system_hardware_sensing_checkers
  src/diagnostics/hw_checker_node.cpp
  src/diagnostics/gpu_checker_node.cpp
  src/diagnostics/network_checker_node.cpp
  src/diagnostics/gnss_checker_node.cpp
  src/diagnostics/imu_checker_node.cpp
  src/diagnostics/lidar_checker_node.cpp
  src/diagnostics/radar_checker_node.cpp
  src/diagnostics/camera_checker_node.cpp
  src/diagnostics/wheel_odometry_checker_node.cpp
  src/diagnostics/cost_grid_checker_node.cpp
  src/diagnostics/velocity_converter_checker_node.cpp
)
target_link_libraries(camrod_system_hardware_sensing_checkers stdc++fs)
rclcpp_components_register_nodes(
  camrod_system_hardware_sensing_checkers
  "HwCheckerNode"
  "GpuCheckerNode"
  "NetworkCheckerNode"
  "GnssCheckerNode"
  "ImuCheckerNode"
  "LidarCheckerNode"
  "RadarCheckerNode"
  "CameraCheckerNode"
  "WheelOdometryCheckerNode"
  "CostGridCheckerNode"
  "VelocityConverterCheckerNode"
)

camrod_system_add_checker_component_library(
  camrod_system_localization_checkers
  src/diagnostics/localization_gnss_checker_node.cpp
  src/diagnostics/localization_mode_checker_node.cpp
  src/diagnostics/localization_pose_checker_node.cpp
  src/diagnostics/localization_init_checker_node.cpp
  src/diagnostics/localization_source_checker_node.cpp
  src/diagnostics/localization_lanelet_checker_node.cpp
)
rclcpp_components_register_nodes(
  camrod_system_localization_checkers
  "LocalizationGnssCheckerNode"
  "LocalizationModeCheckerNode"
  "LocalizationPoseCheckerNode"
  "LocalizationInitCheckerNode"
  "LocalizationSourceCheckerNode"
  "LocalizationLaneletCheckerNode"
)

camrod_system_add_checker_component_library(
  camrod_system_autonomy_checkers
  src/diagnostics/map_cost_grid_checker_node.cpp
  src/diagnostics/perception_obstacle_checker_node.cpp
  src/diagnostics/planning_lifecycle_checker_node.cpp
  src/diagnostics/planning_costmap_checker_node.cpp
  src/diagnostics/planning_nav_status_checker_node.cpp
  src/diagnostics/planning_path_checker_node.cpp
  src/diagnostics/planning_state_checker_node.cpp
)
rclcpp_components_register_nodes(
  camrod_system_autonomy_checkers
  "MapCostGridCheckerNode"
  "PerceptionObstacleCheckerNode"
  "PlanningLifecycleCheckerNode"
  "PlanningCostmapCheckerNode"
  "PlanningNavStatusCheckerNode"
  "PlanningPathCheckerNode"
  "camrod_system::PlanningStateChecker"
)

# HH_260727: Exercise Nav2 goal UUID/state abort deduplication without starting
# the full navigation stack.
if(BUILD_TESTING)
  # HH_260730 - Required graph readiness has a bounded STARTING grace and
  # rejects zero/multiple owners of alternative capabilities deterministically.
  add_executable(
    test_graph_readiness
    test/test_graph_readiness.cpp
  )
  target_compile_features(test_graph_readiness PUBLIC cxx_std_17)
  add_test(
    NAME graph_readiness
    COMMAND test_graph_readiness
  )

  # HH_260728 - Verify physical sensor metadata survives stale conversion and
  # remains compact enough for the multi-sensor SYSTEM console summary.
  add_executable(
    test_diagnostic_detail
    test/test_diagnostic_detail.cpp
  )
  target_compile_features(test_diagnostic_detail PUBLIC cxx_std_17)
  ament_target_dependencies(
    test_diagnostic_detail
    diagnostic_msgs
  )
  add_test(
    NAME diagnostic_detail
    COMMAND test_diagnostic_detail
  )

  # HH_260729 - Keep low-FPS diagnostics from being relabeled by later
  # resolution/encoding checks of equal or lower severity.
  add_executable(
    test_camera_diagnostic_status
    test/test_camera_diagnostic_status.cpp
  )
  target_compile_features(test_camera_diagnostic_status PUBLIC cxx_std_17)
  ament_target_dependencies(
    test_camera_diagnostic_status
    diagnostic_msgs
  )
  add_test(
    NAME camera_diagnostic_status
    COMMAND test_camera_diagnostic_status
  )

  # HH_260729 - Verify that only a fresh explicit true dummy heartbeat can
  # downgrade an intentionally disabled hardware source to WARN.
  add_executable(
    test_dummy_source_monitor
    test/test_dummy_source_monitor.cpp
  )
  target_compile_features(test_dummy_source_monitor PUBLIC cxx_std_17)
  ament_target_dependencies(
    test_dummy_source_monitor
    rclcpp
  )
  add_test(
    NAME dummy_source_monitor
    COMMAND test_dummy_source_monitor
  )

  add_executable(
    test_planning_nav_status_tracker
    test/test_planning_nav_status_tracker.cpp
  )
  target_compile_features(test_planning_nav_status_tracker PUBLIC cxx_std_17)
  target_include_directories(
    test_planning_nav_status_tracker
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/diagnostics
  )
  ament_target_dependencies(
    test_planning_nav_status_tracker
    action_msgs
    unique_identifier_msgs
  )
  add_test(
    NAME planning_nav_status_tracker
    COMMAND test_planning_nav_status_tracker
  )
endif()

add_executable(aggregator_node src/diagnostics/aggregator_node.cpp)
target_compile_features(aggregator_node PUBLIC cxx_std_17)
ament_target_dependencies(aggregator_node rclcpp diagnostic_msgs)
target_link_libraries(aggregator_node ${CAMROD_SYSTEM_YAML_LIB})
install(TARGETS aggregator_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(system_checker_node src/system_checker_node.cpp)
target_compile_features(system_checker_node PUBLIC cxx_std_17)
ament_target_dependencies(system_checker_node
  rclcpp
  diagnostic_msgs
)
install(TARGETS system_checker_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(system_diagnostic_node src/system_diagnostic_node.cpp)
target_compile_features(system_diagnostic_node PUBLIC cxx_std_17)
ament_target_dependencies(system_diagnostic_node
  rclcpp
  diagnostic_msgs
  avg_msgs
)
install(TARGETS system_diagnostic_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(diagnostics_aggregator_node src/diagnostics_aggregator_node.cpp)
target_compile_features(diagnostics_aggregator_node PUBLIC cxx_std_17)
ament_target_dependencies(diagnostics_aggregator_node
  rclcpp
  diagnostic_msgs
)
install(TARGETS diagnostics_aggregator_node
  DESTINATION lib/${PROJECT_NAME})

if(ranger_msgs_FOUND)
  add_executable(ranger_platform_checker_node src/diagnostics/ranger_platform_checker_node.cpp)
  target_compile_features(ranger_platform_checker_node PUBLIC cxx_std_17)
  ament_target_dependencies(ranger_platform_checker_node
    rclcpp
    ranger_msgs
    nav_msgs
    sensor_msgs
    diagnostic_msgs
    diagnostic_updater
    avg_msgs
  )
  install(TARGETS ranger_platform_checker_node
    DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "HH_260326: ranger_msgs not found. ranger_platform_checker_node will not be built.")
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  # HH_260805 - Source-tree imports in contract tests may create Python cache;
  # deployment must install maintained launch sources only.
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
