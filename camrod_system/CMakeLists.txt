cmake_minimum_required(VERSION 3.8)
project(camrod_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# HH_260326: system package unifies robot_diagnostics checkers into camrod_system.
find_package(rclcpp REQUIRED)
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
