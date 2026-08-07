cmake_minimum_required(VERSION 3.8)
project(camrod_localization)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(avg_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(camrod_map REQUIRED)

find_package(yaml-cpp REQUIRED)

# ------------------------------------------------------------------------------
# Executables
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# 1. Input adapters
#   - NavSatFix -> Pose / PoseWithCovariance
#   - Pose -> PoseWithCovariance
#   - Odometry -> Pose / PoseWithCovariance
#   - Wheel input -> unified wheel odometry
# ------------------------------------------------------------------------------
add_executable(localization_input_adapter_node
  src/localization_input_adapter_node.cpp
)
target_include_directories(localization_input_adapter_node PRIVATE include)
ament_target_dependencies(localization_input_adapter_node
  rclcpp
  avg_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

# ------------------------------------------------------------------------------
# HH_260720 - Build the explicitly named localization health monitor.
# 2. Localization health monitor
#   - Sensor freshness monitoring
#   - Filter diagnostics evaluation
#   - Mode / confidence / degraded-state publishing
# ------------------------------------------------------------------------------
add_executable(localization_monitor_node
  src/localization_monitor_node.cpp
)
target_include_directories(localization_monitor_node PRIVATE include)
ament_target_dependencies(localization_monitor_node
  rclcpp
  avg_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

# ------------------------------------------------------------------------------
# 3. Map helper
#   - Lanelet centerline snapping
#   - Drop-zone based initial pose matching
# ------------------------------------------------------------------------------
add_executable(localization_map_helper_node
  src/localization_map_helper_node.cpp
)
ament_target_dependencies(localization_map_helper_node
  rclcpp
  avg_msgs
  geometry_msgs
  std_msgs
  camrod_map
  lanelet2_core
  lanelet2_io
  lanelet2_projection
)
target_link_libraries(localization_map_helper_node
  yaml-cpp
)

# ------------------------------------------------------------------------------
# HH_260721 - Keep one robot_localization EKF backend and remove the unused custom ESKF target.
# 4. Pose selector
# ------------------------------------------------------------------------------
add_executable(localization_pose_selector_node
  src/localization_pose_selector_node.cpp
)
target_include_directories(localization_pose_selector_node PRIVATE include)
ament_target_dependencies(localization_pose_selector_node
  rclcpp
  avg_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
)

# ------------------------------------------------------------------------------
# 5. GNSS reattach helper
#   - Detect persistent localization<->GNSS divergence
#   - Publish reset pose for EKF set_pose / optional avg initialpose path
# ------------------------------------------------------------------------------
add_executable(localization_gnss_reattach_node
  src/localization_gnss_reattach_node.cpp
)
ament_target_dependencies(localization_gnss_reattach_node
  rclcpp
  avg_msgs
  geometry_msgs
)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(TARGETS
  localization_input_adapter_node
  localization_monitor_node
  localization_map_helper_node
  localization_pose_selector_node
  localization_gnss_reattach_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Testing
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # HH_260721 - Lint owned localization code without re-linting vendored robot_localization.
  set(AMENT_LINT_AUTO_FILE_EXCLUDE external)
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cppcheck
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_uncrustify
    ament_cmake_xmllint
  )
  ament_lint_auto_find_test_dependencies()

  # HH_260729 - A disabled GNSS heartbeat must never become an ENU pose.
  ament_add_gtest(
    test_navsat_fix_validation
    test/test_navsat_fix_validation.cpp
  )
  target_include_directories(test_navsat_fix_validation PRIVATE include)
  ament_target_dependencies(test_navsat_fix_validation sensor_msgs)

  # HH_260806 - Verify heading-aware left-antenna to robot-center conversion.
  ament_add_gtest(
    test_gnss_lever_arm
    test/test_gnss_lever_arm.cpp
  )
  target_include_directories(test_gnss_lever_arm PRIVATE include)

  # HH_260807 - A short, GNSS-anchored EKF yaw delta may rotate only the
  # antenna lever arm while receiver heading is temporarily unavailable.
  ament_add_gtest(
    test_gnss_heading_fallback
    test/test_gnss_heading_fallback.cpp
  )
  target_include_directories(test_gnss_heading_fallback PRIVATE include)

  # HH_260807 - Average unique GNSS epochs so one delayed 5 Hz fix does not
  # immediately force DR_ONLY while a sustained former 1 Hz profile still does.
  ament_add_gtest(
    test_gnss_rate_window
    test/test_gnss_rate_window.cpp
  )
  target_include_directories(test_gnss_rate_window PRIVATE include)

  # HH_260730 - Prevent callback order from adding one EKF cycle of selected-pose lag.
  ament_add_gtest(
    test_pose_selector_freshness
    test/test_pose_selector_freshness.cpp
  )
  target_include_directories(test_pose_selector_freshness PRIVATE include)

  # HH_260721 - Scope C++, Python, and XML checks to this package's maintained files.
  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(src LANGUAGE c++)
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(
    src
    FILTERS "-legal/copyright" "-build/include_order"
    MAX_LINE_LENGTH 120
  )
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
    launch
    CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/test/ament_flake8.ini"
  )
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_uncrustify(src)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint(package.xml)

  # HH_260723 - robot_localization silently stops loading numbered sensor
  # inputs at the first gap, so verify GNSS position/yaw inputs remain contiguous.
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_ekf_sensor_inputs
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ekf_sensor_inputs.py
  )

  # HH_260807 - A RELPOSNED unavailable placeholder must not evict the latest
  # valid heading needed by the strict GNSS antenna lever-arm correction.
  add_test(
    NAME test_gnss_heading_cache_contract
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gnss_heading_cache_contract.py
  )
endif()

# ------------------------------------------------------------------------------
ament_package()
