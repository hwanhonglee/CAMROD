cmake_minimum_required(VERSION 3.8)
project(camrod_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(avg_msgs REQUIRED)  # HH_260326: enforce avg_msgs availability for bringup scripts.
find_package(rclcpp REQUIRED)

# HH_260109 Install fake sensor publisher for simulation.
# HH_260422: Added gnss_dr_test_node.py for GNSS DR fallback integration test.
install(
  PROGRAMS
    scripts/fake_sensor_publisher.py
    scripts/planning_scenario_leaderboard.py
    scripts/sync_package_configs_to_bringup.sh
    scripts/gnss_dr_test_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

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
