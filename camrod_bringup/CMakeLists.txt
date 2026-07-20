cmake_minimum_required(VERSION 3.8)
project(camrod_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(avg_msgs REQUIRED)  # enforce avg_msgs availability for bringup scripts.
find_package(rclcpp REQUIRED)

# HH_260630: Install simulation utilities used by bringup smoke tests and
# deterministic validation runs.
install(
  PROGRAMS
    scripts/fake_sensor_publisher.py
    scripts/field_test_tool.sh
    scripts/planning_scenario_leaderboard.py
    scripts/sim_validation_runner.py
    # HH_260623 - Removed sync_package_configs_to_bringup.sh from install; bringup
    # config now carries deployment overrides and must not be force-mirrored.
    scripts/gnss_dr_test_node.py
    scripts/goal_tolerance_probe_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# HH_260720 - Install launch sources without generated Python cache artifacts.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # HH_260721 - Keep bringup lint focused on maintained launch and simulation utilities.
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_pep257
  )
  ament_lint_auto_find_test_dependencies()

  # HH_260721 - Preserve the established quoting convention and validate owned Python paths.
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(
    launch scripts
    CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/test/ament_flake8.ini"
  )
  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257(launch scripts --add-ignore D213)
endif()

ament_package()
