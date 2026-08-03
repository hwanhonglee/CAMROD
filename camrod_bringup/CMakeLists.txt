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
    # HH_260803 / TODOLIST 12 - Reproduce production-owned boundary recovery.
    scripts/automatic_route_recovery_probe.py
    scripts/render_automatic_recovery_results.py
    # HH_260804 - Keep the earlier manual-candidate geometry evidence reproducible.
    scripts/render_boundary_recovery_results.py
    # HH_260730 / TODOLIST 2 - Decode physical NvJPEG output during field acceptance.
    scripts/camera_payload_probe.py
    scripts/fake_sensor_publisher.py
    scripts/field_test_tool.sh
    # HH_260730 - Measure every real localization hand-off in one time window.
    scripts/pose_latency_probe.py
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

  # HH_260722 - Prevent bringup from overriding the verified sensing GNSS route
  # or deploying stale copies of the package-owned GNSS configuration.
  find_package(ament_cmake_pytest REQUIRED)
  # HH_260722 - Isolate package tests from incompatible user-site pytest plugins
  # during both configure-time discovery and CTest execution.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(
    test_gnss_defaults_sync
    test/test_gnss_defaults_sync.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260729 - Keep the seven-channel radar mapping and readable self-echo
  # profile byte-synchronized between sensing and full bringup deployments.
  ament_add_pytest_test(
    test_radar_defaults_sync
    test/test_radar_defaults_sync.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260729 - Keep non-radar hardware dummy heartbeat diagnostics explicit
  # and byte-synchronized between package and deployment checker profiles.
  ament_add_pytest_test(
    test_dummy_checker_config_sync
    test/test_dummy_checker_config_sync.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260729 - Lock aggregate real/dummy routing, including component-camera
  # ownership and the sim-wide no-auxiliary-dummy policy.
  ament_add_pytest_test(
    test_sensor_dummy_launch_routing
    test/test_sensor_dummy_launch_routing.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260730 - Prevent the field helper from silently publishing incompatible
  # gate messages or relying on a platform service that is not in this stack.
  ament_add_pytest_test(
    test_field_test_tool_contract
    test/test_field_test_tool_contract.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260730 - Keep latency percentile/rate calculations deterministic without
  # requiring a live ROS graph in unit tests.
  ament_add_pytest_test(
    test_pose_latency_probe
    test/test_pose_latency_probe.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260730 - Manual goals use RotationShim around RPP while UI goals use
  # RPP directly; lock their path-tracking parameters to one behavior.
  ament_add_pytest_test(
    test_nav2_controller_profile_sync
    test/test_nav2_controller_profile_sync.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260803 - Lock the axle-midpoint frame migration, sensor-coordinate
  # conversion, physical footprint invariants, and deployment config mirrors.
  ament_add_pytest_test(
    test_robot_center_frame_contract
    test/test_robot_center_frame_contract.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
endif()

ament_package()
