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
    scripts/visualization/render_automatic_recovery_results.py
    # HH_260806 - Render the body-hard-stop and recoverable-margin runtime proof.
    scripts/visualization/render_robot_boundary_validation.py
    # HH_260810 - Re-render historical boundary summaries with the active
    # rounded contour while preserving the recorded metrics as provenance.
    scripts/visualization/render_historical_boundary_summaries.py
    # HH_260810 - Render the active source-derived tapered/rounded contour and motion.
    scripts/visualization/render_tapered_rounded_boundary.py
    # HH_260810 - Overlay that contour on the measured map-v17 ROS road run.
    scripts/visualization/render_tapered_rounded_road_sim.py
    # HH_260810 - Reproduce current Park semantic operating coordinates.
    scripts/visualization/render_park_operating_points.py
    # HH_260806 - Render B1-B10 turnaround and B11-B13 roadside arrival evidence.
    scripts/visualization/render_camping_site_sequence_results.py
    # HH_260807 - Render the v2.1.5 repeated-service and safety test reports.
    scripts/visualization/render_v2_1_5_service_results.py
    # HH_260807 - Render the 3 km/h RPP lookahead full-service A/B decision.
    scripts/visualization/render_rpp_service_ab.py
    # HH_260804 - Keep the earlier manual-candidate geometry evidence reproducible.
    scripts/visualization/render_boundary_recovery_results.py
    # HH_260810 - Install the central package-documentation renderer too;
    # direct source and ros2-run workflows share one tool inventory.
    scripts/visualization/render_module_readme_assets.py
    # HH_260810 - Reproduce the measured UI transport and UUID-aware return
    # handoff summary media from their package-owned structured records.
    scripts/visualization/render_operator_transport_handoff_results.py
    # HH_260818 - Reproduce normal/crab, same-anchor return, and final-parking
    # release evidence from canonical YAML plus a B8 simulation report.
    scripts/visualization/render_v2_1_8_return_docking_results.py
    # HH_260819 - Reproduce the measured AMD64 Return/telemetry scheduler A/B.
    scripts/visualization/render_return_resource_profile.py
    # HH_260730 / TODOLIST 2 - Decode physical NvJPEG output during field acceptance.
    scripts/camera_payload_probe.py
    scripts/fake_sensor_publisher.py
    scripts/field_test_tool.sh
    # HH_260819 - Reproduce the live outbound Return motion-barrier timing.
    scripts/manual_return_preemption_probe.py
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
  # HH_260810 - Keep dependency setup/build wrappers and maintained shell
  # entrypoints parseable without executing host, CAN, Docker, or robot actions.
  ament_add_pytest_test(
    test_colcon_build_policy
    test/test_colcon_build_policy.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  ament_add_pytest_test(
    test_workspace_shell_tools
    test/test_workspace_shell_tools.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260807 - Every standalone package owns its default config.  Full
  # bringup carries byte-identical deployment mirrors; reject any future file,
  # comment, or value drift across the complete mirrored trees.
  ament_add_pytest_test(
    test_package_config_mirrors
    test/test_package_config_mirrors.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260807 - The canonical LiDAR preprocessor YAML must match the component
  # schema and feed both standalone and composed launch paths.
  ament_add_pytest_test(
    test_lidar_preprocessor_config_sync
    test/test_lidar_preprocessor_config_sync.py
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
  # HH_260810 - Keep the source-derived tapered/rounded PNG/GIF synchronized
  # with sensor-kit geometry and both deployed Nav2 footprints.
  ament_add_pytest_test(
    test_tapered_rounded_boundary_assets
    test/test_tapered_rounded_boundary_assets.py
    TIMEOUT 90
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260810 - Keep the map-v17 measured road PNG/GIF synchronized with its
  # raw ROS timeline and reject accidental physical-road evidence claims.
  ament_add_pytest_test(
    test_tapered_rounded_road_sim_assets
    test/test_tapered_rounded_road_sim_assets.py
    TIMEOUT 120
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260806 - Prevent the fake WGS84 inverse from masking GNSS lever-arm
  # regressions with a latitude-dependent northing error.
  ament_add_pytest_test(
    test_fake_gnss_projection
    test/test_fake_gnss_projection.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260806 - B1-B10 rotate inside their service area; constrained B11-B13
  # stop at the roadside without an on-site zero-turn.
  ament_add_pytest_test(
    test_camping_site_turnaround_contract
    test/test_camping_site_turnaround_contract.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260804 - Package diagrams must remain regenerable from the active
  # configuration and explicitly classified simulation evidence.
  ament_add_pytest_test(
    test_module_readme_assets
    test/test_module_readme_assets.py
    TIMEOUT 90
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260810 - Keep the post-v2.1.7 UI transport and system handoff media
  # tied to exact AMD64 measurements, config mirrors, and claim boundaries.
  ament_add_pytest_test(
    test_operator_transport_handoff_assets
    test/test_operator_transport_handoff_assets.py
    TIMEOUT 90
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260818 - Bind the v2.1.8 normal/crab, same-anchor campsite return,
  # final-parking slowdown, UI capture, and YAML mirrors to reproducible media.
  ament_add_pytest_test(
    test_v2_1_8_return_docking_assets
    test/test_v2_1_8_return_docking_assets.py
    TIMEOUT 90
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260825 - Keep v2.2.1 current-lane Return, charging dwell, and front
  # radar claims tied to the measured result and synchronized deployment YAML.
  ament_add_pytest_test(
    test_v2_2_1_safety_handoff_assets
    test/test_v2_2_1_safety_handoff_assets.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260819 - Lock camera-range docking, yaw-only stop, and charging handoff.
  ament_add_pytest_test(
    test_apriltag_docking_contract
    test/test_apriltag_docking_contract.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260819 - Bind UI restart recovery, captured lanelet exit and the
  # charging-only controller-status authorization across package boundaries.
  ament_add_pytest_test(
    test_drop_zone_departure_contract
    test/test_drop_zone_departure_contract.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260807 - Keep v2.1.5 service/obstacle/boundary visuals traceable to
  # the committed machine-readable simulation reports.
  ament_add_pytest_test(
    test_v2_1_5_service_results
    test/test_v2_1_5_service_results.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260810 - Bind current Park mission coordinates and their visual report
  # to the exact user-authored OSM and synchronized package configs.
  ament_add_pytest_test(
    test_park_operating_points_assets
    test/test_park_operating_points_assets.py
    TIMEOUT 90
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260810 - Validate the active Park map independently from preserved
  # historical map-v14/v15/v17 runtime evidence.
  ament_add_pytest_test(
    test_lanelet_map_revision_sync
    test/test_lanelet_map_revision_sync.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260805 - Keep all 24 low-rate system checkers in the intended 11/6/7
  # component groups while preserving their standalone debug executables.
  ament_add_pytest_test(
    test_system_checker_composition
    test/test_system_checker_composition.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260805 - Lock the production planner/controller scoped container and
  # standalone fallback while keeping the final command gate in camrod_control.
  ament_add_pytest_test(
    test_nav2_component_launch
    test/test_nav2_component_launch.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260805 - Keep Chromium, ROS intra-process composition, managed DDS SHM,
  # and the default-off LiDAR cost-grid diagnostic contract synchronized.
  ament_add_pytest_test(
    test_runtime_transport_contract
    test/test_runtime_transport_contract.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # Keep the full simulation graph coupled to external physics odometry while
  # preserving fail-closed validation and explicit platform-status ownership.
  ament_add_pytest_test(
    test_external_simulator_contract
    test/test_external_simulator_contract.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
endif()

ament_package()
