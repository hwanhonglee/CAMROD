cmake_minimum_required(VERSION 3.8)
project(camrod_sensing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core build system
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Project message layer
find_package(avg_msgs REQUIRED)

# TF / geometry helpers used in cost-grid nodes
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Point cloud processing
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation sample_consensus search kdtree)
option(CAMROD_ENABLE_CAMERA_PUBLISHER "Build Jetson camera publisher node" ON)

# PCL exports optional compile definitions (e.g., Eigen alignment flags).
if(PCL_DEFINITIONS)
  add_definitions(${PCL_DEFINITIONS})
endif()

# ------------------------------------------------------------------------------
# LiDAR preprocessor
# ------------------------------------------------------------------------------
add_library(lidar_preprocessor_component SHARED src/lidar_preprocessor_node.cpp)
ament_target_dependencies(
  lidar_preprocessor_component
  rclcpp
  rclcpp_components
  avg_msgs
  pcl_conversions
)
target_include_directories(lidar_preprocessor_component PRIVATE ${PCL_INCLUDE_DIRS})
target_include_directories(lidar_preprocessor_component PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(
  lidar_preprocessor_component
  ${PCL_LIBRARIES}
  pcl_common
  pcl_filters
  pcl_sample_consensus
  pcl_segmentation
  pcl_search
  pcl_kdtree
)
rclcpp_components_register_node(
  lidar_preprocessor_component
  PLUGIN "camrod::sensing::LidarPreprocessorNode"
  EXECUTABLE lidar_preprocessor_node
)
install(
  TARGETS lidar_preprocessor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ------------------------------------------------------------------------------
# Camera preprocessor
# ------------------------------------------------------------------------------
# HH_260702 - Kept buildable as an optional relay/debug tool; normal camera
# launch uses econ front/rear publisher outputs directly.
add_executable(camera_preprocessor_node src/camera_preprocessor_node.cpp)
ament_target_dependencies(
  camera_preprocessor_node
  rclcpp
  avg_msgs
)

set(SENSING_NODE_TARGETS
  lidar_preprocessor_node
  camera_preprocessor_node
  platform_velocity_converter_node
  sen0592_radar_node
  radar_cost_grid_node
  lidar_cost_grid_node
  inflation_cost_grid_node
)

# ------------------------------------------------------------------------------
# Camera publisher (Jetson only: ARM + OpenCV + CUDA + VPI)
# ------------------------------------------------------------------------------
if(CAMROD_ENABLE_CAMERA_PUBLISHER)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
    find_package(OpenCV REQUIRED)
    find_package(CUDAToolkit REQUIRED)
    # Rear camera uses cv_bridge; front constructs sensor_msgs/Image directly to
    # avoid loading the system OpenCV ABI into its OpenCV 4.8 component process.
    find_package(cv_bridge REQUIRED)
    find_package(vpi REQUIRED
      PATHS
        /opt/nvidia/vpi3/lib/aarch64-linux-gnu/cmake
        /opt/nvidia/vpi2/lib/aarch64-linux-gnu/cmake
    )

    find_library(NVJPEG_LIB nvjpeg
      HINTS
        ${CUDAToolkit_LIBRARY_DIR}
        /usr/lib/aarch64-linux-gnu/nvidia # HH_260522 - can't find nvjpeg pkg, so add path
        /usr/lib/aarch64-linux-gnu # HH_260522 - can't find nvjpeg pkg, so add path
        /usr/local/cuda/targets/aarch64-linux/lib
        /usr/local/cuda/targets/x86_64-linux/lib
        /usr/local/cuda/lib64
      REQUIRED
    )

    # HH_260528: camera_publisher_component → camera_front_publisher_component (renamed with front node).
    add_library(camera_front_publisher_component SHARED src/camera_front_publisher_node.cpp)
    target_include_directories(camera_front_publisher_component PRIVATE include)
    target_include_directories(camera_front_publisher_component PRIVATE ${OpenCV_INCLUDE_DIRS})
    ament_target_dependencies(
      camera_front_publisher_component
      rclcpp
      rclcpp_components
      sensor_msgs
      avg_msgs
    )
    target_link_libraries(
      camera_front_publisher_component
      ${OpenCV_LIBRARIES}
      vpi
      CUDA::cudart
      ${NVJPEG_LIB}
    )

    rclcpp_components_register_node(
      camera_front_publisher_component
      PLUGIN "camrod::sensing::CameraFrontPublisherNode"
      EXECUTABLE camera_front_publisher_node
    )

    list(APPEND SENSING_NODE_TARGETS camera_front_publisher_node)

    install(
      TARGETS camera_front_publisher_component
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )

    # HH_260805 - Rear camera keeps its generated standalone executable while
    # supporting the rear-camera/rectification/AprilTag intra-process path.
    add_library(camera_rear_publisher_component SHARED src/camera_rear_publisher_node.cpp)
    ament_target_dependencies(
      camera_rear_publisher_component
      builtin_interfaces
      rclcpp
      rclcpp_components
      sensor_msgs
      cv_bridge
    )
    target_include_directories(camera_rear_publisher_component PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(
      camera_rear_publisher_component
      ${OpenCV_LIBRARIES}
    )
    rclcpp_components_register_node(
      camera_rear_publisher_component
      PLUGIN "camrod::sensing::CameraRearPublisherNode"
      EXECUTABLE camera_rear_publisher_node
    )
    install(
      TARGETS camera_rear_publisher_component
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )

    list(APPEND SENSING_NODE_TARGETS camera_rear_publisher_node)

  else()
    message(WARNING "Skipping camera_front/rear_publisher_node: non-ARM architecture (${CMAKE_SYSTEM_PROCESSOR})")
  endif()
endif()

# ------------------------------------------------------------------------------
# Platform velocity converter
# ------------------------------------------------------------------------------
add_executable(platform_velocity_converter_node src/platform_velocity_converter_node.cpp)
ament_target_dependencies(
  platform_velocity_converter_node
  rclcpp
  avg_msgs
)

# ------------------------------------------------------------------------------
# Radar sensor node
# ------------------------------------------------------------------------------
add_executable(sen0592_radar_node src/sen0592_radar_node.cpp)
target_include_directories(sen0592_radar_node PRIVATE include)
ament_target_dependencies(
  sen0592_radar_node
  rclcpp
  rcl_interfaces
  avg_msgs
  # HH_260720 - Link the explicit standard Range visualization boundary.
  sensor_msgs
)

# ------------------------------------------------------------------------------
# Radar cost grid
# ------------------------------------------------------------------------------
add_executable(radar_cost_grid_node src/radar_cost_grid_node.cpp)
target_include_directories(radar_cost_grid_node PRIVATE include)
ament_target_dependencies(
  radar_cost_grid_node
  rclcpp
  rcl_interfaces
  avg_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ------------------------------------------------------------------------------
# LiDAR cost grid
# ------------------------------------------------------------------------------
add_library(lidar_cost_grid_component SHARED src/lidar_cost_grid_node.cpp)
target_include_directories(lidar_cost_grid_component PRIVATE include)
ament_target_dependencies(
  lidar_cost_grid_component
  rclcpp
  rclcpp_components
  avg_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
rclcpp_components_register_node(
  lidar_cost_grid_component
  PLUGIN "camrod::sensing::LidarCostGridNode"
  EXECUTABLE lidar_cost_grid_node
)
install(
  TARGETS lidar_cost_grid_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ------------------------------------------------------------------------------
# Inflation cost grid (lanelet + LiDAR + Radar + global_path → /planning/cost_grid/inflation)
# ------------------------------------------------------------------------------
add_executable(inflation_cost_grid_node src/inflation_cost_grid_node.cpp)
ament_target_dependencies(
  inflation_cost_grid_node
  rclcpp
  avg_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(
  TARGETS ${SENSING_NODE_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# HH_260722 - Install Python sensor-boundary helpers launched by this package.
# HH_260729 - Disabled-hardware dummy publishers keep explicit transport and
# diagnostic contracts alive without opening physical sensor devices.
install(
  PROGRAMS
    scripts/rtcm_serial_writer.py
    scripts/radar_dummy_publisher.py
    # HH_260807 - Display only the seven real SEN0592 Range streams; this tool
    # never substitutes or publishes dummy radar data.
    scripts/radar_status_gui.py
    scripts/sensing_dummy_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # HH_260720 - Verify route-lanelet clipping and metric margin behavior
  # independently from live sensors and Lanelet2 map loading.
  ament_add_gtest(
    test_route_lanelet_cost_filter
    test/test_route_lanelet_cost_filter.cpp
  )
  target_include_directories(test_route_lanelet_cost_filter PRIVATE include)
  ament_target_dependencies(test_route_lanelet_cost_filter avg_msgs)
  # HH_260728 - Lock the measured per-sensor notch behavior independently of
  # TF, hardware, and the live cost-grid node.
  ament_add_gtest(
    test_radar_self_echo_filter
    test/test_radar_self_echo_filter.cpp
  )
  target_include_directories(test_radar_self_echo_filter PRIVATE include)
  # HH_260807 - Verify that the 0.10 m stop-candidate gate does not truncate
  # the driver's independent 0.50 m level-1 observation contract.
  ament_add_gtest(
    test_radar_stop_range_filter
    test/test_radar_stop_range_filter.cpp
  )
  target_include_directories(test_radar_stop_range_filter PRIVATE include)
  # HH_260729 - Lock the no-target value and per-channel dummy marker naming
  # used when one SEN0592 port is intentionally disabled.
  ament_add_gtest(
    test_radar_dummy_contract
    test/test_radar_dummy_contract.cpp
  )
  target_include_directories(test_radar_dummy_contract PRIVATE include)
  # Validate front-camera frame/JPEG boundaries without camera, CUDA, or decoding.
  ament_add_gtest(
    test_camera_front_contract
    test/test_camera_front_contract.cpp
  )
  target_include_directories(test_camera_front_contract PRIVATE include)
  # HH_260722 - Lock single/dual GNSS RTCM topic and USB-input launch behavior.
  # Disable unrelated user-site pytest plugins during configure-time discovery;
  # the current anyio plugin is incompatible with the ROS system pytest version.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(
    test_gnss_launch_routing
    test/test_gnss_launch_routing.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260729 - Keep disabled-hardware dummy outputs aligned with all seven
  # canonical radar and RViz topic boundaries from the SEN0592 YAML.
  ament_add_pytest_test(
    test_radar_dummy_publisher
    test/test_radar_dummy_publisher.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260807 - Lock the physical radar GUI's no-target classification, robot
  # axes, beam arcs, and all-seven-channel live YAML contract.
  ament_add_pytest_test(
    test_radar_status_gui
    test/test_radar_status_gui.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260729 - Validate low-rate GNSS/IMU/LiDAR/camera dummy schemas without
  # starting hardware drivers or a ROS graph.
  ament_add_pytest_test(
    test_sensing_dummy_publisher
    test/test_sensing_dummy_publisher.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  # HH_260729 - Evaluate the aggregate launch expressions so each disabled
  # physical group gets exactly one dummy and every real/external source wins.
  ament_add_pytest_test(
    test_sensing_dummy_launch
    test/test_sensing_dummy_launch.py
    TIMEOUT 30
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
endif()

ament_package()
