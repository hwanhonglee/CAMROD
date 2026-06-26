cmake_minimum_required(VERSION 3.8)
project(camrod_sensing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core build system
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

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
add_executable(lidar_preprocessor_node src/lidar_preprocessor_node.cpp)
ament_target_dependencies(
  lidar_preprocessor_node
  rclcpp
  avg_msgs
  pcl_conversions
)
target_include_directories(lidar_preprocessor_node PRIVATE ${PCL_INCLUDE_DIRS})
target_include_directories(lidar_preprocessor_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(
  lidar_preprocessor_node
  ${PCL_LIBRARIES}
  pcl_common
  pcl_filters
  pcl_sample_consensus
  pcl_segmentation
  pcl_search
  pcl_kdtree
)

set(SENSING_NODE_TARGETS
  lidar_preprocessor_node
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
    # HJ_260626: cv_bridge moved up so camera_front_publisher_component can depend on it.
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
      cv_bridge   # HJ_260626: front node now includes cv_bridge/cv_bridge.h
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

    # HH_260528: Rear camera publisher — econ_camera_node (OpenCV + GStreamer, no GPU).
    # HH_260601: removed yaml-cpp dependency; calibration now loaded from ROS parameters.
    # HJ_260626: find_package(cv_bridge) moved above the front component.
    add_executable(camera_rear_publisher_node src/camera_rear_publisher_node.cpp)
    ament_target_dependencies(
      camera_rear_publisher_node
      rclcpp
      sensor_msgs
      cv_bridge
    )
    target_include_directories(camera_rear_publisher_node PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(
      camera_rear_publisher_node
      ${OpenCV_LIBRARIES}
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
ament_target_dependencies(
  sen0592_radar_node
  rclcpp
  avg_msgs
)

# ------------------------------------------------------------------------------
# Radar cost grid
# ------------------------------------------------------------------------------
add_executable(radar_cost_grid_node src/radar_cost_grid_node.cpp)
ament_target_dependencies(
  radar_cost_grid_node
  rclcpp
  avg_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ------------------------------------------------------------------------------
# LiDAR cost grid
# ------------------------------------------------------------------------------
add_executable(lidar_cost_grid_node src/lidar_cost_grid_node.cpp)
ament_target_dependencies(
  lidar_cost_grid_node
  rclcpp
  avg_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
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

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
