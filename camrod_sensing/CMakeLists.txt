cmake_minimum_required(VERSION 3.8)
project(camrod_sensing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core build system
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

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

# ------------------------------------------------------------------------------
# Camera preprocessor
# ------------------------------------------------------------------------------
add_executable(camera_preprocessor_node src/camera_preprocessor_node.cpp)
ament_target_dependencies(
  camera_preprocessor_node
  rclcpp
  avg_msgs
)

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
  TARGETS
    lidar_preprocessor_node
    camera_preprocessor_node
    platform_velocity_converter_node
    sen0592_radar_node
    radar_cost_grid_node
    lidar_cost_grid_node
    inflation_cost_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
