cmake_minimum_required(VERSION 3.8)
project(camping_cart_map)

#---------------------------------------------
# HH_260114 Base setup: use C++17.
#---------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#---------------------------------------------
# HH_260114 Find required ROS2 / Lanelet2 packages.
#---------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)

#---------------------------------------------
# HH_260114 Include directories for libraries/nodes.
#---------------------------------------------
include_directories(
  include
)

#---------------------------------------------
# HH_260114 Lanelet2 Map Loader library (shared across nodes).
#---------------------------------------------
add_library(camping_cart_map_lib
  src/lanelet2_map_loader.cpp
)

# HH_260114 Library dependencies.
ament_target_dependencies(camping_cart_map_lib
  rclcpp
  lanelet2_core
  lanelet2_io
  lanelet2_projection
)

#---------------------------------------------
# HH_260114 Executable node: lanelet2_map_node.
#---------------------------------------------
add_executable(lanelet2_map_node
  src/lanelet2_map_node.cpp
)

# HH_260114 Link node to Map Loader library.
target_link_libraries(lanelet2_map_node
  camping_cart_map_lib
)

# HH_260114 Node dependencies.
ament_target_dependencies(lanelet2_map_node
  rclcpp
  std_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  lanelet2_core
  lanelet2_io
  lanelet2_projection
)

# HH_251230: Nav2 costmap layer plugin
add_library(lanelet_cost_layer SHARED
  src/cost_map/lanelet_cost_layer.cpp
)
ament_target_dependencies(lanelet_cost_layer
  nav2_costmap_2d
  pluginlib
  rclcpp
  nav_msgs
)
target_include_directories(lanelet_cost_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
pluginlib_export_plugin_description_file(nav2_costmap_2d plugin_cost_map.xml)

# HH_260109 Removed cost_grid_publisher (test-only, replaced by lanelet_cost_grid_node).

# HH_260114 Lanelet-based OccupancyGrid generator (no LiDAR required).
add_executable(lanelet_cost_grid_node
  src/cost_map/lanelet_cost_grid_node.cpp
)
ament_target_dependencies(lanelet_cost_grid_node
  rclcpp
  nav_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_include_directories(lanelet_cost_grid_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 2026-01-30: Drop zone exporter (Lanelet type -> YAML list).
add_executable(drop_zone_exporter_node
  src/drop_zone_exporter_node.cpp
)
ament_target_dependencies(drop_zone_exporter_node
  rclcpp
  lanelet2_core
  lanelet2_io
  lanelet2_projection
)
target_link_libraries(drop_zone_exporter_node
  camping_cart_map_lib
)

# 2026-02-24: Visualization nodes are owned by map/platform modules (no visualizer package).
add_executable(cost_field_marker_node
  src/cost_field_marker_node.cpp
)
ament_target_dependencies(cost_field_marker_node
  rclcpp
  nav_msgs
  visualization_msgs
)

add_executable(cost_field_node
  src/cost_field_node.cpp
)
ament_target_dependencies(cost_field_node
  rclcpp
  geometry_msgs
  visualization_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
)

#---------------------------------------------
# HH_260114 Install executables and libraries.
#---------------------------------------------
install(TARGETS
  camping_cart_map_lib
  lanelet2_map_node
  lanelet_cost_layer
  lanelet_cost_grid_node
  drop_zone_exporter_node
  cost_field_marker_node
  cost_field_node
  DESTINATION lib/${PROJECT_NAME}
)

#---------------------------------------------
# HH_260114 Install headers (include/${PROJECT_NAME}/... per ROS2 convention).
#---------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

#---------------------------------------------
# HH_260114 Export include dirs so downstream packages can find custom rules.
#---------------------------------------------
ament_export_include_directories(include)

#---------------------------------------------
# HH_260114 Install config / launch / rviz directories (optional to avoid failures).
#---------------------------------------------
# HH_260109 Install directories and plugin XML separately (XML is a file, not a directory).
install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)
install(FILES
  plugin_cost_map.xml
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

#---------------------------------------------
# HH_260114 Declare ROS2 package.
#---------------------------------------------
ament_package()
