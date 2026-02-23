cmake_minimum_required(VERSION 3.8)
project(camping_cart_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# HH_260109 Build visualization-only nodes.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(camping_cart_map REQUIRED)
find_package(camping_cart_sensor_kit REQUIRED)

add_executable(robot_visualization_node src/robot_visualization_node.cpp)
ament_target_dependencies(robot_visualization_node
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(robot_visualization_node
  camping_cart_sensor_kit::camping_cart_sensor_kit_lib  # HH_260109 Link shared robot params helper.
)
target_include_directories(robot_visualization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(cost_field_marker_node src/cost_field_marker_node.cpp)
ament_target_dependencies(cost_field_marker_node
  rclcpp
  nav_msgs
  visualization_msgs
)

add_executable(cost_field_node src/cost_field_node.cpp)
# HH_260109 Depend on camping_cart_map for custom regulatory headers.
ament_target_dependencies(cost_field_node
  rclcpp
  geometry_msgs
  visualization_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  camping_cart_map
)
target_include_directories(cost_field_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../camping_cart_map/include
)

install(TARGETS
  robot_visualization_node
  cost_field_marker_node
  cost_field_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
