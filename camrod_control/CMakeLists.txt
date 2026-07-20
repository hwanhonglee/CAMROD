cmake_minimum_required(VERSION 3.8)
project(camrod_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# HH_260720 - Build flattened Python control nodes and the C++ parking controller together.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(avg_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(apriltag_parking_controller_node
  # HH_260720 - Keep every runtime node in one source directory.
  src/apriltag_parking_controller_node.cpp
)
ament_target_dependencies(apriltag_parking_controller_node
  rclcpp
  avg_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

install(TARGETS apriltag_parking_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS src/cmd_vel_safety_gate_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cmd_vel_safety_gate_node
)
install(PROGRAMS src/camping_site_maneuver_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camping_site_maneuver_controller_node
)
install(PROGRAMS src/drop_zone_maneuver_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME drop_zone_maneuver_controller_node
)
install(PROGRAMS src/reverse_parking_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME reverse_parking_controller_node
)

# HH_260720 - Install only the two helper modules required by flattened runtime nodes.
install(FILES
  src/control_support.py
  src/parking_geometry.py
  DESTINATION lib/${PROJECT_NAME}
)

# HH_260720 - Keep runtime cache artifacts out of the installed control package.
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN ".pytest_cache" EXCLUDE
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # HH_260720 - Run both final gate behavior and parking geometry regression tests.
  ament_add_pytest_test(test_cmd_vel_safety_gate_logic test/test_cmd_vel_safety_gate_logic.py)
  ament_add_pytest_test(test_parking_geometry test/test_parking_geometry.py)
endif()

ament_package()
