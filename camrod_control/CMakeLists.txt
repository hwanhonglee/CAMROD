cmake_minimum_required(VERSION 3.8)
project(camrod_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# HH_260721 - Build native C++ control nodes from shared policy and geometry headers.
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(avg_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

# HH_260721 - Compile the final velocity gate with explicit command and motion-cost stop components.
add_executable(cmd_vel_safety_gate_node
  src/cmd_vel_safety_gate_node.cpp
  src/motion_cost_stop.cpp
)
ament_target_dependencies(cmd_vel_safety_gate_node
  rclcpp
  rcl_interfaces
  avg_msgs
  # HH_260721 - Resolve shared control diagnostics included by boundary helpers.
  diagnostic_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(cmd_vel_safety_gate_node yaml-cpp)

add_executable(apriltag_parking_controller_node
  # HH_260720 - Keep every runtime node in one source directory.
  src/apriltag_parking_controller_node.cpp
)

# HH_260721 - Require CAN charging confirmation in the native reverse controller.
add_executable(reverse_parking_controller_node
  src/reverse_parking_controller_node.cpp
)
ament_target_dependencies(reverse_parking_controller_node
  rclcpp
  avg_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
)
target_link_libraries(reverse_parking_controller_node yaml-cpp)

# HH_260721 - Keep drop-zone departure and parking alignment native and explicit.
add_executable(drop_zone_maneuver_controller_node
  src/drop_zone_maneuver_controller_node.cpp
)
ament_target_dependencies(drop_zone_maneuver_controller_node
  rclcpp
  avg_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
)
target_link_libraries(drop_zone_maneuver_controller_node yaml-cpp)

# HH_260803 / TODOLIST 12 - Keep bounded route recovery separate from the
# final safety gate and from campsite/drop-zone maneuver ownership.
add_executable(route_safety_recovery_controller_node
  src/route_safety_recovery_controller_node.cpp
)
ament_target_dependencies(route_safety_recovery_controller_node
  rclcpp
  avg_msgs
)

# HH_260721 - Compile the complete campsite maneuver state machine natively.
add_executable(camping_site_maneuver_controller_node
  src/camping_site_maneuver_controller_node.cpp
)
ament_target_dependencies(camping_site_maneuver_controller_node
  rclcpp
  action_msgs
  avg_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
)
target_link_libraries(camping_site_maneuver_controller_node yaml-cpp)
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

install(TARGETS
  apriltag_parking_controller_node
  camping_site_maneuver_controller_node
  cmd_vel_safety_gate_node
  drop_zone_maneuver_controller_node
  reverse_parking_controller_node
  route_safety_recovery_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# HH_260721 - Install no Python runtime helpers after all control nodes became native C++.

# HH_260720 - Keep runtime cache artifacts out of the installed control package.
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # HH_260721 - Run native gate, cost-stop, charging, and reverse-axis regression tests.
  ament_add_gtest(test_control_policies
    test/test_control_policies.cpp
    src/motion_cost_stop.cpp
  )
  target_include_directories(test_control_policies PRIVATE include)
  ament_target_dependencies(test_control_policies avg_msgs)

  ament_add_gtest(test_reverse_parking_axis test/test_reverse_parking_axis.cpp)
  target_include_directories(test_reverse_parking_axis PRIVATE include)
  ament_target_dependencies(test_reverse_parking_axis avg_msgs geometry_msgs diagnostic_msgs rclcpp)
endif()

ament_package()
