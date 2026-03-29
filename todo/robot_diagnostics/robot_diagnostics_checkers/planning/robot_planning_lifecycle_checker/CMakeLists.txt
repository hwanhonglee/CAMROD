cmake_minimum_required(VERSION 3.8)
project(robot_planning_lifecycle_checker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(robot_diagnostics_base REQUIRED)

set(DEPS
  rclcpp
  lifecycle_msgs
  diagnostic_msgs
  diagnostic_updater
  robot_diagnostics_base
)

add_executable(planning_lifecycle_checker_node
  src/planning_lifecycle_checker_node.cpp)
ament_target_dependencies(planning_lifecycle_checker_node ${DEPS})

install(TARGETS
  planning_lifecycle_checker_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
