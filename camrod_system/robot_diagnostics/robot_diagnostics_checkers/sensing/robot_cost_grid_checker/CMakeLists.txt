cmake_minimum_required(VERSION 3.8)
project(robot_cost_grid_checker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(robot_diagnostics_base REQUIRED)

set(DEPS
  rclcpp
  nav_msgs
  diagnostic_msgs
  diagnostic_updater
  robot_diagnostics_base
)

add_executable(cost_grid_checker_node src/cost_grid_checker_node.cpp)
ament_target_dependencies(cost_grid_checker_node ${DEPS})

install(TARGETS cost_grid_checker_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
